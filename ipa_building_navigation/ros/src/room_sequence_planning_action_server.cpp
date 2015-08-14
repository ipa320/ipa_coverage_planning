/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_building_navigation
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 08.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ipa_building_navigation/room_sequence_planning_action_server.h>

RoomSequencePlanning::RoomSequencePlanning(ros::NodeHandle nh)
:	node_handle_(nh),
	room_sequence_with_checkpoints_server_(node_handle_, "room_sequence_planning_server", boost::bind(&RoomSequencePlanning::findRoomSequenceWithCheckpointsServer, this, _1), false)
{
	std::cout << "test" << std::endl;

	// start action server
	room_sequence_with_checkpoints_server_.start();

	// Parameters
	std::cout << "\n--------------------------\nRoom Sequence Planner Parameters:\n--------------------------\n";
	node_handle_.param("tsp_solver", tsp_solver_, 3);
	std::cout << "room_sequence_planning/tsp_solver = " << tsp_solver_ << std::endl;
	if (tsp_solver_ == 1)
		ROS_INFO("You have chosen the Nearest Neighbor TSP method.");
	else if (tsp_solver_ == 2)
		ROS_INFO("You have chosen the Genetic TSP method.");
	else if (tsp_solver_ == 3)
		ROS_INFO("You have chosen the Concorde TSP solver.");
	node_handle_.param("display_map", display_map_, false);
	std::cout << "room_sequence_planning/display_map = " << display_map_ << std::endl;
}

void RoomSequencePlanning::findRoomSequenceWithCheckpointsServer(const ipa_building_navigation::FindRoomSequenceWithCheckpointsGoalConstPtr &goal)
{

//	sensor_msgs/Image input_map 				# floor plan map [mono8 format], 0=obstacle or unknown, 255=free space
//	float32 map_resolution						# the resolution of the map in [meter/cell]
//	geometry_msgs/Pose map_origin				# the origin of the map in [meter]
//	float64 max_clique_path_length				# max A* path length between two rooms that are assigned to the same clique
//	geometry_msgs/Pose robot_start_coordinate	# current robot location (used to determine the closest checkpoint in the sequence of checkpoints) [in meter]
//ipa_building_navigation/RoomSequence[] checkpoints

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat floor_plan = cv_ptr_obj->image;

	//get map origin and convert robot start coordinate to [pixel]
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	cv::Point robot_start_coordinate((goal->robot_start_coordinate.position.x - map_origin.x)/goal->map_resolution, (goal->robot_start_coordinate.position.y - map_origin.y)/goal->map_resolution);

	//get room center
	std::vector<cv::Point> room_centers(goal->room_information_in_pixel.size());
	for (size_t i=0; i<room_centers.size(); ++i)
		room_centers[i] = cv::Point(goal->room_information_in_pixel[i].room_center.x, goal->room_information_in_pixel[i].room_center.y);

	// 1. determine cliques of rooms
	std::vector< std::vector<int> > cliques = set_cover_solver_.solveSetCover(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, goal->max_clique_path_length);

	// 2. determine trolley position within each clique
	std::vector<cv::Point> trolley_positions = trolley_position_finder_.findTrolleyPositions(floor_plan, cliques, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);

	// 3. determine optimal sequence of trolley positions (solve TSP problem)
	//		a) find nearest trolley location to current robot location
	//		b) solve the TSP for the trolley positions
	// reduce image size already here to avoid resizing in the planner each time
	const double one_by_downsampling_factor = 1./goal->map_downsampling_factor;
	cv::Mat downsampled_map;
	a_star_path_planner_.downsampleMap(floor_plan, downsampled_map, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
	//find nearest trolley position as start point for TSP
	double min_dist = 1e10;
	size_t optimal_trolley_start_position = 0;
	const cv::Point start_point = goal->map_downsampling_factor * robot_start_coordinate;
	for (size_t i=0; i<trolley_positions.size(); ++i)
	{
		const cv::Point end_point = goal->map_downsampling_factor * trolley_positions[i];
		double dist = one_by_downsampling_factor * a_star_path_planner_.planPath(downsampled_map, start_point, end_point, 1., 0., goal->map_resolution);
		if (dist < min_dist)
		{
			min_dist = dist;
			optimal_trolley_start_position = i;
		}
	}
	//solve the TSP
	std::vector<int> optimal_trolley_sequence;
	if(tsp_solver_ == 1) //nearest neighbor TSP solver
	{
		optimal_trolley_sequence = nearest_neighbor_tsp_solver_.solveNearestTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
	}
	if(tsp_solver_ == 2) //genetic TSP solver
	{
		optimal_trolley_sequence = genetic_tsp_solver_.solveGeneticTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
	}
	if(tsp_solver_ == 3) //concorde TSP solver
	{
		optimal_trolley_sequence = concorde_tsp_solver_.solveConcordeTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
	}

	// 4. determine optimal sequence of rooms with each clique (solve TSP problem)
	//		a) find start point for each clique closest to the trolley position
	//		b) create a vector< vector <Point> > to fill the groups into the TSP solver
	//		c) solve the TSP for each clique
	//find starting points
	std::vector<int> clique_starting_points(cliques.size());
	for(size_t i=0; i<cliques.size(); ++i)
	{
		const cv::Point start_point = goal->map_downsampling_factor * trolley_positions[i];
		size_t starting_index_for_clique;
		double best_distance = 1e10;
		for (size_t j=0; j<trolley_positions.size(); ++j)
		{
			const cv::Point end_point = goal->map_downsampling_factor * room_centers[cliques[i][j]];
			double dist = one_by_downsampling_factor * a_star_path_planner_.planPath(downsampled_map, start_point, end_point, 1., 0., goal->map_resolution);
			if (dist < best_distance)
			{
				best_distance = dist;
				starting_index_for_clique = i;
			}
		}
		clique_starting_points[i] = (int) starting_index_for_clique;
	}
	//create vector of cv::Point for TSP solver
	std::vector<std::vector<cv::Point> > room_cliques_as_points(cliques.size());
	for(size_t i=0; i<cliques.size(); ++i)
	{
		for(size_t j=0; j<cliques[i].size(); ++j)
		{
			room_cliques_as_points[i].push_back(room_centers[cliques[i][j]]);
		}
	}
	//solve TSPs
	std::vector< std::vector <int> > optimal_room_sequences(cliques.size());
	if(tsp_solver_ == 1) //nearest neighbor TSP solver
	{
		for(size_t i=0; i<cliques.size(); ++i)
		{
			optimal_room_sequences[i] = nearest_neighbor_tsp_solver_.solveNearestTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
		}
	}
	if(tsp_solver_ == 2) //genetic TSP solver
	{
		for(size_t i=0; i<cliques.size(); ++i)
		{
			optimal_room_sequences[i] = genetic_tsp_solver_.solveGeneticTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
		}
	}
	if(tsp_solver_ == 3) //concorde TSP solver
	{
		for(size_t i=0; i<cliques.size(); ++i)
		{
			optimal_room_sequences[i] = concorde_tsp_solver_.solveConcordeTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
		}
	}
	// return results

	ipa_building_navigation::FindRoomSequenceWithCheckpointsResult action_result;

	std::vector<ipa_building_navigation::RoomSequence> room_sequences(cliques.size());


	for(size_t i=0; i<cliques.size(); ++i)
	{
		//convert signed int to unsigned int (necessary for this msg type)
		std::vector<unsigned int> cliques_unsigned_int(cliques[i].begin(), cliques[i].end());
		room_sequences[i].room_indices = cliques_unsigned_int;
		room_sequences[i].checkpoint_position_in_pixel.x = trolley_positions[i].x;
		room_sequences[i].checkpoint_position_in_pixel.y = trolley_positions[i].y;
		room_sequences[i].checkpoint_position_in_meter.x = convert_pixel_to_meter_for_x_coordinate(trolley_positions[i].x, goal->map_resolution, map_origin);
		room_sequences[i].checkpoint_position_in_meter.y =convert_pixel_to_meter_for_y_coordinate(trolley_positions[i].y, goal->map_resolution, map_origin);
	}
	action_result.checkpoints = room_sequences;

	room_sequence_with_checkpoints_server_.setSucceeded(action_result);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_sequence_planning_server");
	ros::NodeHandle nh;

	RoomSequencePlanning sqp(nh);
	ROS_INFO("Action server for room sequence planning has been initialized......");
	ros::spin();

	return 0;
}

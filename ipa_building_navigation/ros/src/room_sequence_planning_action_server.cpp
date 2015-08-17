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
	room_sequence_with_checkpoints_server_(nh, "room_sequence_planning_server", boost::bind(&RoomSequencePlanning::findRoomSequenceWithCheckpointsServer, this, _1), false)
{
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
	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat floor_plan = cv_ptr_obj->image;

	//get map origin and convert robot start coordinate to [pixel]
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	cv::Point robot_start_coordinate((goal->robot_start_coordinate.position.x - map_origin.x)/goal->map_resolution, (goal->robot_start_coordinate.position.y - map_origin.y)/goal->map_resolution);

	std::cout << "robot_start_coordinate: " << robot_start_coordinate << std::endl;


	//get room center and check how many of them are reachable
	cv::Mat downsampled_map_for_accessability_checking;
	a_star_path_planner_.downsampleMap(floor_plan, downsampled_map_for_accessability_checking, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
	std::vector<cv::Point> room_centers;
	for (size_t i=0; i<goal->room_information_in_pixel.size(); ++i)
	{
		cv::Point current_center(goal->room_information_in_pixel[i].room_center.x, goal->room_information_in_pixel[i].room_center.y);
		if(a_star_path_planner_.planPath(downsampled_map_for_accessability_checking, goal->map_downsampling_factor * robot_start_coordinate, goal->map_downsampling_factor * current_center, 1., 0., goal->map_resolution) < 9000)
			room_centers.push_back(current_center);
	}

	std::cout << "Anzahl erreichbarer Raumzentren: " << room_centers.size() << std::endl;

	if(room_centers.size() == 0)
	{
		ROS_ERROR("No given roomcenter reachable from starting Position.");
		return;
	}
	// 1. determine cliques of rooms
	std::vector< std::vector<int> > cliques = set_cover_solver_.solveSetCover(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, goal->max_clique_path_length/goal->map_resolution);

	// 2. determine trolley position within each clique (same indexing as in cliques)
	std::vector<cv::Point> trolley_positions = trolley_position_finder_.findTrolleyPositions(floor_plan, cliques, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);

	std::cout << std::endl << "Gefundende Trolleypositionen: " << std::endl;
	for(size_t i=0; i<trolley_positions.size(); ++i)
	{
		std::cout << trolley_positions[i] << std::endl;
	}
	std::cout << std::endl;

	// 3. determine optimal sequence of trolley positions (solve TSP problem)
	//		a) find nearest trolley location to current robot location
	//		b) solve the TSP for the trolley positions
	// reduce image size already here to avoid resizing in the planner each time
	size_t optimal_trolley_start_position = getNearestLocation(floor_plan, robot_start_coordinate, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
	std::cout << "optimal_trolley_start_position: " << trolley_positions[optimal_trolley_start_position] << std::endl;
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
	std::cout << "done Trolley sequence" << std::endl;

	// 4. determine optimal sequence of rooms with each clique (solve TSP problem)
	//		a) find start point for each clique closest to the trolley position
	//		b) create a vector< vector <Point> > to fill the groups into the TSP solver
	//		c) solve the TSP for each clique
	//create vector of cv::Point for TSP solver
	std::vector< std::vector<cv::Point> > room_cliques_as_points;
	for(size_t i=0; i<cliques.size(); ++i)
	{
		std::vector<cv::Point> current_clique;
		for(size_t j=0; j<cliques[i].size(); ++j)
		{
			current_clique.push_back(room_centers[cliques[i][j]]);
		}
		room_cliques_as_points.push_back(current_clique);
	}
	//find starting points
	std::vector<size_t> clique_starting_points(cliques.size());
	for(size_t i=0; i<cliques.size(); ++i)
		clique_starting_points[i] = getNearestLocation(floor_plan, trolley_positions[i], room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
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

	// display
	if (display_map_ == true)
	{
		cv::Mat display;
		cv::cvtColor(floor_plan, display, CV_GRAY2BGR);

		std::cout << "Alle Raumpunkte in Cliquen: " << std::endl;
		for(size_t i=0; i<room_cliques_as_points.size(); ++i)
		{
			std::cout << room_cliques_as_points[i] << std::endl;
		}

		std::cout << "Optimale Reihenfolge der Trolleypositionen (als Indizes): " << std::endl;
		for(size_t i=0; i<optimal_trolley_sequence.size(); ++i)
		{
			std::cout << optimal_trolley_sequence[i] << std::endl;
		}

		std::cout << std::endl;

		for (size_t t=0; t<trolley_positions.size(); ++t)
		{
			// trolley positions + connections
			const int ot = optimal_trolley_sequence[t];
			if (t>0)
			{
				std::cout << "Trolley Position: " << trolley_positions[ot] << std::endl;
				cv::circle(display, trolley_positions[ot], 5, CV_RGB(0,0,255), CV_FILLED);
				cv::line(display, trolley_positions[ot], trolley_positions[optimal_trolley_sequence[t-1]], CV_RGB(128,128,255), 1);
			}
			else
			{
				cv::circle(display, trolley_positions[ot], 5, CV_RGB(255,0,0), CV_FILLED);
				std::cout << trolley_positions[ot] << std::endl << " Anzahl Trolleypositionen: " << trolley_positions.size() << " Anzahl Raumsequenzen: " << optimal_room_sequences.size() << " Anzahl Cliquen: " << room_cliques_as_points.size() << std::endl << std::endl;
			}

			std::cout << "Raumpunkte der Clique (Größe: " << room_cliques_as_points[ot].size() << ", cliquenindex: " << ot << ", Größe der Sequenz: " << optimal_room_sequences[ot].size() << ")" << std::endl;
			// room positions and connections
			for (size_t r=0; r<optimal_room_sequences[ot].size(); ++r)
			{

				cv::circle(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r]], 3, CV_RGB(0,255,0), CV_FILLED);
				if (r==0)
					cv::line(display, trolley_positions[ot], room_cliques_as_points[ot][optimal_room_sequences[ot][r]], CV_RGB(255,0,0), 1);
				else
				{
					if(r==optimal_room_sequences[ot].size()-1 && optimal_room_sequences.size()>2)
						cv::line(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r]], trolley_positions[ot], CV_RGB(255,0,255), 1);
					cv::line(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r-1]], room_cliques_as_points[ot][optimal_room_sequences[ot][r]], CV_RGB(128,255,128), 1);
				}
				std::cout << room_cliques_as_points[ot][optimal_room_sequences[ot][r]] << "mit Index: " << optimal_room_sequences[ot][r] << std::endl << std::endl;
			}
		}

		cv::imshow("sequence planning", display);
		cv::waitKey();
	}

	// return results
	ipa_building_navigation::FindRoomSequenceWithCheckpointsResult action_result;
	std::vector<ipa_building_navigation::RoomSequence> room_sequences(cliques.size());
	for(size_t i=0; i<cliques.size(); ++i)
	{
		//convert signed int to unsigned int (necessary for this msg type)
		room_sequences[i].room_indices = std::vector<unsigned int>(cliques[i].begin(), cliques[i].end());
		room_sequences[i].checkpoint_position_in_pixel.x = trolley_positions[i].x;
		room_sequences[i].checkpoint_position_in_pixel.y = trolley_positions[i].y;
		room_sequences[i].checkpoint_position_in_meter.x = convert_pixel_to_meter_for_x_coordinate(trolley_positions[i].x, goal->map_resolution, map_origin);
		room_sequences[i].checkpoint_position_in_meter.y = convert_pixel_to_meter_for_y_coordinate(trolley_positions[i].y, goal->map_resolution, map_origin);
	}
	action_result.checkpoints = room_sequences;

	room_sequence_with_checkpoints_server_.setSucceeded(action_result);
}

size_t RoomSequencePlanning::getNearestLocation(const cv::Mat& floor_plan, const cv::Point start_coordinate, const std::vector<cv::Point>& positions,
		const double map_downsampling_factor, const double robot_radius, const double map_resolution)
{
	const double one_by_downsampling_factor = 1./map_downsampling_factor;
	cv::Mat downsampled_map;
	a_star_path_planner_.downsampleMap(floor_plan, downsampled_map, map_downsampling_factor, robot_radius, map_resolution);
	//find nearest trolley position as start point for TSP
	double min_dist = 1e10;
	size_t nearest_position = 0;
	const cv::Point start_point = map_downsampling_factor * start_coordinate;
	for (size_t i=0; i<positions.size(); ++i)
	{
		const cv::Point end_point = map_downsampling_factor * positions[i];
		double dist = one_by_downsampling_factor * a_star_path_planner_.planPath(downsampled_map, start_point, end_point, 1., 0., map_resolution);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_position = i;
		}
	}

	return nearest_position;
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

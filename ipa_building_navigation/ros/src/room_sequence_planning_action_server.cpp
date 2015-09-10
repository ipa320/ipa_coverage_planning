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

RoomSequencePlanningServer::RoomSequencePlanningServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_sequence_with_checkpoints_server_(node_handle_, name_of_the_action, boost::bind(&RoomSequencePlanningServer::findRoomSequenceWithCheckpointsServer, this, _1), false),
	action_name_(name_of_the_action)
{
	// start action server
	room_sequence_with_checkpoints_server_.start();

	// Parameters
	std::cout << "\n--------------------------\nRoom Sequence Planner Parameters:\n--------------------------\n";
	node_handle_.param("tsp_solver", tsp_solver_, 3);
	std::cout << "room_sequence_planning/tsp_solver = " << tsp_solver_ << std::endl;
	node_handle_.param("planning_method", planning_method_, 3);
	std::cout << "room_sequence_planning/tsp_solver = " << tsp_solver_ << std::endl;
	if (tsp_solver_ == 1)
		ROS_INFO("You have chosen the Nearest Neighbor TSP method.");
	else if (tsp_solver_ == 2)
		ROS_INFO("You have chosen the Genetic TSP method.");
	else if (tsp_solver_ == 3)
		ROS_INFO("You have chosen the Concorde TSP solver.");
	else
		ROS_INFO("Undefined TSP Solver.");
	if (planning_method_ == 1)
		ROS_INFO("You have chosen the Trolley dragging method method.");
	else if (planning_method_ == 2)
		ROS_INFO("You have chosen the roomgroup planning method.");
	else
		ROS_INFO("Undefined planning method.");
	node_handle_.param("display_map", display_map_, false);
	std::cout << "room_sequence_planning/display_map = " << display_map_ << std::endl;
}

void RoomSequencePlanningServer::findRoomSequenceWithCheckpointsServer(const ipa_building_navigation::FindRoomSequenceWithCheckpointsGoalConstPtr &goal)
{
	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat floor_plan = cv_ptr_obj->image;

	//get map origin and convert robot start coordinate to [pixel]
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	cv::Point robot_start_coordinate((goal->robot_start_coordinate.position.x - map_origin.x)/goal->map_resolution, (goal->robot_start_coordinate.position.y - map_origin.y)/goal->map_resolution);


	//create a star pathplanner to plan a path from Point A to Point B in a given gridmap
	AStarPlanner a_star_path_planner;

	//get room center and check how many of them are reachable
	cv::Mat downsampled_map_for_accessability_checking;
	if(goal->check_accessibility_of_rooms == true)
	{
		a_star_path_planner.downsampleMap(floor_plan, downsampled_map_for_accessability_checking, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
	}
	std::vector<cv::Point> room_centers;
	for (size_t i=0; i<goal->room_information_in_pixel.size(); ++i)
	{
		cv::Point current_center(goal->room_information_in_pixel[i].room_center.x, goal->room_information_in_pixel[i].room_center.y);
		if(goal->check_accessibility_of_rooms == true)
		{
			std::cout << "checking for accessibility of rooms" << std::endl;
			if(a_star_path_planner.planPath(downsampled_map_for_accessability_checking, goal->map_downsampling_factor * robot_start_coordinate, goal->map_downsampling_factor * current_center, 1., 0., goal->map_resolution) < 9000)
				room_centers.push_back(current_center);
		}
		else
		{
			room_centers.push_back(current_center);
		}
	}
	downsampled_map_for_accessability_checking.release(); //release not anymore needed space

	std::cout << "number of reachable roomcenters: " << room_centers.size() << std::endl;

	if(room_centers.size() == 0)
	{
		ROS_ERROR("No given roomcenter reachable from starting position.");
		return;
	}

	if(goal->planning_method > 0 && goal->planning_method < 3)
	{
		std::cout << "set planning method from goal" << std::endl;
		planning_method_ = goal->planning_method;
		if(planning_method_ == 1)
			ROS_INFO("You have chosen the drag planning method.");
		if(planning_method_ == 2)
			ROS_INFO("You have chosen the grouping planning method.");
	}

	if(goal->tsp_solver > 0 && goal->tsp_solver < 4)
	{
		std::cout << "set tsp solver from goal" << std::endl;
		tsp_solver_ = goal->tsp_solver;
		if(tsp_solver_ == 1)
			ROS_INFO("You have chosen the nearest neighbor solver.");
		if(tsp_solver_ == 2)
			ROS_INFO("You have chosen the genetic TSP solver.");
		if(tsp_solver_ == 3)
			ROS_INFO("You have chosen the concorde TSP solver.");
	}
	//saving vectors needed from both planning methods
	std::vector<std::vector<int> > cliques;
	std::vector<cv::Point> trolley_positions;
	std::vector< std::vector<cv::Point> > room_cliques_as_points;

	//image container to draw the sequence in if needed
	cv::Mat display;


	if(planning_method_ == 1) //Drag Trolley if the next room is too far away
	{
		std::cout << "Maximal cliquedistance [m]: "<< goal->max_clique_path_length  << " Maximal cliquedistance [Pixel]: "<< goal->max_clique_path_length/goal->map_resolution << std::endl;

		//calculate the index of the best starting position
		size_t optimal_start_position = getNearestLocation(floor_plan, robot_start_coordinate, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);

		//plan the optimal path trough all given rooms
		std::vector<int> optimal_room_sequence;
		if(tsp_solver_ == 1) //nearest neighbor TSP solver
		{
			NearestNeighborTSPSolver nearest_neighbor_tsp_solver;
			optimal_room_sequence = nearest_neighbor_tsp_solver.solveNearestTSP(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_start_position);
		}
		if(tsp_solver_ == 2) //genetic TSP solver
		{
			GeneticTSPSolver genetic_tsp_solver;
			optimal_room_sequence = genetic_tsp_solver.solveGeneticTSP(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_start_position);
		}
		if(tsp_solver_ == 3) //concorde TSP solver
		{
			ConcordeTSPSolver concorde_tsp_solver;
			optimal_room_sequence = concorde_tsp_solver.solveConcordeTSP(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_start_position);
		}

		//put the rooms that are close enough together into the same clique, if a new clique is needed put the first roomcenter as a trolleyposition
		std::vector<int> current_clique;
		trolley_positions.push_back(robot_start_coordinate); //trolley stands close to robot on startup
		//sample down map one time to reduce calculation time
		cv::Mat downsampled_map;
		a_star_path_planner.downsampleMap(floor_plan, downsampled_map, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
		double one_by_downsampling_factor = 1 / goal->map_downsampling_factor;
		for(size_t i=0; i<optimal_room_sequence.size(); ++i)
		{
			double distance_to_trolley = one_by_downsampling_factor * a_star_path_planner.planPath(downsampled_map, goal->map_downsampling_factor * trolley_positions.back(), goal->map_downsampling_factor * room_centers[optimal_room_sequence[i]], 1., 0, goal->map_resolution);
			if( distance_to_trolley <= goal->max_clique_path_length/goal->map_resolution) //expand current clique by next roomcenter
			{
				current_clique.push_back(optimal_room_sequence[i]);
			}
			else //start new clique and put old in the cliques vector
			{
				cliques.push_back(current_clique);
				current_clique.clear();
				current_clique.push_back(optimal_room_sequence[i]);
				trolley_positions.push_back(room_centers[optimal_room_sequence[i]]);
			}
		}
		//add last clique
		cliques.push_back(current_clique);

		//fill vector of cv::Point for display purpose
		for(size_t i=0; i<cliques.size(); ++i)
		{
			std::vector<cv::Point> current_clique;
			for(size_t j=0; j<cliques[i].size(); ++j)
			{
				current_clique.push_back(room_centers[cliques[i][j]]);
			}
			room_cliques_as_points.push_back(current_clique);
		}

		if(goal->return_sequence_map == true)
				{
					cv::cvtColor(floor_plan, display, CV_GRAY2BGR);

					for (size_t t=0; t<trolley_positions.size(); ++t)
					{
						// trolley positions + connections
						if (t>0)
						{
							cv::circle(display, trolley_positions[t], 5, CV_RGB(0,0,255), CV_FILLED);
							cv::line(display, trolley_positions[t], trolley_positions[t-1], CV_RGB(128,128,255), 1);
						}
						else
						{
							cv::circle(display, trolley_positions[t], 5, CV_RGB(255,0,0), CV_FILLED);
						}

						// room positions and connections
						for (size_t r=0; r<cliques[t].size(); ++r)
						{
							cv::circle(display, room_cliques_as_points[t][r], 3, CV_RGB(0,255,0), CV_FILLED);
							if (r==0)
								cv::line(display, trolley_positions[t], room_cliques_as_points[t][r], CV_RGB(255,0,0), 1);
							else
							{
								if(r==cliques[t].size()-1)
									cv::line(display, room_cliques_as_points[t][r], trolley_positions[t], CV_RGB(255,0,255), 1);
								cv::line(display, room_cliques_as_points[t][r-1], room_cliques_as_points[t][r], CV_RGB(128,255,128), 1);
							}
						}
					}
				}
				// display
				if (display_map_ == true && goal->return_sequence_map == true)
				{
					cv::imshow("sequence planning", display);
					cv::waitKey();
				}

	}

	else if(planning_method_ == 2) //calculate roomgroups and corresponding trolley positions
	{
		std::cout << "Maximal cliquedistance [m]: "<< goal->max_clique_path_length  << " Maximal cliquedistance [Pixel]: "<< goal->max_clique_path_length/goal->map_resolution << std::endl;

		std::cout << "finding trolley positions" << std::endl;
		// 1. determine cliques of rooms
		SetCoverSolver set_cover_solver;
		cliques = set_cover_solver.solveSetCover(floor_plan, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, goal->max_clique_path_length/goal->map_resolution);

		// 2. determine trolley position within each clique (same indexing as in cliques)
		TrolleyPositionFinder trolley_position_finder;
		trolley_positions = trolley_position_finder.findTrolleyPositions(floor_plan, cliques, room_centers, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);

		// 3. determine optimal sequence of trolley positions (solve TSP problem)
		//		a) find nearest trolley location to current robot location
		//		b) solve the TSP for the trolley positions
		// reduce image size already here to avoid resizing in the planner each time
		size_t optimal_trolley_start_position = getNearestLocation(floor_plan, robot_start_coordinate, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);

		//solve the TSP
		std::vector<int> optimal_trolley_sequence;
		std::cout << "finding optimal trolley sequence. Start: " << optimal_trolley_start_position << std::endl;
		if(tsp_solver_ == 1) //nearest neighbor TSP solver
		{
			NearestNeighborTSPSolver nearest_neighbor_tsp_solver;
			optimal_trolley_sequence = nearest_neighbor_tsp_solver.solveNearestTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
		}
		if(tsp_solver_ == 2) //genetic TSP solver
		{
			GeneticTSPSolver genetic_tsp_solver;
			optimal_trolley_sequence = genetic_tsp_solver.solveGeneticTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
		}
		if(tsp_solver_ == 3) //concorde TSP solver
		{
			ConcordeTSPSolver concorde_tsp_solver;
			optimal_trolley_sequence = concorde_tsp_solver.solveConcordeTSP(floor_plan, trolley_positions, goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, (int) optimal_trolley_start_position);
		}

		// 4. determine optimal sequence of rooms with each clique (solve TSP problem)
		//		a) find start point for each clique closest to the trolley position
		//		b) create a vector< vector <Point> > to fill the groups into the TSP solver
		//		c) solve the TSP for each clique


		//fill vector of cv::Point for TSP solver
		for(size_t i=0; i<cliques.size(); ++i)
		{
			std::vector<cv::Point> current_clique;
			for(size_t j=0; j<cliques[i].size(); ++j)
			{
				current_clique.push_back(room_centers[cliques[i][j]]);
			}
			room_cliques_as_points.push_back(current_clique);
		}

		std::cout << "cliques: " << std::endl;
		for(size_t i = 0; i < cliques.size(); ++i)
		{
			for(size_t j = 0; j < cliques[i].size(); ++j)
			{
				std::cout << room_cliques_as_points[i][j] << std::endl;
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		std::cout << "trolley indexes: " << std::endl;
		for(size_t i = 0; i < optimal_trolley_sequence.size(); ++i)
		{
			std::cout << optimal_trolley_sequence[i] << std::endl;
		}
		std::cout << std::endl;

		std::cout << "number of trolley positions: " << trolley_positions.size() << " " << optimal_trolley_sequence.size() << std::endl;
		//find starting points
		//find starting points
		std::vector<size_t> clique_starting_points(cliques.size());
		for(size_t i=0; i<cliques.size(); ++i)
			clique_starting_points[i] = getNearestLocation(floor_plan, trolley_positions[i], room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution);
		//solve TSPs
		std::vector< std::vector <int> > optimal_room_sequences(cliques.size());
		// Create the TSP solver objects two times and not one time for the whole function because by this way only two objects has to be
		// created and else three
		if(tsp_solver_ == 1) //nearest neighbor TSP solver
		{
			NearestNeighborTSPSolver nearest_neighbor_tsp_solver;
			for(size_t i=0; i<cliques.size(); ++i)
			{
				optimal_room_sequences[i] = nearest_neighbor_tsp_solver.solveNearestTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
				std::cout << "done one clique" << std::endl;
			}
		}
		if(tsp_solver_ == 2) //genetic TSP solver
		{
			GeneticTSPSolver genetic_tsp_solver;
			for(size_t i=0; i<cliques.size(); ++i)
			{
				optimal_room_sequences[i] = genetic_tsp_solver.solveGeneticTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
				std::cout << "done one clique" << std::endl;
			}
		}
		if(tsp_solver_ == 3) //concorde TSP solver
		{
			ConcordeTSPSolver concorde_tsp_solver;
			for(size_t i=0; i<cliques.size(); ++i)
			{
				optimal_room_sequences[i] = concorde_tsp_solver.solveConcordeTSP(floor_plan, room_cliques_as_points[i], goal->map_downsampling_factor, goal->robot_radius, goal->map_resolution, clique_starting_points[i]);
				std::cout << "done one clique" << std::endl;
			}
		}

		if(goal->return_sequence_map == true)
		{
			cv::cvtColor(floor_plan, display, CV_GRAY2BGR);

			for (size_t t=0; t<trolley_positions.size(); ++t)
			{
				// trolley positions + connections
				const int ot = optimal_trolley_sequence[t];
				std::cout << "starting to draw one clique. Position: " << trolley_positions[ot] << std::endl;
				if (t>0)
				{
					cv::circle(display, trolley_positions[ot], 5, CV_RGB(0,0,255), CV_FILLED);
					cv::line(display, trolley_positions[ot], trolley_positions[optimal_trolley_sequence[t-1]], CV_RGB(128,128,255), 1);
				}
				else
				{
					cv::circle(display, trolley_positions[ot], 5, CV_RGB(255,0,0), CV_FILLED);
				}

				// room positions and connections
				for (size_t r=0; r<optimal_room_sequences[ot].size(); ++r)
				{
					cv::circle(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r]], 3, CV_RGB(0,255,0), CV_FILLED);
					if (r==0)
						cv::line(display, trolley_positions[ot], room_cliques_as_points[ot][optimal_room_sequences[ot][r]], CV_RGB(255,0,0), 1);
					else
					{
						if(r==optimal_room_sequences[ot].size()-1)
							cv::line(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r]], trolley_positions[ot], CV_RGB(255,0,255), 1);
						cv::line(display, room_cliques_as_points[ot][optimal_room_sequences[ot][r-1]], room_cliques_as_points[ot][optimal_room_sequences[ot][r]], CV_RGB(128,255,128), 1);
					}
				}
				std::cout << "finished to draw one clique" << std::endl;
			}
		}
		// display
		if (display_map_ == true && goal->return_sequence_map == true)
		{
			cv::imshow("sequence planning", display);
			cv::waitKey();
		}
	}
	else
	{
		ROS_ERROR("Undefined planning method.");
		return;
	}

	std::cout << "done sequence planning" << std::endl;

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
	if(goal->return_sequence_map == true)
	{
		//converting the cv format in map msg format
		cv_bridge::CvImage cv_image;
		cv_image.header.stamp = ros::Time::now();
		cv_image.encoding = "bgr8";
		cv_image.image = display;
		cv_image.toImageMsg(action_result.sequence_map);
	}

	room_sequence_with_checkpoints_server_.setSucceeded(action_result);

	//garbage collection
	action_result.checkpoints.clear();

	return;
}

size_t RoomSequencePlanningServer::getNearestLocation(const cv::Mat& floor_plan, const cv::Point start_coordinate, const std::vector<cv::Point>& positions,
		const double map_downsampling_factor, const double robot_radius, const double map_resolution)
{
	const double one_by_downsampling_factor = 1./map_downsampling_factor;
	cv::Mat downsampled_map;
	AStarPlanner a_star_path_planner;
	a_star_path_planner.downsampleMap(floor_plan, downsampled_map, map_downsampling_factor, robot_radius, map_resolution);
	//find nearest trolley position as start point for TSP
	double min_dist = 1e10;
	size_t nearest_position = 0;
	const cv::Point start_point = map_downsampling_factor * start_coordinate;
	for (size_t i=0; i<positions.size(); ++i)
	{
		const cv::Point end_point = map_downsampling_factor * positions[i];
		double dist = one_by_downsampling_factor * a_star_path_planner.planPath(downsampled_map, start_point, end_point, 1., 0., map_resolution);
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

	RoomSequencePlanningServer sqp(nh, ros::this_node::getName());
	ROS_INFO("Action server for room sequence planning has been initialized......");
	ros::spin();

	return 0;
}

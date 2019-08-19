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
#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>

#include <fstream>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ipa_building_navigation/BuildingNavigationConfig.h>

//TSP solver
#include <ipa_building_navigation/tsp_solver_defines.h>
#include <ipa_building_navigation/nearest_neighbor_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>
#include <ipa_building_navigation/concorde_TSP.h>

//Set Cover solver to find room groups
#include <ipa_building_navigation/set_cover_solver.h>

//finder of trolley positions for each room group
#include <ipa_building_navigation/trolley_position_finder.h>

// A* planner
#include <ipa_building_navigation/A_star_pathplanner.h>

// action
#include <actionlib/server/simple_action_server.h>
#include <ipa_building_msgs/FindRoomSequenceWithCheckpointsAction.h>

class RoomSequencePlanningServer
{
public:
	RoomSequencePlanningServer(ros::NodeHandle nh, std::string name_of_the_action);

	~RoomSequencePlanningServer()
	{
	}

protected:
	//!!Important!!
	// define the Nodehandle before the action server, or else the server won't start
	ros::NodeHandle node_handle_;

	ros::Publisher room_sequence_visualization_pub_;	// visualization of the room sequence
	visualization_msgs::MarkerArray room_sequence_visualization_msg_;

	actionlib::SimpleActionServer<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction> room_sequence_with_checkpoints_server_;

	std::string action_name_;

	//converter-> Pixel to meter for X coordinate
	double convert_pixel_to_meter_for_x_coordinate(const int pixel_valued_object_x, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution) + map_origin.x;
		return meter_value_obj_x;
	}
	//converter-> Pixel to meter for Y coordinate
	double convert_pixel_to_meter_for_y_coordinate(int pixel_valued_object_y, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution) + map_origin.y;
		return meter_value_obj_y;
	}

	// this is the execution function used by action server
	void findRoomSequenceWithCheckpointsServer(const ipa_building_msgs::FindRoomSequenceWithCheckpointsGoalConstPtr &goal);

	size_t getNearestLocation(const cv::Mat& floor_plan, const cv::Point start_coordinate, const std::vector<cv::Point>& positions,
			const double map_downsampling_factor, const double robot_radius, const double map_resolution);

	void publishSequenceVisualization(const std::vector<ipa_building_msgs::RoomSequence>& room_sequences, const std::vector<cv::Point>& room_centers,
			std::vector< std::vector<int> >& cliques, const double map_resolution, const cv::Point2d& map_origin);

	// callback function for dynamic reconfigure
	void dynamic_reconfigure_callback(ipa_building_navigation::BuildingNavigationConfig &config, uint32_t level);

	dynamic_reconfigure::Server<ipa_building_navigation::BuildingNavigationConfig> room_sequence_planning_dynamic_reconfigure_server_;

	// params
	int tsp_solver_;		// TSP solver: 1 = Nearest Neighbor,  2 = Genetic solver,  3 = Concorde solver
	int problem_setting_;	// problem setting of the sequence planning problem
							//   1 = SimpleOrderPlanning (plan the optimal order of a simple set of locations)
							//   2 = CheckpointBasedPlanning (two-stage planning that creates local cliques of locations (= checkpoints) and determines
							//        the optimal order through the members of each clique as well as the optimal order through the cliques)
	int planning_method_;	// Method of planning the sequence: 1 = drag trolley if next room is too far away, 2 = calculate cliques as roomgroups with trolleypositions
	double max_clique_path_length_;	// max A* path length between two rooms that are assigned to the same clique, in [m]
	double map_downsampling_factor_;	// the map may be downsampled during computations (e.g. of A* path lengths) in order to speed up the algorithm, range of the factor [0 < factor <= 1], if set to 1 the map will have original size, if set to 0 the algorithm won't work
	bool check_accessibility_of_rooms_;	// boolean to tell the sequence planner if it should check the given room centers for accessibility from the starting position
	bool return_sequence_map_;	// boolean to tell the server if the map with the sequence drawn in should be returned
	int max_clique_size_; // maximal number of nodes belonging to one clique, when planning trolley positions
	bool display_map_;		// displays the map with paths upon service call (only if return_sequence_map=true)
};

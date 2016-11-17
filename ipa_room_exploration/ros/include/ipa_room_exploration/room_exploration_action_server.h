#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>

#include <ipa_room_exploration/RoomExplorationConfig.h>

#include <Eigen/Dense>

#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h>

#include <ipa_building_msgs/RoomExplorationAction.h>
#include <cob_map_accessibility_analysis/CheckPerimeterAccessibility.h>
#include <ipa_room_exploration/concorde_TSP.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

#include <ipa_room_exploration/grid_point_explorator.h>
#include <ipa_room_exploration/boustrophedon_explorator.h>

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
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 03.2016
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

#define PI 3.14159265359

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#pragma once

class RoomExplorationServer
{
protected:

	int path_planning_algorithm_; // variable to specify which algorithm is going to be used to plan a path
									// 1: grid point explorator
									// 2: boustrophedon explorator

	double left_sections_min_area_; // variable to determine the minimal area that not seen sections must have before they
									// are revisited after one go trough the room

	gridPointExplorator grid_point_planner; // object that uses the grid point method to plan a path trough a room
	boustrophedonExplorer boustrophedon_explorer_; // object that uses the boustrophedon exploration method to plan a path trough the room

	// parameters for the different planners
	int grid_line_length_; // size of the grid-lines that the grid-point-explorator lays over the map
	int path_eps_; // the distance between points when generating a path with the boustrophedon exploration method
	bool plan_for_footprint_; // boolean that implies if the path should be planned for the footprint and not for the field of view


	// callback function for dynamic reconfigure
	void dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level);

	// this is the execution function used by action server
	void exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal);

	// function to publish a navigation goal, it returns true if the goal could be reached
	// eps_x and eps_y are used to define a epsilon neighborhood around the goal in which a new nav_goal gets published
	// 	--> may smooth the process, move_base often slows before and stops at the goal
	bool publishNavigationGoal(const geometry_msgs::Pose2D& nav_goal, const std::string map_frame,
			const std::string camera_frame, std::vector<geometry_msgs::Pose2D>& robot_poses, const double eps = 0.0);

	// converter-> Pixel to meter for X coordinate
	double convertPixelToMeterForXCoordinate(const int pixel_valued_object_x, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution) + map_origin.x;
		return meter_value_obj_x;
	}
	// converter-> Pixel to meter for Y coordinate
	double convertPixelToMeterForYCoordinate(int pixel_valued_object_y, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution) + map_origin.y;
		return meter_value_obj_y;
	}

	// function to transform the given map in a way s.t. the OpenCV and room coordinate system are the same
	//	--> the map_saver from ros saves maps as images with the origin laying in the lower left corner of it, but openCV assumes
	//		that the origin is in the upper left corner, also they are rotated around the image-x-axis about each other
	void transformImageToRoomCordinates(cv::Mat& map)
	{
		cv::Point2f src_center(map.cols/2.0F, map.rows/2.0F);
		cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
		cv::Mat dst;
		cv::warpAffine(map, dst, rot_mat, map.size());
		cv::flip(dst, map, 1);
	}

	// function to draw the points that have been covered by the field of view, when the robot moved trough the room
	//		--> use given Poses and original field of view points to do so
	void drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
			const std::vector<geometry_msgs::Point32>& field_of_view_points, const Eigen::Matrix<float, 2, 1> raycasting_corner_1,
			const Eigen::Matrix<float, 2, 1> raycasting_corner_2, const float map_resolution, const cv::Point2d map_origin);

	// function to draw the robot footprint into the given map, when coverage for the robot is wanted
	void drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
				const std::vector<geometry_msgs::Point32>& robot_footprint, const float map_resolution,
				const cv::Point2d map_origin);

	// !!Important!!
	//  define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_building_msgs::RoomExplorationAction> room_exploration_server_;
	dynamic_reconfigure::Server<ipa_room_exploration::RoomExplorationConfig> room_exploration_dynamic_reconfigure_server_;

public:
	// initialize the action-server
	RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action);

	// default destructor for the class
	~RoomExplorationServer(void)
	{
	}
};

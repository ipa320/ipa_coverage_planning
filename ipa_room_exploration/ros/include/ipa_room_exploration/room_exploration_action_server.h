/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
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

#pragma once


// Ros specific
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
// OpenCV specific
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// Eigen library
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
// standard c++ libraries
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
// services and actions
#include <ipa_building_msgs/RoomExplorationAction.h>
#include <cob_map_accessibility_analysis/CheckPerimeterAccessibility.h>
#include <ipa_building_msgs/CheckCoverage.h>
// messages
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// specific from this package
#include <ipa_building_navigation/concorde_TSP.h>
#include <ipa_room_exploration/RoomExplorationConfig.h>
#include <ipa_room_exploration/grid_point_explorator.h>
#include <ipa_room_exploration/boustrophedon_explorator.h>
#include <ipa_room_exploration/neural_network_explorator.h>
#include <ipa_room_exploration/convex_sensor_placement_explorator.h>
#include <ipa_room_exploration/flow_network_explorator.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/energy_functional_explorator.h>
#include <ipa_room_exploration/voronoi.hpp>
#include <ipa_room_exploration/room_rotator.h>
#include <ipa_room_exploration/coverage_check_server.h>


#define PI 3.14159265359

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RoomExplorationServer
{
protected:

	int planning_mode_; // 1 = plans a path for coverage with the robot footprint, 2 = plans a path for coverage with the robot's field of view

	ros::Publisher path_pub_; // a publisher sending the path as a nav_msgs::Path before executing

	GridPointExplorator grid_point_planner; // object that uses the grid point method to plan a path trough a room
	BoustrophedonExplorer boustrophedon_explorer_; // object that uses the boustrophedon exploration method to plan a path trough the room
	NeuralNetworkExplorator neural_network_explorator_; // object that uses the neural network method to create an exploration path
	convexSPPExplorator convex_SPP_explorator_; // object that uses the convex spp exploration methd to create an exploration path
	FlowNetworkExplorator flow_network_explorator_; // object that uses the flow network exploration method to create an exploration path
	EnergyFunctionalExplorator energy_functional_explorator_; // object that uses the energy functional exploration method to create an exploration path
	BoustrophedonVariantExplorer boustrophedon_variant_explorer_; // object that uses the boustrophedon variant exploration method to plan a path trough the room

	// parameters
	int room_exploration_algorithm_;	// variable to specify which algorithm is going to be used to plan a path
										// 1: grid point explorator
										// 2: boustrophedon explorator
										// 3: neural network explorator
										// 4: convexSPP explorator
										// 5: flowNetwork explorator
										// 6: energyFunctional explorator
										// 7: Voronoi explorator
										// 8: boustrophedon variant explorator
	bool display_trajectory_;		// display final trajectory plan step by step

	// parameters on map correction
	int map_correction_closing_neighborhood_size_;	// Applies a closing operation to neglect inaccessible areas and map errors/artifacts if the
													// map_correction_closing_neighborhood_size parameter is larger than 0.
													// The parameter then specifies the iterations (or neighborhood size) of that closing operation.

	// parameters specific to the navigation of the robot along the computed coverage trajectory
	bool return_path_;				// boolean used to determine if the server should return the computed coverage path in the response message
	bool execute_path_;				// boolean used to determine whether the server should navigate the robot along the computed coverage path
	double goal_eps_;				// distance between the published navigation goal and the robot to publish the next
									// navigation goal in the path
	bool use_dyn_goal_eps_;		// using a dynamic goal distance criterion: the larger the path's curvature, the more accurate the navigation
	bool interrupt_navigation_publishing_;	// variable that interrupts the publishing of navigation goals as long as needed, e.g. when during the execution
											// of the coverage path a trash bin is found and one wants to empty it directly and resume the path later.
	bool revisit_areas_;			// variable that turns functionality on/off to revisit areas that haven't been seen during the
									// execution of the coverage path, due to uncertainties or dynamic obstacles
	double left_sections_min_area_; // variable to determine the minimal area that not seen sections must have before they
									// are revisited after one run through the room
	std::string global_costmap_topic_;	// name of the global costmap topic
	std::string coverage_check_service_name_;	// name of the service to call for a coverage check of the driven trajectory
	std::string map_frame_;			// string that carries the name of the map frame, used for tracking of the robot
	std::string camera_frame_;				// string that carries the name of the camera frame, that is in the same kinematic chain as the map_frame and shows the camera pose

	// parameters specific to the grid point explorator
	int tsp_solver_;	// indicates which TSP solver should be used
						//   1 = Nearest Neighbor
						//   2 = Genetic solver
						//   3 = Concorde solver
	int64_t tsp_solver_timeout_;	// a sophisticated solver like Concorde or Genetic can be interrupted if it does not find a solution within this time, in [s], and then falls back to the nearest neighbor solver

	// parameters specific for the boustrophedon explorator
	double min_cell_area_;			// minimal area a cell can have, when using the boustrophedon explorator
	double path_eps_;		// the distance between points when generating a path
	double grid_obstacle_offset_;	// in [m], the additional offset of the grid to obstacles, i.e. allows to displace the grid by more than the standard half_grid_size from obstacles
	int max_deviation_from_track_;	// in [pixel], maximal allowed shift off the ideal boustrophedon track to both sides for avoiding obstacles on track
									// setting max_deviation_from_track=grid_spacing is usually a good choice
									// for negative values (e.g. max_deviation_from_track: -1) max_deviation_from_track is automatically set to grid_spacing
	int cell_visiting_order_;		// cell visiting order
									//   1 = optimal visiting order of the cells determined as TSP problem
									//   2 = alternative ordering from left to right (measured on y-coordinates of the cells), visits the cells in a more obvious fashion to the human observer (though it is not optimal)


	// parameters specific for the neural network explorator, see "A Neural Network Approach to Complete Coverage Path Planning" from Simon X. Yang and Chaomin Luo
	double step_size_; // step size for integrating the state dynamics
	int A_; // decaying parameter that pulls the activity of a neuron closer to zero, larger value means faster decreasing
	int B_; // increasing parameter that tries to increase the activity of a neuron when it's not too big already, higher value means a higher desired value and a faster increasing at the beginning
	int D_; // decreasing parameter when the neuron is labeled as obstacle, higher value means faster decreasing
	int E_; // external input parameter of one neuron that is used in the dynamics corresponding to if it is an obstacle or uncleaned/cleaned, E>>B
	double mu_; // parameter to set the importance of the states of neighboring neurons to the dynamics, higher value means higher influence
	double delta_theta_weight_; // parameter to set the importance of the traveleing direction from the previous step and the next step, a higher value means that the robot should turn less

	// parameters specific for the convexSPP explorator
	int cell_size_;				// size of one cell that is used to discretize the free space
	double delta_theta_;			// sampling angle when creating possible sensing poses in the convexSPP explorator

	// parameters specific for the flowNetwork explorator
	double curvature_factor_; // double that shows the factor, an arc can be longer than a straight arc when using the flowNetwork explorator
	double max_distance_factor_; // double that shows how much an arc can be longer than the maximal distance of the room, which is determined by the min/max coordinates that are set in the goal


	// callback function for dynamic reconfigure
	void dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level);

	// this is the execution function used by action server
	void exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal);

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
	bool removeUnconnectedRoomParts(cv::Mat& room_map);

	// clean path from subsequent double occurrences of the same pose
	// min_dist_squared is the squared minimum distance between two points on the trajectory, in [pixel] (i.e. grid cells)
	void downsampleTrajectory(const std::vector<geometry_msgs::Pose2D>& path_uncleaned, std::vector<geometry_msgs::Pose2D>& path, const double min_dist_squared);


	// excute the planned trajectory and drive to unexplored areas after moving along the computed path
	void navigateExplorationPath(const std::vector<geometry_msgs::Pose2D>& exploration_path, const std::vector<geometry_msgs::Point32>& field_of_view,
			const geometry_msgs::Point32& field_of_view_origin, const double coverage_radius, const double distance_robot_fov_middlepoint,
			const float map_resolution, const geometry_msgs::Pose& map_origin, const double grid_spacing_in_pixel, const double map_height);

	// function to publish a navigation goal, it returns true if the goal could be reached
	// eps_x and eps_y are used to define a epsilon neighborhood around the goal in which a new nav_goal gets published
	// 	--> may smooth the process, move_base often slows before and stops at the goal
	bool publishNavigationGoal(const geometry_msgs::Pose2D& nav_goal, const std::string map_frame,
			const std::string camera_frame, std::vector<geometry_msgs::Pose2D>& robot_poses,
			const double robot_to_fov_middlepoint_distance, const double eps = 0.0,
			const bool perimeter_check = false);

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

	// function to create an occupancyGrid map out of a given cv::Mat
	void matToMap(nav_msgs::OccupancyGrid &map, const cv::Mat &mat)
	{
		map.info.width  = mat.cols;
		map.info.height = mat.rows;
		map.data.resize(mat.cols*mat.rows);

		for(int x=0; x<mat.cols; x++)
			for(int y=0; y<mat.rows; y++)
				map.data[y*mat.cols+x] = mat.at<int8_t>(y,x)?0:100;
	}

	// function to create a cv::Mat out of a given occupancyGrid map
	void mapToMat(const nav_msgs::OccupancyGrid &map, cv::Mat &mat)
	{
		mat = cv::Mat(map.info.height, map.info.width, CV_8U);

		for(int x=0; x<mat.cols; x++)
			for(int y=0; y<mat.rows; y++)
				mat.at<int8_t>(y,x) = map.data[y*mat.cols+x];
	}

	// !!Important!!
	//  define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_building_msgs::RoomExplorationAction> room_exploration_server_;
	dynamic_reconfigure::Server<ipa_room_exploration::RoomExplorationConfig> room_exploration_dynamic_reconfigure_server_;

public:
	enum PlanningMode {PLAN_FOR_FOOTPRINT=1, PLAN_FOR_FOV=2};

	// initialize the action-server
	RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action);

	// default destructor for the class
	~RoomExplorationServer(void)
	{
	}
};

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
 * Author: Florian Jordan, Richard Bormann
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


#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::exploreRoom, this, _1), false)
{
	// dynamic reconfigure
	room_exploration_dynamic_reconfigure_server_.setCallback(boost::bind(&RoomExplorationServer::dynamic_reconfigure_callback, this, _1, _2));

	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("room_exploration_algorithm", room_exploration_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << room_exploration_algorithm_ << std::endl;
	node_handle_.param("display_trajectory", display_trajectory_, false);
	std::cout << "room_exploration/display_trajectory = " << display_trajectory_ << std::endl;

	node_handle_.param("map_correction_closing_neighborhood_size", map_correction_closing_neighborhood_size_, 2);
	std::cout << "room_exploration/map_correction_closing_neighborhood_size = " << map_correction_closing_neighborhood_size_ << std::endl;

	node_handle_.param("return_path", return_path_, true);
	std::cout << "room_exploration/return_path = " << return_path_ << std::endl;
	node_handle_.param("execute_path", execute_path_, false);
	std::cout << "room_exploration/execute_path = " << execute_path_ << std::endl;
	node_handle_.param("goal_eps", goal_eps_, 0.35);
	std::cout << "room_exploration/goal_eps = " << goal_eps_ << std::endl;
	node_handle_.param("use_dyn_goal_eps", use_dyn_goal_eps_, false);
	std::cout << "room_exploration/use_dyn_goal_eps = " << use_dyn_goal_eps_ << std::endl;
	node_handle_.param("interrupt_navigation_publishing", interrupt_navigation_publishing_, false);
	std::cout << "room_exploration/interrupt_navigation_publishing = " << interrupt_navigation_publishing_ << std::endl;
	node_handle_.param("revisit_areas", revisit_areas_, false);
	std::cout << "room_exploration/revisit_areas = " << revisit_areas_ << std::endl;
	node_handle_.param("left_sections_min_area", left_sections_min_area_, 0.01);
	std::cout << "room_exploration/left_sections_min_area_ = " << left_sections_min_area_ << std::endl;
	global_costmap_topic_ = "/move_base/global_costmap/costmap";
	node_handle_.param<std::string>("global_costmap_topic", global_costmap_topic_);
	std::cout << "room_exploration/global_costmap_topic = " << global_costmap_topic_ << std::endl;
	node_handle_.param<std::string>("coverage_check_service_name", coverage_check_service_name_, "/room_exploration/coverage_check_server/coverage_check");
	std::cout << "room_exploration/coverage_check_service_name = " << coverage_check_service_name_ << std::endl;
	map_frame_ = "map";
	node_handle_.param<std::string>("map_frame", map_frame_);
	std::cout << "room_exploration/map_frame = " << map_frame_ << std::endl;
	camera_frame_ = "base_link";
	node_handle_.param<std::string>("camera_frame", camera_frame_);
	std::cout << "room_exploration/camera_frame = " << camera_frame_ << std::endl;


	if (room_exploration_algorithm_ == 1)
		ROS_INFO("You have chosen the grid exploration method.");
	else if(room_exploration_algorithm_ == 2)
		ROS_INFO("You have chosen the boustrophedon exploration method.");
	else if(room_exploration_algorithm_ == 3)
		ROS_INFO("You have chosen the neural network exploration method.");
	else if(room_exploration_algorithm_ == 4)
		ROS_INFO("You have chosen the convexSPP exploration method.");
	else if(room_exploration_algorithm_ == 5)
		ROS_INFO("You have chosen the flow network exploration method.");
	else if(room_exploration_algorithm_ == 6)
		ROS_INFO("You have chosen the energy functional exploration method.");
	else if(room_exploration_algorithm_ == 7)
		ROS_INFO("You have chosen the voronoi exploration method.");
	else if(room_exploration_algorithm_ == 8)
		ROS_INFO("You have chosen the boustrophedon variant exploration method.");

	if (room_exploration_algorithm_ == 1) // get grid point exploration parameters
	{
		node_handle_.param("tsp_solver", tsp_solver_, (int)TSP_CONCORDE);
		std::cout << "room_exploration/tsp_solver = " << tsp_solver_ << std::endl;
		int timeout=0;
		node_handle_.param("tsp_solver_timeout", timeout, 600);
		tsp_solver_timeout_ = timeout;
		std::cout << "room_exploration/tsp_solver_timeout = " << tsp_solver_timeout_ << std::endl;

	}
	else if ((room_exploration_algorithm_ == 2) || (room_exploration_algorithm_ == 8)) // set boustrophedon (variant) exploration parameters
	{
		node_handle_.param("min_cell_area", min_cell_area_, 10.0);
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
		node_handle_.param("path_eps", path_eps_, 2.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("grid_obstacle_offset", grid_obstacle_offset_, 0.0);
		std::cout << "room_exploration/grid_obstacle_offset_ = " << grid_obstacle_offset_ << std::endl;
		node_handle_.param("max_deviation_from_track", max_deviation_from_track_, -1);
		std::cout << "room_exploration/max_deviation_from_track_ = " << max_deviation_from_track_ << std::endl;
		node_handle_.param("cell_visiting_order", cell_visiting_order_, 1);
		std::cout << "room_exploration/cell_visiting_order = " << cell_visiting_order_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 3) // set neural network explorator parameters
	{
		node_handle_.param("step_size", step_size_, 0.008);
		std::cout << "room_exploration/step_size_ = " << step_size_ << std::endl;
		node_handle_.param("A", A_, 17);
		std::cout << "room_exploration/A_ = " << A_ << std::endl;
		node_handle_.param("B", B_, 5);
		std::cout << "room_exploration/B_ = " << B_ << std::endl;
		node_handle_.param("D", D_, 7);
		std::cout << "room_exploration/D_ = " << D_ << std::endl;
		node_handle_.param("E", E_, 80);
		std::cout << "room_exploration/E_ = " << E_ << std::endl;
		node_handle_.param("mu", mu_, 1.03);
		std::cout << "room_exploration/mu_ = " << mu_ << std::endl;
		node_handle_.param("delta_theta_weight", delta_theta_weight_, 0.15);
		std::cout << "room_exploration/delta_theta_weight_ = " << delta_theta_weight_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 4) // set convexSPP explorator parameters
	{
		node_handle_.param("cell_size", cell_size_, 0);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("delta_theta", delta_theta_, 1.570796);
		std::cout << "room_exploration/delta_theta = " << delta_theta_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 5) // set flowNetwork explorator parameters
	{
		node_handle_.param("curvature_factor", curvature_factor_, 1.1);
		std::cout << "room_exploration/curvature_factor = " << curvature_factor_ << std::endl;
		node_handle_.param("max_distance_factor", max_distance_factor_, 1.0);
		std::cout << "room_exploration/max_distance_factor_ = " << max_distance_factor_ << std::endl;
		node_handle_.param("cell_size", cell_size_, 0);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("path_eps", path_eps_, 3.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 6) // set energyfunctional explorator parameters
	{
	}
	else if (room_exploration_algorithm_ == 7) // set voronoi explorator parameters
	{
	}

	if (revisit_areas_ == true)
		ROS_INFO("Areas not seen after the initial execution of the path will be revisited.");
	else
		ROS_INFO("Areas not seen after the initial execution of the path will NOT be revisited.");


	// min area for revisiting left sections

	path_pub_ = node_handle_.advertise<nav_msgs::Path>("coverage_path", 2);

	//Start action server
	room_exploration_server_.start();

	ROS_INFO("Action server for room exploration has been initialized......");
}


// Callback function for dynamic reconfigure.
void RoomExplorationServer::dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level)
{
	// set segmentation algorithm
	std::cout << "######################################################################################" << std::endl;
	std::cout << "Dynamic reconfigure request:" << std::endl;

	room_exploration_algorithm_ = config.room_exploration_algorithm;
	std::cout << "room_exploration/path_planning_algorithm_ = " << room_exploration_algorithm_ << std::endl;

	map_correction_closing_neighborhood_size_ = config.map_correction_closing_neighborhood_size;
	std::cout << "room_exploration/map_correction_closing_neighborhood_size_ = " << map_correction_closing_neighborhood_size_ << std::endl;

	return_path_ = config.return_path;
	std::cout << "room_exploration/return_path_ = " << return_path_ << std::endl;
	execute_path_ = config.execute_path;
	std::cout << "room_exploration/execute_path_ = " << execute_path_ << std::endl;
	goal_eps_ = config.goal_eps;
	std::cout << "room_exploration/goal_eps_ = " << goal_eps_ << std::endl;
	use_dyn_goal_eps_  = config.use_dyn_goal_eps;
	std::cout << "room_exploration/use_dyn_goal_eps_ = " << use_dyn_goal_eps_ << std::endl;
	interrupt_navigation_publishing_ = config.interrupt_navigation_publishing;
	std::cout << "room_exploration/interrupt_navigation_publishing_ = " << interrupt_navigation_publishing_ << std::endl;
	revisit_areas_ = config.revisit_areas;
	std::cout << "room_exploration/revisit_areas_ = " << revisit_areas_ << std::endl;
	left_sections_min_area_ = config.left_sections_min_area;
	std::cout << "room_exploration/left_sections_min_area = " << left_sections_min_area_ << std::endl;
	global_costmap_topic_ = config.global_costmap_topic;
	std::cout << "room_exploration/global_costmap_topic_ = " << global_costmap_topic_ << std::endl;
	coverage_check_service_name_ = config.coverage_check_service_name;
	std::cout << "room_exploration/coverage_check_service_name_ = " << coverage_check_service_name_ << std::endl;
	map_frame_ = config.map_frame;
	std::cout << "room_exploration/map_frame_ = " << map_frame_ << std::endl;
	camera_frame_ = config.camera_frame;
	std::cout << "room_exploration/camera_frame_ = " << camera_frame_ << std::endl;

	// set parameters regarding the chosen algorithm
	if (room_exploration_algorithm_ == 1) // set grid point exploration parameters
	{
		tsp_solver_ = config.tsp_solver;
		std::cout << "room_exploration/tsp_solver_ = " << tsp_solver_ << std::endl;
		tsp_solver_timeout_ = config.tsp_solver_timeout;
		std::cout << "room_exploration/tsp_solver_timeout_ = " << tsp_solver_timeout_ << std::endl;
	}
	else if ((room_exploration_algorithm_ == 2) || (room_exploration_algorithm_ == 8)) // set boustrophedon (variant) exploration parameters
	{
		min_cell_area_ = config.min_cell_area;
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		grid_obstacle_offset_ = config.grid_obstacle_offset;
		std::cout << "room_exploration/grid_obstacle_offset_ = " << grid_obstacle_offset_ << std::endl;
		max_deviation_from_track_ = config.max_deviation_from_track;
		std::cout << "room_exploration/max_deviation_from_track_ = " << max_deviation_from_track_ << std::endl;
		cell_visiting_order_ = config.cell_visiting_order;
		std::cout << "room_exploration/cell_visiting_order = " << cell_visiting_order_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 3) // set neural network explorator parameters
	{
		step_size_ = config.step_size;
		std::cout << "room_exploration/step_size_ = " << step_size_ << std::endl;
		A_ = config.A;
		std::cout << "room_exploration/A_ = " << A_ << std::endl;
		B_ = config.B;
		std::cout << "room_exploration/B_ = " << B_ << std::endl;
		D_ = config.D;
		std::cout << "room_exploration/D_ = " << D_ << std::endl;
		E_ = config.E;
		std::cout << "room_exploration/E_ = " << E_ << std::endl;
		mu_ = config.mu;
		std::cout << "room_exploration/mu_ = " << mu_ << std::endl;
		delta_theta_weight_ = config.delta_theta_weight;
		std::cout << "room_exploration/delta_theta_weight_ = " << delta_theta_weight_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 4) // set convexSPP explorator parameters
	{
		cell_size_ = config.cell_size;
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		delta_theta_ = config.delta_theta;
		std::cout << "room_exploration/delta_theta_ = " << delta_theta_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 5) // set flowNetwork explorator parameters
	{
		curvature_factor_ = config.curvature_factor;
		std::cout << "room_exploration/delta_theta_ = " << delta_theta_ << std::endl;
		max_distance_factor_ = config.max_distance_factor;
		std::cout << "room_exploration/max_distance_factor_ = " << max_distance_factor_ << std::endl;
		cell_size_ = config.cell_size;
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
	}
	else if (room_exploration_algorithm_ == 6) // set energyFunctional explorator parameters
	{
	}
	else if (room_exploration_algorithm_ == 7) // set voronoi explorator parameters
	{
	}


	if (revisit_areas_ == true)
		std::cout << "Areas not seen after the initial execution of the path will be revisited." << std::endl;
	else
		std::cout << "Areas not seen after the initial execution of the path will NOT be revisited." << std::endl;

	std::cout << "######################################################################################" << std::endl;
}


// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal)
{
	ROS_INFO("*****Room Exploration action server*****");

	// ***************** I. read the given parameters out of the goal *****************
	// todo: this is only correct if the map is not rotated
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const float map_resolution = goal->map_resolution;	// in [m/cell]
	const float map_resolution_inverse = 1./map_resolution;
	std::cout << "map origin: " << map_origin << " m       map resolution: " << map_resolution << " m/cell" << std::endl;

	const float robot_radius = goal->robot_radius;
	const int robot_radius_in_pixel = (robot_radius / map_resolution);
	std::cout << "robot radius: " << robot_radius << " m   (" << robot_radius_in_pixel << " px)" << std::endl;

	const cv::Point starting_position((goal->starting_position.x-map_origin.x)/map_resolution, (goal->starting_position.y-map_origin.y)/map_resolution);
	std::cout << "starting point: (" << goal->starting_position.x << ", " << goal->starting_position.y << ") m   (" << starting_position << " px)" << std::endl;

	planning_mode_ = goal->planning_mode;
	if (planning_mode_==PLAN_FOR_FOOTPRINT)
		std::cout << "planning mode: planning coverage path with robot's footprint" <<std::endl;
	else if (planning_mode_==PLAN_FOR_FOV)
		std::cout << "planning mode: planning coverage path with robot's field of view" <<std::endl;

	// todo: receive map data in nav_msgs::OccupancyGrid format
	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;

	// determine room size
	int area_px = 0;		// room area in pixels
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map.at<uchar>(v,u) >= 250)
				area_px++;
	std::cout << "### room area = " << area_px*map_resolution*map_resolution << " m^2" << std::endl;

	// closing operation to neglect inaccessible areas and map errors/artifacts
	cv::Mat temp;
	cv::erode(room_map, temp, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);
	cv::dilate(temp, room_map, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
	const bool room_not_empty = removeUnconnectedRoomParts(room_map);
	if (room_not_empty == false)
	{
		std::cout << "RoomExplorationServer::exploreRoom: Warning: the requested room is too small for generating exploration trajectories." << std::endl;
		ipa_building_msgs::RoomExplorationResult action_result;
		room_exploration_server_.setAborted(action_result);
		return;
	}

	// get the grid size, to check the areas that should be revisited later
	double grid_spacing_in_meter = 0.0;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius
	float fitting_circle_radius_in_meter = 0;
	Eigen::Matrix<float, 2, 1> fitting_circle_center_point_in_meter;	// this is also considered the center of the field of view, because around this point the maximum radius incircle can be found that is still inside the fov
	std::vector<Eigen::Matrix<float, 2, 1> > fov_corners_meter(4);
	const double fov_resolution = 1000;		// in [cell/meter]
	if(planning_mode_ == PLAN_FOR_FOV) // read out the given fov-vectors, if needed
	{
		// Get the size of one grid cell s.t. the grid can be completely covered by the field of view (fov) from all rotations around it.
		for(int i = 0; i < 4; ++i)
			fov_corners_meter[i] << goal->field_of_view[i].x, goal->field_of_view[i].y;
		computeFOVCenterAndRadius(fov_corners_meter, fitting_circle_radius_in_meter, fitting_circle_center_point_in_meter, fov_resolution);
		// get the edge length of the grid square that fits into the fitting_circle_radius
		grid_spacing_in_meter = fitting_circle_radius_in_meter*std::sqrt(2);
	}
	else // if planning should be done for the footprint, read out the given coverage radius
	{
		grid_spacing_in_meter = goal->coverage_radius*std::sqrt(2);
	}
	// map the grid size to an int in pixel coordinates, using floor method
	const double grid_spacing_in_pixel = grid_spacing_in_meter/map_resolution;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius, multiply with sqrt(2) to receive the whole working width
	std::cout << "grid size: " << grid_spacing_in_meter << " m   (" << grid_spacing_in_pixel << " px)" << std::endl;
	// set the cell_size_ for #4 convexSPP explorator or #5 flowNetwork explorator if it is not provided
	if (cell_size_ <= 0)
		cell_size_ = std::floor(grid_spacing_in_pixel);


	// ***************** II. plan the path using the wanted planner *****************
	// todo: consider option to provide the inflated map or the robot radius to the functions instead of inflating with half cell size there
	Eigen::Matrix<float, 2, 1> zero_vector;
	zero_vector << 0, 0;
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if (room_exploration_algorithm_ == 1) // use grid point explorator
	{
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			grid_point_planner.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, std::floor(grid_spacing_in_pixel), false, fitting_circle_center_point_in_meter, tsp_solver_, tsp_solver_timeout_);
		else
			grid_point_planner.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, std::floor(grid_spacing_in_pixel), true, zero_vector, tsp_solver_, tsp_solver_timeout_);
	}
	else if (room_exploration_algorithm_ == 2) // use boustrophedon explorator
	{
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, false, fitting_circle_center_point_in_meter, min_cell_area_, max_deviation_from_track_);
		else
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, true, zero_vector, min_cell_area_, max_deviation_from_track_);
	}
	else if (room_exploration_algorithm_ == 3) // use neural network explorator
	{
		neural_network_explorator_.setParameters(A_, B_, D_, E_, mu_, step_size_, delta_theta_weight_);
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, false, fitting_circle_center_point_in_meter, false);
		else
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, true, zero_vector, false);
	}
	else if (room_exploration_algorithm_ == 4) // use convexSPP explorator
	{
		// plan coverage path
		if(planning_mode_ == PLAN_FOR_FOV)
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, fov_corners_meter, fitting_circle_center_point_in_meter, 0., 7, false);
		else
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, fov_corners_meter, zero_vector, goal->coverage_radius, 7, true);
	}
	else if (room_exploration_algorithm_ == 5) // use flow network explorator
	{
		if(planning_mode_ == PLAN_FOR_FOV)
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, fitting_circle_center_point_in_meter, grid_spacing_in_pixel, false, path_eps_, curvature_factor_, max_distance_factor_);
		else
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, zero_vector, grid_spacing_in_pixel, true, path_eps_, curvature_factor_, max_distance_factor_);
	}
	else if (room_exploration_algorithm_ == 6) // use energy functional explorator
	{
		if(planning_mode_ == PLAN_FOR_FOV)
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, false, fitting_circle_center_point_in_meter);
		else
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, true, zero_vector);
	}
	else if (room_exploration_algorithm_ == 7) // use voronoi explorator
	{
		// create a usable occupancyGrid map out of the given room map
		nav_msgs::OccupancyGrid room_gridmap;
		matToMap(room_gridmap, room_map);

		// do not find nearest pose to starting-position and start there because of issue in planner when starting position is provided
		if(planning_mode_==PLAN_FOR_FOV)
		{
//			cv::Mat distance_transform;
//			cv::distanceTransform(room_map, distance_transform, CV_DIST_L2, CV_DIST_MASK_PRECISE);
//			cv::Mat display = room_map.clone();
//			// todoo: get max dist from map and parametrize loop
//			for (int s=5; s<100; s+=10)
//			{
//				for (int v=0; v<distance_transform.rows; ++v)
//				{
//					for (int u=0; u<distance_transform.cols; ++u)
//					{
//						if (int(distance_transform.at<float>(v,u)) == s)
//						{
//							display.at<uchar>(v,u) = 0;
//						}
//					}
//				}
//			}
//			cv::imshow("distance_transform", distance_transform);
//			cv::imshow("trajectories", display);
//			cv::waitKey();

			// convert fov-radius to pixel integer
			const int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel);
			std::cout << "grid spacing in pixel: " << grid_spacing_as_int << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, grid_spacing_as_int, 2, true); // a perfect alignment of the paths cannot be assumed here (in contrast to footprint planning) because the well-aligned fov trajectory is mapped to robot locations that may not be on parallel tracks
			// get the exploration path
			std::vector<geometry_msgs::Pose2D> fov_path_uncleaned;
			vm.setSingleRoom(true); //to force to consider all rooms
			vm.generatePath(fov_path_uncleaned, cv::Mat(), starting_position.x, starting_position.y);	// start position in room center

			// clean path from subsequent double occurrences of the same pose
			std::vector<geometry_msgs::Pose2D> fov_path;
			downsampleTrajectory(fov_path_uncleaned, fov_path, 2.*2.); //5*5);

			// convert to poses with angles
			RoomRotator room_rotation;
			room_rotation.transformPointPathToPosePath(fov_path);

			// map fov-path to robot-path
			//cv::Point start_pos(fov_path.begin()->x, fov_path.begin()->y);
			//mapPath(room_map, exploration_path, fov_path, fitting_circle_center_point_in_meter, map_resolution, map_origin, start_pos);
			ROS_INFO("Starting to map from field of view pose to robot pose");
			cv::Point robot_starting_position = (fov_path.size()>0 ? cv::Point(fov_path[0].x, fov_path[0].y) : starting_position);
			cv::Mat inflated_room_map;
			cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), (int)std::floor(goal->robot_radius/map_resolution));
			mapPath(inflated_room_map, exploration_path, fov_path, fitting_circle_center_point_in_meter, map_resolution, map_origin, robot_starting_position);
		}
		else
		{
			// convert coverage-radius to pixel integer
			//int coverage_diameter = (int)std::floor(2.*goal->coverage_radius/map_resolution);
			//std::cout << "coverage radius in pixel: " << coverage_diameter << std::endl;
			const int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel);
			std::cout << "grid spacing in pixel: " << grid_spacing_as_int << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, grid_spacing_as_int, 2, true);	//coverage_diameter-1); // diameter in pixel (full working width can be used here because tracks are planned in parallel motion)
			// get the exploration path
			std::vector<geometry_msgs::Pose2D> exploration_path_uncleaned;
			vm.setSingleRoom(true); //to force to consider all rooms
			vm.generatePath(exploration_path_uncleaned, cv::Mat(), starting_position.x, starting_position.y);	// start position in room center

			// clean path from subsequent double occurrences of the same pose
			downsampleTrajectory(exploration_path_uncleaned, exploration_path, 3.5*3.5); //3.5*3.5);

			// convert to poses with angles
			RoomRotator room_rotation;
			room_rotation.transformPointPathToPosePath(exploration_path);

			// transform to global coordinates
			for(size_t pos=0; pos<exploration_path.size(); ++pos)
			{
				exploration_path[pos].x = (exploration_path[pos].x * map_resolution) + map_origin.x;
				exploration_path[pos].y = (exploration_path[pos].y * map_resolution) + map_origin.y;
			}
		}
	}
	else if (room_exploration_algorithm_ == 8) // use boustrophedon variant explorator
	{
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			boustrophedon_variant_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, false, fitting_circle_center_point_in_meter, min_cell_area_, max_deviation_from_track_);
		else
			boustrophedon_variant_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, true, zero_vector, min_cell_area_, max_deviation_from_track_);
	}

	// display finally planned path
	if (display_trajectory_ == true)
	{
		std::cout << "printing path" << std::endl;
		cv::Mat fov_path_map;
		for(size_t step=1; step<exploration_path.size(); ++step)
		{
			fov_path_map = room_map.clone();
			cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
			if (exploration_path.size() > 0)
#if CV_MAJOR_VERSION<=3
				cv::circle(fov_path_map, 2*cv::Point((exploration_path[0].x-map_origin.x)/map_resolution, (exploration_path[0].y-map_origin.y)/map_resolution), 2, cv::Scalar(150), CV_FILLED);
#else
				cv::circle(fov_path_map, 2*cv::Point((exploration_path[0].x-map_origin.x)/map_resolution, (exploration_path[0].y-map_origin.y)/map_resolution), 2, cv::Scalar(150), cv::FILLED);
#endif
			for(size_t i=1; i<=step; ++i)
			{
				cv::Point p1((exploration_path[i-1].x-map_origin.x)/map_resolution, (exploration_path[i-1].y-map_origin.y)/map_resolution);
				cv::Point p2((exploration_path[i].x-map_origin.x)/map_resolution, (exploration_path[i].y-map_origin.y)/map_resolution);
#if CV_MAJOR_VERSION<=3
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(200), CV_FILLED);
#else
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(200), cv::FILLED);
#endif
				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
				cv::Point p3(p2.x+5*cos(exploration_path[i].theta), p2.y+5*sin(exploration_path[i].theta));
				if (i==step)
				{
#if CV_MAJOR_VERSION<=3
					cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(80), CV_FILLED);
#else
					cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(80), cv::FILLED);
#endif
					cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
					cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(50), 1);
				}
			}
//			cv::imshow("cell path", fov_path_map);
//			cv::waitKey();
		}
		cv::imshow("cell path", fov_path_map);
		cv::waitKey();
	}

	ROS_INFO("Room exploration planning finished.");

	ipa_building_msgs::RoomExplorationResult action_result;
	// check if the size of the exploration path is larger then zero
	if(exploration_path.size()==0)
	{
		room_exploration_server_.setAborted(action_result);
		return;
	}

	// if wanted, return the path as the result
	if(return_path_ == true)
	{
		action_result.coverage_path = exploration_path;
		// return path in PoseStamped format as well (e.g. necessary for move_base commands)
		std::vector<geometry_msgs::PoseStamped> exploration_path_pose_stamped(exploration_path.size());
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "/map";
		for (size_t i=0; i<exploration_path.size(); ++i)
		{
			exploration_path_pose_stamped[i].header = header;
			exploration_path_pose_stamped[i].header.seq = i;
			exploration_path_pose_stamped[i].pose.position.x = exploration_path[i].x;
			exploration_path_pose_stamped[i].pose.position.y = exploration_path[i].y;
			exploration_path_pose_stamped[i].pose.position.z = 0.;
			Eigen::Quaterniond quaternion;
			quaternion = Eigen::AngleAxisd((double)exploration_path[i].theta, Eigen::Vector3d::UnitZ());
			tf::quaternionEigenToMsg(quaternion, exploration_path_pose_stamped[i].pose.orientation);
		}
		action_result.coverage_path_pose_stamped = exploration_path_pose_stamped;

		nav_msgs::Path coverage_path;
		coverage_path.header.frame_id = "map";
		coverage_path.header.stamp = ros::Time::now();
		coverage_path.poses = exploration_path_pose_stamped;
		path_pub_.publish(coverage_path);
	}

	// ***************** III. Navigate trough all points and save the robot poses to check what regions have been seen *****************
	// [optionally] execute the path
	if(execute_path_ == true)
	{
		navigateExplorationPath(exploration_path, goal->field_of_view, goal->field_of_view_origin, goal->coverage_radius, fitting_circle_center_point_in_meter.norm(),
					map_resolution, goal->map_origin, grid_spacing_in_pixel, room_map.rows * map_resolution);
		ROS_INFO("Explored room.");
	}

	room_exploration_server_.setSucceeded(action_result);

	return;
}

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
bool RoomExplorationServer::removeUnconnectedRoomParts(cv::Mat& room_map)
{
	// create new map with segments labeled by increasing labels from 1,2,3,...
	cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
	for (int v=0; v<room_map.rows; ++v)
	{
		for (int u=0; u<room_map.cols; ++u)
		{
			if (room_map.at<uchar>(v,u) == 255)
				room_map_int.at<int32_t>(v,u) = -100;
			else
				room_map_int.at<int32_t>(v,u) = 0;
		}
	}

	std::map<int, int> area_to_label_map;	// maps area=number of segment pixels (keys) to the respective label (value)
	int label = 1;
	for (int v=0; v<room_map_int.rows; ++v)
	{
		for (int u=0; u<room_map_int.cols; ++u)
		{
			if (room_map_int.at<int32_t>(v,u) == -100)
			{
				const int area = cv::floodFill(room_map_int, cv::Point(u,v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
				area_to_label_map[area] = label;
				++label;
			}
		}
	}
	// abort if area_to_label_map.size() is empty
	if (area_to_label_map.size() == 0)
		return false;

	// remove all room pixels from room_map which are not accessible
	const int label_of_biggest_room = area_to_label_map.rbegin()->second;
	std::cout << "label_of_biggest_room=" << label_of_biggest_room << std::endl;
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map_int.at<int32_t>(v,u) != label_of_biggest_room)
				room_map.at<uchar>(v,u) = 0;

	return true;
}


void RoomExplorationServer::downsampleTrajectory(const std::vector<geometry_msgs::Pose2D>& path_uncleaned, std::vector<geometry_msgs::Pose2D>& path, const double min_dist_squared)
{
	// clean path from subsequent double occurrences of the same pose
	path.push_back(path_uncleaned[0]);
	cv::Point last_added_point(path_uncleaned[0].x, path_uncleaned[0].y);
	for (size_t i=1; i<path_uncleaned.size(); ++i)
	{
		const cv::Point current_point(path_uncleaned[i].x, path_uncleaned[i].y);
		cv::Point vector = current_point - last_added_point;
		if (vector.x*vector.x+vector.y*vector.y > min_dist_squared || i==path_uncleaned.size()-1)
		{
			path.push_back(path_uncleaned[i]);
			last_added_point = current_point;
		}
	}
}


void RoomExplorationServer::navigateExplorationPath(const std::vector<geometry_msgs::Pose2D>& exploration_path,
		const std::vector<geometry_msgs::Point32>& field_of_view, const geometry_msgs::Point32& field_of_view_origin,
		const double coverage_radius, const double distance_robot_fov_middlepoint, const float map_resolution,
		const geometry_msgs::Pose& map_origin, const double grid_spacing_in_pixel, const double map_height)
{
	// ***************** III. Navigate trough all points and save the robot poses to check what regions have been seen *****************
	// 1. publish navigation goals
	std::vector<geometry_msgs::Pose2D> robot_poses;
	geometry_msgs::Pose2D last_pose;
	geometry_msgs::Pose2D pose;
	for(size_t map_oriented_pose = 0; map_oriented_pose < exploration_path.size(); ++map_oriented_pose)
	{
		// check if the path should be continued or not
		bool interrupted = false;
		if(interrupt_navigation_publishing_==true)
		{
			ROS_INFO("Interrupt order received, resuming coverage path later.");
			interrupted = true;
		}
		while(interrupt_navigation_publishing_==true)
		{
			// sleep for 1s because else this loop would produce errors
			std::cout << "sleeping... (-.-)zzZZ" << std::endl;
			ros::Duration sleep_rate(1);
			sleep_rate.sleep();
		}
		if(interrupted==true)
			ROS_INFO("Interrupt order canceled, resuming coverage path now.");

		// if no interrupt is wanted, publish the navigation goal
		pose = exploration_path[map_oriented_pose];
		// todo: convert map to image properly, then this coordinate correction here becomes obsolete
		//pose.y = map_height - (pose.y - map_origin.position.y) + map_origin.position.y;
		double temp_goal_eps = 0;
		if (use_dyn_goal_eps_)
		{
		if (map_oriented_pose != 0)
		{
			double delta_theta = std::fabs(last_pose.theta - pose.theta);
			if (delta_theta > M_PI * 0.5)
				delta_theta = M_PI * 0.5;
			temp_goal_eps = (M_PI * 0.5 - delta_theta) / (M_PI * 0.5) * goal_eps_;
			}
		}
		else
		{
			temp_goal_eps = goal_eps_;
		}
		publishNavigationGoal(pose, map_frame_, camera_frame_, robot_poses, distance_robot_fov_middlepoint, temp_goal_eps, true); // eps = 0.35
		last_pose = pose;
	}

	std::cout << "published all navigation goals, starting to check seen area" << std::endl;

	// 2. get the global costmap, that has initially not known objects in to check what regions have been seen
	nav_msgs::OccupancyGrid global_costmap;
	global_costmap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(global_costmap_topic_));
	ROS_INFO("Found global gridmap.");

	std::vector<signed char> pixel_values;
	pixel_values = global_costmap.data;

	// if wanted check for areas that haven't been seen during the execution of the path and revisit them, if wanted
	if(revisit_areas_ == true)
	{
		// save the costmap as Mat of the same type as the given map (8UC1)
		cv::Mat costmap_as_mat;//(global_map.cols, global_map.rows, CV_8UC1);

		mapToMat(global_costmap, costmap_as_mat);

		// 70% probability of being an obstacle
		cv::threshold(costmap_as_mat, costmap_as_mat, 75, 255, cv::THRESH_BINARY_INV);

		// 3. draw the seen positions so the server can check what points haven't been seen
		std::cout << "checking coverage using the coverage_check_server" << std::endl;
		cv::Mat coverage_map, number_of_coverage_image;
		// use the coverage check server to check which areas have been seen
		//   --> convert path to cv format
		std::vector<cv::Point3d> path;
		for (size_t i=0; i<robot_poses.size(); ++i)
			path.push_back(cv::Point3d(robot_poses[i].x, robot_poses[i].y, robot_poses[i].theta));
		//   --> convert field of view to Eigen format
		std::vector<Eigen::Matrix<float, 2, 1> > fov;
		for(size_t i = 0; i < field_of_view.size(); ++i)
		{
			Eigen::Matrix<float, 2, 1> current_vector;
			current_vector << field_of_view[i].x, field_of_view[i].y;
			fov.push_back(current_vector);
		}
		//   --> convert field of view origin to Eigen format
		Eigen::Matrix<float, 2, 1> fov_origin;
		fov_origin <<field_of_view_origin.x, field_of_view_origin.y;
		//   --> call coverage checker
		CoverageCheckServer coverage_checker;
		if (coverage_checker.checkCoverage(costmap_as_mat, map_resolution, cv::Point2d(map_origin.position.x, map_origin.position.y),
				path, fov, fov_origin, coverage_radius, (planning_mode_==PLAN_FOR_FOOTPRINT), false, coverage_map, number_of_coverage_image) == true)
		{
			std::cout << "got the service response" << std::endl;
		}
		else
		{
			ROS_WARN("Coverage check failed, is the coverage_check_server running?");
			room_exploration_server_.setAborted();
			return;
		}

//		// service interface - can be deleted
//		// define the request for the coverage check
//		ipa_building_msgs::CheckCoverageRequest coverage_request;
//		ipa_building_msgs::CheckCoverageResponse coverage_response;
//		// fill request
//		sensor_msgs::ImageConstPtr service_image;
//		cv_bridge::CvImage cv_image;
//		cv_image.encoding = "mono8";
//		cv_image.image = costmap_as_mat;
//		service_image = cv_image.toImageMsg();
//		coverage_request.input_map = *service_image;
//		coverage_request.map_resolution = map_resolution;
//		coverage_request.map_origin = map_origin;
//		coverage_request.path = robot_poses;
//		coverage_request.field_of_view = field_of_view;
//		coverage_request.field_of_view_origin = field_of_view_origin;
//		coverage_request.coverage_radius = coverage_radius;
//		coverage_request.check_number_of_coverages = false;
//		std::cout << "filled service request for the coverage check" << std::endl;
//		if(planning_mode_ == PLAN_FOR_FOV)
//			coverage_request.check_for_footprint = false;
//		else
//			coverage_request.check_for_footprint = true;
//		// send request
//		if(ros::service::call(coverage_check_service_name_, coverage_request, coverage_response) == true)
//		{
//			std::cout << "got the service response" << std::endl;
//			cv_bridge::CvImagePtr cv_ptr_obj;
//			cv_ptr_obj = cv_bridge::toCvCopy(coverage_response.coverage_map, sensor_msgs::image_encodings::MONO8);
//			coverage_map = cv_ptr_obj->image;
//		}
//		else
//		{
//			ROS_WARN("Coverage check failed, is the coverage_check_server running?");
//			room_exploration_server_.setAborted();
//			return;
//		}

		// testing, parameter to show
//		cv::namedWindow("initially seen areas", cv::WINDOW_NORMAL);
//		cv::imshow("initially seen areas", seen_positions_map);
//		cv::resizeWindow("initially seen areas", 600, 600);
//		cv::waitKey();

		// apply a binary filter on the image, making the drawn seen areas black
		cv::threshold(coverage_map, coverage_map, 150, 255, cv::THRESH_BINARY);

		// ***************** IV. Find leftover areas and lay a grid over it, then plan a path trough all grids s.t. they can be covered by the fov. *****************
		// 1. find regions with an area that is bigger than a defined value, which have not been seen by the fov.
		// 	  hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
		// 	  child-contour = 1 if it has one, = -1 if not, same for parent_contour
		std::vector < std::vector<cv::Point> > left_areas, areas_to_revisit;
		std::vector < cv::Vec4i > hierarchy;
		cv::findContours(coverage_map, left_areas, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

		// find valid regions
		for(size_t area = 0; area < left_areas.size(); ++area)
		{
			// don't look at hole contours
			if (hierarchy[area][3] == -1)
			{
				double room_area = map_resolution * map_resolution * cv::contourArea(left_areas[area]);
				//subtract the area from the hole contours inside the found contour, because the contour area grows extremly large if it is a closed loop
				for(int hole = 0; hole < left_areas.size(); ++hole)
				{
					if(hierarchy[hole][3] == area)//check if the parent of the hole is the current looked at contour
					{
						room_area -= map_resolution * map_resolution * cv::contourArea(left_areas[hole]);
					}
				}

				// save the contour if the area of it is larger than the defined value
				if(room_area >= left_sections_min_area_)
					areas_to_revisit.push_back(left_areas[area]);
			}
		}

		// check if areas need to be visited again, if not cancel here
		if(areas_to_revisit.size() == 0)
		{
			ROS_INFO("Explored room.");

			room_exploration_server_.setSucceeded();

			return;
		}

		// draw found regions s.t. they can be intersected later
		cv::Mat black_map(costmap_as_mat.cols, costmap_as_mat.rows, costmap_as_mat.type(), cv::Scalar(0));
#if CV_MAJOR_VERSION<=3
		cv::drawContours(black_map, areas_to_revisit, -1, cv::Scalar(255), CV_FILLED);
#else
		cv::drawContours(black_map, areas_to_revisit, -1, cv::Scalar(255), cv::FILLED);
#endif
		for(size_t contour = 0; contour < left_areas.size(); ++contour)
			if(hierarchy[contour][3] != -1)
#if CV_MAJOR_VERSION<=3
				cv::drawContours(black_map, left_areas, contour, cv::Scalar(0), CV_FILLED);
#else
				cv::drawContours(black_map, left_areas, contour, cv::Scalar(0), cv::FILLED);
#endif

		// 2. Intersect the left areas with respect to the calculated grid length.
		geometry_msgs::Polygon min_max_coordinates;	// = goal->room_min_max;
		for(size_t i = 0/*min_max_coordinates.points[0].y*/; i < black_map.cols; i += std::floor(grid_spacing_in_pixel))
			cv::line(black_map, cv::Point(0, i), cv::Point(black_map.cols, i), cv::Scalar(0), 1);
		for(size_t i = 0/*min_max_coordinates.points[0].x*/; i < black_map.rows; i += std::floor(grid_spacing_in_pixel))
			cv::line(black_map, cv::Point(i, 0), cv::Point(i, black_map.rows), cv::Scalar(0), 1);

		// 3. find the centers of the global_costmap areas
		std::vector < std::vector<cv::Point> > grid_areas;
		cv::Mat contour_map = black_map.clone();
		cv::findContours(contour_map, grid_areas, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

		// get the moments
		std::vector<cv::Moments> moments(grid_areas.size());
		for( int i = 0; i < grid_areas.size(); i++)
		{
			moments[i] = cv::moments(grid_areas[i], false);
		}

		// get the mass centers
		std::vector<cv::Point> area_centers(grid_areas.size());
		for( int i = 0; i < grid_areas.size(); i++ )
		{
			// check if the current contour has an area and isn't just a few pixels
			if(moments[i].m10 != 0 && moments[i].m01 != 0)
			{
				area_centers[i] = cv::Point( moments[i].m10/moments[i].m00 , moments[i].m01/moments[i].m00 );
			}
			// if contour is too small for moment calculation, take one point on this contour and use it as center
			else
			{
				area_centers[i] = grid_areas[i][0];
			}
		}
		
		// testing
//		black_map = room_map.clone();
//		for(size_t i = 0; i < area_centers.size(); ++i)
//		{
//			cv::circle(black_map, area_centers[i], 2, cv::Scalar(127), CV_FILLED);
//			std::cout << area_centers[i] << std::endl;
//		}
//		cv::namedWindow("revisiting areas", cv::WINDOW_NORMAL);
//		cv::imshow("revisiting areas", black_map);
//		cv::resizeWindow("revisiting areas", 600, 600);
//		cv::waitKey();

		// 4. plan a tsp path trough the centers of the left areas
		// find the center that is nearest to the current robot position, which becomes the start node for the tsp
		geometry_msgs::Pose2D current_robot_pose = robot_poses.back();
		cv::Point current_robot_point(current_robot_pose.x, current_robot_pose.y);
		double min_dist = 9001;
		int min_index = 0;
		for(size_t current_center_index = 0; current_center_index < area_centers.size(); ++current_center_index)
		{
			cv::Point current_center = area_centers[current_center_index];
			double current_squared_distance = std::pow(current_center.x - current_robot_point.x, 2.0) + std::pow(current_center.y - current_robot_point.y, 2.0);

			if(current_squared_distance <= min_dist)
			{
				min_dist = current_squared_distance;
				min_index = current_center_index;
			}
		}
		ConcordeTSPSolver tsp_solver;
		std::vector<int> revisiting_order = tsp_solver.solveConcordeTSP(costmap_as_mat, area_centers, 0.25, 0.0, map_resolution, min_index, 0);

		// 5. go to each center and use the map_accessability_server to find a robot pose around it s.t. it can be covered by the field of view or robot center
		const double pi_8 = PI/8;
		const std::string perimeter_service_name = "/room_exploration/map_accessibility_analysis/map_perimeter_accessibility_check";
	//	robot_poses.clear();
		for(size_t center = 0; center < revisiting_order.size(); ++center)
		{
			geometry_msgs::Pose2D current_center;
			current_center.x = (area_centers[revisiting_order[center]].x * map_resolution) + map_origin.position.x;
			current_center.y = (area_centers[revisiting_order[center]].y * map_resolution) + map_origin.position.y;

			// define request
			cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request check_request;
			cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response response;
			check_request.center = current_center;
			if(planning_mode_ == PLAN_FOR_FOV)
			{
				check_request.radius = distance_robot_fov_middlepoint;
				check_request.rotational_sampling_step = pi_8;
			}
			else
			{
				check_request.radius = 0.0;
				check_request.rotational_sampling_step = 2.0*PI;
			}
			std::cout << "checking center: " << std::endl << current_center << "radius: " << check_request.radius << std::endl;

			// send request
			if(ros::service::call(perimeter_service_name, check_request, response) == true)
			{
				std::cout << "successful check of accessibility" << std::endl;
				// go trough the found accessible positions and try to reach one of them
				for(std::vector<geometry_msgs::Pose2D>::iterator pose = response.accessible_poses_on_perimeter.begin(); pose != response.accessible_poses_on_perimeter.end(); ++pose)
					if(publishNavigationGoal(*pose, map_frame_, camera_frame_, robot_poses, 0.0) == true)
						break;
			}
			else
			{
				// todo: return areas that were not visible on radius
				std::cout << "center not reachable on perimeter" << std::endl;
			}
		}
		
//		drawSeenPoints(copy, robot_poses, goal->field_of_view, corner_point_1, corner_point_2, map_resolution, map_origin);
//		cv::namedWindow("seen areas", cv::WINDOW_NORMAL);
//		cv::imshow("seen areas", copy);
//		cv::resizeWindow("seen areas", 600, 600);
//		cv::waitKey();
	}
}


// Function to publish a navigation goal for move_base. It returns true, when the goal could be reached.
// The function tracks the robot pose while moving to the goal and adds these poses to the given pose-vector. This is done
// because it allows to calculate where the robot field of view has theoretically been and identify positions of the map that
// the robot hasn't seen.
bool RoomExplorationServer::publishNavigationGoal(const geometry_msgs::Pose2D& nav_goal, const std::string map_frame,
		const std::string camera_frame, std::vector<geometry_msgs::Pose2D>& robot_poses, const double robot_to_fov_middlepoint_distance,
		const double eps, const bool perimeter_check)
{
	// move base client, that sends navigation goals to a move_base action server
	MoveBaseClient mv_base_client("/move_base", true);

	// wait for the action server to come up
	while(mv_base_client.waitForServer(ros::Duration(5.0)) == false)
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}


	geometry_msgs::Pose2D map_oriented_pose;

	map_oriented_pose.x = nav_goal.x;
	map_oriented_pose.y = nav_goal.y;
	map_oriented_pose.theta = nav_goal.theta;

	std::cout << "navigation goal: (" << map_oriented_pose.x << ", "  << map_oriented_pose.y << ", " << map_oriented_pose.theta << ")" << std::endl;

	move_base_msgs::MoveBaseGoal move_base_goal;

	// create move_base_goal
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();


	move_base_goal.target_pose.pose.position.x = map_oriented_pose.x;
	move_base_goal.target_pose.pose.position.y = map_oriented_pose.y;
	move_base_goal.target_pose.pose.orientation.z = std::sin(map_oriented_pose.theta/2);
	move_base_goal.target_pose.pose.orientation.w = std::cos(map_oriented_pose.theta/2);

	// send goal to the move_base sever, when one is found
	ROS_INFO_STREAM("Sending goal with eps " << eps);
	mv_base_client.sendGoal(move_base_goal);

	// wait until goal is reached or the goal is aborted
//	ros::Duration sleep_rate(0.1);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	ros::Duration sleep_duration(0.15); // todo: param
	bool near_pos;
	do
	{
		near_pos = false;
		double roll, pitch, yaw;
		// try to get the transformation from map_frame to base_frame, wait max. 2 seconds for this transform to come up
		try
		{
			ros::Time time = ros::Time(0);
			listener.waitForTransform(map_frame, camera_frame, time, ros::Duration(2.0)); // 5.0
			listener.lookupTransform(map_frame, camera_frame, time, transform);

			sleep_duration.sleep();

			// save the current pose if a transform could be found
			geometry_msgs::Pose2D current_pose;

			current_pose.x = transform.getOrigin().x();
			current_pose.y = transform.getOrigin().y();
			transform.getBasis().getRPY(roll, pitch, yaw);
			current_pose.theta = yaw;

			if((current_pose.x-map_oriented_pose.x)*(current_pose.x-map_oriented_pose.x) + (current_pose.y-map_oriented_pose.y)*(current_pose.y-map_oriented_pose.y) <= eps*eps)
				near_pos = true;

			robot_poses.push_back(current_pose);
		}
		catch(tf::TransformException &ex)
		{
			ROS_WARN_STREAM("Couldn't get transform from " << camera_frame << " to " << map_frame << "!");// %s", ex.what());
		}

	}while(mv_base_client.getState() != actionlib::SimpleClientGoalState::ABORTED && mv_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED
			&& near_pos == false);

	// check if point could be reached or not
	if(mv_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || near_pos == true)
	{
		ROS_INFO("current goal could be reached.");
		return true;
	}
	// if the goal couldn't be reached, find another point around the desired fov-position
	else if(perimeter_check == true)
	{
		ROS_INFO("current goal could not be reached, checking for other goal.");

		// get the desired fov-position
		geometry_msgs::Pose2D relative_vector;
		relative_vector.x = std::cos(map_oriented_pose.theta)*robot_to_fov_middlepoint_distance;
		relative_vector.y = std::sin(map_oriented_pose.theta)*robot_to_fov_middlepoint_distance;
		geometry_msgs::Pose2D center;
		center.x = map_oriented_pose.x + relative_vector.x;
		center.y = map_oriented_pose.y + relative_vector.y;

		// check for another robot pose to reach the desired fov-position
		const std::string perimeter_service_name = "/room_exploration/map_accessibility_analysis/map_perimeter_accessibility_check";
		cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response response;
		cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request check_request;
		check_request.center = center;
		if(planning_mode_ == PLAN_FOR_FOV)
		{
			check_request.radius = robot_to_fov_middlepoint_distance;
			check_request.rotational_sampling_step = PI/8;
		}
		else
		{
			check_request.radius = 0.0;
			check_request.rotational_sampling_step = 2.0*PI;
		}

		// send request
		if(ros::service::call(perimeter_service_name, check_request, response) == true)
		{
			// go trough the found accessible positions and try to reach one of them
			for(std::vector<geometry_msgs::Pose2D>::iterator pose = response.accessible_poses_on_perimeter.begin(); pose != response.accessible_poses_on_perimeter.end(); ++pose)
			{
				if(publishNavigationGoal(*pose, map_frame, camera_frame, robot_poses, 0.0) == true)
				{
					ROS_INFO("Perimeter check for not reachable goal succeeded.");
					return true;
				}
			}
		}
		else
		{
			ROS_INFO("Desired position not reachable.");
		}
		return false;
	}
	else
	{
		return false;
	}
}



// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");
	ros::Time::init();

	ros::NodeHandle nh("~");

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ros::spin();

	return 0;
}

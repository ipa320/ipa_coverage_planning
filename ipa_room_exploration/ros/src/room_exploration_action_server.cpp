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
	node_handle_.param("room_exploration_algorithm", path_planning_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << path_planning_algorithm_ << std::endl << std::endl;
	node_handle_.param("goal_eps", goal_eps_, 0.35);
	std::cout << "room_exploration/goal_eps = " << goal_eps_ << std::endl;
	node_handle_.param("return_path", return_path_, true);
	std::cout << "room_exploration/return_path = " << return_path_ << std::endl;
	node_handle_.param("execute_path", execute_path_, false);
	std::cout << "room_exploration/execute_path = " << execute_path_ << std::endl;
	global_costmap_topic_ = "/move_base/global_costmap/costmap";
	node_handle_.param<std::string>("global_costmap_topic", global_costmap_topic_);
	std::cout << "room_exploration/global_costmap_topic = " << global_costmap_topic_ << std::endl;
	coverage_check_service_name_ = "/coverage_check_server/coverage_check";
	node_handle_.param<std::string>("coverage_check_service_name", coverage_check_service_name_);
	std::cout << "room_exploration/coverage_check_service_name = " << coverage_check_service_name_ << std::endl;
	map_frame_ = "map";
	node_handle_.param<std::string>("map_frame", map_frame_);
	std::cout << "room_exploration/map_frame = " << map_frame_ << std::endl;
	camera_frame_ = "camera";
	node_handle_.param<std::string>("camera_frame", camera_frame_);
	std::cout << "room_exploration/camera_frame = " << camera_frame_ << std::endl;


	if (path_planning_algorithm_ == 1)
		ROS_INFO("You have chosen the grid exploration method.");
	else if(path_planning_algorithm_ == 2)
		ROS_INFO("You have chosen the boustrophedon exploration method.");
	else if(path_planning_algorithm_ == 3)
		ROS_INFO("You have chosen the neural network exploration method.");
	else if(path_planning_algorithm_ == 4)
		ROS_INFO("You have chosen the convexSPP exploration method.");
	else if(path_planning_algorithm_ == 5)
		ROS_INFO("You have chosen the flow network exploration method.");
	else if(path_planning_algorithm_ == 6)
		ROS_INFO("You have chosen the energy functional exploration method.");
	else if(path_planning_algorithm_ == 7)
		ROS_INFO("You have chosen the voronoi exploration method.");

	if (path_planning_algorithm_ == 1) // get grid point exploration parameters
	{
		node_handle_.param("tsp_solver", tsp_solver_, (int)TSP_CONCORDE);
		std::cout << "room_exploration/tsp_solver = " << tsp_solver_ << std::endl;
		int timeout=0;
		node_handle_.param("tsp_solver_timeout", timeout, 600);
		tsp_solver_timeout_ = timeout;
		std::cout << "room_exploration/tsp_solver_timeout = " << tsp_solver_timeout_ << std::endl;

	}
	else if(path_planning_algorithm_ == 2) // set boustrophedon exploration parameters
	{
		node_handle_.param("path_eps", path_eps_, 3.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("min_cell_area", min_cell_area_, 0.5);
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
	}
	else if(path_planning_algorithm_ == 3) // set neural network explorator parameters
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
	else if(path_planning_algorithm_ == 4) // set convexSPP explorator parameters
	{
		node_handle_.param("cell_size", cell_size_, 0);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("delta_theta", delta_theta_, 1.570796);
		std::cout << "room_exploration/delta_theta = " << delta_theta_ << std::endl;
	}
	else if(path_planning_algorithm_ == 5) // set flowNetwork explorator parameters
	{
		node_handle_.param("path_eps", path_eps_, 3.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("cell_size", cell_size_, 0);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("curvature_factor", curvature_factor_, 1.1);
		std::cout << "room_exploration/curvature_factor = " << curvature_factor_ << std::endl;
		node_handle_.param("max_distance_factor", max_distance_factor_, 1.0);
		std::cout << "room_exploration/max_distance_factor_ = " << max_distance_factor_ << std::endl;
	}
	else if(path_planning_algorithm_ == 6) // set energyfunctional explorator parameters
	{
	}
	else if(path_planning_algorithm_ == 7) // set voronoi explorator parameters
	{
	}

	// boolean to set the functionality of revisiting not seen areas
	node_handle_.param("revisit_areas", revisit_areas_, true);
	if(revisit_areas_ == true)
		ROS_INFO("Areas not seen after the initial execution of the path will be revisited.");
	else
		ROS_INFO("Areas not seen after the initial execution of the path will NOT be revisited.");

	// min area for revisiting left sections
	node_handle_.param("left_sections_min_area", left_sections_min_area_, 10.0);
	std::cout << "room_exploration/left_sections_min_area_ = " << left_sections_min_area_ << std::endl;

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

	path_planning_algorithm_ = config.room_exploration_algorithm;
	std::cout << "room_exploration/path_planning_algorithm_ = " << path_planning_algorithm_ << std::endl;
	goal_eps_ = config.goal_eps;
	std::cout << "room_exploration/goal_eps_ = " << goal_eps_ << std::endl;
	return_path_ = config.return_path;
	std::cout << "room_exploration/return_path_ = " << return_path_ << std::endl;
	execute_path_ = config.execute_path;
	std::cout << "room_exploration/execute_path_ = " << execute_path_ << std::endl;
	global_costmap_topic_ = config.global_costmap_topic;
	std::cout << "room_exploration/global_costmap_topic_ = " << global_costmap_topic_ << std::endl;
	coverage_check_service_name_ = config.coverage_check_service_name;
	std::cout << "room_exploration/coverage_check_service_name_ = " << coverage_check_service_name_ << std::endl;
	map_frame_ = config.map_frame;
	std::cout << "room_exploration/map_frame_ = " << map_frame_ << std::endl;
	camera_frame_ = config.camera_frame;
	std::cout << "room_exploration/camera_frame_ = " << camera_frame_ << std::endl;

	// set parameters regarding the chosen algorithm
	if (path_planning_algorithm_ == 1) // set grid point exploration parameters
	{
		tsp_solver_ = config.tsp_solver;
		std::cout << "room_exploration/tsp_solver_ = " << tsp_solver_ << std::endl;
		tsp_solver_timeout_ = config.tsp_solver_timeout;
		std::cout << "room_exploration/tsp_solver_timeout_ = " << tsp_solver_timeout_ << std::endl;
	}
	else if(path_planning_algorithm_ == 2) // set boustrophedon exploration parameters
	{
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		min_cell_area_ = config.min_cell_area;
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
	}
	else if(path_planning_algorithm_ == 3) // set neural network explorator parameters
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
	else if(path_planning_algorithm_ == 4) // set convexSPP explorator parameters
	{
		cell_size_ = config.cell_size;
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		delta_theta_ = config.delta_theta;
		std::cout << "room_exploration/delta_theta_ = " << delta_theta_ << std::endl;
	}
	else if(path_planning_algorithm_ == 5) // set flowNetwork explorator parameters
	{
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		cell_size_ = config.cell_size;
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		curvature_factor_ = config.curvature_factor;
		std::cout << "room_exploration/delta_theta_ = " << delta_theta_ << std::endl;
		max_distance_factor_ = config.max_distance_factor;
		std::cout << "room_exploration/max_distance_factor_ = " << max_distance_factor_ << std::endl;
	}
	else if(path_planning_algorithm_ == 6) // set energyFunctional explorator parameters
	{
	}
	else if(path_planning_algorithm_ == 7) // set voronoi explorator parameters
	{
	}


	revisit_areas_ = config.revisit_areas;
	if(revisit_areas_ == true)
		std::cout << "Areas not seen after the initial execution of the path will be revisited." << std::endl;
	else
		std::cout << "Areas not seen after the initial execution of the path will NOT be revisited." << std::endl;

	left_sections_min_area_ = config.left_sections_min_area;
	std::cout << "room_exploration/left_sections_min_area = " << left_sections_min_area_ << std::endl;

	interrupt_navigation_publishing_ = config.interrupt_navigation_publishing;
	std::cout << "room_exploration/interrupt_navigation_publishing = " << interrupt_navigation_publishing_ << std::endl;

	std::cout << "######################################################################################" << std::endl;
}


// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal)
{
	ROS_INFO("*****Room Exploration action server*****");

	// ***************** I. read the given parameters out of the goal *****************
	// todo: this is only correct if the map is not rotated
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const float map_resolution = goal->map_resolution;
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

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;

	// erode map so that not reachable areas are not considered - we are using the closing operation instead to work on the original but cleaned up map
	//cv::erode(room_map, room_map, cv::Mat(), cv::Point(-1, -1), robot_radius_in_pixel);

	// closing operation to neglect inaccessible areas and map errors/artifacts
	// todo: make closing neighborhood size a parameter
	cv::Mat temp;
	cv::erode(room_map, temp, cv::Mat(), cv::Point(-1, -1), 2);
	cv::dilate(temp, room_map, cv::Mat(), cv::Point(-1, -1), 2);

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
	removeUnconnectedRoomParts(room_map);

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
	Eigen::Matrix<float, 2, 1> zero_vector;
	zero_vector << 0, 0;
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if(path_planning_algorithm_ == 1) // use grid point explorator
	{
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			grid_point_planner.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, std::floor(grid_spacing_in_pixel), false, fitting_circle_center_point_in_meter, tsp_solver_, tsp_solver_timeout_);
		else
			grid_point_planner.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, std::floor(grid_spacing_in_pixel), true, zero_vector, tsp_solver_, tsp_solver_timeout_);
	}
	else if(path_planning_algorithm_ == 2) // use boustrophedon explorator
	{
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, path_eps_, false, fitting_circle_center_point_in_meter, min_cell_area_);
		else
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, path_eps_, true, zero_vector, min_cell_area_);
	}
	else if(path_planning_algorithm_ == 3) // use neural network explorator
	{
		neural_network_explorator_.setParameters(A_, B_, D_, E_, mu_, step_size_, delta_theta_weight_);
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, false, fitting_circle_center_point_in_meter, false);
		else
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, true, zero_vector, false);
	}
	else if(path_planning_algorithm_ == 4) // use convexSPP explorator
	{
		// plan coverage path
		if(planning_mode_ == PLAN_FOR_FOV)
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, fov_corners_meter, fitting_circle_center_point_in_meter, 0., 7, false);
		else
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, fov_corners_meter, zero_vector, goal->coverage_radius, 7, true);
	}
	else if(path_planning_algorithm_ == 5) // use flow network explorator
	{
		if(planning_mode_ == PLAN_FOR_FOV)
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, fitting_circle_center_point_in_meter, grid_spacing_in_pixel, false, path_eps_, curvature_factor_, max_distance_factor_);
		else
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, zero_vector, grid_spacing_in_pixel, true, path_eps_, curvature_factor_, max_distance_factor_);
	}
	else if(path_planning_algorithm_ == 6) // use energy functional explorator
	{
		if(planning_mode_ == PLAN_FOR_FOV)
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, false, fitting_circle_center_point_in_meter);
		else
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, true, zero_vector);
	}
	else if(path_planning_algorithm_ == 7) // use voronoi explorator
	{
		// create a usable occupancyGrid map out of the given room map
		nav_msgs::OccupancyGrid room_gridmap;
		matToMap(room_gridmap, room_map);

		// do not find nearest pose to starting-position and start there because of issue in planner when starting position is provided
		if(planning_mode_==PLAN_FOR_FOV)
		{
			// convert fov-radius to pixel integer
			const int fov_diameter_as_int = (int)std::floor(2.*fitting_circle_radius_in_meter/map_resolution);
			std::cout << "fov diameter in pixel: " << fov_diameter_as_int << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, fov_diameter_as_int); // diameter in pixel (full working width can be used here because tracks are planned in parallel motion)
			// get the exploration path
			std::vector<geometry_msgs::Pose2D> fov_path_uncleaned;
			vm.generatePath(fov_path_uncleaned, cv::Mat());

			// clean path from subsequent double occurrences of the same pose
			std::vector<geometry_msgs::Pose2D> fov_path;
			downsampleTrajectory(fov_path_uncleaned, fov_path, 5*5);

			// convert to poses with angles
			RoomRotator room_rotation;
			room_rotation.transformPointPathToPosePath(fov_path);

			// map fov-path to robot-path
			//cv::Point start_pos(fov_path.begin()->x, fov_path.begin()->y);
			//mapPath(room_map, exploration_path, fov_path, fitting_circle_center_point_in_meter, map_resolution, map_origin, start_pos);
			ROS_INFO("Starting to map from field of view pose to robot pose");
			cv::Point robot_starting_position = (fov_path.size()>0 ? cv::Point(fov_path[0].x, fov_path[0].y) : starting_position);
			cv::Mat inflated_room_map;
			cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), (int)std::floor(2.*fitting_circle_radius_in_meter/map_resolution));
			mapPath(inflated_room_map, exploration_path, fov_path, fitting_circle_center_point_in_meter, map_resolution, map_origin, robot_starting_position);
		}
		else
		{
			// convert coverage-radius to pixel integer
			int coverage_diameter = (int)std::floor(2.*goal->coverage_radius/map_resolution);
			std::cout << "coverage radius in pixel: " << coverage_diameter << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, coverage_diameter); // diameter in pixel (full working width can be used here because tracks are planned in parallel motion)
			// get the exploration path
			std::vector<geometry_msgs::Pose2D> exploration_path_uncleaned;
			vm.generatePath(exploration_path_uncleaned, cv::Mat());

			// clean path from subsequent double occurrences of the same pose
			downsampleTrajectory(exploration_path_uncleaned, exploration_path, 3.5*3.5);

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
	}

	// ***************** III. Navigate trough all points and save the robot poses to check what regions have been seen *****************
	// [optionally] execute the path
	if(execute_path_ == true)
	{
		navigateExplorationPath(exploration_path, goal->field_of_view, goal->coverage_radius, fitting_circle_center_point_in_meter.norm(),
				map_resolution, goal->map_origin, grid_spacing_in_pixel);
		ROS_INFO("Explored room.");
	}

	room_exploration_server_.setSucceeded(action_result);

	return;
}

void RoomExplorationServer::removeUnconnectedRoomParts(cv::Mat& room_map)
{
	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
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
	// remove all room pixels from room_map which are not accessible
	const int label_of_biggest_room = area_to_label_map.rbegin()->second;
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map_int.at<int32_t>(v,u) != label_of_biggest_room)
				room_map.at<uchar>(v,u) = 0;
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
		const std::vector<geometry_msgs::Point32>& field_of_view, const double coverage_radius, const double distance_robot_fov_middlepoint,
		const float map_resolution, const geometry_msgs::Pose& map_origin, const double grid_spacing_in_pixel)
{
	// ***************** III. Navigate trough all points and save the robot poses to check what regions have been seen *****************
	// 1. publish navigation goals
	std::vector<geometry_msgs::Pose2D> robot_poses;
	for(size_t nav_goal = 0; nav_goal < exploration_path.size(); ++nav_goal)
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
		publishNavigationGoal(exploration_path[nav_goal], map_frame_, camera_frame_, robot_poses, distance_robot_fov_middlepoint, goal_eps_, true); // eps = 0.35
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

//		// fill one row and then go to the next one (storing method of ros)
//		for(size_t u = 0; u < costmap_as_mat.cols; ++u)
//		{
//			for(size_t v = 0; v < costmap_as_mat.rows; ++v)
//			{
//				costmap_as_mat.at<uchar>(u,v) = (uchar) pixel_values[v+u*global_map.rows];
//			}
//		}
		mapToMat(global_costmap, costmap_as_mat);

		// 70% probability of being an obstacle
		cv::threshold(costmap_as_mat, costmap_as_mat, 75, 255, cv::THRESH_BINARY_INV);

		// 3. draw the seen positions so the server can check what points haven't been seen
		std::cout << "checking coverage using the coverage_check_server" << std::endl;
		cv::Mat seen_positions_map;
		// define the request for the coverage check
		ipa_building_msgs::CheckCoverageRequest coverage_request;
		ipa_building_msgs::CheckCoverageResponse coverage_response;
		// fill request
		sensor_msgs::ImageConstPtr service_image;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = costmap_as_mat;
		service_image = cv_image.toImageMsg();
		coverage_request.input_map = *service_image;
		coverage_request.path = robot_poses;
		coverage_request.field_of_view = field_of_view;
		coverage_request.coverage_radius = coverage_radius;
		coverage_request.map_origin = map_origin;
		coverage_request.map_resolution = map_resolution;
		coverage_request.check_number_of_coverages = false;
		std::cout << "filled service request for the coverage check" << std::endl;
		if(planning_mode_ == PLAN_FOR_FOV)
		{
			coverage_request.check_for_footprint = false;
			// send request
			if(ros::service::call(coverage_check_service_name_, coverage_request, coverage_response) == true)
			{
				std::cout << "got the service response" << std::endl;
				cv_bridge::CvImagePtr cv_ptr_obj;
				cv_ptr_obj = cv_bridge::toCvCopy(coverage_response.coverage_map, sensor_msgs::image_encodings::MONO8);
				seen_positions_map = cv_ptr_obj->image;
			}
			else
			{
				ROS_WARN("Coverage check failed, is the coverage_check_server running?");
				room_exploration_server_.setAborted();
				return;
			}
		}
		else
		{
			coverage_request.check_for_footprint = true;
			// send request
			if(ros::service::call(coverage_check_service_name_, coverage_request, coverage_response) == true)
			{
				std::cout << "got the service response" << std::endl;
				cv_bridge::CvImagePtr cv_ptr_obj;
				cv_ptr_obj = cv_bridge::toCvCopy(coverage_response.coverage_map, sensor_msgs::image_encodings::MONO8);
				seen_positions_map = cv_ptr_obj->image;
			}
			else
			{
				ROS_WARN("Coverage check failed, is the coverage_check_server running?");
				room_exploration_server_.setAborted();
				return;
			}
		}
//		cv::imshow("covered", seen_positions_map);
//		cv::waitKey();
//		cv::Mat copy = room_map.clone();

		// testing, parameter to show
//		cv::namedWindow("initially seen areas", cv::WINDOW_NORMAL);
//		cv::imshow("initially seen areas", seen_positions_map);
//		cv::resizeWindow("initially seen areas", 600, 600);
//		cv::waitKey();

		// apply a binary filter on the image, making the drawn seen areas black
		cv::threshold(seen_positions_map, seen_positions_map, 150, 255, cv::THRESH_BINARY);

		// ***************** IV. Find left areas and lay a grid over it, then plan a path trough all grids s.t. they can be covered by the fov. *****************
		// 1. find regions with an area that is bigger than a defined value, which have not been seen by the fov.
		// 	  hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
		// 	  child-contour = 1 if it has one, = -1 if not, same for parent_contour
		std::vector < std::vector<cv::Point> > left_areas, areas_to_revisit;
		std::vector < cv::Vec4i > hierarchy;
		cv::findContours(seen_positions_map, left_areas, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

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
		cv::drawContours(black_map, areas_to_revisit, -1, cv::Scalar(255), CV_FILLED);
		for(size_t contour = 0; contour < left_areas.size(); ++contour)
			if(hierarchy[contour][3] != -1)
				cv::drawContours(black_map, left_areas, contour, cv::Scalar(0), CV_FILLED);

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
	//	cv::waitKey();

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

		// 5. go to each center and use the map_accessability_server to find a robot pose around it s.t. it can be covered
		//	  by the fov
		double pi_8 = PI/8;
		std::string perimeter_service_name = "/map_accessibility_analysis/map_perimeter_accessibility_check";	// todo: replace with library interface
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
				// TODO: return areas that were not visible on radius
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

	std::cout << "navigation goal: (" << nav_goal.x << ", "  << nav_goal.y << ", " << nav_goal.theta << ")" << std::endl;

	move_base_msgs::MoveBaseGoal move_base_goal;

	// create move_base_goal
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();

	move_base_goal.target_pose.pose.position.x = nav_goal.x;
	move_base_goal.target_pose.pose.position.y = nav_goal.y;
	move_base_goal.target_pose.pose.orientation.z = std::sin(nav_goal.theta/2);
	move_base_goal.target_pose.pose.orientation.w = std::cos(nav_goal.theta/2);

	// send goal to the move_base sever, when one is found
	ROS_INFO("Sending goal");
	mv_base_client.sendGoal(move_base_goal);

	// wait until goal is reached or the goal is aborted
//	ros::Duration sleep_rate(0.1);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	ros::Duration sleep_duration(0.15); // TODO: param
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

//			ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getOrigin().y());
			sleep_duration.sleep();

			// save the current pose if a transform could be found
			geometry_msgs::Pose2D current_pose;

			current_pose.x = transform.getOrigin().x();
			current_pose.y = transform.getOrigin().y();
			transform.getBasis().getRPY(roll, pitch, yaw);
			current_pose.theta = yaw;

			if((current_pose.x-nav_goal.x)*(current_pose.x-nav_goal.x) + (current_pose.y-nav_goal.y)*(current_pose.y-nav_goal.y) <= eps*eps)
				near_pos = true;

			robot_poses.push_back(current_pose);
		}
		catch(tf::TransformException &ex)
		{
			ROS_INFO("Couldn't get transform!");// %s", ex.what());
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
		relative_vector.x = std::cos(nav_goal.theta)*robot_to_fov_middlepoint_distance;
		relative_vector.y = std::sin(nav_goal.theta)*robot_to_fov_middlepoint_distance;
		geometry_msgs::Pose2D center;
		center.x = nav_goal.x + relative_vector.x;
		center.y = nav_goal.y + relative_vector.y;

		// check for another robot pose to reach the desired fov-position
		std::string perimeter_service_name = "/map_accessibility_analysis/map_perimeter_accessibility_check";	// todo: replace with library interface
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

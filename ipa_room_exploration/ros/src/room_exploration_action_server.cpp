#include <ipa_room_exploration/room_exploration_action_server.h>

// Callback function for dynamic reconfigure.
void RoomExplorationServer::dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level)
{
	// set segmentation algorithm
	std::cout << "######################################################################################" << std::endl;
	std::cout << "Dynamic reconfigure request:" << std::endl;

	plan_for_footprint_ = config.plan_for_footprint;
	std::cout << "room_exploration/plan_for_footprint_ = " << plan_for_footprint_ << std::endl;

	path_planning_algorithm_ = config.room_exploration_algorithm;
	std::cout << "room_exploration/path_planning_algorithm_ = " << path_planning_algorithm_ << std::endl;

	goal_eps_ = config.goal_eps;
	std::cout << "room_exploration/goal_eps_ = " << goal_eps_ << std::endl;

	// set parameters regarding the chosen algorithm
	if (path_planning_algorithm_ == 1) // set grid point exploration parameters
	{
		grid_line_length_ = config.grid_line_length;
		std::cout << "room_exploration/grid_line_length_ = " << grid_line_length_ << std::endl;
	}
	else if(path_planning_algorithm_ == 2) // set boustrophedon exploration parameters
	{
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		min_cell_size_ = config.min_cell_size;
		std::cout << "room_exploration/min_cell_size_ = " << min_cell_size_ << std::endl;
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

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::exploreRoom, this, _1), false)
{
	// dynamic reconfigure
	room_exploration_dynamic_reconfigure_server_.setCallback(boost::bind(&RoomExplorationServer::dynamic_reconfigure_callback, this, _1, _2));

	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("plan_for_footprint", plan_for_footprint_, false);
	std::cout << "room_exploration/plan_for_footprint_ = " << plan_for_footprint_ << std::endl;

	node_handle_.param("room_exploration_algorithm", path_planning_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << path_planning_algorithm_ << std::endl << std::endl;

	node_handle_.param("goal_eps", goal_eps_, 0.35);
	std::cout << "room_exploration/goal_eps_ = " << goal_eps_ << std::endl;

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
		node_handle_.param("grid_line_length", grid_line_length_, 10);
		std::cout << "room_exploration/grid_line_length = " << grid_line_length_ << std::endl;
	}
	else if(path_planning_algorithm_ == 2) // set boustrophedon exploration parameters
	{
		node_handle_.param("path_eps", path_eps_, 3.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("min_cell_size", min_cell_size_, 0.5);
		std::cout << "room_exploration/min_cell_size_ = " << min_cell_size_ << std::endl;
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
		node_handle_.param("cell_size", cell_size_, 10);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("delta_theta", delta_theta_, 1.570796);
		std::cout << "room_exploration/delta_theta = " << delta_theta_ << std::endl;
	}
	else if(path_planning_algorithm_ == 5) // set flowNetwork explorator parameters
	{
		node_handle_.param("path_eps", path_eps_, 3.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("cell_size", cell_size_, 10);
		std::cout << "room_exploration/cell_size_ = " << cell_size_ << std::endl;
		node_handle_.param("curvature_factor", curvature_factor_, 1.1);
		std::cout << "room_exploration/curvature_factor = " << curvature_factor_ << std::endl;
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
	ros::Duration sleep_duration(0.15); // TODO: param!!!!
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

			if(std::pow(current_pose.x - nav_goal.x, 2.0) + std::pow(current_pose.y - nav_goal.y, 2.0) <= eps*eps)
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
		std::string perimeter_service_name = "/map_accessibility_analysis/map_perimeter_accessibility_check";
		cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response response;
		cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request check_request;
		check_request.center = center;

		if(plan_for_footprint_ == false)
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

// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Room Exploration action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	// ***************** I. read the given parameters out of the goal *****************
	// todo: test for adding right coordinates!!
	cv::Point2d map_origin;
	map_origin.x = goal->map_origin.x + goal->region_of_interest_coordinates.points[0].x;
	map_origin.y = goal->map_origin.y + goal->region_of_interest_coordinates.points[0].y;

	const float map_resolution = goal->map_resolution;

	const float robot_radius = goal->robot_radius;

	std::cout << "******************* robot radius ********************" << robot_radius << std::endl;

	cv::Point starting_position;
	starting_position.x = (goal->starting_position.x - map_origin.x)/map_resolution;
	starting_position.y = (goal->starting_position.y - map_origin.y)/map_resolution;
	std::cout << "starting point: " << starting_position << std::endl;
	geometry_msgs::Polygon min_max_coordinates = goal->room_min_max;

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat global_map = cv_ptr_obj->image;
//	transformImageToRoomCordinates(global_map); // TODO: add again

	// extract the subregion of the given map that should be explored
	int roi_x_start = goal->region_of_interest_coordinates.points[0].x;
	int roi_y_start = goal->region_of_interest_coordinates.points[0].y;
	int roi_x_end = goal->region_of_interest_coordinates.points[1].x;
	int roi_y_end = goal->region_of_interest_coordinates.points[1].y;

	cv::Mat room_map = global_map(cv::Range(roi_y_start, roi_y_end), cv::Range(roi_x_start, roi_x_end));
//	cv::Mat room_map = global_map;

	// erode map so that not reachable areas are not considered
	const int robot_radius_in_pixel = (robot_radius / map_resolution);
	cv::erode(room_map, room_map, cv::Mat(), cv::Point(-1, -1), robot_radius_in_pixel);
//	cv::circle(room_map, cv::Point(min_max_coordinates.points[0].x, min_max_coordinates.points[0].y), 2, cv::Scalar(127), CV_FILLED);
//	cv::circle(room_map, cv::Point(min_max_coordinates.points[1].x, min_max_coordinates.points[1].y), 2, cv::Scalar(127), CV_FILLED);
//	cv::imshow("room", room_map);
//	cv::waitKey();

	// get the grid size, to check the areas that should be revisited later
	int grid_length = 0;
	double grid_length_as_double = 0.0;
	float fitting_circle_radius = 0;
	Eigen::Matrix<float, 2, 1> middle_point;
	std::vector<Eigen::Matrix<float, 2, 1> > fov_vectors;
	Eigen::Matrix<float, 2, 1> middle_point_1, middle_point_2, middle_point_3, middle_point_4;
	if(plan_for_footprint_ == false) // read out the given fov-vectors, if needed
	{
		for(int i = 0; i < 4; ++i)
		{
			Eigen::Matrix<float, 2, 1> current_vector;
			current_vector << goal->field_of_view[i].x, goal->field_of_view[i].y;
			fov_vectors.push_back(current_vector);
		}
		// Get the size of one grid cell s.t. the grid can be completely covered by the field of view (fov) from all rotations around it.
		// For this fit a circle in the fov, which gives the diagonal length of the square. Then use Pythagoras to get the fov middle point.
		middle_point = (fov_vectors[0] + fov_vectors[1] + fov_vectors[2] + fov_vectors[3]) / 4;
//		std::cout << "middle point: " << middle_point << std::endl;

		// get middle points of edges of the fov
		middle_point_1 = (fov_vectors[0] + fov_vectors[1]) / 2;
		middle_point_2 = (fov_vectors[1] + fov_vectors[2]) / 2;
		middle_point_3 = (fov_vectors[2] + fov_vectors[3]) / 2;
		middle_point_4 = (fov_vectors[3] + fov_vectors[0]) / 2;
//		std::cout << "middle-points: " << std::endl << middle_point_1 << " (" << middle_point - middle_point_1 << ")" << std::endl << middle_point_2 << " (" << middle_point - middle_point_2 << ")" << std::endl << middle_point_3 << " (" << middle_point - middle_point_3 << ")" << std::endl << middle_point_4 << " (" << middle_point - middle_point_4 << ")" << std::endl;

		// get the radius of the circle in the fov as min distance from the fov-middle point to the edge middle points
		float distance_1 = (middle_point - middle_point_1).norm();
		float distance_2 = (middle_point - middle_point_2).norm();
		float distance_3 = (middle_point - middle_point_3).norm();
		float distance_4 = (middle_point - middle_point_4).norm();
		fitting_circle_radius = std::min(std::min(distance_1, distance_2), std::min(distance_3, distance_4));
		std::cout << "fitting_circle_radius: " << fitting_circle_radius << std::endl;

		// get the edge length of the grid square as float and map it to an int in pixel coordinates, using floor method
		grid_length_as_double = fitting_circle_radius*std::sqrt(2);
	}
	else // if planning should be done for the footprint, read out the given coverage radius
	{
		grid_length_as_double = goal->coverage_radius*std::sqrt(2);
	}
	grid_length = std::floor(grid_length_as_double/map_resolution);
	std::cout << "grid size: " << grid_length_as_double << ", as int: " << grid_length << std::endl;



	// ***************** II. plan the path using the wanted planner *****************
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if(path_planning_algorithm_ == 1) // use grid point explorator
	{
		// set wanted grid size
		//grid_point_planner.setGridLineLength(grid_line_length_);	// todo: why not grid_length which is already computed to fit the working device
		grid_point_planner.setGridLineLength(grid_length);

		// plan path
		grid_point_planner.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, min_max_coordinates, map_origin);
	}
	else if(path_planning_algorithm_ == 2) // use boustrophedon explorator
	{
		// plan path
		if(plan_for_footprint_ == false)
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, fitting_circle_radius/map_resolution, path_eps_, plan_for_footprint_, middle_point, min_cell_size_);
		else
		{
			Eigen::Matrix<float, 2, 1> zero_vector;
			zero_vector << 0, 0;
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, goal->coverage_radius/map_resolution, path_eps_, plan_for_footprint_, zero_vector, min_cell_size_);
		}
	}
	else if(path_planning_algorithm_ == 3) // use neural network explorator
	{
		// plan path
		if(plan_for_footprint_ == false)
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, fitting_circle_radius/map_resolution, plan_for_footprint_, middle_point, min_max_coordinates, false);
		else
		{
			Eigen::Matrix<float, 2, 1> zero_vector;
			zero_vector << 0, 0;
			neural_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, goal->coverage_radius/map_resolution, plan_for_footprint_, zero_vector, min_max_coordinates, false);
		}
	}
	else if(path_planning_algorithm_ == 4) // use convexSPP explorator
	{
		// find the maximum angle that is spanned between the closer or more distant corners, used to determine the visibility of cells
		float max_angle = 0.0;
		float dot = fov_vectors[0].transpose()*fov_vectors[1];
		float abs = fov_vectors[0].norm()*fov_vectors[1].norm();
		float quotient = dot/abs;
		if(quotient > 1) // prevent errors resulting from round errors
			quotient = 1;
		else if(quotient < -1)
			quotient = -1;
		float angle_1 = std::acos(quotient);
		dot = fov_vectors[2].transpose()*fov_vectors[3];
		abs = fov_vectors[2].norm()*fov_vectors[3].norm();
		quotient = dot/abs;
		if(quotient > 1) // prevent errors resulting from round errors
			quotient = 1;
		else if(quotient < -1)
			quotient = -1;
		float angle_2 = std::acos(dot/abs);

		if(angle_1 > angle_2)
			max_angle = angle_1;
		else
			max_angle = angle_2;

		// plan coverage path
		if(plan_for_footprint_ == false)
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, min_max_coordinates, goal->field_of_view, middle_point, max_angle, middle_point_1.norm(), fov_vectors[3].norm(), 7, false);
		else
			convex_SPP_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, delta_theta_, min_max_coordinates, goal->footprint, middle_point, max_angle, 0.0, goal->coverage_radius/map_resolution, 7, true);
	}
	else if(path_planning_algorithm_ == 5) // use flow network explorator
	{
		if(plan_for_footprint_ == false)
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, min_max_coordinates, middle_point, fitting_circle_radius/map_resolution, false, path_eps_, curvature_factor_);
		else
		{
			Eigen::Matrix<float, 2, 1> zero_vector;
			zero_vector << 0, 0;
			flow_network_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, cell_size_, min_max_coordinates, zero_vector, goal->coverage_radius/map_resolution, true, path_eps_, curvature_factor_);
		}
	}
	else if(path_planning_algorithm_ == 6) // use energy functional explorator
	{
		if(plan_for_footprint_ == false)
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, fitting_circle_radius/map_resolution, false, middle_point);
		else
		{
			Eigen::Matrix<float, 2, 1> zero_vector;
			zero_vector << 0, 0;
			energy_functional_explorator_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, fitting_circle_radius/map_resolution, true, zero_vector);
		}
	}
	else if(path_planning_algorithm_ == 7) // use voronoi explorator
	{
		// create a usable occupancyGrid map out of the given room map
		nav_msgs::OccupancyGrid room_gridmap;
		matToMap(room_gridmap, room_map);

		// TODO: find nearest pose to starting-position and start there
		if(plan_for_footprint_==false)
		{
			// convert fov-radius to pixel integer
			int fov_radius_as_int = (int) std::floor(fitting_circle_radius/map_resolution);
			std::cout << "fov radius in pixel: " << fov_radius_as_int << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, fov_radius_as_int); // radius in pixel

			// get the exploration path
			std::vector<geometry_msgs::Pose2D> fov_path;
			vm.generatePath(fov_path, cv::Mat());

			// map fov-path to robot-path
			cv::Point start_pos(fov_path.begin()->x, fov_path.begin()->y);
			mapPath(room_map, exploration_path, fov_path, middle_point, map_resolution, map_origin, start_pos);
//			for(size_t pos=0; pos<fov_path.size(); ++pos)
//			{
//				geometry_msgs::Pose2D current_pose;
//				current_pose.x = (fov_path[pos].x * map_resolution) + map_origin.x;
//				current_pose.y = (fov_path[pos].y * map_resolution) + map_origin.y;
//				current_pose.theta = fov_path[pos].theta;
//				exploration_path.push_back(current_pose);
//			}
		}
		else
		{
			// convert coverage-radius to pixel integer
			int coverage_radius = (int) std::floor(goal->coverage_radius/map_resolution);
			std::cout << "coverage radius in pixel: " << coverage_radius << std::endl;

			// create the object that plans the path, based on the room-map
			VoronoiMap vm(room_gridmap.data.data(), room_gridmap.info.width, room_gridmap.info.height, coverage_radius); // radius in pixel

			// get the exploration path
			vm.generatePath(exploration_path, cv::Mat());

			// transform to global coordinates
			for(size_t pos=0; pos<exploration_path.size(); ++pos)
			{
				exploration_path[pos].x = (exploration_path[pos].x * map_resolution) + map_origin.x;
				exploration_path[pos].y = (exploration_path[pos].y * map_resolution) + map_origin.y;
			}
		}

//		cv::Mat test_map = room_map.clone();
//		for(std::vector<geometry_msgs::Pose2D>::iterator pose=exploration_path.begin(); pose!=exploration_path.end()-1; ++pose)
//		{
//			cv::circle(test_map, cv::Point((pose->x-map_origin.x)/map_resolution, (pose->y-map_origin.y)/map_resolution), 2, cv::Scalar(127), CV_FILLED);
//			cv::line(test_map, cv::Point((pose->x-map_origin.x)/map_resolution, (pose->y-map_origin.y)/map_resolution), cv::Point(((pose+1)->x-map_origin.x)/map_resolution, ((pose+1)->y-map_origin.y)/map_resolution), cv::Scalar(100), 1);
////			cv::imshow("path", test_map);
////			cv::waitKey();
//		}
//		cv::imshow("path", test_map);
//		cv::waitKey();
	}

	// check if the size of the exploration path is larger then zero
	if(exploration_path.size()==0)
		return;

	// if wanted, return the path as the result
	ipa_building_msgs::RoomExplorationResult action_result;
	if(goal->return_path == true)
		action_result.coverage_path = exploration_path;

	// check if the path should be executed, if not end here
	if(goal->execute_path == false)
	{
		room_exploration_server_.setSucceeded(action_result);
		return;
	}

	// ***************** III. Navigate trough all points and save the robot poses to check what regions have been seen *****************
	// 1. publish navigation goals
	double distance_robot_fov_middlepoint = middle_point.norm();
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
		publishNavigationGoal(exploration_path[nav_goal], goal->map_frame, goal->camera_frame, robot_poses, distance_robot_fov_middlepoint, goal_eps_, true); // eps = 0.35
	}

	std::cout << "published all navigation goals, starting to check seen area" << std::endl;

	// 2. get the global costmap, that has initially not known objects in to check what regions have been seen
	const std::string costmap_topic = "/move_base/global_costmap/costmap";
	nav_msgs::OccupancyGrid global_costmap;
	global_costmap = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(costmap_topic));
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
		std::string coverage_service_name = "/coverage_check_server/coverage_check";
		cv::Mat seen_positions_map;
		// define the request for the coverage check
		ipa_building_msgs::checkCoverageRequest coverage_request;
		ipa_building_msgs::checkCoverageResponse coverage_response;
		// fill request
		sensor_msgs::ImageConstPtr service_image;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = costmap_as_mat;
		service_image = cv_image.toImageMsg();
		coverage_request.input_map = *service_image;
		coverage_request.path = robot_poses;
		coverage_request.field_of_view = goal->field_of_view;
		coverage_request.footprint = goal->footprint;
		coverage_request.map_origin = goal->map_origin;
		coverage_request.map_resolution = map_resolution;
		coverage_request.check_number_of_coverages = false;
		std::cout << "filled service request for the coverage check" << std::endl;
		if(plan_for_footprint_ == false)
		{
			coverage_request.check_for_footprint = false;
			// send request
			if(ros::service::call(coverage_service_name, coverage_request, coverage_response) == true)
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
			if(ros::service::call(coverage_service_name, coverage_request, coverage_response) == true)
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

		// testing, TODO: parameter to show
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
		black_map = cv::Scalar(0);
		cv::drawContours(black_map, areas_to_revisit, -1, cv::Scalar(255), CV_FILLED);
		for(size_t contour = 0; contour < left_areas.size(); ++contour)
			if(hierarchy[contour][3] != -1)
				cv::drawContours(black_map, left_areas, contour, cv::Scalar(0), CV_FILLED);

		// 2. Intersect the left areas with respect to the calculated grid length.
		for(size_t i =  min_max_coordinates.points[0].y; i < black_map.cols; i += grid_length)
			cv::line(black_map, cv::Point(0, i), cv::Point(black_map.cols, i), cv::Scalar(0), 1);
		for(size_t i =  min_max_coordinates.points[0].x; i < black_map.rows; i += grid_length)
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
//		 black_map = room_map.clone();
//		 for(size_t i = 0; i < area_centers.size(); ++i)
//		 {
//			 cv::circle(black_map, area_centers[i], 2, cv::Scalar(127), CV_FILLED);
//			 std::cout << area_centers[i] << std::endl;
//		 }
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
		std::string perimeter_service_name = "/map_accessibility_analysis/map_perimeter_accessibility_check";
	//	robot_poses.clear();
		for(size_t center = 0; center < revisiting_order.size(); ++center)
		{
			geometry_msgs::Pose2D current_center;
			current_center.x = (area_centers[revisiting_order[center]].x * map_resolution) + map_origin.x;
			current_center.y = (area_centers[revisiting_order[center]].y * map_resolution) + map_origin.y;

			// define request
			cob_map_accessibility_analysis::CheckPerimeterAccessibility::Request check_request;
			cob_map_accessibility_analysis::CheckPerimeterAccessibility::Response response;
			check_request.center = current_center;
			if(plan_for_footprint_ == false)
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
				std::cout << "successful check of accessiblity" << std::endl;
				// go trough the found accessible positions and try to reach one of them
				for(std::vector<geometry_msgs::Pose2D>::iterator pose = response.accessible_poses_on_perimeter.begin(); pose != response.accessible_poses_on_perimeter.end(); ++pose)
					if(publishNavigationGoal(*pose, goal->map_frame, goal->camera_frame, robot_poses, 0.0) == true)
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

	ROS_INFO("Explored room.");

	room_exploration_server_.setSucceeded(action_result);

	return;
}

// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");
	ros::Time::init();

	ros::NodeHandle nh("~");

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room exploration has been initialized......");
	ros::spin();

	return 0;
}

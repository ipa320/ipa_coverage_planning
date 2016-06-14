#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::execute_exploration_server, this, _1), false)
{
	//Start action server
	room_exploration_server_.start();

	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("room_segmentation_algorithm", path_planning_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << path_planning_algorithm_ << std::endl << std::endl;
	if (path_planning_algorithm_ == 1)
		ROS_INFO("You have chosen the grid exploration method.");

	if (path_planning_algorithm_ == 1) // get grid point exploration parameters
	{
		node_handle_.param("grid_line_length", grid_line_length_, 10);
		std::cout << "room_exploration/grid_line_length = " << grid_line_length_ << std::endl;
	}
}

// Function to publish a navigation goal for move_base. It returns true, when the goal could be reached.
// The function tracks the robot pose while moving to the goal and adds these poses to the given pose-vector. This is done
// because it allows to calculate where the robot field of view has theoretically been and identify positions of the map that
// the robot hasn't seen.
bool RoomExplorationServer::publish_navigation_goal(const geometry_msgs::Pose2D& nav_goal, const std::string map_frame,
		const std::string base_frame, std::vector<geometry_msgs::Pose2D>& robot_poses)
{
	// move base client, that sends navigation goals to a move_base action server
	MoveBaseClient mv_base_client("/move_base", true);

	// wait for the action server to come up
	while(!mv_base_client.waitForServer(ros::Duration(5.0)))
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
	do
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;

		// try to get the transformation from map_frame to base_frame, wait max 5 seconds for this transform to come up
		try
		{
			ros::Time time = ros::Time(0);
			listener.waitForTransform("/map", "/base_footprint", time, ros::Duration(5.0));
			listener.lookupTransform("/map", "/base_footprint", time, transform);

			ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getOrigin().y());

			// save the current pose
			geometry_msgs::Pose2D current_pose;

			current_pose.x = transform.getOrigin().x();
			current_pose.y = transform.getOrigin().y();
			double roll, pitch, yaw;
			transform.getBasis().getRPY(roll, pitch, yaw);
			current_pose.theta = yaw;

			robot_poses.push_back(current_pose);
		}
		catch(tf::TransformException ex)
		{
			ROS_INFO("Couldn't get transform!");// %s", ex.what());
		}

	}while(mv_base_client.getState() != actionlib::SimpleClientGoalState::ABORTED && mv_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	// check if point could be reached or not
	if(mv_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, the base moved to the goal");
		return true;
	}
	else
	{
		ROS_INFO("The base failed to move to the goal for some reason");
		return false;
	}
}

// Function executed by Call.
void RoomExplorationServer::execute_exploration_server(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Room Exploration action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	// read the given parameters out of the goal
	cv::Point2d map_origin;
	map_origin.x = goal->map_origin.x;
	map_origin.y = goal->map_origin.y;

	float map_resolution = goal->map_resolution;

	float robot_radius = goal->robot_radius;

	geometry_msgs::Pose2D starting_position = goal->starting_position;
	geometry_msgs::Polygon min_max_coordinates = goal->room_min_max;

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;
	transform_image_to_room_cordinates(room_map);

	// plan the path using the wanted planner
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if(path_planning_algorithm_ == 1) // use grid point explorator
	{
		// set grid size
		grid_point_planner.setGridLineLength(grid_line_length_);

		// plan path
//		grid_point_planner.getExplorationPath(room_map, exploration_path, robot_radius, map_resolution, starting_position, min_max_coordinates, map_origin);
	}

	// after planning a path, navigate trough all points and save the robot poses to check what regions have been seen
	std::vector<geometry_msgs::Pose2D> robot_poses;
	for(size_t nav_goal = 0; nav_goal < 8; ++nav_goal)
	{
//		cv::Mat map_copy = room_map.clone();
//
//		cv::circle(map_copy, cv::Point(exploration_path[nav_goal].y / map_resolution, exploration_path[nav_goal].x / map_resolution), 3, cv::Scalar(127), CV_FILLED);
//		cv::imshow("current_goal", map_copy);
//		cv::waitKey();

//		publish_navigation_goal(exploration_path[nav_goal], goal->map_frame, goal->base_frame, robot_poses);
	}

	geometry_msgs::Pose2D nav_goal;
	nav_goal.x = convert_pixel_to_meter_for_x_coordinate(150, map_resolution, map_origin);
	nav_goal.y = convert_pixel_to_meter_for_y_coordinate(100, map_resolution, map_origin);
	nav_goal.theta = -0.5*3.14159;

//	std::cout << "nav goal: (" << nav_goal.x << ", "  << nav_goal.y << ", " << nav_goal.theta << ")" << std::endl;

	publish_navigation_goal(nav_goal, goal->map_frame, goal->base_frame, robot_poses);

//	tf::TransformListener listener;
//	tf::StampedTransform transform;
//
//	ros::Time begin = ros::Time::now();
//	listener.waitForTransform("/map", "/base_footprint", begin, ros::Duration(10.0));
//	listener.lookupTransform("/map", "/base_footprint", begin, transform);
//
//	ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getOrigin().y());

	for(size_t i = 0; i < robot_poses.size(); ++i)
	{
//		std::cout << "Pose: " << robot_poses[i] << std::endl;
		std::vector<geometry_msgs::Pose2D> test;
		publish_navigation_goal(robot_poses[i], goal->map_frame, goal->base_frame, test);
	}

	room_exploration_server_.setSucceeded();
}

// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");

	ros::NodeHandle nh;

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room exploration has been initialized......");
	ros::spin();

	return 0;
}

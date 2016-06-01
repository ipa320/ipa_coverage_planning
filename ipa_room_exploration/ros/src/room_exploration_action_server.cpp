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
bool RoomExplorationServer::publish_navigation_goal(const geometry_msgs::Pose2D& nav_goal)
{
	// move base client, that sends navigation goals to a move_base action server
	MoveBaseClient mv_base_client_("move_base", true);

	// wait for the action server to come up
	while(!mv_base_client_.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal move_base_goal;

	// create mve_base_goal
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();

	move_base_goal.target_pose.pose.position.x = nav_goal.x;
	move_base_goal.target_pose.pose.position.y = nav_goal.y;
	move_base_goal.target_pose.pose.orientation.z = std::sin(nav_goal.theta/2);
	move_base_goal.target_pose.pose.orientation.w = std::cos(nav_goal.theta/2);

	// send goal to the move_base sever, when one is found
	ROS_INFO("Sending goal");
	mv_base_client_.sendGoal(move_base_goal);

	// wait until action is done
	mv_base_client_.waitForResult();

	// check if point could be reached or not
	if(mv_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
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

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;

	// plan the path using the wanted planner
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if(path_planning_algorithm_ == 1) // use grid point explorator
	{
		// set grid size
		grid_point_planner.setGridLineLength(grid_line_length_);

		// plan path
		grid_point_planner.getExplorationPath(room_map, exploration_path, robot_radius, starting_position);
	}

	geometry_msgs::Pose2D nav_goal;
	nav_goal.x = convert_pixel_to_meter_for_x_coordinate(150, map_resolution, map_origin);
	nav_goal.y = convert_pixel_to_meter_for_y_coordinate(100, map_resolution, map_origin);
	nav_goal.theta = 0.0;

	std::cout << "nav goal: (" << nav_goal.x << ", "  << nav_goal.y << ")" << std::endl;

	publish_navigation_goal(nav_goal);

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

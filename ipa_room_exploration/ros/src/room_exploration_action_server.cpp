#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::execute_exploration_server, this, _1), false)
{
	//Start action server
	room_exploration_server_.start();
}

// Function to publish a navigation goal for move_base. It returns true, when the goal could be reached.
bool RoomExplorationServer::publish_navigation_goal(const geometry_msgs::Pose2D& nav_goal)
{
	// move base client, that sends navigation goals to a move_base action server
	MoveBaseClient mv_base_client_("move_base", true);

	//wait for the action server to come up
	while(!mv_base_client_.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO("Waiting for the move_base action server to come up");
	}

	double x = 5.5;
	double y = 5.0;
	double theta = 0.5*0;

	move_base_msgs::MoveBaseGoal move_base_goal;

	//we'll send a goal to the robot to move 1 meter forward
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();

	move_base_goal.target_pose.pose.position.x = x;
	move_base_goal.target_pose.pose.position.y = y;
	move_base_goal.target_pose.pose.orientation.z = std::sin(theta/2);
	move_base_goal.target_pose.pose.orientation.w = std::cos(theta/2);

	ROS_INFO("Sending goal");
	mv_base_client_.sendGoal(move_base_goal);

	mv_base_client_.waitForResult();

	if(mv_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("Hooray, the base moved ");
	else
	  ROS_INFO("The base failed to move for some reason");

	return true;
}

// Function executed by Call.
void RoomExplorationServer::execute_exploration_server(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Room Exploration action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	geometry_msgs::Pose2D nav_goal;

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

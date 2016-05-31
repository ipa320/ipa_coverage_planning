#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::execute_exploration_server, this, _1), false)
{
	//Start action server
	room_exploration_server_.start();
}

// function executed by call
void RoomExplorationServer::execute_exploration_server(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Room Exploration action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO("Waiting for the move_base action server to come up");
	}

	double x = 5.5;
	double y = 5.0;
	double theta = 0.5*0;

	move_base_msgs::MoveBaseGoal nav_goal;

	//we'll send a goal to the robot to move 1 meter forward
	nav_goal.target_pose.header.frame_id = "map";
	nav_goal.target_pose.header.stamp = ros::Time::now();

	nav_goal.target_pose.pose.position.x = x;
	nav_goal.target_pose.pose.position.y = y;
	nav_goal.target_pose.pose.orientation.z = std::sin(theta/2);
	nav_goal.target_pose.pose.orientation.w = std::cos(theta/2);

	ROS_INFO("Sending goal");
	ac.sendGoal(nav_goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("Hooray, the base moved ");
	else
	  ROS_INFO("The base failed to move for some reason");

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

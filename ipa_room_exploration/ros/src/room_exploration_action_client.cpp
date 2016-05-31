#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_exploration/RoomExplorationAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_client");

	actionlib::SimpleActionClient<ipa_room_exploration::RoomExplorationAction> ac("/room_exploration_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	ipa_room_exploration::RoomExplorationGoal goal;
	ac.sendGoal(goal);
	return 0;
}

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <ipa_room_exploration/RoomExplorationAction.h>

#include <ipa_room_exploration/grid_point_explorator.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RoomExplorationServer
{
protected:

	int path_planning_algorithm_; // variable to specify which algorithm is going to be used to plan a path

	// This is the execution function used by action server
	void execute_exploration_server(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal);

	// !!Important!!
	//  define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_room_exploration::RoomExplorationAction> room_exploration_server_;

public:
	// initialize the action-server
	RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action);

	// Default destructor for the class
	~RoomExplorationServer(void)
	{
	}
};

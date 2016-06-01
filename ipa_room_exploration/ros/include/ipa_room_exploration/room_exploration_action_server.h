#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <ipa_room_exploration/RoomExplorationAction.h>

#include <ipa_room_exploration/grid_point_explorator.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RoomExplorationServer
{
protected:

	int path_planning_algorithm_; // variable to specify which algorithm is going to be used to plan a path
									// 1: grid point explorator

	gridPointExplorator grid_point_planner; // object that uses the grid point method to plan a path trough a room

	// parameters for the different planners
	int grid_line_length_; // size of the grid-lines that the grid-point-explorator lays over the map

	// this is the execution function used by action server
	void execute_exploration_server(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal);

	// function to publish a navigation goal, it returns true if the goal could be reached
	bool publish_navigation_goal(const geometry_msgs::Pose2D& nav_goal);

	//converter-> Pixel to meter for X coordinate
	double convert_pixel_to_meter_for_x_coordinate(const int pixel_valued_object_x, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution) + map_origin.x;
		return meter_value_obj_x;
	}
	//converter-> Pixel to meter for Y coordinate
	double convert_pixel_to_meter_for_y_coordinate(int pixel_valued_object_y, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution) + map_origin.y;
		return meter_value_obj_y;
	}

	// !!Important!!
	//  define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_room_exploration::RoomExplorationAction> room_exploration_server_;

public:
	// initialize the action-server
	RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action);

	// default destructor for the class
	~RoomExplorationServer(void)
	{
	}
};

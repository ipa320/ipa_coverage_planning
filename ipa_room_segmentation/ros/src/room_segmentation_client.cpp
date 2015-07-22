#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_action");

	cv::Mat map = cv::imread("room_segmentation/ipa_map.png", 0);

	sensor_msgs::Image labeling;

	cv_bridge::CvImage cv_image;
//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(labeling);
	// create the action client --> "name of server"
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> ac("/room_segmentation/room_segmentation_server", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	ipa_room_segmentation::MapSegmentationGoal goal;
	goal.input_map = labeling;
	goal.map_origin_x = 0;
	goal.map_origin_y = 0;
	goal.map_resolution = 0.05;
	goal.return_format_in_meter = false;
	goal.return_format_in_pixel = true;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

	if(finished_before_timeout)
	{
		ROS_INFO("alles gut!");
	}

	//exit
	return 0;
}

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_building_msgs/MapSegmentationAction.h>
#include <ipa_building_msgs/FindRoomSequenceWithCheckpointsAction.h>

#include <ipa_building_navigation/dynamic_reconfigure_client.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_sequence_planning_client");
	ros::NodeHandle nh;

	std::vector< std::string > map_names;
	map_names.push_back("lab_ipa.png");
//	map_names.push_back("freiburg_building101.png");
	map_names.push_back("freiburg_building52.png");
	map_names.push_back("freiburg_building79.png");
//	map_names.push_back("intel_map.png");
//	map_names.push_back("lab_a.png");
//	map_names.push_back("lab_b.png");
	map_names.push_back("lab_c.png");
	map_names.push_back("lab_d.png");
//	map_names.push_back("lab_e.png");

	for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
	{
		std::string image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_names[image_index];
		cv::Mat map = cv::imread(image_filename.c_str(), 0);
		//make non-white pixels black
		for (int y = 0; y < map.rows; y++)
		{
			for (int x = 0; x < map.cols; x++)
			{
				//find not reachable regions and make them black
				if (map.at<unsigned char>(y, x) != 255)
				{
					map.at<unsigned char>(y, x) = 0;
				}
			}
		}

		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
	//	cv_image.header.stamp = ros::Time::now();
		cv_image.encoding = "mono8";
		cv_image.image = map;
		cv_image.toImageMsg(map_msg);
		// create the action client --> "name of server"
		// true causes the client to spin its own thread
		actionlib::SimpleActionClient<ipa_building_msgs::MapSegmentationAction> ac_seg("/room_segmentation/room_segmentation_server", true);
		ROS_INFO("Waiting for action server '/room_segmentation/room_segmentation_server' to start.");
		// wait for the action server to start
		ac_seg.waitForServer(); //will wait for infinite time

		// set algorithm parameters
		ROS_INFO("Action server started, sending goal.");
		DynamicReconfigureClient drc_seg(nh, "/room_segmentation/room_segmentation_server/set_parameters", "/room_segmentation/room_segmentation_server/parameter_updates");
		drc_seg.setConfig("room_segmentation_algorithm", 1);

		// send a goal to the action
		ipa_building_msgs::MapSegmentationGoal goal_seg;
		goal_seg.input_map = map_msg;
		goal_seg.map_origin.position.x = 0;
		goal_seg.map_origin.position.y = 0;
		goal_seg.map_resolution = 0.05;
		goal_seg.return_format_in_meter = false;
		goal_seg.return_format_in_pixel = true;
		ac_seg.sendGoal(goal_seg);

		//wait for the action to return
		bool finished_before_timeout = ac_seg.waitForResult(ros::Duration(300.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room segmentation.");
			return -1;
		}
		ipa_building_msgs::MapSegmentationResultConstPtr result_seg = ac_seg.getResult();
		ROS_INFO("Finished segmentation successfully!");

		// solve sequence problem
		actionlib::SimpleActionClient<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction> ac_seq("/room_sequence_planning/room_sequence_planning_server", true);
		ROS_INFO("Waiting for action server '/room_sequence_planning/room_sequence_planning_server' to start.");
		// wait for the action server to start
		ac_seq.waitForServer(); //will wait for infinite time

		// set algorithm parameters
		ROS_INFO("Action server started, sending goal_seq.");
		DynamicReconfigureClient drc_seq(nh, "/room_sequence_planning/room_sequence_planning_server/set_parameters", "/room_sequence_planning/room_sequence_planning_server/parameter_updates");
		drc_seq.setConfig("planning_method", 1);
		drc_seq.setConfig("tsp_solver", 3);
		drc_seq.setConfig("return_sequence_map", true);
		drc_seq.setConfig("display_map", true);

		// send a goal_seg to the action
		ipa_building_msgs::FindRoomSequenceWithCheckpointsGoal goal_seq;
		goal_seq.input_map = map_msg;
		goal_seq.map_resolution = goal_seg.map_resolution;
		goal_seq.map_origin.position.x = goal_seg.map_origin.position.x;
		goal_seq.map_origin.position.y = goal_seg.map_origin.position.y;
		goal_seq.room_information_in_pixel = result_seg->room_information_in_pixel;
		goal_seq.robot_radius = 0.3;
		cv::Mat map_eroded;
		cv::erode(map, map_eroded, cv::Mat(), cv::Point(-1,-1), goal_seq.robot_radius/goal_seq.map_resolution+2);
		cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image
		bool robot_start_coordinate_set = false;
		for (int v=0; v<map_eroded.rows && robot_start_coordinate_set==false; ++v)
			for (int u=0; u<map_eroded.cols && robot_start_coordinate_set==false; ++u)
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<uchar>(v,u) > 15)
				{
					goal_seq.robot_start_coordinate.position.x = u*goal_seq.map_resolution + goal_seg.map_origin.position.x;
					goal_seq.robot_start_coordinate.position.y = v*goal_seq.map_resolution + goal_seg.map_origin.position.y;
					robot_start_coordinate_set = true;
				}
		ac_seq.sendGoal(goal_seq);

		//wait for the action to return
		finished_before_timeout = ac_seq.waitForResult(ros::Duration(300.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room sequence planning.");
			return -1;
		}
		ROS_INFO("Finished sequence planning successfully!");

	}

	//exit
	return 0;
}

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_segmentation_client");

	std::vector< std::string > map_names;
	map_names.push_back("lab_ipa.png");
//	map_names.push_back("freiburg_building101.png");
//	map_names.push_back("freiburg_building52.png");
//	map_names.push_back("freiburg_building79.png");
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
		goal.map_origin.position.x = 0;
		goal.map_origin.position.y = 0;
		goal.map_resolution = 0.05;
		goal.return_format_in_meter = false;
		goal.return_format_in_pixel = true;
		ac.sendGoal(goal);

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

		if (finished_before_timeout)
			ROS_INFO("Finished successfully!");
	}

	//exit
	return 0;
}

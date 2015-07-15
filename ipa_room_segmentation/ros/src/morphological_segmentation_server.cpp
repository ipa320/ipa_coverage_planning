#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#include <ipa_room_segmentation/morphological_segmentation.h>


int main(int argc, char** argv)
{
	cv::Mat original_map_from_subscription = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", -1);

	if (original_map_from_subscription.empty())
	{
		ROS_INFO("Fehler Bildladen");
		return false;
	}
	//make non-white pixels black
	for (int y_coordinate = 0; y_coordinate < original_map_from_subscription.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < original_map_from_subscription.rows; x_coordinate++)
		{
			//find not reachable regions and make them black
			if (original_map_from_subscription.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				original_map_from_subscription.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	ros::init(argc, argv, "morpholigical_segmentation_server");
	ros::NodeHandle n;
	morphological_segmentation segment(original_map_from_subscription, 0.05, 1.0, 40.0);
	//ros::spin();

	return 0;
}

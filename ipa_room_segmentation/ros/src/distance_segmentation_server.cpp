#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

#include <ipa_room_segmentation/distance_segmentation.h>


int main(int argc, char **argv)
{
	cv::Mat original_map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	for (int y_coordinate = 0; y_coordinate < original_map.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < original_map.rows; x_coordinate++)
		{
			//find not reachable regions andmake them black
			if (original_map.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				original_map.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	ros::init(argc, argv, "distance_segmentation_server");
	ros::NodeHandle n;
	ROS_INFO("Raumsegmentierung auf Basis einer distanztransformierten Karte");
	distance_segmentation segmenter(original_map, 0.05, 1.0, 45.0);
	ROS_INFO("Done segmenting the map");

	return 0;
}

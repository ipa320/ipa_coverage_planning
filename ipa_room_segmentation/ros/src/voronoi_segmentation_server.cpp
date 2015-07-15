#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

#include <ipa_room_segmentation/voronoi_segmentation.h>

#define PI 3.14159265

int main(int argc, char **argv)
{
	ros::init(argc, argv, "voronoi_segmentation_server");
	ros::NodeHandle n;
//ros::ServiceServer service = n.advertiseService("roomsegmentation", segmentation_algorithm);
	ROS_INFO("Segmentation of a gridmap based on a voronoi-graph");
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
//	for (int y_coordinate = 0; y_coordinate < map.cols; y_coordinate++)
//	{
//		for (int x_coordinate = 0; x_coordinate < map.rows; x_coordinate++)
//		{
//			//find not reachable regions andmake them black
//			if (map.at<unsigned char>(x_coordinate, y_coordinate) != 255)
//			{
//				map.at<unsigned char>(x_coordinate, y_coordinate) = 0;
//			}
//		}
//	}
	if (map.empty())
	{
		ROS_INFO("Fehler Bildladen");
		return -1;
	}

	std::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	voronoi_segmentation segmenter(map, 0.05, 2.0, 40.0);

	//ros::spin();
	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);

	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;

	return 0;
}

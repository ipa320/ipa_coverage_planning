#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

#include <ctime>
#include <stdlib.h>

#include <ipa_room_segmentation/adaboost_classifier.h>

#define PI 3.14159265


int main(int argc, char **argv)
{
	ros::init(argc, argv, "semantic_segmentation_server");
	ros::NodeHandle n;
	ROS_INFO("Semantic labeling of places using the generalized AdaBoost-Algorithm from OpenCV. Detects rooms and hallways.");
//	ros::spin();
	// todo: no absolute paths
	cv::Mat first_room_training_map = cv::imread("/home/rmb-fj/Pictures/maps/room_training_map.png", 0);
	cv::Mat second_room_training_map = cv::imread("/home/rmb-fj/Pictures/maps/lab_d_room_training_map.png", 0);
	cv::Mat first_hallway_training_map = cv::imread("/home/rmb-fj/Pictures/maps/hallway_training_map.png", 0);
	cv::Mat second_hallway_training_map = cv::imread("/home/rmb-fj/Pictures/maps/lab_a_hallway_training_map.png", 0);
//	cv::Mat map_to_be_labeled = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	cv::Mat map_to_be_labeled = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	for (int y_coordinate = 0; y_coordinate < map_to_be_labeled.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < map_to_be_labeled.rows; x_coordinate++)
		{
			//find not reachable regions and make them black
			if (map_to_be_labeled.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				map_to_be_labeled.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}

	std::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	adaboost_classifier segmenter(map_to_be_labeled, 0.05, 1.0, 40.0);

	segmenter.trainClassifiers(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);

	cv::Mat segmented_Map = segmenter.semanticLabeling(map_to_be_labeled);

	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);

	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;

	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/boost_labeled.png", segmented_Map);

	return 0;
}

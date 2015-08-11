#include "ros/ros.h"
#include <ros/package.h>

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

#include <omp.h>

#include <ipa_building_navigation/A_star_pathplanner.h>

int main(int argc, char **argv)
{
	srand(5); //time(NULL));
	ros::init(argc, argv, "boosttester");
	ros::NodeHandle nh;

	// This statement should only print once
	printf("Starting Program!\n");

#pragma omp parallel
	{
		// This statement will run on each thread.
		// If there are 4 threads, this will execute 4 times in total
		std::cout << "Running on multiple trheads" << std::endl;
	}

	// We're out of the parallelized secion.
	// Therefor, this should execute only once
	std::cout << "Finished" << std::endl;

//	AStarPlanner planner;
//
//	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
//
//	double downsampling_factor = 0.5;
//
//	cv::Mat downsampled_map;
//	cv::resize(map, downsampled_map, cv::Size(0, 0), downsampling_factor, downsampling_factor, cv::INTER_LINEAR);
//
//	cv::Point start_point(150, 150);
//	cv::Point end_point(300, 300);
//
//	int start_x = downsampling_factor * start_point.x;
//	int start_y = downsampling_factor * start_point.y;
//	int end_x = downsampling_factor * end_point.x;
//	int end_y = downsampling_factor * end_point.y;
//
//	double length = planner.PlanPath(map, start_point, end_point, downsampling_factor);
//
//	cv::circle(map, start_point, 2, cv::Scalar(127), CV_FILLED);
//	cv::circle(map, end_point, 2, cv::Scalar(127), CV_FILLED);
//	cv::circle(downsampled_map, cv::Point(start_x, start_y), 1, cv::Scalar(127), CV_FILLED);
//	cv::circle(downsampled_map, cv::Point(end_x, end_y), 1, cv::Scalar(127), CV_FILLED);
//
//	std::cout << length << std::endl;
//
//	cv::imshow("original", map);
//	cv::imshow("downsampled", downsampled_map);
//	cv::waitKey();

	return 0;
}

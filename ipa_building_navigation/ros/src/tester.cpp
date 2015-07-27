#include "ros/ros.h"

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

#include <ipa_building_navigation/A_star_pathplanner.h>
//#include <ipa_building_navigation/nearest_neighbor_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>

int main(int argc, char **argv)
{
	srand (5);//time(NULL));
	ros::init(argc, argv, "a_star_tester");
	ros::NodeHandle nh;
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);

	AStarPlanner planner;
//	NearestNeigborTSPSolver TSPsolver;
	GeneticTSPSolver genTSPsolver;


	std::vector<cv::Point> centers;

	cv::Mat pathlengths(cv::Size(27, 27), CV_64F);
	cv::Mat eroded_map;

	cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1,-1), 4);

	for(int i = 0; i < 27; i++)//add Points for TSP to test the solvers
	{
		bool done = false;
		do
		{
			int x = rand() % map.rows;
			int y = rand() % map.cols;
			if(eroded_map.at<unsigned char>(x,y) == 255)
			{
				centers.push_back(cv::Point(x,y));
				done = true;
			}
		}while(!done);
	}

	cv::Mat testermap = map.clone();

	for(int i = 0; i < centers.size(); i++)
	{
		cv::circle(testermap, cv::Point(centers[i].y, centers[i].x), 2, cv::Scalar(127), CV_FILLED);
		for(int p = 0; p < centers.size(); p++)
		{
			if(p != i)
			{
				if(p > i)//only compute upper right triangle of matrix, rest is symmetrically added
				{
					double length = planner.PlanPath(map, centers[i], centers[p]);
					pathlengths.at<double>(i, p) = length;
					pathlengths.at<double>(p, i) = length; //symmetrical-Matrix --> saves half the computation time
				}
			}
			else
			{
				pathlengths.at<double>(i, p) = -1;
			}
		}
	}

	for(int row = 0; row < pathlengths.rows; row++)
	{
		for(int col = 0; col < pathlengths.cols; col++)
		{
			std::cout << pathlengths.at<double>(row, col) << " ";
		}
		std::cout << std::endl;
	}

	std::vector<int> TSPorder = genTSPsolver.solveGeneticTSP(pathlengths, 0);

	cv::circle(testermap, cv::Point(centers[0].y, centers[0].x), 2, cv::Scalar(73), CV_FILLED);

	for(int i = 0; i < TSPorder.size()-1; i++)
	{
		cv::line(testermap, cv::Point(centers[TSPorder[i]].y, centers[TSPorder[i]].x), cv::Point(centers[TSPorder[i+1]].y, centers[TSPorder[i+1]].x), cv::Scalar(127));
	}

//	cv::imshow("test", testermap);
	cv::imwrite("/home/rmb-fj/Pictures/TSP/genetic.png", testermap);
//	cv::waitKey(1000000);

	return 0;
}

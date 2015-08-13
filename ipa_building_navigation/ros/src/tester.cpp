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

//#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_building_navigation/nearest_neighbor_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>
#include <ipa_building_navigation/concorde_TSP.h>

#include <ipa_building_navigation/maximal_clique_finder.h>
#include <ipa_building_navigation/set_cover_solver.h>

#include <ipa_building_navigation/trolley_position_finder.h>

int main(int argc, char **argv)
{
	srand(5); //time(NULL));
	ros::init(argc, argv, "a_star_tester");
	ros::NodeHandle nh;

	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);

	AStarPlanner planner;
	NearestNeighborTSPSolver TSPsolver;
	GeneticTSPSolver genTSPsolver;
	concordeTSPSolver conTSPsolver;

	cliqueFinder finder; //Object to find all maximal cliques for the given map

	setCoverSolver setsolver;	//Object to find the groups based on the found cliques

	trolleyPositionFinder tolley_finder;

	std::vector < cv::Point > centers;

	int n = 9;
	double downfactor = 0.25;
	double map_resolution_factor = 0.05;
	double robot_radius_test = 0.5;

	cv::Mat pathlengths (cv::Size(n, n), CV_64F);
	cv::Mat distancematrix;
//	cv::Mat eroded_map;
//
//	cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1, -1), 4);

	//Testcenters:
//	x: 494 y: 535
//	x: 218 y: 176
//	x: 152 y: 148
//	x: 475 y: 417
//	x: 342 y: 333
//	x: 283 y: 205
//	x: 149 y: 229
//	x: 201 y: 456
//	x: 286 y: 125


//	for (int i = 0; i < n; i++) //add Points for TSP to test the solvers
//	{
//		bool done = false;
//		do
//		{
//			int x = rand() % map.cols;
//			int y = rand() % map.rows;
//			if (eroded_map.at<unsigned char>(y, x) == 255)
//			{
//				centers.push_back(cv::Point(x, y));
//				done = true;
//			}
//		} while (!done);
//	}

	centers.push_back(cv::Point(494, 535));
	centers.push_back(cv::Point(218, 176));
	centers.push_back(cv::Point(152, 148));
	centers.push_back(cv::Point(475, 417));
	centers.push_back(cv::Point(342, 333));
	centers.push_back(cv::Point(283, 205));
	centers.push_back(cv::Point(149, 229));
	centers.push_back(cv::Point(201, 456));
	centers.push_back(cv::Point(286, 125));

	std::vector<int> nearest_neighbor_order = genTSPsolver.solveGeneticTSP(map, n, centers, downfactor, robot_radius_test, map_resolution_factor, 0);

	std::cout << "without distance matrix:" << std::endl;
	for(int i = 0; i < nearest_neighbor_order.size(); i++)
	{
		std::cout << nearest_neighbor_order[i] << std::endl;
	}
	std::cout << std::endl;

	nearest_neighbor_order = genTSPsolver.solveGeneticTSP(map, n, centers, downfactor, robot_radius_test, map_resolution_factor, 0, distancematrix);

	std::cout << "without distance matrix, returned:" << std::endl;
	for(int i = 0; i < nearest_neighbor_order.size(); i++)
	{
		std::cout << nearest_neighbor_order[i] << std::endl;
	}
	std::cout << std::endl;

	std::cout << "distance matrix out of solver: " << std::endl;
	for (int row = 0; row < distancematrix.rows; row++)
	{
		for (int col = 0; col < distancematrix.cols; col++)
		{
			std::cout << distancematrix.at<double>(row, col) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

//	cv::Mat testermap = map.clone();
//
	for (int i = 0; i < centers.size(); i++)
	{
		cv::Point center = centers[i];
		for (int p = 0; p < centers.size(); p++)
		{
			if (p != i)
			{
				if (p > i) //only compute upper right triangle of matrix, rest is symmetrically added
				{
					double length = planner.PlanPath(map, center, centers[p], downfactor, robot_radius_test, map_resolution_factor);
					pathlengths.at<double>(i, p) = length;
					pathlengths.at<double>(p, i) = length; //symmetrical-Matrix --> saves half the computationtime
				}
			}
			else
			{
				pathlengths.at<double>(i, p) = 0;
			}
		}
	}

	nearest_neighbor_order = genTSPsolver.solveGeneticTSP(pathlengths, 0);

	std::cout << "with distance matrix:" << std::endl;
	for(int i = 0; i < nearest_neighbor_order.size(); i++)
	{
		std::cout << nearest_neighbor_order[i] << std::endl;
	}
	std::cout << std::endl;

	std::cout << "distance matrix out of main: " << std::endl;
	for (int row = 0; row < pathlengths.rows; row++)
	{
		for (int col = 0; col < pathlengths.cols; col++)
		{
			std::cout << pathlengths.at<double>(row, col) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
//
//	std::vector<int> TSPorder = genTSPsolver.solveGeneticTSP(pathlengths, 0);
//
//	conTSPsolver.solveConcordeTSP(pathlengths, 0);
//
//	cv::circle(testermap, centers[0], 2, cv::Scalar(73), CV_FILLED);
//
//	for (int i = 0; i < TSPorder.size() - 1; i++)
//	{
//		std::cout << TSPorder[i] << std::endl;
//		cv::line(testermap, centers[TSPorder[i]], centers[TSPorder[i + 1]], cv::Scalar(127));
//	}
//
//	std::cout << std::endl;
//	std::cout << "All maximum cliques in the graph:" << std::endl;
//
//	std::vector < std::vector<int> > cliques = finder.getCliques(pathlengths, 150.0);
//
//	for (int i = 0; i < cliques.size(); i++)
//	{
//		for (int j = 0; j < cliques[i].size(); j++)
//		{
//			std::cout << cliques[i][j] << std::endl;
//		}
//		std::cout << std::endl;
//	}
//
//	ROS_INFO("Starting to solve the setcover problem.");
//
//	std::vector<std::vector<int> > groups = setsolver.solveSetCover(cliques, n);
//
//	ROS_INFO("Starting to find the trolley positions.");
//
//	std::vector<cv::Point> trolley_positions = tolley_finder.findTrolleyPositions(map, groups, centers, downfactor, robot_radius_test, map_resolution_factor);
//
//	std::cout << groups.size() << " " << trolley_positions.size() << std::endl;
//
//	for(int i = 0; i < groups.size(); i++)
//	{
//		cv::Scalar group_colour(rand() % 200 + 20);
//		cv::circle(testermap, trolley_positions[i], 4, cv::Scalar(0), 1);
//		for(int j = 0; j < groups[i].size(); j++)
//		{
//			cv::circle(testermap, centers[groups[i][j]], 2, group_colour, CV_FILLED);
//			std::cout << groups[i][j] << std::endl;
//		}
//		std::cout << "group done" << std::endl;
//	}
//
//	cv::imwrite("/home/rmb-fj/Pictures/TSP/genetic.png", testermap);

	return 0;
}

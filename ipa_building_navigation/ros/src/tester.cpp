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

#include <ipa_building_navigation/A_star_pathplanner.h>
//#include <ipa_building_navigation/nearest_neighbor_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>
#include <ipa_building_navigation/maximal_clique_finder.h>

void writeToFile(cv::Mat& pathlength_matrix)
{
	std::ofstream saving_file("/home/rmb-fj/saver.txt");
	if (saving_file.is_open())
	{
		std::cout << "starting saving classifier parameters" << std::endl;
		saving_file << "NAME: ipa-building-navigation" << std::endl << "TYPE: TSP" << std::endl << "COMMENT: This is the TSPlib file for using concorde"
		        << std::endl;
		saving_file << "DIMENSION: " << pathlength_matrix.cols << std::endl;
		saving_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
		saving_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
		saving_file << "EDGE_WEIGHT_SECTION" << std::endl;

		for (int row = 0; row < pathlength_matrix.rows; row++)
		{
			for (int col = 0; col < pathlength_matrix.cols; col++)
			{
				saving_file << " " << (int) pathlength_matrix.at<double>(row, col);
			}
			saving_file << std::endl;
		}

		saving_file << "EOF";

		std::cout << "finished saving" << std::endl;
		saving_file.close();
	}
	else
	{
		std::cout << "nicht geöffnet1" << std::endl;
	}
}

int main(int argc, char **argv)
{
	srand(5); //time(NULL));
	ros::init(argc, argv, "a_star_tester");
	ros::NodeHandle nh;

	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);

	AStarPlanner planner;
//	NearestNeigborTSPSolver TSPsolver;
	GeneticTSPSolver genTSPsolver;

	cliqueFinder finder;

	std::vector < cv::Point > centers;

	int n = 8;

	cv::Mat pathlengths(cv::Size(n, n), CV_64F);
	cv::Mat eroded_map;

	cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1, -1), 4);

	for (int i = 0; i < n; i++) //add Points for TSP to test the solvers
	{
		bool done = false;
		do
		{
			int x = rand() % map.cols;
			int y = rand() % map.rows;
			if (eroded_map.at<unsigned char>(y, x) == 255)
			{
				centers.push_back(cv::Point(x, y));
				done = true;
			}
		} while (!done);
	}

	cv::Mat testermap = map.clone();

	for (int i = 0; i < centers.size(); i++)
	{
		cv::circle(testermap, centers[i], 2, cv::Scalar(127), CV_FILLED);
		for (int p = 0; p < centers.size(); p++)
		{
			if (p != i)
			{
				if (p > i) //only compute upper right triangle of matrix, rest is symmetrically added
				{
					double length = planner.PlanPath(map, centers[i], centers[p]);
					pathlengths.at<double>(i, p) = length;
					pathlengths.at<double>(p, i) = length; //symmetrical-Matrix --> saves half the computation time
				}
			}
			else
			{
				pathlengths.at<double>(i, p) = 0;
			}
		}
	}

	writeToFile(pathlengths);

//	std::string cmd = ros::package::getPath("concorde_tsp_solver") + "/bin/concorde .......";
//	result = system(cmd.c_str());
//	assert(!result);

	for (int row = 0; row < pathlengths.rows; row++)
	{
		for (int col = 0; col < pathlengths.cols; col++)
		{
			std::cout << pathlengths.at<double>(row, col) << " ";
		}
		std::cout << std::endl;
	}

	std::vector<int> TSPorder = genTSPsolver.solveGeneticTSP(pathlengths, 0);

	cv::circle(testermap, centers[0], 2, cv::Scalar(73), CV_FILLED);

	for (int i = 0; i < TSPorder.size() - 1; i++)
	{
		cv::line(testermap, centers[TSPorder[i]], centers[TSPorder[i + 1]], cv::Scalar(127));
	}

	cv::imwrite("/home/rmb-fj/Pictures/TSP/genetic.png", testermap);

	std::cout << std::endl;
	std::cout << "All maximum cliques in the graph:" << std::endl;

	std::vector < std::vector<int> > cliques = finder.getCliques(pathlengths, 300.0);

	for (int i = 0; i < cliques.size(); i++)
	{
		for (int j = 0; j < cliques[i].size(); j++)
		{
			std::cout << cliques[i][j] << std::endl;
		}
		std::cout << std::endl;
	}

	return 0;
}

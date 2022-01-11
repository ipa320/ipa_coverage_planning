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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <time.h>
#include <sys/time.h>

#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_building_navigation/contains.h>

#include <ipa_building_navigation/nearest_neighbor_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>
#include <ipa_building_navigation/concorde_TSP.h>

#include <ipa_building_navigation/distance_matrix.h>

int main(int argc, char **argv)
{
	srand(time(NULL));
	ros::init(argc, argv, "TSPevaluation");
	ros::NodeHandle nh;

	//define parameters to describe the Graph
	const int dimension = 600;

	//define parameters for solving the TSP
	double downsampling = 0.25;
	double robot_radius = 0.;
	double map_resolution = 0.05;
	int start_node = 0;

	//saving variables for pathlengths
	std::vector<double> nearest_pathlengths;
	std::vector<double> genetic_pathlengths;
	std::vector<double> concorde_pathlengths;

	//create empty map to random generate Points in it
	cv::Mat map(dimension, dimension, CV_8UC1, cv::Scalar(255));

	//path to save the results
	const std::string data_storage_path = "tsp_evaluation/";
	const std::string upper_command = "mkdir -p " + data_storage_path;
	int return_value = system(upper_command.c_str());

	//stingstreams to save the parameters
	std::stringstream pathlength_output;
	std::stringstream times_output;

	for(int number_of_nodes = 10; number_of_nodes <= 210; number_of_nodes += 20)
	{
		std::stringstream folder_name;
		folder_name << number_of_nodes << "nodes";
		const std::string evaluation_path = data_storage_path + folder_name.str() + "/";
		const std::string command = "mkdir -p " + evaluation_path;
		int return_value = system(command.c_str());

		//generate random Points as nodes for the TSP solvers
		int point_counter = 0;
		std::vector<cv::Point> nodes;
		std::cout << "getting random Points" << std::endl;
		do
		{
			int rand_x = (rand() % (dimension-30)) + 30;
			int rand_y = (rand() % (dimension-30)) + 30;
			if(map.at<unsigned char>(rand_y, rand_x) == 255)
			{
				nodes.push_back(cv::Point(rand_x, rand_y));
				point_counter++;
			}
		}while(point_counter < number_of_nodes);

		NearestNeighborTSPSolver nearest_solver;
		GeneticTSPSolver genetic_solver;
		ConcordeTSPSolver concorde_solver;

		//solve the TSPs and save the calculation time and orders
		cv::Mat distance_matrix;
		struct timespec t0, t1, t2, t3;

		//construct distance matrix once
		std::cout << "constructing distance matrix" << std::endl;
		AStarPlanner planner;
		DistanceMatrix distance_matrix_computation;
		distance_matrix_computation.constructDistanceMatrix(distance_matrix, map, nodes, downsampling, robot_radius, map_resolution, planner);

		std::cout << "solving TSPs" << std::endl;
		clock_gettime(CLOCK_MONOTONIC,  &t0);
		std::vector<int> nearest_order = nearest_solver.solveNearestTSP(distance_matrix, start_node);
		std::cout << "solved nearest TSP" << std::endl;
		clock_gettime(CLOCK_MONOTONIC,  &t1);
		std::vector<int> genetic_order = genetic_solver.solveGeneticTSP(distance_matrix, start_node);
		std::cout << "solved genetic TSP" << std::endl;
		clock_gettime(CLOCK_MONOTONIC,  &t2);
		std::vector<int> concorde_order = concorde_solver.solveConcordeTSP(distance_matrix, start_node);
		std::cout << "solved concorde TSP" << std::endl;
		clock_gettime(CLOCK_MONOTONIC,  &t3);

		std::cout << "number of nodes in the paths: " << nearest_order.size() << " " << genetic_order.size() << " " << concorde_order.size() << std::endl;

		//create maps to draw the paths in
		cv::Mat nearest_map = map.clone();
#if CV_MAJOR_VERSION<=3
		cv::cvtColor(nearest_map, nearest_map, CV_GRAY2BGR);
#else
		cv::cvtColor(nearest_map, nearest_map, cv::COLOR_GRAY2BGR);
#endif
		cv::Mat genetic_map = nearest_map.clone();
		cv::Mat concorde_map = nearest_map.clone();

		//draw the order into the maps
		//	draw the start node as red
		std::cout << "starting to draw the maps" << std::endl;
#if CV_MAJOR_VERSION<=3
		cv::circle(nearest_map, nodes[nearest_order[0]], 2, CV_RGB(255,0,0), CV_FILLED);
		cv::circle(genetic_map, nodes[genetic_order[0]], 2, CV_RGB(255,0,0), CV_FILLED);
		cv::circle(concorde_map, nodes[concorde_order[0]], 2, CV_RGB(255,0,0), CV_FILLED);
#else
		cv::circle(nearest_map, nodes[nearest_order[0]], 2, CV_RGB(255,0,0), cv::FILLED);
		cv::circle(genetic_map, nodes[genetic_order[0]], 2, CV_RGB(255,0,0), cv::FILLED);
		cv::circle(concorde_map, nodes[concorde_order[0]], 2, CV_RGB(255,0,0), cv::FILLED);
#endif
		for(size_t i = 1; i < nearest_order.size(); ++i)
		{
			cv::line(nearest_map,  nodes[nearest_order[i-1]],  nodes[nearest_order[i]], CV_RGB(128,128,255), 1);
			cv::line(genetic_map,  nodes[genetic_order[i-1]],  nodes[genetic_order[i]], CV_RGB(128,128,255), 1);
			cv::line(concorde_map,  nodes[concorde_order[i-1]],  nodes[concorde_order[i]], CV_RGB(128,128,255), 1);
#if CV_MAJOR_VERSION<=3
			cv::circle(nearest_map, nodes[nearest_order[i]], 2, CV_RGB(0,0,0), CV_FILLED);
			cv::circle(genetic_map, nodes[genetic_order[i]], 2, CV_RGB(0,0,0), CV_FILLED);
			cv::circle(concorde_map, nodes[concorde_order[i]], 2, CV_RGB(0,0,0), CV_FILLED);
#else
			cv::circle(nearest_map, nodes[nearest_order[i]], 2, CV_RGB(0,0,0), cv::FILLED);
			cv::circle(genetic_map, nodes[genetic_order[i]], 2, CV_RGB(0,0,0), cv::FILLED);
			cv::circle(concorde_map, nodes[concorde_order[i]], 2, CV_RGB(0,0,0), cv::FILLED);
#endif
		}
		//draw line back to start
		cv::line(nearest_map,  nodes[nearest_order[0]],  nodes[nearest_order.back()], CV_RGB(128,128,255), 1);
		cv::line(genetic_map,  nodes[genetic_order[0]],  nodes[genetic_order.back()], CV_RGB(128,128,255), 1);
		cv::line(concorde_map,  nodes[concorde_order[0]],  nodes[concorde_order.back()], CV_RGB(128,128,255), 1);

		//save the maps
		std::string nearest_path = evaluation_path + "nearest_order.png";
		std::string genetic_path = evaluation_path + "genetic_order.png";
		std::string concorde_path = evaluation_path + "concorde_order.png";
		cv::imwrite(nearest_path.c_str(), nearest_map);
		cv::imwrite(genetic_path.c_str(), genetic_map);
		cv::imwrite(concorde_path.c_str(), concorde_map);
		std::cout << "saved the maps" << std::endl;

		//get the pathlengths for each solver
		double nearest_pathlength= 0;
		double genetic_pathlength = 0;
		double concorde_pathlength = 0;
		//add each pathlength
		std::cout << "starting to calculate the pathlengths " << distance_matrix.cols << std::endl;
		for(size_t i = 1; i < nearest_order.size(); ++i)
		{
			nearest_pathlength += distance_matrix.at<double>(nearest_order[i-1], nearest_order[i]);
			genetic_pathlength += distance_matrix.at<double>(genetic_order[i-1], genetic_order[i]);
			concorde_pathlength += distance_matrix.at<double>(concorde_order[i-1], concorde_order[i]);
			std::cout << "done node: " << (int) i << std::endl;
		}
		//add path from end to start
		std::cout << "doing last paths. Indexes: " << nearest_order.back() << " " << genetic_order.back() << " " << concorde_order.back() << std::endl;
		int last_index = nearest_order.back();
		nearest_pathlength += distance_matrix.at<double>(nearest_order[0], last_index);
		std::cout << "finished nearest path" << std::endl;
		last_index = genetic_order.back();
		genetic_pathlength += distance_matrix.at<double>(genetic_order[0], last_index);
		std::cout << "finished genetic path." << std::endl;
		last_index = concorde_order.back();
		concorde_pathlength += distance_matrix.at<double>(concorde_order[0], last_index);
		std::cout << "finished concorde path" << std::endl;

		//calculate computation times
		double nearest_time = (t1.tv_sec - t0.tv_sec) + (double) (t1.tv_nsec - t0.tv_nsec) * 1e-9;
		double genetic_time = (t2.tv_sec - t1.tv_sec) + (double) (t2.tv_nsec - t1.tv_nsec) * 1e-9;
		double concorde_time = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;

		//save the pathlengths and computation times
		pathlength_output << "number of nodes: " << number_of_nodes << std::endl
				<< nearest_pathlength << std::endl << genetic_pathlength << std::endl << concorde_pathlength << std::endl << std::endl;
		times_output << "number of nodes: " << number_of_nodes << std::endl
				<< nearest_time << std::endl << genetic_time << std::endl << concorde_time << std::endl << std::endl;
	}
	std::string pathlength_log_filename = data_storage_path + "pathlengths.txt";
	std::ofstream pathlength_file(pathlength_log_filename.c_str(), std::ios::out);
	if (pathlength_file.is_open()==true)
		pathlength_file << pathlength_output.str();
	pathlength_file.close();
	std::cout << "finished to save the pathlengths" << std::endl;

	std::string genetic_log_filename = data_storage_path + "times.txt";
	std::ofstream genetic_file(genetic_log_filename.c_str(), std::ios::out);
	if (genetic_file.is_open()==true)
		genetic_file << times_output.str();
	genetic_file.close();
	std::cout << "finished to save the times" << std::endl;


	return 0;
}

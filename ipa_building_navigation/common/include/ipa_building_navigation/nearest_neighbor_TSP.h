#include "ros/ros.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ipa_building_navigation/contains.h>
#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_building_navigation/distance_matrix.h>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
			 //regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This class provides a solution for the TSP by taking the nearest neighbor from the current Point as next Point.
//It needs a symmetrical matrix of pathlenghts between the nodes and the starting-point index in this matrix.
//If the path from one node to another doesn't exist or the path is from one node to itself, the entry in the matrix must
//be 0 or smaller. so the format for this matrix is:
// row: node to start from, column: node to go to
//		---					   ---
//		| 0.0 1.0  3.5  5.8  1.2 |
//		| 1.0 0.0  2.4  3.3  9.0 |
//		| 3.5 2.4  0.0 	7.7  88.0|
//		| 5.8 3.3  7.7  0.0  0.0 |
//		| 1.2 9.0  88.0 0.0  0.0 |
//		---					   ---

class NearestNeighborTSPSolver
{
protected:

	//Astar pathplanner to find the pathlengths from cv::Point to cv::Point
	AStarPlanner pathplanner_;

//	//Function to construct the distance matrix, showing the pathlength from node to node
//	void NearestNeighborTSPSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map,
//			const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution);

public:
	//constructor
	NearestNeighborTSPSolver();

	//Solving-algorithms for the given TSP. It returns a vector of int, which is the order from this solution. The int shows
	//the index in the Matrix. There are two functions for different cases:
	//		1. The distance matrix already exists
	//		2. The distance matrix has to be computed and maybe returned

	//with given distance matrix
	std::vector<int> solveNearestTSP(const cv::Mat& path_length_matrix, const int start_node); //with given distance matrix

	// compute TSP and distance matrix without cleaning it
	// this version does not exclude infinite paths from the TSP ordering
	std::vector<int> solveNearestTSP(const cv::Mat& original_map, const std::vector<cv::Point>& points, double downsampling_factor,
			double robot_radius, double map_resolution, const int start_node, cv::Mat* distance_matrix=0);

	// compute TSP from a cleaned distance matrix (does not contain any infinity paths) that has to be computed
	std::vector<int> solveNearestTSPClean(const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, const int start_node);

	// compute TSP with pre-computed cleaned distance matrix (does not contain any infinity paths)
	std::vector<int> solveNearestTSPWithCleanedDistanceMatrix(const cv::Mat& distance_matrix,
			const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node);
};

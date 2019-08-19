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

#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_building_navigation/distance_matrix.h>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
//regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This class applies an object to solve a given TSP problem using the concorde TSP solver. This solver can be downloaded at:
//		http://www.math.uwaterloo.ca/tsp/concorde.html
//A short explanation on how to build the solver is given at:
//		http://www.math.uwaterloo.ca/tsp/concorde/DOC/README.html
//If you have build the solver navigate to the./TSP folder and type " ./concorde -h " to see how to use this solver. This class
//uses the concorde solver by using a systemcall.
//Make sure you have a "/common/files/TSP_order.txt" and a "/common/files/TSPlib_file.txt" file. These files are used to tell
//concorde the current problem and save the output of it.
//
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

class ConcordeTSPSolver
{
protected:

	//Astar pathplanner to find the pathlengths from cv::Point to cv::Point
	AStarPlanner pathplanner_;

	//Function to create neccessary TSPlib file to tell concorde what the problem is.
	void writeToFile(const cv::Mat& pathlength_matrix, const std::string& tsp_lib_filename, const std::string& tsp_order_filename);

	//Function to read the saved TSP order.
	std::vector<int> readFromFile(const std::string& tsp_order_filename);

	void distance_matrix_thread(DistanceMatrix& distance_matrix_computation, cv::Mat& distance_matrix,
			const cv::Mat& original_map, const std::vector<cv::Point>& points, double downsampling_factor,
			double robot_radius, double map_resolution, AStarPlanner& path_planner);

	bool abort_computation_;
	std::string unique_file_identifier_;

public:
	//Constructor
	ConcordeTSPSolver();

	void abortComputation();

	//Functions to solve the TSP. It needs a distance matrix, that shows the pathlengths between two nodes of the problem.
	//This matrix has to be symmetrical or else the TSPlib must be changed. The int shows the index in the Matrix.
	//There are two functions for different cases:
	//		1. The distance matrix already exists
	//		2. The distance matrix has to be computed and maybe returned

	//with given distance matrix
	std::vector<int> solveConcordeTSP(const cv::Mat& path_length_Matrix, const int start_Node);

	// compute distance matrix and maybe return it
	// this version does not exclude infinite paths from the TSP ordering
	std::vector<int> solveConcordeTSP(const cv::Mat& original_map, const std::vector<cv::Point>& points, double downsampling_factor,
			double robot_radius, double map_resolution, const int start_Node, cv::Mat* distance_matrix = 0);

	// compute TSP from a cleaned distance matrix (does not contain any infinity paths) that has to be computed
	std::vector<int> solveConcordeTSPClean(const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, const int start_node);

	// compute TSP with pre-computed cleaned distance matrix (does not contain any infinity paths)
	std::vector<int> solveConcordeTSPWithCleanedDistanceMatrix(const cv::Mat& distance_matrix,
			const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node);
};

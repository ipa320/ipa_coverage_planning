#include "ros/ros.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ipa_building_navigation/contains.h>

//This class provides a solution for the TSP by taking the nearest neighbor from the current Point as next Point.
//It need a symmetrical matrix of pathlenghts between the nodes and the starting-point index in this matrix.
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
public:
	//constructor
	NearestNeighborTSPSolver();

	//Solving-algorithm for the given TSP. It returns a vector of int, which is the order from this solution. The int shows
	//the index in the Matrix.
	std::vector<int> solveNearestTSP(const cv::Mat& path_length_Matrix, const int start_node);
};

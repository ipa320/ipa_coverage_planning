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
//It gets a Matrix of pathlenghts between the Points and the starting-point index in this Matrix.
class NearestNeighborTSPSolver
{
//protected:


public:
	//constructor
	NearestNeighborTSPSolver();

	//Solving-algorithm for the given TSP. It returns a vector of int, which is the order from this solution. The int shows
	//the index in the Matrix.
	std::vector<int> solveNearestTSP(const cv::Mat& path_length_Matrix, const int start_node);
};

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
#include <ipa_building_navigation/nearest_neighbor_TSP.h>

//This class provides a solution for the TSP by taking the nearest-neighbor path and applying a genetic algorithm on it.
//It gets a Matrix of pathlenghts between the Points and the starting-point index in this Matrix.
class GeneticTSPSolver
{
protected:
	//function to get the length of a given path
	double getPathLength(const cv::Mat& path_length_Matrix, std::vector<int> given_path);

	//function to mutate (randomly change) a given Parent-path
	std::vector<int> mutatePath(const std::vector<int>& parent_path);

	//function that selects the best path from the given paths
	std::vector<int> getBestPath(const std::vector<std::vector<int> > paths, const cv::Mat& pathlength_Matrix, bool& changed);

public:
	//constructor
	GeneticTSPSolver();

	//Solving-algorithm for the given TSP. It returns a vector of int, which is the order from this solution. The int shows
	//the index in the Matrix.
	std::vector<int> solveGeneticTSP(const cv::Mat& path_length_Matrix, const int start_Node);
};

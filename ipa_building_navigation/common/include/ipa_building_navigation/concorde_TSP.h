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

//This class applies an object to solve a given TSP problem using the concorde TSP solver. This solver can be downloaded at:
//		http://www.math.uwaterloo.ca/tsp/concorde.html
//A short explanation on how to build the solver is given at:
//		http://www.math.uwaterloo.ca/tsp/concorde/DOC/README.html
//If you have build the solver navigate to the./TSP folder and type " ./concorde -h " to see how to use this solver. This class
//uses the concorde solver by using a systemcall.
//Make sure you have a "/common/files/TSP_order.txt" and a "/common/files/TSPlib_file.txt" file. These files are used to tell
//concorde the current problem and save the output of it.

class concordeTSPSolver
{
protected:
	//Function to create neccessary TSPlib file to tell concorde what the problem is.
	void writeToFile(const cv::Mat& pathlength_matrix);

	//Function to read the saved TSP order.
	std::vector<int> readFromFile();

public:
	//Constructor
	concordeTSPSolver();

	//Function to solve the TSP. It needs a distance matrix, that shows the pathlengths between two nodes of the problem. This
	//matrix has to be symmetrical or else the TSPlib must be changed.
	std::vector<int> solveConcordeTSP(const cv::Mat& path_length_Matrix, const int start_Node);
};

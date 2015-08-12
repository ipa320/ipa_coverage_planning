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

class concordeTSPSolver
{
protected:
	void writeToFile(const cv::Mat& pathlength_matrix);

	std::vector<int> readFromFile();
public:
	concordeTSPSolver();

	std::vector<int> solveConcordeTSP(const cv::Mat& path_length_Matrix, const int start_Node);
};

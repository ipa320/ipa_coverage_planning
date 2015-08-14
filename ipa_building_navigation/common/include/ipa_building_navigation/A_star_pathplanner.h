// Astar.cpp
// http://en.wikipedia.org/wiki/A*
// Compiler: Dev-C++ 4.9.9.2
// FB - 201012256
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

#include <ipa_building_navigation/node.h>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
			 //regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This class provides an AStar pathplanner, which calculates the pathlength from one cv::Point to another. It was taken from
// http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/ and slightly changed (to use it with openCV).
//It needs node.h to work, which gives an implementation of nodes of the graph.
//
//!!!!!!!!!Important!!!!!!!!!!!!!
//It downsamples the map mith the given factor (0 < factor < 1) so the map gets reduced and calculationtime gets better.
//If it is set to 1 the map will have original size, if it is 0 the algorithm won't work, so make sure to not set it to 0.
//The algorithm also needs the Robot radius [m] and the map resolution [m²/pixel] to calculate the needed
//amount of erosions to include the radius in the planning.
//

class AStarPlanner
{
protected:
	int n;
	int m;

	std::string pathFind(const int& xStart, const int& yStart, const int& xFinish, const int& yFinish, const cv::Mat& map);

public:
	AStarPlanner();

	double planPath(const cv::Mat& map, const cv::Point& start_point, const cv::Point& end_point,
			const double downsampling_factor, const double robot_radius, const double map_resolution);

	void downsampleMap(const cv::Mat& map, cv::Mat& downsampled_map, const double downsampling_factor, const double robot_radius, const double map_resolution);
};

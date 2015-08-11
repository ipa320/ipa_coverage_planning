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

//This class provides an AStar pathplanner, which calculates the pathlength from one cv::Point to another. It was taken from
// http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/ and slightly changed (to use it with openCV).
//
//!!!!!!!!!Important!!!!!!!!!!!!!
//It downsamples the map mith the given factor (0 < factor < 1) so the map gets reduced and calculationtime gets better.
//If it is set to 1 the map will have original size, if it is 0 the algorithm won't work, so make sure to not set it to 0.
//

class AStarPlanner
{
protected:
	int n;
	int m;

	std::string pathFind(const int& xStart, const int& yStart, const int& xFinish, const int& yFinish, const cv::Mat& map_from_subscription);

public:
	AStarPlanner();

	double PlanPath(const cv::Mat& map_from_subscription, cv::Point& start_point, cv::Point& end_point, double downsampling_factor);
};

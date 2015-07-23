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

class AStarPlanner
{
protected:
	int n;
	int m;

	std::string pathFind(const int& xStart, const int& yStart, const int& xFinish, const int& yFinish, const cv::Mat& map_from_subscription);

public:
	AStarPlanner();

	double PlanPath(const cv::Mat& map_from_subscription, cv::Point& start_point, cv::Point& end_point);
};

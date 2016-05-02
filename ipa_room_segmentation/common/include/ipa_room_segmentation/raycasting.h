#pragma once

#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#define PI 3.14159265

class LaserScannerRaycasting
{
public:
	LaserScannerRaycasting();

	//raycasting function using the simple method that tracks a ray until its end
	void raycasting(const cv::Mat& map, const cv::Point& location, std::vector<double>& distances);

	//raycasting function based on the bresenham algorithm
	void bresenham_raycasting(const cv::Mat& map, const cv::Point& location, std::vector<double>& distances);

private:

	std::vector<double> precomputed_cos_;
	std::vector<double> precomputed_sin_;
};

std::vector<double> raycasting(const cv::Mat& map, const cv::Point& location);

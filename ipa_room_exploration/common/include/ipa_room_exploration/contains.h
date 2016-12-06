#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#pragma once

//This function searches for the fiven element in the given vector and returns true if it is in it or false if not. Here for
//int elements.
bool contains(std::vector<int> vector, int element);

bool contains(std::vector<cv::Point> vector, cv::Point element);

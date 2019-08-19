#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <set>
#include <math.h>

bool contains(std::vector<cv::Scalar> vector, cv::Scalar element);
bool contains(std::vector<cv::Point> vector, cv::Point element);
bool contains(std::vector<int> vector, int element);
bool contains(std::vector<std::vector<unsigned int> > vector, std::vector<unsigned int> element);

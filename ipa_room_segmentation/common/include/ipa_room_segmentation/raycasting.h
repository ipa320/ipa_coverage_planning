#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#define PI 3.14159265

std::vector<double> raycasting(cv::Mat map, cv::Point location);

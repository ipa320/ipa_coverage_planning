#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#define PI 3.14159265

//raycasting function using the simple method that tracks a ray until its end
std::vector<double> raycasting(const cv::Mat& map, const cv::Point& location);

//raycasting function based on the bresenham algorithm
std::vector<double> bresenham_raycasting(const cv::Mat& map, const cv::Point& location);

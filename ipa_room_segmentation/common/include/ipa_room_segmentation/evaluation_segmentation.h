
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ctime>

class EvaluationSegmentation
{
public:
	std::vector<cv::Mat> gt_calculation(cv::Mat&);
	std::vector<cv::Mat> calculate_white_pixels(cv::Mat& map);
};

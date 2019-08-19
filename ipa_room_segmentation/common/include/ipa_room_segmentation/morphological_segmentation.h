#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>


class MorphologicalSegmentation
{
public:
	MorphologicalSegmentation();
	//algorithm to segment the map
	void segmentMap(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription, double room_area_factor_lower_limit, double room_area_factor_upper_limit);
};

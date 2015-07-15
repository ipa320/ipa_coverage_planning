#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ipa_room_segmentation/watershed_region_spreading.h>
#include <ipa_room_segmentation/contains.h>

#include <ctime>

class distance_segmentation
{
private:
	//variables for calculating-purpose
	double map_resolution_from_subscription_;
	double room_area_factor_lower_limit_;
	double room_area_factor_upper_limit_;
	//variable for the distance-transformed map
	cv::Mat distance_map_;
	//saving-vector for the found contours
	std::vector < std::vector<cv::Point> > saved_contours;
	//saving-variable for already used fill-colours
	std::vector<cv::Scalar> already_used_coloures_;
	//algorithm to segment the map
	void segmentationAlgorithm(cv::Mat map_to_be_labeled);
public:
	distance_segmentation(cv::Mat original_map_from_subscription, double map_resolution_from_subscription, double room_area_factor_lower_limit, double room_area_factor_upper_limit);
};

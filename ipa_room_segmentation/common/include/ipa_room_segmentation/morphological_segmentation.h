#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#include <ipa_room_segmentation/watershed_region_spreading.h>
#include <ipa_room_segmentation/contains.h>

class morphological_segmentation
{
private:
	//saving variable for every contour that is between the upper and the lower limit
	std::vector < std::vector<cv::Point> > saved_contours;
	//map to find the rooms and eroding
	cv::Mat temporary_map_to_find_rooms_;
	//map for drawing the found contours
	cv::Mat new_map_to_draw_contours_;
	//map to spread the colored Pixels to the surrounding white regions
	cv::Mat temporary_map_to_fill_white_pixels;
	//map to find the colored rooms and save the min/max x/y value of them
	cv::Mat temporary_map_to_find_room_values_;
	//vector for saving the already used coloures
	std::vector<cv::Scalar> already_used_coloures_;
public:
	//variables for calculating purpose
	double map_resolution_from_subscription_;
	double room_area_factor_lower_limit_;
	double room_area_factor_upper_limit_;
	morphological_segmentation();
	//algorithm to segment the map
	cv::Mat segmentationAlgorithm(cv::Mat map_to_be_labeled);
	void clear_all_vectors();
	int get_size_color();
};

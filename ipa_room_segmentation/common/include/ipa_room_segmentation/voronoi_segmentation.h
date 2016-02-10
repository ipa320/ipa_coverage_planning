#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

#include <ipa_room_segmentation/room_class.h>

#define PI 3.14159265

class VoronoiSegmentation
{
private:

	bool determineRoomIndexFromRoomID(const std::vector<Room>& rooms, const int room_id, size_t& room_index);

	void mergeRoomPair(std::vector<Room>& rooms, const int target_index, const int room_to_merge_index, cv::Mat& segmented_map, const double map_resolution);

	//function to draw the generalized voronoi-diagram into a given map, not drawing lines that start or end at black Pixels
	void drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color,
			//const std::vector<cv::Point>& contour, const std::vector<std::vector<cv::Point> >& hole_contours);
			const cv::Mat& eroded_map);

	//function to get the voronoi-diagram drawn into the map
	void createVoronoiGraph(cv::Mat& map_for_voronoi_generation);

	//function to merge rooms together
	void mergeRooms(cv::Mat& map_to_merge_rooms, std::vector<Room>& rooms, double map_resolution_from_subscription, double max_area_for_merging);

public:

	VoronoiSegmentation();

	//the segmentation-algorithm
	void segmentationAlgorithm(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
			double room_area_factor_lower_limit, double room_area_factor_upper_limit, int neighborhood_index, int max_iterations,
			double min_critical_point_distance_factor, double max_area_for_merging, bool display_map=false);
};

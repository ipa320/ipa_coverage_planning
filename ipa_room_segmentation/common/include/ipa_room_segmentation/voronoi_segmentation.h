#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

#define PI 3.14159265

#include <ipa_room_segmentation/watershed_region_spreading.h>
#include <ipa_room_segmentation/contains.h>

class voronoi_segmentation
{
private:
	//variable to save the given map for drawing in the voronoi-diagram
	cv::Mat map_to_draw_voronoi_in_;
	//variables to find the facets and centers of the voronoi-cells
	std::vector<std::vector<cv::Point2f> > voronoi_facets_;
	std::vector<cv::Point2f> voronoi_centers_;
	//variables to save the hole-contours
	std::vector<std::vector<cv::Point> > hole_contours_;
	//variable to save the largest contour of the map --> the contour of the map itself
	std::vector<cv::Point> largest_contour_;
	//variable to save the given map in the createVoronoiGraph-function
	cv::Mat temporary_map_to_calculate_voronoi_;
	//voronoi-map for the segmentation-algotihm
	cv::Mat voronoi_map_;
	//variables for Nodepoint-extraction
	std::vector<cv::Point> node_Points_;
	//map to draw the critical lines and fill the map with random coloures
	cv::Mat temporary_map_to_draw_critical_lines_and_colouring_;
	//distance-map of the original-map (used to check the distance of each Point to nearest black Pixel)
	cv::Mat distance_map_;
	//saving-variable for the critical Points found on the voronoi-graph
	std::vector<cv::Point> critical_Points_;
	//the angles between the basis-lines of each critical Point
	std::vector<double> angles_;
	//saving-vector to save the already used coloures
	std::vector<cv::Scalar> already_used_coloures_;
	//function to draw the generalized voronoi-diagram into a given map, not drawing lines that start or end at black Pixels
	void drawVoronoi(cv::Mat &img, std::vector<std::vector<cv::Point2f> > facets_of_voronoi, cv::Scalar voronoi_color, std::vector<cv::Point> contour,
	        std::vector<std::vector<cv::Point> > hole_contours);
	//function to get the voronoi-diagram drawn into the map
	cv::Mat createVoronoiGraph(cv::Mat map_for_voronoi_generation);

public:
	//variables for calculating-purpose
	double map_resolution_from_subscription_;
	double room_area_factor_lower_limit_;
	double room_area_factor_upper_limit_;
	voronoi_segmentation();
	//the segmentation-algorithm
	cv::Mat segmentationAlgorithm(cv::Mat map_to_be_labeled);
	void clear_all_vectors();
};

#pragma once

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

#include <ipa_room_segmentation/room_class.h>

#define PI 3.14159265

// Struct that compares two given points and returns if the y-coordinate of the first is smaller or if they are equal if the
// x-coordinate of the first is smaller. This is used for sets to easily store cv::Point objects and search for specific objects.
struct cv_Point_comp
{
	bool operator()(const cv::Point& lhs, const cv::Point& rhs) const
	{
		return ((lhs.y < rhs.y) || (lhs.y == rhs.y && lhs.x < rhs.x));
	}
};

class AbstractVoronoiSegmentation
{
protected:

	// Function to get the ID of a room, when given an index in the storing vector.
	// This function takes a vector of rooms and an index in this vector and returns the ID of this room, meaning the color it has
	// been drawn in the map.
	bool determineRoomIndexFromRoomID(const std::vector<Room>& rooms, const int room_id, size_t& room_index);

	// Function to merge two rooms together on the given already segmented map.
	// This function is used to merge two rooms together in the given already segmented map. It takes two indexes, showing which
	// room is the bigger one and which is the smaller one. The smaller one should be merged together with the bigger one, what is
	// done by changing the color of this room in the map to the color of the bigger room and inserting the members into the bigger
	// room.
	void mergeRoomPair(std::vector<Room>& rooms, const int target_index, const int room_to_merge_index, cv::Mat& segmented_map, const double map_resolution);

	// Function to draw the generalized voronoi-diagram into a given map, not drawing lines that start or end at black pixels
	// This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
	// map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
	// inside a hole-contour and doesn't draw the lines that are not.
	// Function to draw the approximated voronoi graph into a given map. It doesn't draw lines of the graph that start or end
	// in a black region. This is necessary because the voronoi graph gets approximated by diskretizing the maps contour and
	// using these points as centers for the graph. It gets wrong lines, that are eliminated in this function. See the .cpp
	// files for further information.
	void drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color, const cv::Mat& eroded_map);

	// Function to get the voronoi-diagram drawn into the map
	// This function is here to create the generalized voronoi-graph in the given map. It does following steps:
	//	1. It finds every discretized contour in the given map (they are saved as vector<Point>). Then it takes these
	//	   contour-Points and adds them to the OpenCV Delaunay generator from which the voronoi-cells can be generated.
	//	2. Finally it gets the boundary-Points of the voronoi-cells with getVoronoiFacetList. It takes these facets
	//	   and draws them using the drawVoronoi function. This function draws the facets that only have Points inside
	//	   the map-contour (other lines go to not-reachable places and are not necessary to be looked at).
	//	3. It returns the map that has the generalized voronoi-graph drawn in.
	void createVoronoiGraph(cv::Mat& map_for_voronoi_generation);

	// This function prunes the generalized Voronoi-graph in the given map.
	// It reduces the graph down to the nodes in the graph. A node is a point on the Voronoi graph, that has at least 3
	// neighbors. This deletes errors from the approximate generation of the graph that hasn't been eliminated from
	// the drawVoronoi function. the resulting graph is the pruned generalized voronoi graph.
	// It does following steps:
	//   1. Extract node-points of the Voronoi-Diagram, which have at least 3 neighbors.
	//   2. Reduce the leave-nodes (Point on graph with only one neighbor) of the graph until the reduction
	//      hits a node-Point. This is done to reduce the lines along the real voronoi-graph, coming from the discretisation
	//      of the contour.
	//   3. It returns the map that has the pruned generalized voronoi-graph drawn in.
	void pruneVoronoiGraph(cv::Mat& voronoi_map, std::set<cv::Point, cv_Point_comp>& node_points);

	// Function to merge rooms together
	// Function that goes trough each given room and checks if it should be merged together wit another bigger room, if it is too small.
	// This function takes the segmented Map from the original Voronoi-segmentation-algorithm and merges rooms together,
	// that are small enough and have only two or one neighbor.
	void mergeRooms(cv::Mat& map_to_merge_rooms, std::vector<Room>& rooms, double map_resolution_from_subscription, double max_area_for_merging, bool display_map);

public:

	AbstractVoronoiSegmentation();
};

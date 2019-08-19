#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <ipa_building_navigation/contains.h>

#include <ipa_building_navigation/A_star_pathplanner.h>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
			 //regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This object provides an algorithm to search for the best trolley position for a group of points. A trolley position is
//the Point where the trolley of a robot should be placed during the cleaning of the group. This is done by searching in the
//bounding box of these points for the point which minimizes the pathlength to every group member. If the goup has only two
//members the algorithm chooses a Point on the optimal path between these two Points that is in the middlest of this path.
//This algorithm needs as input:
//		1. The original occupancy gridmap to get the pathlength between two Points.
//		2. A vector of found groups. This vector stores the group as integer that show the Position of the node in the
//		   roomcenters vector. To find the groups it is good to find all maximal cliques in the graph and then applying
//		   a set Cover solver on these.
//		3. A vector of roomcenters that stores the centers of each room. This algorithm was implemented for planning
//		   the order of cleaning rooms so this variable is called room_centers, but it can store every cv::Point with that
//		   you want to find the best trolley position.
//		4. A downsampling factor to reduce the size of the map. This is used by the A_star_pathplanner to heavily reduce
//		   calculationtime. It has to be (0, 1]. If it is 1 the map will be took as it is.
//		5. The Radius of the robot and the map resolution to make sure the A_star pathplanner stays in enough distance to the
//		   walls and obstacles. (See A_star_pathplanner.cpp for further information)

class TrolleyPositionFinder
{
protected:

	AStarPlanner path_planner_; //Object to plan a path from Point A to Point B in a given gridmap

	//Function to find a trolley position for one group
	cv::Point findOneTrolleyPosition(const std::vector<cv::Point> group_points, const cv::Mat& original_map,
			const double downsampling_factor, const double robot_radius, const double map_resolution);

public:

	//constructor
	TrolleyPositionFinder();

	//Function to find a trolley position for each group by using the findOneTrolleyPosition function
	std::vector<cv::Point> findTrolleyPositions(const cv::Mat& original_map, const std::vector<std::vector<int> >& found_groups,
			const std::vector<cv::Point>& room_centers, const double downsampling_factor, const double robot_radius,
			const double map_resolution);
};

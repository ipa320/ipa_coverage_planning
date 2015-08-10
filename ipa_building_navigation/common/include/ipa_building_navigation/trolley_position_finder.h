#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

#include <ipa_building_navigation/contains.h>

#include <ipa_building_navigation/A_star_pathplanner.h>

class trolleyPositionFinder
{
protected:

	AStarPlanner path_planner_; //Object to plan a path from Point A to Point B in a given gridmap

	cv::Point findOneTrolleyPosition(const std::vector<cv::Point> group_points, const cv::Mat& original_map);

public:
	trolleyPositionFinder();

	std::vector<cv::Point> findTrolleyPositions(const cv::Mat& original_map, const std::vector<std::vector<int> >& found_groups,
			const std::vector<cv::Point>& room_centers);
};

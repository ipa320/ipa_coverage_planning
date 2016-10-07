#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include <ipa_room_exploration/concorde_TSP.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

//#include <ipa_building_navigation/concorde_TSP.h>

// Class that generates a room exploration path by laying a small grid over the given map and then planning the best path trough
// all points, by defining an traveling salesman problem (TSP). This class only produces a static path, regarding the given map
// in form of a point series. To react on dynamic obstacles, one has to do this in upper algorithms.
//
class gridPointExplorator
{
protected:
//	 object that solves the TSP, when the grid has been computed
//	ConcordeTSPSolver tsp_solver_;

	// length of the grid cell lines [pixel]
	int grid_line_length_;

public:
	// constructor
	gridPointExplorator(int grid_line_length = 5);

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
			const float map_resolution, const geometry_msgs::Pose2D starting_position,
			const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin);

	// function to set the grid size
	void setGridLineLength(int new_line_length)
	{
		grid_line_length_ = new_line_length;
	}
};

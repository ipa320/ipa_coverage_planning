// Ros
#include <ros/ros.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// Eigen library
#include <Eigen/Dense>
// c++ standard libraries
#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
// services
#include <ipa_building_msgs/CheckCoverage.h>

// Class that provides a service server to check which areas have been covered, when going along the given poses. It returns an image that
// has all covered areas drawn in as 127.
// REMARK: The given map has to be a 8 bit single channel image, with 0 as obstacle and 255 as free space drawn in.
// If wanted, the server can check how often a pixel has been covered during the execution. When this is wished, an image is returned, in
// which the number of observations (or coverages) is assigned to each pixel (32 bit image in this case).
class CoverageCheckServer
{
protected:
	// node handle
	ros::NodeHandle node_handle_;

	// Function to draw the covered areas into the given map. This is done by going through all given robot-poses and calculating
	// the field of view. The field of view is given in robot base coordinates (x-axis shows to the front and y-axis to left side).
	// The function then calculates the field_of_view in the global frame by using the given robot pose.
	// After this the function does a ray casting from the field of view origin (i.e. the camera location expressed in the robot base coordinate system)
	// to each cell of the field of view in order to verify whether the cell has been obstructed by an obstacle and could not be observed.
	// This ensures that no point is wrongly classified as seen.
	// @param fov_origin The mounting position of the camera spanning the field of view, given in robot base coordinates, in [m]
	void drawCoveredPointsPolygon(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
			const std::vector<Eigen::Matrix<float, 2, 1>>& field_of_view_points, const Eigen::Matrix<float, 2, 1>& fov_origin,
			const float map_resolution, const cv::Point2d map_origin, cv::Mat* number_of_coverages_image=NULL);

	// Function that takes the given robot poses and draws the circular footprint with coverage_radius at these positions into the given map.
	// Used when the server should plan a coverage path for the robot coverage area (a circle). This drawing function does not test occlusions
	// since a footprint can usually be assumed to reach all covered map positions (otherwise it would be a collision).
	void drawCoveredPointsCircle(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
				const double coverage_radius, const float map_resolution,
				const cv::Point2d map_origin, cv::Mat* number_of_coverages_image=NULL);

	// reduces image coordinates of a point to valid values inside the image, i.e. between [0,rows-1]/[0,cols-1]
	cv::Point clampImageCoordinates(const cv::Point& p, const int rows, const int cols);

	// ros server object
	ros::ServiceServer coverage_check_server_;
public:
	// constructor
	CoverageCheckServer();
	CoverageCheckServer(ros::NodeHandle nh);

	// callback function for the server
	bool checkCoverage(ipa_building_msgs::CheckCoverageRequest& request, ipa_building_msgs::CheckCoverageResponse& response);

	// ROS-independent coverage check library interface
	bool checkCoverage(const cv::Mat& map, const float map_resolution, const cv::Point2d& map_origin, const std::vector<cv::Point3d>& path,
			const std::vector<Eigen::Matrix<float, 2, 1> >& field_of_view, const Eigen::Matrix<float, 2, 1>& fov_origin, const float coverage_radius,
			const bool check_for_footprint, const bool check_number_of_coverages, cv::Mat& coverage_map, cv::Mat& number_of_coverage_image);

};

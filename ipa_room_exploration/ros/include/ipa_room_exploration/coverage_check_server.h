// Ros
#include <ros/ros.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
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
//	REMARK: The given map has to be a 8Bit single channel image, with 0 as obstacle and 255 as free sapce drawn in.
// If wanted, the server can check how often a pixel has been covered during the execution. When this is wished, an image is returned, in
// which the number of coverages is assigned to each pixel (32bit image in this case).
class coverageCheckServer
{
protected:
	// node handle
	ros::NodeHandle node_handle_;

	// function to draw the covered areas into the given map, when checking for the field of view, done by doing a raycasting between the
	// fov origin and the fov
	void drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
			const std::vector<geometry_msgs::Point32>& field_of_view_points, const Eigen::Matrix<float, 2, 1> raycasting_corner_1,
			const Eigen::Matrix<float, 2, 1> raycasting_corner_2, const float map_resolution, const cv::Point2d map_origin,
			cv::Mat* number_of_coverages_image=NULL);

	// function to draw the covered ares into the given map, when checking for the robot footprint, done by simply drawing the footprint
	// at the given poses into the map
	void drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
				const double coverage_radius, const float map_resolution,
				const cv::Point2d map_origin, cv::Mat* number_of_coverages_image=NULL);

	// ros server object
	ros::ServiceServer coverage_check_server_;
public:
	// constructor
	coverageCheckServer(ros::NodeHandle nh);

	// callback function for the server
	bool checkCoverage(ipa_building_msgs::CheckCoverageRequest& request, ipa_building_msgs::CheckCoverageResponse& response);
};

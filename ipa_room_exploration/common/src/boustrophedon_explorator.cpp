#include <ipa_room_exploration/boustrophedon_explorator.h>

// Constructor
boustrophedonExplorer::boustrophedonExplorer()
{

}

// Function to resample the given map. This produces generalized Polygons that represent the obstacles in the map, by
// setting the vertexes as a set of points and the edges as a set of vectors, in a counter-clockwise manner. To do so
// the following steps are done:
//	1. 	Find the contours that represent the obstacles, by using OpenCV.
//	2.	Use the previously found sets of points to find the vertexes that represent each obstacle. This is done by taking
//		a start point and searching in the counter-clockwise direction for a point that is at leas min_point_dist away
//		from the previous point. This creates a sampled obstacle, which is easy to represent as a generalized Polygon.
//	3.	Create the generalized Polygons and store them into the given vector. Start with the Polygon that has the smallest
//		minimal x-value and go upward (Convention in the given paper, to easily compute the cells later).
void boustrophedonExplorer::resampleMap(const cv::Mat& original_map, std::vector<generalizedPolygon>& obstacles,
		const float min_point_dist, cv::Mat* resampled_map)
{
	// *********** 1. Find the contours of the obstacles in the given map. ***********
	std::vector < std::vector<cv::Point> > contours;
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;
	cv::Mat contour_map = original_map.clone();
	// invert the binary image, such that obstacles are represented as white contours
	cv::threshold (contour_map, contour_map, 70, 255, CV_THRESH_BINARY_INV);
	cv::findContours(contour_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// testing
	cv::Mat test_map = cv::Mat(original_map.cols, original_map.rows, original_map.type(), cv::Scalar(255));
	for(size_t i = 0; i < contours.size(); ++i)
	{
		cv::drawContours(test_map, contours, i, cv::Scalar(0), CV_FILLED);
		cv::imshow("test", test_map);
		cv::waitKey();
	}
}


// Function that creates a room exploration path for the given map, by using the cellular decomposition method proposed in
// "Choset, H., & Pignon, P. (1998). Coverage path planning: The boustrophedon cellular decomposition. In Field and Service Robotics (pp. 203-209). Springer London."
// This method takes the given map and separates it into several cells. Each cell is obstacle free and so allows an
// easier path planning. For each cell then a boustrophedon path is planned, which goes up, down and parallel to the
// upper and lower boundaries of the cell, see the referenced paper for details. This function does the following steps:
//	I.	Resample the given map, such that each obstacle can be represented in a certain way. The vertexes are stored as a set
//		of points and the edges as a set of vectors, in a counter-clockwise manner. This needs to be done to simplify the
//		computation of the cells, because it allows to use the computed points and vectors as vertexes and edges for these.
//		To do this the upper function resampleMap() is used, see it for further references.
void boustrophedonExplorer::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
		const float map_resolution, const geometry_msgs::Pose2D starting_position,
		const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin,
		const float fow_fitting_circle_radius, const float min_resample_dist)
{
	// *********************** I. Resample the given map. ***********************
	std::vector<generalizedPolygon> obstacle_polygons;
	resampleMap(room_map, obstacle_polygons, min_resample_dist, 0);
}

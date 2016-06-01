#include <ipa_room_exploration/grid_point_explorator.h>

// Constructor
gridPointExplorator::gridPointExplorator(int grid_line_length)
{
	grid_line_length_ = grid_line_length;
}

// Function to create a static pose series that has the goal to inspect the complete floor of the given room.
// This is done in the following steps:
//		I. It lays a grid over the given map, with a line_size defined by the constructor/set-function. All intersection points
//		   that are not laying on an obstacle, or near to one, become one point of the path.
//		II. It plans an optimal series of the previously found points by solving a Traveling-Salesman-Problem (TSP). This produces
//			a path that covers all nodes and ends at the node where the path started. depending on this series the angle of the
//			Poses are computed, by calculating a vector from the old node to the next and using the angle of this with the x-axis
//			as angle for the Poses.
void gridPointExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
		const geometry_msgs::Pose2D starting_position)
{
	//******************* I. Get grid points *************************
	// vector to store all found points
	std::vector<cv::Point> grid_points;

	// erode map so points that are too near to obstacels don't get chosen
	cv::Mat eroded_map;
	cv::erode(room_map, eroded_map, cv::Mat(), cv::Point(-1, -1), robot_radius);

	// iterate trough the columns and rows with stepsize as the grid_size
	std::cout << "size of one grid line: " << grid_line_length_ << std::endl;
	for(unsigned int u = 0; u < eroded_map.rows; u += grid_line_length_)
	{
		for(unsigned int v = 0; v < eroded_map.cols; v += grid_line_length_)
		{
			// check if point is in the free space
			if(eroded_map.at<unsigned char>(u, v) == 255)
			{
				grid_points.push_back(cv::Point(v, u));
			}
		}
	}

	// print results
	cv::Mat point_map = room_map.clone();
	for(std::vector<cv::Point>::iterator point = grid_points.begin(); point != grid_points.end(); ++point)
		cv::circle(point_map, *point, 2, cv::Scalar(127), CV_FILLED);

	cv::imshow("grid", point_map);
	cv::waitKey();
}

#include <ipa_room_exploration/grid_point_explorator.h>

// Constructor
gridPointExplorator::gridPointExplorator(int grid_line_length)
{
	grid_line_length_ = grid_line_length;
}

// Function to create a static pose series that has the goal to inspect the complete floor of the given room.
// This is done in the following steps:
//		I. It lays a grid over the given map, with a line_size defined by the constructor/set-function. All intersection points
//		   that are not laying on an obstacle, or near to one, become one point of the path. The grid starts at the upper left
//		   point that is reachable.
//		II. It plans an optimal series of the previously found points by solving a Traveling-Salesman-Problem (TSP). This produces
//			a path that covers all nodes and ends at the node where the path started. depending on this series the angle of the
//			Poses are computed, by calculating a vector from the old node to the next and using the angle of this with the x-axis
//			as angle for the Poses.
void gridPointExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float robot_radius, const float map_resolution, const geometry_msgs::Pose2D starting_position,
		const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin)
{
	//******************* I. Get grid points *************************
	// vector to store all found points
	std::vector<cv::Point> grid_points;

	// erode map so points that are too near to obstacels don't get chosen
//	cv::Mat eroded_map;
//	int number_of_erosions = (robot_radius / map_resolution);
//	cv::erode(room_map, eroded_map, cv::Mat(), cv::Point(-1, -1), number_of_erosions);

	// iterate trough the columns and rows with stepsize as the grid_size, start at the upper left point
	//		the given min/max-Polygon stores the min/max coordinates in two points: the first showing the min and the other
	//		showing the max coordinates
	std::cout << "size of one grid line: " << grid_line_length_ << std::endl;
	for(unsigned int u = room_min_max_coordinates.points[0].y; u < room_min_max_coordinates.points[1].y; u += grid_line_length_)
	{
		for(unsigned int v = room_min_max_coordinates.points[0].x; v < room_min_max_coordinates.points[1].x; v += grid_line_length_)
		{
			// check if point is in the free space
			if(room_map.at<unsigned char>(u, v) == 255)
			{
				grid_points.push_back(cv::Point(v, u));
			}
		}
	}

	// print results
	cv::Mat point_map = room_map.clone();
	for(std::vector<cv::Point>::iterator point = grid_points.begin(); point != grid_points.end(); ++point)
		cv::circle(point_map, *point, 2, cv::Scalar(127), CV_FILLED);


	//******************* II. Plan a path trough the found points *************************
	// find the index of the point, which is closest to the starting position
	int min_index = 0;
	double min_distance = 1e7;
	cv::Point starting_point(starting_position.x, starting_position.y); // conversion of Pose2D to cv::Point for convenience

	for(std::vector<cv::Point>::iterator point = grid_points.begin(); point != grid_points.end(); ++point)
	{
		double distance = cv::norm(starting_point - *point);

		if(distance <= min_distance)
		{
			min_distance = distance;
			min_index = point - grid_points.begin();
		}
	}


	// solve the Traveling Salesman Problem
	std::cout << "Finding optimal order of the found points. Start-index: " << min_index << std::endl;
	ConcordeTSPSolver tsp_solver;
//	double map_downsampling_factor = 0.25;
	std::vector<int> optimal_order = tsp_solver.solveConcordeTSP(room_map, grid_points, 0.25, 0.0, map_resolution, min_index, 0);

	// resave the found points in the optimal order and convert them to the right format
	for(unsigned int point_index = 0; point_index < optimal_order.size(); ++point_index)
	{
		// get the vector from the current point to the next point
		cv::Point current_point = grid_points[optimal_order[point_index]];
		cv::Point next_point = grid_points[optimal_order[(point_index+1)%(optimal_order.size())]];
		cv::Point vector = next_point - current_point;

		float quotient = vector.x / (sqrtf(vector.x * vector.x + vector.y * vector.y));

		float angle = std::acos(quotient);

		// correct angle if robot moves in negative y-direction
		if(vector.y < 0 && vector.x >= 0)
			angle -= 3.14159;

		// add the next navigation goal to the path
		geometry_msgs::Pose2D navigation_goal;
		navigation_goal.x = (current_point.x * map_resolution) + map_origin.x; // coordinate systems are rotated to each other
		navigation_goal.y = (current_point.y * map_resolution) + map_origin.y;
		navigation_goal.theta = angle;

		path.push_back(navigation_goal);

//		std::cout << "angle: " << angle << ", vector: " << vector << std::endl;

		cv::line(point_map, current_point, next_point, cv::Scalar(127), 1);
	}

//	cv::imshow("grid", point_map);
//	cv::waitKey();
}

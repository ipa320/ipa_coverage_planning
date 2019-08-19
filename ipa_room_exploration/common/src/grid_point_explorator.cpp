#include <ipa_room_exploration/grid_point_explorator.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <pthread.h>
#include <signal.h>

// Constructor
GridPointExplorator::GridPointExplorator()
{
}

void GridPointExplorator::tsp_solver_thread_concorde(ConcordeTSPSolver& tsp_solver, std::vector<int>& optimal_order,
		const cv::Mat& distance_matrix, const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node)
{
	try
	{
		optimal_order = tsp_solver.solveConcordeTSPWithCleanedDistanceMatrix(distance_matrix, cleaned_index_to_original_index_mapping, start_node);
	}
	catch (boost::thread_interrupted&)
	{
		std::cout << "GridPointExplorator::tsp_solver_thread_concorde: Thread with Concorde TSP solver was interrupted." << std::endl;
	}

	std::cout << "GridPointExplorator::tsp_solver_thread_concorde: finished TSP with solver 3=Concorde and optimal_order.size=" << optimal_order.size() << std::endl;
}

void GridPointExplorator::tsp_solver_thread_genetic(GeneticTSPSolver& tsp_solver, std::vector<int>& optimal_order,
		const cv::Mat& distance_matrix, const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node)
{
	try
	{
		optimal_order = tsp_solver.solveGeneticTSPWithCleanedDistanceMatrix(distance_matrix, cleaned_index_to_original_index_mapping, start_node);
	}
	catch (boost::thread_interrupted&)
	{
		std::cout << "GridPointExplorator::tsp_solver_thread_genetic: Thread with Genetic TSP solver was interrupted." << std::endl;
	}

	std::cout << "GridPointExplorator::tsp_solver_thread_genetic: finished TSP with solver 2=Genetic and optimal_order.size=" << optimal_order.size() << std::endl;
}

void GridPointExplorator::tsp_solver_thread(const int tsp_solver, std::vector<int>& optimal_order, const cv::Mat& original_map,
		const std::vector<cv::Point>& points, const double downsampling_factor, const double robot_radius, const double map_resolution,
		const int start_node)
{
	try
	{
		if (tsp_solver == TSP_NEAREST_NEIGHBOR)
		{
			NearestNeighborTSPSolver tsp_solve;
			optimal_order = tsp_solve.solveNearestTSP(original_map, points, downsampling_factor, robot_radius, map_resolution, start_node, 0);
		}
		else if (tsp_solver == TSP_GENETIC)
		{
			GeneticTSPSolver tsp_solve;
			optimal_order = tsp_solve.solveGeneticTSP(original_map, points, downsampling_factor, robot_radius, map_resolution, start_node, 0);
		}
		else if (tsp_solver == TSP_CONCORDE)
		{
			ConcordeTSPSolver tsp_solve;
			optimal_order = tsp_solve.solveConcordeTSP(original_map, points, downsampling_factor, robot_radius, map_resolution, start_node, 0);
		}
		else
		{
			std::cout << "GridPointExplorator::tsp_solver_thread: Error: tsp_solver " << tsp_solver << " is undefined." << std::endl;
		}
	}
	catch (boost::thread_interrupted&)
	{
		std::cout << "GridPointExplorator::tsp_solver_thread: Thread with TSP solver was interrupted." << std::endl;
	}

	std::cout << "GridPointExplorator::tsp_solver_thread: finished TSP with solver " << tsp_solver << " and optimal_order.size=" << optimal_order.size() << std::endl;
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
// room_map = expects to receive the original, not inflated room map
void GridPointExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const double map_resolution,
		const cv::Point starting_position, const cv::Point2d map_origin, const int cell_size, const bool plan_for_footprint,
		const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, int tsp_solver, int64_t tsp_solver_timeout)
{
	const int half_cell_size = cell_size/2;

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// rotate map
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

	// transform starting position
	std::vector<cv::Point> starting_point_vector(1, starting_position); // opencv syntax
	cv::transform(starting_point_vector, starting_point_vector, R);
	cv::Point rotated_starting_position = starting_point_vector[0];

	//******************* II. Get grid points *************************
	// vector to store all found points
	std::vector<cv::Point> grid_points;

	// compute the basic Boustrophedon grid lines
	cv::Mat inflated_rotated_room_map;
	BoustrophedonGrid grid_lines;
	GridGenerator::generateBoustrophedonGrid(rotated_room_map, inflated_rotated_room_map, half_cell_size, grid_lines, cv::Vec4i(-1, -1, -1, -1),
			cell_size, half_cell_size, cell_size-1);		// using cell_size-1 instead of cell_size for grid_spacing_horizontal helps
															// the TSP planner to avoid unnecessary rotations by following a preferred direction
	// convert grid points format
	for (BoustrophedonGrid::iterator line=grid_lines.begin(); line!=grid_lines.end(); ++line)
	{
		grid_points.insert(grid_points.end(), line->upper_line.begin(), line->upper_line.end());
		if (line->has_two_valid_lines == true)
			grid_points.insert(grid_points.end(), line->lower_line.begin(), line->lower_line.end());
	}

//	// print results
//	cv::Mat point_map = rotated_room_map.clone();
//	for(std::vector<cv::Point>::iterator point = grid_points.begin(); point != grid_points.end(); ++point)
//	{
//		cv::circle(point_map, *point, 2, cv::Scalar(127), CV_FILLED);
//		std::cout << "  - " << *point << "\n";
//	}
//	cv::imshow("grid", point_map);
//	cv::waitKey();

	//******************* II. Plan a path trough the found points *************************
	// find the index of the point, which is closest to the starting position
	int min_index = 0;
	double min_distance = 1e10;

	for(std::vector<cv::Point>::iterator point = grid_points.begin(); point != grid_points.end(); ++point)
	{
		double distance = cv::norm(rotated_starting_position - *point);

		if(distance < min_distance)
		{
			min_distance = distance;
			min_index = point - grid_points.begin();
		}
	}


	// solve the Traveling Salesman Problem
	std::cout << "Finding optimal order of the " << grid_points.size() << " found points. Start-index: " << min_index << std::endl;
	const double map_downsampling_factor = 0.25;
	// compute distance matrix for TSP (outside of time limits for solving TSP)
	cv::Mat distance_matrix_cleaned;
	std::map<int,int> cleaned_index_to_original_index_mapping;	// maps the indices of the cleaned distance_matrix to the original indices of the original distance_matrix
	AStarPlanner path_planner;
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.computeCleanedDistanceMatrix(rotated_room_map, grid_points, map_downsampling_factor, 0.0, map_resolution, path_planner,
			distance_matrix_cleaned, cleaned_index_to_original_index_mapping, min_index);

	// solve TSP
	bool finished = false;
	std::vector<int> optimal_order;
	if (tsp_solver == TSP_CONCORDE)
	{
		// start TSP solver in extra thread
		ConcordeTSPSolver tsp_solve;
		boost::thread t(boost::bind(&GridPointExplorator::tsp_solver_thread_concorde, this, boost::ref(tsp_solve), boost::ref(optimal_order),
				boost::cref(distance_matrix_cleaned), boost::cref(cleaned_index_to_original_index_mapping), min_index));
		if (tsp_solver_timeout > 0)
		{
			finished = t.try_join_for(boost::chrono::seconds(tsp_solver_timeout));
			if (finished == false)
			{
				tsp_solve.abortComputation();
				std::cout << "GridPointExplorator::getExplorationPath: INFO: Terminated tsp_solver " << tsp_solver << " because of time out. Taking the Nearest Neighbor TSP instead." << std::endl;
			}
		}
		else
			finished = true;
		t.join();
	}
	else if (tsp_solver == TSP_GENETIC)
	{
		// start TSP solver in extra thread
		GeneticTSPSolver tsp_solve;
		boost::thread t(boost::bind(&GridPointExplorator::tsp_solver_thread_genetic, this, boost::ref(tsp_solve), boost::ref(optimal_order),
				boost::cref(distance_matrix_cleaned), boost::cref(cleaned_index_to_original_index_mapping), min_index));
		if (tsp_solver_timeout > 0)
		{
			finished = t.try_join_for(boost::chrono::seconds(tsp_solver_timeout));
			if (finished == false)
			{
				tsp_solve.abortComputation();
				std::cout << "GridPointExplorator::getExplorationPath: INFO: Terminated tsp_solver " << tsp_solver << " because of time out. Taking the Nearest Neighbor TSP instead." << std::endl;
			}
		}
		else
			finished = true;
		t.join();
	}
	// fall back to nearest neighbor TSP if the other approach was timed out
	if (tsp_solver==TSP_NEAREST_NEIGHBOR || finished==false)
	{
		NearestNeighborTSPSolver tsp_solve;
		optimal_order = tsp_solve.solveNearestTSPWithCleanedDistanceMatrix(distance_matrix_cleaned, cleaned_index_to_original_index_mapping, min_index);
		std::cout << "GridPointExplorator::getExplorationPath: finished TSP with solver 1 and optimal_order.size=" << optimal_order.size() << std::endl;
	}

	// rearrange the found points in the optimal order and convert them to the right format
	std::vector<cv::Point2f> fov_middlepoint_path(optimal_order.size());
	for(unsigned int point_index = 0; point_index < optimal_order.size(); ++point_index)
		fov_middlepoint_path[point_index] = cv::Point2f(grid_points[optimal_order[point_index]].x, grid_points[optimal_order[point_index]].y);

	// transform the calculated path back to the originally rotated map and create poses with an angle
	std::vector<geometry_msgs::Pose2D> path_fov_poses;
	room_rotation.transformPathBackToOriginalRotation(fov_middlepoint_path, path_fov_poses, R);

//	for(unsigned int point_index = 0; point_index < fov_middlepoint_path.size(); ++point_index)
//	{
//		// get the vector from the current point to the next point
//		cv::Point current_point = grid_points[optimal_order[point_index]];
//		cv::Point next_point = grid_points[optimal_order[(point_index+1)%(optimal_order.size())]];
//		cv::Point vector = next_point - current_point;
//
//		float angle = std::atan2(vector.y, vector.x);//std::acos(quotient);
//
//		// add the next navigation goal to the path
//		geometry_msgs::Pose2D navigation_goal;
//		navigation_goal.x = (current_point.x * map_resolution) + map_origin.x; // coordinate systems are rotated to each other
//		navigation_goal.y = (current_point.y * map_resolution) + map_origin.y;
//		navigation_goal.theta = angle;
//
//		path.push_back(navigation_goal);
//	}

	// if the path should be planned for the robot footprint create the path and return here
	if(plan_for_footprint == true)
	{
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=path_fov_poses.begin(); pose != path_fov_poses.end(); ++pose)
		{
			geometry_msgs::Pose2D current_pose;
			current_pose.x = (pose->x * map_resolution) + map_origin.x;
			current_pose.y = (pose->y * map_resolution) + map_origin.y;
			current_pose.theta = pose->theta;
			path.push_back(current_pose);
		}
		return;
	}

	// *********************** III. Get the robot path out of the fov path. ***********************
	// go trough all computed fov poses and compute the corresponding robot pose
	//mapPath(room_map, path, path_fov_poses, robot_to_fov_vector, map_resolution, map_origin, starting_position);
	ROS_INFO("Starting to map from field of view pose to robot pose");
	cv::Point robot_starting_position = (path_fov_poses.size()>0 ? cv::Point(path_fov_poses[0].x, path_fov_poses[0].y) : starting_position);
	cv::Mat inflated_room_map;
	cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_cell_size);
	mapPath(inflated_room_map, path, path_fov_poses, robot_to_fov_vector, map_resolution, map_origin, robot_starting_position);
}

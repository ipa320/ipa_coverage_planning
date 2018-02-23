#include <ipa_building_navigation/nearest_neighbor_TSP.h>

//Default Constructor
NearestNeighborTSPSolver::NearestNeighborTSPSolver()
{

}

//This function calculates the order of the TSP, using the nearest neighbor method. It uses a pathlength Matrix, which
//should be calculated once. This Matrix should save the pathlengths with this logic:
//		1. The rows show from which Node the length is calculated.
//		2. For the columns in a row the Matrix shows the distance to the Node in the column.
//		3. From the node to itself the distance is 0.
std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& path_length_matrix, const int start_node)
{
	std::vector<int> calculated_order; //solution order

	if(path_length_matrix.rows > 1) //check if clique has more than one member or else this algorithm produces a order of size=3
	{
		int last_node; //index of the last spectated node
		std::vector<bool> visited(path_length_matrix.rows, false);

		int current_node = start_node; //index of the current spectated node
		calculated_order.push_back(current_node);
		visited[current_node] = true;

		//check every Point for the next nearest neighbor and add it to the order
		do
		{
			int next_node; //saver for next node
			double min_distance = 1e100; //saver for distance to current next node
			for (int current_neighbor = 0; current_neighbor < path_length_matrix.cols; current_neighbor++)
			{
				if (visited[current_neighbor]==false) //check if current neighbor hasn't been visited yet
				{
					const double length = path_length_matrix.at<double>(current_node, current_neighbor);
					if (length < min_distance && length > 0)
					{
						next_node = current_neighbor;
						min_distance = length;
					}
				}
			}
			calculated_order.push_back(next_node); //add the found nearest neighbor to the order-vector
			visited[next_node] = true;
			current_node = next_node;
		} while (calculated_order.size() < path_length_matrix.rows); //when the order has as many elements as the pathlength Matrix has the solver is ready

	}
	else
	{
		calculated_order.push_back(start_node);
	}

	return calculated_order;
}

// compute TSP and distance matrix without cleaning it
// this version does not exclude infinite paths from the TSP ordering
std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, const int start_node, cv::Mat* distance_matrix)
{
	std::cout << "NearestNeighborTSPSolver::solveNearestTSP: Constructing distance matrix..." << std::endl;
	//calculate the distance matrix
	cv::Mat distance_matrix_ref;
	if (distance_matrix != 0)
		distance_matrix_ref = *distance_matrix;
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.constructDistanceMatrix(distance_matrix_ref, original_map, points, downsampling_factor, robot_radius, map_resolution, pathplanner_);

	return solveNearestTSP(distance_matrix_ref, start_node);
}


// compute TSP from a cleaned distance matrix (does not contain any infinity paths) that has to be computed
std::vector<int> NearestNeighborTSPSolver::solveNearestTSPClean(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, const int start_node)
{
	// compute a cleaned distance matrix
	cv::Mat distance_matrix_cleaned;
	std::map<int,int> cleaned_index_to_original_index_mapping;	// maps the indices of the cleaned distance_matrix to the original indices of the original distance_matrix
	int new_start_node = start_node;
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.computeCleanedDistanceMatrix(original_map, points, downsampling_factor, robot_radius, map_resolution, pathplanner_,
			distance_matrix_cleaned, cleaned_index_to_original_index_mapping, new_start_node);

	// solve TSP and re-index points to original indices
	return solveNearestTSPWithCleanedDistanceMatrix(distance_matrix_cleaned, cleaned_index_to_original_index_mapping, new_start_node);
}


// compute TSP with pre-computed cleaned distance matrix (does not contain any infinity paths)
std::vector<int> NearestNeighborTSPSolver::solveNearestTSPWithCleanedDistanceMatrix(const cv::Mat& distance_matrix,
		const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node)
{
	// solve TSP and re-index points to original indices
	std::vector<int> optimal_order = solveNearestTSP(distance_matrix, start_node);
	for (size_t i=0; i<optimal_order.size(); ++i)
		optimal_order[i] = cleaned_index_to_original_index_mapping.at(optimal_order[i]);

	return optimal_order;
}

#include <ipa_building_navigation/nearest_neighbor_TSP.h>

//Default Constructor
NearestNeighborTSPSolver::NearestNeighborTSPSolver()
{

}

////Function to construct the symmetrical distance matrix from the given points. The rows show from which node to start and
////the columns to which node to go. If the path between nodes doesn't exist or the node to go to is the same as the one to
////start from, the entry of the matrix is 0.
//void NearestNeighborTSPSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map,
//		const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution)
//{
//	//create the distance matrix with the right size
//	cv::Mat pathlengths(cv::Size((int)points.size(), (int)points.size()), CV_64F);
//
//	// reduce image size already here to avoid resizing in the planner each time
//	const double one_by_downsampling_factor = 1./downsampling_factor;
//	cv::Mat downsampled_map;
//	pathplanner_.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);
//
//	for (int i = 0; i < points.size(); i++)
//	{
//		cv::Point current_center = downsampling_factor * points[i];
//		for (int j = 0; j < points.size(); j++)
//		{
//			if (j != i)
//			{
//				if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
//				{
//					cv::Point neighbor = downsampling_factor * points[j];
//					double length = one_by_downsampling_factor * pathplanner_.planPath(downsampled_map, current_center, neighbor, 1., 0., map_resolution);
//					pathlengths.at<double>(i, j) = length;
//					pathlengths.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computationtime
//				}
//			}
//			else
//			{
//				pathlengths.at<double>(i, j) = 0;
//			}
//		}
//	}
//
//	distance_matrix = pathlengths.clone();
//}

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

		int current_node = start_node; //index of the current spectated node
		calculated_order.push_back(current_node);

		//check every Point for the next nearest neighbor and add it to the order
		do
		{
			int next_node; //saver for next node
			double saved_distance = 100000000000000; //saver for distance to current next node
			for (int current_neighbor = 0; current_neighbor < path_length_matrix.cols; current_neighbor++)
			{
				if (!contains<int>(calculated_order, current_neighbor)) //check if current neighbor hasn't been visited yet
				{
					if (path_length_matrix.at<double>(current_node, current_neighbor) < saved_distance
							&& path_length_matrix.at<double>(current_node, current_neighbor) > 0)
					{
						next_node = current_neighbor;
						saved_distance = path_length_matrix.at<double>(current_node, current_neighbor);
					}
				}
			}
			calculated_order.push_back(next_node); //add the found nearest neighbor to the order-vector
			current_node = next_node;
		} while (calculated_order.size() < path_length_matrix.rows); //when the order has as many elements as the pathlength Matrix has the solver is ready

	}
	else
	{
		calculated_order.push_back(start_node);
	}

	return calculated_order;
}

// compute distance matrix without returning it
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

	// check whether distance matrix contains infinite path lengths and if this is true, create a new distance matrix with maximum size clique of reachable points
	cv::Mat distance_matrix_cleaned;
	std::map<int,int> cleaned_index_to_original_index_mapping;	//maps the indices of the cleaned distance_matrix to the original indices of the original distance_matrix
	distance_matrix_computation.cleanDistanceMatrix(distance_matrix_ref, distance_matrix_cleaned, cleaned_index_to_original_index_mapping);

	// re-assign the start node to cleaned indices (use 0 if the original start node was removed from distance_matrix_cleaned)
	int new_start_node = 0;
	for (std::map<int,int>::iterator it=cleaned_index_to_original_index_mapping.begin(); it!=cleaned_index_to_original_index_mapping.end(); ++it)
		if (it->second == start_node)
			new_start_node = it->first;

	// solve TSP and re-index points to original indices
	std::vector<int> optimal_order = solveNearestTSP(distance_matrix_cleaned, new_start_node);
	for (size_t i=0; i<optimal_order.size(); ++i)
		optimal_order[i] = cleaned_index_to_original_index_mapping[optimal_order[i]];

	// todo: and do not forget to copy fix to ipa_building_navigation

	return optimal_order;
}


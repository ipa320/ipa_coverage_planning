#include <ipa_building_navigation/nearest_neighbor_TSP.h>

//Default Constructor
NearestNeighborTSPSolver::NearestNeighborTSPSolver()
{

}

//Function to construct the distance matrix from the given points. See the definition below for the style of this matrix.
void NearestNeighborTSPSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const int number_of_nodes,
        const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution)
{
	//create the distance matrix with the right size
	cv::Mat pathlengths(cv::Size(number_of_nodes, number_of_nodes), CV_64F);

	for (int i = 0; i < points.size(); i++)
	{
		cv::Point current_center = points[i];
		for (int p = 0; p < points.size(); p++)
		{
			if (p != i)
			{
				if (p > i) //only compute upper right triangle of matrix, rest is symmetrically added
				{
					cv::Point neighbor = points[p];
					double length = pathplanner_.planPath(original_map, current_center, neighbor, downsampling_factor, robot_radius, map_resolution);
					pathlengths.at<double>(i, p) = length;
					pathlengths.at<double>(p, i) = length; //symmetrical-Matrix --> saves half the computationtime
				}
			}
			else
			{
				pathlengths.at<double>(i, p) = 0;
			}
		}
	}

	distance_matrix = pathlengths.clone();
}

//This function calculates the order of the TSP, using the nearest neighbor method. It uses a pathlength Matrix, which
//should be calculated once. This Matrix should save the pathlengths with this logic:
//		1. The rows show from which Node the length is calculated.
//		2. For the columns in a row the Matrix shows the distance to the Node in the column.
//		3. From the node to itself the distance is 0.
std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& path_length_matrix, const int start_node)
{
	std::vector<int> calculated_order; //solution order

	int last_node; //index of the last spectated node

	int current_node = start_node; //index of the current spectated node
	calculated_order.push_back(current_node);

	//check every Point for the next nearest neighbor and add it to the order
	do
	{
		int next_node; //saver for next node
		double saved_distance = 100000000000000; //saver for distance to current next node
		for (int current_neighbor = 0; current_neighbor < path_length_matrix.rows; current_neighbor++)
		{
			if (!contains(calculated_order, current_neighbor)) //check if current neighbor hasn't been visited yet
			{
				if (path_length_matrix.at<double>(current_node, current_neighbor) < saved_distance
				        && path_length_matrix.at<double>(current_node, current_neighbor) > 0)
				{
					next_node = current_neighbor;
					saved_distance = path_length_matrix.at<double>(current_node, current_neighbor);
				}
			}
		}
		calculated_order.push_back(next_node); //add the found nearest nighbor to the order-vector
	} while (calculated_order.size() < path_length_matrix.rows); //when the order has as many elements as the pathlength Matrix has the solver is ready

	//add the starting-node at the end of the vector, so the Start will be reached again.
	calculated_order.push_back(start_node);

	return calculated_order;
}

//compute distancematrix without returning it
std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& original_map, const int number_of_nodes, const std::vector<cv::Point>& points,
        double downsampling_factor, double robot_radius, double map_resolution, const int start_node)
{
	cv::Mat distance_matrix;
	constructDistanceMatrix(distance_matrix, original_map, number_of_nodes, points, downsampling_factor, robot_radius, map_resolution);

	return (solveNearestTSP(distance_matrix, start_node));
}

//compute distancematrix without returning it
std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& original_map, const int number_of_nodes, const std::vector<cv::Point>& points,
        double downsampling_factor, double robot_radius, double map_resolution, const int start_node, cv::Mat& pathlength_matrix)
{
	constructDistanceMatrix(pathlength_matrix, original_map, number_of_nodes, points, downsampling_factor, robot_radius, map_resolution);

	return (solveNearestTSP(pathlength_matrix, start_node));
}

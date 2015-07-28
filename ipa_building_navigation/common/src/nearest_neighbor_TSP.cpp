#include <ipa_building_navigation/nearest_neighbor_TSP.h>

NearestNeighborTSPSolver::NearestNeighborTSPSolver()
{

}

std::vector<int> NearestNeighborTSPSolver::solveNearestTSP(const cv::Mat& path_length_Matrix, const int start_node)
{
	//This function calculates the order of the TSP, using the nearest neighbor method. It uses a pathlength Matrix, which
	//should be calculated once. This Matrix should save the pathlengths with this logic:
	//		1. The rows show from which Node the length is calculated.
	//		2. For the columns in a row the Matrix shows the distance to the Node in the column.
	//		3. From the node to itself the distance is 0.

	std::vector<int> calculated_order; //solution order

	int last_node; //index of the last spectated node

	int current_node = start_node; //index of the current spectated node
	calculated_order.push_back(current_node);

	//check every Point for the next nearest neighbor and add it to the order
	do
	{
		int next_node; //saver for next node
		double saved_distance = 100000000000000; //saver for distance to current next node
		for (int current_neighbor = 0; current_neighbor < path_length_Matrix.rows; current_neighbor++)
		{
			if (!contains(calculated_order, current_neighbor)) //check if current neighbor hasn't been visited yet
			{
				if (path_length_Matrix.at<double>(current_node, current_neighbor) < saved_distance
				        && path_length_Matrix.at<double>(current_node, current_neighbor) > 0)
				{
					next_node = current_neighbor;
					saved_distance = path_length_Matrix.at<double>(current_node, current_neighbor);
				}
			}
		}
		calculated_order.push_back(next_node); //add the found nearest nighbor to the order-vector
	} while (calculated_order.size() < path_length_Matrix.rows); //when the order has as many elements as the pathlength Matrix has the solver is ready

	//add the starting-node at the end of the vector, so the Start will be reached again.
	calculated_order.push_back(start_node);

	return calculated_order;
}

#include <ipa_room_exploration/energy_functional_explorator.h>

// Constructor
energyFunctionalExplorator::energyFunctionalExplorator()
{

}

// Function that plans a coverage path trough the given map, using the method proposed in
//
//	Bormann, Richard, Joshua Hampp, and Martin Hägele. "New brooms sweep clean-an autonomous robotic cleaning assistant for
//	professional office cleaning." Robotics and Automation (ICRA), 2015 IEEE International Conference on. IEEE, 2015.
//
// This method discretizes the free space, that should be covered, into several nodes. Each of the node has to be covered, in order
// to cover the whole area. The path starts at the node that is closest to the given starting position and chooses the next node as
// the one that minimizes the energy functional, provided in the paper above. To do this here the following steps are done.
//	I.	The free area gets discretized into several nodes, using the given cell_size parameter, starting at the upper left white pixel of
//		the room. Whenever the overlaid grid then hits a white pixel, this point is added as a node. Then after all nodes have been found
//		the direct 8 neighbors for each node are found, which will be used later in the energy functional.
//	II.	After all nodes have been found, the coverage path is computed.
//			i.	The start node gets chosen as the one that is closest to the given starting position and is an edge of the given room, i.e
//				a node that has less than 4 neighbors.
//			ii.	The next node is then iteratively chosen from the directly neighboring ones, by finding the node that minimizes the given
//				energy functional and wasn't visited before.
//			iii.If in the neighborhood no accessible point could be found, search for the next node in the whole grid to continue the path.
//			iv.	This procedure is repeated, until all created nodes have been covered.
//
void energyFunctionalExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
			const cv::Point starting_position, const cv::Point2d map_origin, const int cell_size, const geometry_msgs::Polygon room_min_max_coordinates,
			const float fitting_circle_radius, const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fow_vector)
{
	// *********************** I. Find the nodes and their neighbors ***********************
	// get the nodes in the free space
	std::vector<std::vector<energyExploratorNode> > nodes; // 2-dimensional vector to easily find the neighbors
	for(size_t y=room_min_max_coordinates.points[0].y+cell_size; y<room_min_max_coordinates.points[1].y-cell_size; y+=2.0*cell_size)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<energyExploratorNode> current_row;
		for(size_t x=room_min_max_coordinates.points[0].x+cell_size; x<room_min_max_coordinates.points[1].x-cell_size; x+=2.0*cell_size)
		{
			// create node if the current point is in the free space
			if(room_map.at<uchar>(y,x) == 255)
			{
				energyExploratorNode current_node;
				current_node.center_ = cv::Point(x,y);
				current_row.push_back(current_node);
			}
		}

		// insert the current row into grid
		nodes.push_back(current_row);
	}

	// find the neighbors for each node
	for(size_t row=0; row<nodes.size(); ++row)
	{
		for(size_t column=0; column<nodes[row].size(); ++column)
		{
			for(int dy=-1; dy<=1; ++dy)
			{
				// don't exceed the current row
				if(row+dy < 0 || row+dy >= nodes.size())
					continue;

				// get the neighbors left from the current neuron
				if(column > 0)
					nodes[row][column].neighbors_.push_back(nodes[row+dy][column-1].center_);

				// get the nodes on the same column as the current neuron
				if(dy != 0)
					nodes[row][column].neighbors_.push_back(nodes[row+dy][column].center_);

				// get the nodes right from the current neuron
				if(column < nodes[row].size()-1)
					nodes[row][column].neighbors_.push_back(nodes[row+dy][column+1].center_);
			}
		}
	}

//	testing
	for(size_t i=0; i<nodes.size(); ++i)
	{
		cv::Mat test_map = room_map.clone();
		for(size_t j=0; j<nodes[i].size(); ++j)
		{
			std::vector<cv::Point> neighbors = nodes[i][j].neighbors_;
			for(std::vector<cv::Point>::iterator n=neighbors.begin(); n!=neighbors.end(); ++n)
				cv::circle(test_map, *n, 2, cv::Scalar(127), CV_FILLED);
		}
		cv::imshow("neighbors", test_map);
		cv::waitKey();
	}
}

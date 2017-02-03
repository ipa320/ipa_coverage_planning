#include <ipa_room_exploration/energy_functional_explorator.h>

// Constructor
energyFunctionalExplorator::energyFunctionalExplorator()
{

}

// Function that computes the energy functional for each pair of nodes.
float E(const energyExploratorNode& location, const energyExploratorNode& neighbor,
		std::set<cv::Point, cv_Point_cmp> visited_nodes, const int cell_size, const double previous_travel_angle)
{
	float energy_functional = 0.0;

	// 1. translational distance
	cv::Point diff = location.center_ - neighbor.center_;
	energy_functional += cv::norm(diff)/cell_size;

	// 2. rotational distance
	double travel_angle_to_node = std::atan2(diff.y, diff.x);
	energy_functional += std::abs(previous_travel_angle - travel_angle_to_node)/PI_2;

	// 3. neighboring function, determining how many neighbors of the neighbor have been visited
	// TODO: finish
	int visited_neighbors = 0;
//	for(std::vector<cv::Point>::iterator n=neighbor.neighbors_.begin(); n!=neighbor.neighbors_.end(); ++n)
//		if(visited_nodes.find(*n->center_)!=visited_nodes.end())
//			++visited_neighbors;
//	energy_functional += 4 - visited_neighbors/2;

	return energy_functional;
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
			const cv::Point starting_position, const cv::Point2d map_origin, const geometry_msgs::Polygon room_min_max_coordinates,
			const float fitting_circle_radius, const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fow_vector)
{
	// *********************** I. Find the nodes and their neighbors ***********************
	// get the nodes in the free space
	std::vector<std::vector<energyExploratorNode> > nodes; // 2-dimensional vector to easily find the neighbors
	int radius_as_int = (int) std::floor(fitting_circle_radius);
	int number_of_free_nodes = 0;
	for(size_t y=room_min_max_coordinates.points[0].y+radius_as_int; y<room_min_max_coordinates.points[1].y-radius_as_int; y+=2.0*radius_as_int)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<energyExploratorNode> current_row;
		for(size_t x=room_min_max_coordinates.points[0].x+radius_as_int; x<room_min_max_coordinates.points[1].x-radius_as_int; x+=2.0*radius_as_int)
		{
			// create node if the current point is in the free space
			energyExploratorNode current_node;
			current_node.center_ = cv::Point(x,y);
			if(room_map.at<uchar>(y,x) == 255)
			{
				current_node.obstacle_ = false;
				++number_of_free_nodes;
			}
			// add the obstacle nodes to easily find the neighbors for each free node by using the grid structure
			else
				current_node.obstacle_ = true;
			current_row.push_back(current_node);
		}

		// insert the current row into grid
		nodes.push_back(current_row);
	}
	std::cout << "found " << number_of_free_nodes <<  " free nodes" << std::endl;

	// find the neighbors for each node
	std::vector<energyExploratorNode> corner_nodes; // vector that stores the corner nodes, i.e. nodes with 3 or less neighbors
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
				if(column > 0 && nodes[row+dy][column-1].obstacle_==false)
					nodes[row][column].neighbors_.push_back(&nodes[row+dy][column-1]);

				// get the nodes on the same column as the current neuron
				if(dy != 0 && nodes[row+dy][column].obstacle_==false)
					nodes[row][column].neighbors_.push_back(&nodes[row+dy][column]);

				// get the nodes right from the current neuron
				if(column < nodes[row].size()-1 && nodes[row+dy][column+1].obstacle_==false)
					nodes[row][column].neighbors_.push_back(&nodes[row+dy][column+1]);
			}

			// check if the current node is a corner
			if(nodes[row][column].neighbors_.size()<=3)
				corner_nodes.push_back(nodes[row][column]);
		}
	}
	std::cout << "found neighbors" << std::endl;

//	testing
//	for(size_t i=0; i<nodes.size(); ++i)
//	{
//		for(size_t j=0; j<nodes[i].size(); ++j)
//		{
//			cv::Mat test_map = room_map.clone();
//
//			std::vector<cv::Point> neighbors = nodes[i][j].neighbors_;
//			for(std::vector<cv::Point>::iterator n=neighbors.begin(); n!=neighbors.end(); ++n)
//				cv::circle(test_map, *n, 2, cv::Scalar(127), CV_FILLED);
//
//			cv::imshow("neighbors", test_map);
//			cv::waitKey();
//		}
//	}

	// *********************** II. Plan the coverage path ***********************
	// i. find the start node of the path as a corner that is closest to the starting position
	energyExploratorNode start_node;
	float min_distance = 1e4;
	for(std::vector<energyExploratorNode>::iterator corner=corner_nodes.begin(); corner!=corner_nodes.end(); ++corner)
	{
		cv::Point diff = corner->center_ - starting_position;
		float current_distance = diff.x*diff.x+diff.y*diff.y;
		if(current_distance<=min_distance)
		{
			start_node = *corner;
			min_distance = current_distance;
		}
	}

	// insert start node into coverage path
	std::vector<cv::Point> fow_coverage_path;
	fow_coverage_path.push_back(start_node.center_);

	// ii. starting at the start node, find the coverage path, by choosing the node that min. the energy functional
	std::set<cv::Point, cv_Point_cmp> visited_nodes;
	energyExploratorNode last_node = start_node;
	visited_nodes.insert(start_node.center_);
	double previous_travel_angle = std::atan2(starting_position.y-start_node.center_.y, starting_position.x-start_node.center_.x);
	do
	{
		// check the direct neighbors, if at least one is not already visited
		// TODO: finish --> pointers as neighbors
		std::vector<energyExploratorNode> not_visited_neighbors;
//		for(std::)

	}while(visited_nodes.size()<number_of_free_nodes);
}

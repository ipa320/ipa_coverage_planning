#include <ipa_room_exploration/neural_network_explorator.h>

// Default constructor
neuralNetworkExplorator::neuralNetworkExplorator()
{
	// default values
	step_size_ = 0.01;
	A_ = 20;
	B_ = 1;
	D_ = 1;
	E_ = 50; // E >> B
	mu_ = 0.7;
	delta_theta_weight_ = 1.5;
}

// Function that calculates an exploration path trough the given map s.t. everything has been covered by the robot-footprint
// or the field of view. The algorithm is programmed after
//
// Yang, Simon X., and Chaomin Luo. "A neural network approach to complete coverage path planning." IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics) 34.1 (2004): 718-724.
//
// and uses a artificial neural network to produce the path. For this the following steps are done:
// I. 	Construct the neural network by sampling the given map, using the given fitting circle radius as step size. This is
//		done because it allows that when all neurons (the samples) are covered the whole space have been cleaned/inspected.
// II.	Starting with the given robot pose go trough the found network. At every time-step choose the next Neuron by
//		solving x_n = max(x_j + c*y_j), with c as a positive scalar and y_j a function penalizing movements of the robot
//		into a direction he is currently not pointing at. At every time-step an update of the states of the neurons is done.
//
void neuralNetworkExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
					 const geometry_msgs::Pose2D starting_position, const cv::Point2d map_origin, const float fitting_circle_radius,
					 const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fow_vector,
					 const geometry_msgs::Polygon room_min_max_coordinates)
{
	// ****************** I. Create the neural network ******************
	// reset previously computed neurons
	neurons_.clear();

	// go trough the map and create the neurons
	int fitting_radius_as_int = (int) std::floor(fitting_circle_radius);
	int number_of_neurons = 0;
	for(size_t y=room_min_max_coordinates.points[0].y+fitting_radius_as_int; y<room_min_max_coordinates.points[1].y-fitting_radius_as_int; y+=fitting_radius_as_int)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<Neuron> current_network_row;
		for(size_t x=room_min_max_coordinates.points[0].x+fitting_radius_as_int; x<room_min_max_coordinates.points[1].x-fitting_radius_as_int; x+=fitting_radius_as_int)
		{
			// create free neuron
			if(room_map.at<uchar>(y,x) == 255)
			{
				Neuron current_neuron(cv::Point(x,y), A_, B_, D_, E_, mu_, step_size_, false);
				current_network_row.push_back(current_neuron);
				++number_of_neurons;
			}
			else // obstacle neuron
			{
				Neuron current_neuron(cv::Point(x,y), A_, B_, D_, E_, mu_, step_size_, true);
				current_network_row.push_back(current_neuron);
				++number_of_neurons;
			}
		}

		// insert the current row into the network
		neurons_.push_back(current_network_row);
	}

	// go trough the found neurons and get the direct neighbors of each
	for(size_t row=0; row<neurons_.size(); ++row)
	{
		for(size_t column=0; column<neurons_[row].size(); ++column)
		{
			for(int dy=-1; dy<=1; ++dy)
			{
				// don't exceed the current column
				if(row+dy < 0 || row+dy >= neurons_.size())
					continue;

				// get the neighbors left from the current neuron
				if(column > 0)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column-1]);

				// get the neurons on the same column as the current neuron
				if(dy != 0)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column]);

				// get the neurons right from the current neuron
				if(column < neurons_[row].size()-1)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column+1]);
			}
		}
	}

//	testing
//	cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, room_map.type(), cv::Scalar(0));
//	for(size_t i=0; i<neurons_.size(); ++i)
//	{
//		for(size_t j=0; j<neurons_[i].size(); ++j)
//		{
//			std::vector<Neuron*> neighbors;
//			neurons_[i][j].getNeighbors(neighbors);
//			for(size_t k=0; k<neighbors.size(); ++k)
//			{
//				cv::circle(black_map, neighbors[k]->getPosition(), 2, cv::Scalar(127), CV_FILLED);
//			}
//		}
//	}
//	cv::imshow("neighbors", black_map);
//	cv::waitKey();

	// ****************** II. Find the coverage path ******************
	// mark the first non-obstacle neuron as starting node
	Neuron* starting_neuron;
	bool found = false;
	for(size_t row=0; row<neurons_.size(); ++row)
	{
		for(size_t column=0; column<neurons_[row].size(); ++column)
		{
			if(neurons_[row][column].isObstacle() == false && found == false)
			{
				found = true;
				starting_neuron = &neurons_[row][column];
				break;
			}
		}

		if(found == true)
			break;
	}
	starting_neuron->markAsVisited();

	// initial update of the states to mark obstacles and unvisited free neurons as such
	for(size_t row=0; row<neurons_.size(); ++row)
		for(size_t column=0; column<neurons_[row].size(); ++column)
			neurons_[row][column].saveState();
	for(size_t row=0; row<neurons_.size(); ++row)
		for(size_t column=0; column<neurons_[row].size(); ++column)
			neurons_[row][column].updateState();

//	testing
//	cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, CV_64F, cv::Scalar(0));
//	for(size_t row=0; row<neurons_.size(); ++row)
//	{
//		for(size_t column=0; column<neurons_[row].size(); ++column)
//		{
//			std::cout << neurons_[row][column].getState(false) << " ";
//			black_map.at<double>(row*fitting_radius_as_int, column*fitting_radius_as_int) = neurons_[row][column].getState(false);
//		}
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;
//	cv::imshow("states", black_map);
//	cv::waitKey();

	// iteratively choose the next neuron until all neurons have been visited
	int visited_neurons = 1;
	std::vector<geometry_msgs::Pose2D> fow_path; // vector that stores the computed path, that is generated for the field of view (fow)
	geometry_msgs::Pose2D initial_pose;
	initial_pose.x = starting_neuron->getPosition().x;
	initial_pose.y = starting_neuron->getPosition().y;
	initial_pose.theta = 0.0;
	fow_path.push_back(initial_pose);
	double previous_traveling_angle = 0.0; // save the travel direction to the current neuron to determine the next neuron
	cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, CV_64F, cv::Scalar(0));
	Neuron* previous_neuron = starting_neuron;
	do
	{
		// get the current neighbors and choose the next out of them
		std::vector<Neuron*> neighbors;
		previous_neuron->getNeighbors(neighbors);
		Neuron* next_neuron;

		// go trough the neighbors and find the next one
		std::cout << "checking neighbors" << std::endl;
		double max_value = -1e3, travel_angle = 0.0;
		for(size_t neighbor=0; neighbor<neighbors.size(); ++neighbor)
		{
			// get travel angle to this neuron
			travel_angle = std::atan2(fow_path.back().y-neighbors[neighbor]->getPosition().y, fow_path.back().x-neighbors[neighbor]->getPosition().x);

			// compute penalizing function y_j
			double y = 1 - (std::abs(previous_traveling_angle - travel_angle)/PI);
			std::cout << "y: " << y << ", value: ";

			// compute transition function value
			double trans_fct_value = neighbors[neighbor]->getState(false) + delta_theta_weight_ * y;
			std::cout << trans_fct_value << std::endl;

			// check if neighbor is next neuron to be visited
			if(trans_fct_value > max_value)
			{
				std::cout << "found new optimum candidate" << std::endl;
				max_value = trans_fct_value;
				next_neuron = neighbors[neighbor];
			}
		}

		// mark next neuron as visited
		next_neuron->markAsVisited();
		++visited_neurons;
		previous_traveling_angle = travel_angle;

		// add neuron to path
		geometry_msgs::Pose2D current_pose;
		current_pose.x = next_neuron->getPosition().x;
		current_pose.y = next_neuron->getPosition().y;
		current_pose.theta = travel_angle;
		fow_path.push_back(current_pose);

		// update the states of the network
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].saveState();
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].updateState();

		// save neuron that has been visited
		previous_neuron = next_neuron;

//		testing
		cv::circle(black_map, next_neuron->getPosition(), 2, cv::Scalar(127), CV_FILLED);
		cv::imshow("next_neuron", black_map);
		cv::waitKey();

	}while(visited_neurons < number_of_neurons);
}

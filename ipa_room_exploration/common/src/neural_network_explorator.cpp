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
}

// Function that calculates an exploration path trough the given map s.t. everything has been covered by the robot-footprint
// or the field of view. The algorithm is programmed after
//
// Yang, Simon X., and Chaomin Luo. "A neural network approach to complete coverage path planning." IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics) 34.1 (2004): 718-724.
//
// and uses a artificial neural network to produce the path. For this the following steps are done:
// I. 	Construct the neural network by sampling the given map, using the given fitting circle radius as step size. This is
//		done because it allows that when all neurons (the samples) are covered the whole space have been cleaned/inspected.
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
	for(size_t y=room_min_max_coordinates.points[0].y+fitting_radius_as_int; y<room_min_max_coordinates.points[1].y-fitting_radius_as_int; y+=fitting_radius_as_int)
	{
		for(size_t x=room_min_max_coordinates.points[0].x+fitting_radius_as_int; x<room_min_max_coordinates.points[1].x-fitting_radius_as_int; x+=fitting_radius_as_int)
		{
			// create free neuron
			if(room_map.at<uchar>(y,x) == 255)
			{
				Neuron current_neuron(cv::Point(x,y), A_, B_, D_, E_, mu_, step_size_, false);
				neurons_.push_back(current_neuron);
			}
			else // obstacle neuron
			{
				Neuron current_neuron(cv::Point(x,y), A_, B_, D_, E_, mu_, step_size_, true);
				neurons_.push_back(current_neuron);
			}
		}
	}

	// go trough the found neurons and get the direct neighbors of each
	// TODO: find neighbors
}

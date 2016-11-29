#include <ipa_room_exploration/neural_network_explorator.h>

// Default constructor
neuralNetworkExplorator::neuralNetworkExplorator()
{
	// default values TODO: param
	step_size_ = 0.008; // 0.008
	A_ = 17; // 17
	B_ = 5; // 5
	D_ = 7; // 7
	E_ = 80; // E >> B, 80
	mu_ = 1.03; // 1.03
	delta_theta_weight_ = 0.15; // 0.15
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
//		After this step a path consisting of poses for the fow middlepoint is obtained. If the plan should be planned for
//		the robot footprint, then the algorithm returns this path and is finished.
// III.	If wanted use the given vector from the robot middlepoint to the fow middlepoint to map the fow poses to the
//		robot footprint poses. To do so simply a vector operation is applied. If the computed robot pose is not in the
//		free space, another accessible point is generated by finding it on the radius around the fow middlepoint s.t.
//		the distance to the last robot position is minimized. If this is not wanted one has to set the corresponding
//		Boolean to false (shows that the path planning should be done for the robot footprint).
void neuralNetworkExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
					 const cv::Point starting_position, const cv::Point2d map_origin, const float fitting_circle_radius,
					 const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fow_vector,
					 const geometry_msgs::Polygon room_min_max_coordinates, bool show_path_computation)
{
	// ****************** I. Create the neural network ******************
	// reset previously computed neurons
	neurons_.clear();

	// go trough the map and create the neurons
	int fitting_radius_as_int = (int) std::floor(fitting_circle_radius);
	int number_of_neurons = 0;
	for(size_t y=room_min_max_coordinates.points[0].y+fitting_radius_as_int; y<room_min_max_coordinates.points[1].y-fitting_radius_as_int; y+=2.0*fitting_radius_as_int)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<Neuron> current_network_row;
		for(size_t x=room_min_max_coordinates.points[0].x+fitting_radius_as_int; x<room_min_max_coordinates.points[1].x-fitting_radius_as_int; x+=2.0*fitting_radius_as_int)
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

	// initial updates of the states to mark obstacles and unvisited free neurons as such
	for(size_t init=1; init<=3; ++init)
	{
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].saveState();
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].updateState();
	}

//	testing
//	cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, CV_32F, cv::Scalar(0));
//	for(size_t row=0; row<neurons_.size(); ++row)
//	{
//		for(size_t column=0; column<neurons_[row].size(); ++column)
//		{
//			std::cout << neurons_[row][column].getState(false) << " ";
//			black_map.at<float>(row*fitting_radius_as_int, column*fitting_radius_as_int) = (float) 1000.0*neurons_[row][column].getState(false);
//		}
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;
//	cv::namedWindow("states", cv::WINDOW_NORMAL);
//	cv::imshow("states", black_map);
//	cv::resizeWindow("states", 600, 600);
//	cv::waitKey();

	// iteratively choose the next neuron until all neurons have been visited or the algorithm is stuck in a
	// limit cycle like path (i.e. the same neurons get visited over and over)
	int visited_neurons = 1;
	bool stuck_in_cycle = false;
	std::vector<geometry_msgs::Pose2D> fow_path; // vector that stores the computed path, that is generated for the field of view (fow)
	geometry_msgs::Pose2D initial_pose;
	initial_pose.x = starting_neuron->getPosition().x;
	initial_pose.y = starting_neuron->getPosition().y;
//	initial_pose.theta = 0.0;
	fow_path.push_back(initial_pose);
	double previous_traveling_angle = 0.0; // save the travel direction to the current neuron to determine the next neuron
	cv::Mat black_map = room_map.clone();
	Neuron* previous_neuron = starting_neuron;
	do
	{
		// get the current neighbors and choose the next out of them
		std::vector<Neuron*> neighbors;
		previous_neuron->getNeighbors(neighbors);
		Neuron* next_neuron;

		// go trough the neighbors and find the next one
		double max_value = -1e3, travel_angle = 0.0, best_angle = 0.0;
		for(size_t neighbor=0; neighbor<neighbors.size(); ++neighbor)
		{
			// get travel angle to this neuron
			travel_angle = std::atan2(neighbors[neighbor]->getPosition().y-previous_neuron->getPosition().y, neighbors[neighbor]->getPosition().x-previous_neuron->getPosition().x);

			// compute penalizing function y_j
			double y = 1 - (std::abs(previous_traveling_angle - travel_angle)/PI);

			// compute transition function value
			double trans_fct_value = neighbors[neighbor]->getState(false) + delta_theta_weight_ * y;

			// check if neighbor is next neuron to be visited
			if(trans_fct_value > max_value && room_map.at<uchar>(neighbors[neighbor]->getPosition()) != 0)
			{
				max_value = trans_fct_value;
				next_neuron = neighbors[neighbor];
				best_angle = travel_angle;
			}
		}

		// if the next neuron was previously uncleaned, increase number of visited neurons
		if(next_neuron->visitedNeuron() == false)
			++visited_neurons;

		// mark next neuron as visited
		next_neuron->markAsVisited();
		previous_traveling_angle = best_angle;

		// add neuron to path
		geometry_msgs::Pose2D current_pose;
		current_pose.x = next_neuron->getPosition().x;
		current_pose.y = next_neuron->getPosition().y;
		fow_path.push_back(current_pose);

		// check the fow path for a limit cycle by searching the path for the next neuron, if it occurs too often
		// and the previous/after neuron is always the same the algorithm probably is stuck in a cycle
		int number_of_neuron_in_path = 0;
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=fow_path.begin(); pose!=fow_path.end(); ++pose)
			if(*pose==current_pose)
				++number_of_neuron_in_path;

		if(number_of_neuron_in_path >= 20)
		{
			// check number of previous neuron
			geometry_msgs::Pose2D previous_pose = fow_path[fow_path.size()-2];
			int number_of_previous_neuron_in_path = 0;
			for(std::vector<geometry_msgs::Pose2D>::iterator pose=fow_path.begin(); pose!=fow_path.end()-1; ++pose)
			{
				// check if the the previous pose always has the current pose as neighbor
				if(*pose==previous_pose)
				{
					if(*(pose+1)==current_pose)
						++number_of_previous_neuron_in_path;
					else if(*(pose-1)==current_pose)
						++number_of_previous_neuron_in_path;
				}
			}

			if(number_of_previous_neuron_in_path >= number_of_neuron_in_path)
				stuck_in_cycle = true;
		}

		// update the states of the network
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].saveState();
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].updateState();

		// save neuron that has been visited
		previous_neuron = next_neuron;

//		printing of the path computation
		if(show_path_computation == true)
		{
			cv::circle(black_map, next_neuron->getPosition(), 2, cv::Scalar((visited_neurons*5)%250), CV_FILLED);
			cv::imshow("next_neuron", black_map);
			cv::waitKey(20);
		}

	}while(visited_neurons < number_of_neurons && stuck_in_cycle == false); //TODO: test terminal condition

	// go trough the found fow-path and compute the angles of the poses s.t. it points to the next pose that should be visited
	for(unsigned int point_index=0; point_index<fow_path.size(); ++point_index)
	{
		// get the vector from the current point to the next point
		geometry_msgs::Pose2D current_point = fow_path[point_index];
		geometry_msgs::Pose2D next_point = fow_path[(point_index+1)%(fow_path.size())];

		float angle = std::atan2(next_point.y - current_point.y, next_point.x - current_point.x);

		// save the found angle
		fow_path[point_index].theta = angle;
	}

	// if the path should be planned for the robot footprint create the path and return here
	if(plan_for_footprint == true)
	{
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=fow_path.begin(); pose != fow_path.end(); ++pose)
		{
			geometry_msgs::Pose2D current_pose;
			current_pose.x = (pose->x * map_resolution) + map_origin.x;
			current_pose.y = (pose->y * map_resolution) + map_origin.y;
			current_pose.theta = pose->theta;
			path.push_back(current_pose);
		}
		return;
	}

	// ****************** III. Map the found fow path to the robot path ******************
	mapPath(room_map, path, fow_path, robot_to_fow_vector, map_resolution, map_origin, starting_position);
}

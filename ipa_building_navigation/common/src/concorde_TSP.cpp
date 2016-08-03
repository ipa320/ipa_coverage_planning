# include <ipa_building_navigation/concorde_TSP.h>

//Default constructor
ConcordeTSPSolver::ConcordeTSPSolver()
{

}

////Function to construct the distance matrix from the given points. See the definition in the header for the style of this matrix.
//void ConcordeTSPSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const int number_of_nodes,
//        const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution)
//{
//	//create the distance matrix with the right size
//	cv::Mat pathlengths(cv::Size(number_of_nodes, number_of_nodes), CV_64F);
//
//	for (int i = 0; i < points.size(); i++)
//	{
//		cv::Point current_center = points[i];
//		for (int p = 0; p < points.size(); p++)
//		{
//			if (p != i)
//			{
//				if (p > i) //only compute upper right triangle of matrix, rest is symmetrically added
//				{
//					cv::Point neighbor = points[p];
//					double length = pathplanner_.planPath(original_map, current_center, neighbor, downsampling_factor, robot_radius, map_resolution);
//					pathlengths.at<double>(i, p) = length;
//					pathlengths.at<double>(p, i) = length; //symmetrical-Matrix --> saves half the computation time
//				}
//			}
//			else
//			{
//				pathlengths.at<double>(i, p) = 0;
//			}
//		}
//	}
//
//	distance_matrix = pathlengths.clone();
//}

//This function generates a file with the current TSP in TSPlib format. This is neccessary because concorde needs this file
//as input to solve the TSP. See http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/ for documentation.
void ConcordeTSPSolver::writeToFile(const cv::Mat& pathlength_matrix)
{
	std::string path_for_saving_file = "TSPlib_file.txt";//ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSPlib_file.txt";
	std::ofstream saving_file(path_for_saving_file.c_str());
	if (saving_file.is_open()) {
		std::cout << "Starting to create the TSPlib file." << std::endl;
		//specify name of the Problem, Type (TSP = symmetrical TSP) and add a comment to the file. Name and Type are neccessary, comment is for better understanding when you open the file.
		saving_file << "NAME: ipa-building-navigation" << std::endl
				<< "TYPE: TSP" << std::endl
				<< "COMMENT: This is the TSPlib file for using concorde. See http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/ for documentation."
				<< std::endl;
		saving_file << "DIMENSION: " << pathlength_matrix.cols << std::endl; //Shows the Dimension of the problem --> the number of nodes (Neccessary).
		//Write the distance-matrix into the file as a full-matrix.
		saving_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
		saving_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
		saving_file << "EDGE_WEIGHT_SECTION" << std::endl;

		for (int row = 0; row < pathlength_matrix.rows; row++) {
			for (int col = 0; col < pathlength_matrix.cols; col++) {
				saving_file << " "
						<< (int) pathlength_matrix.at<double>(row, col);
			}
			saving_file << std::endl;
		}
		//shows the end of the file
		saving_file << "EOF";

		std::cout << "Created the TSPlib file." << std::endl;
		saving_file.close();
	}
	else
	{
		std::cout << "Saving file '" << path_for_saving_file << "' for concorde could not be opened." << std::endl;
	}
}

//This function opens the file which saves the output from the concorde solver and reads the saved order. The names of the
//nodes in the graph are stored as positions in the distance matrix in this case. The first integer in the file is the number
//of nodes of this problem, so this one is not necessary.
std::vector<int> ConcordeTSPSolver::readFromFile()
{
	std::string path_for_order_file = "TSP_order.txt";//ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSP_order.txt"; //get path to file
	std::ifstream reading_file(path_for_order_file.c_str()); //open file

	std::vector<int> order_vector; //vector that stores the calculated TSP order

	std::string line; //current line of the file

	int value; //current node in the line

	int line_counter = 0; //variable to make sure that the first line isn't stored

	if (reading_file.is_open()) {
		//get new line in the file
		while (getline(reading_file, line)) {
			std::istringstream iss(line);
			while (iss >> value) {
				if (line_counter > 0) //first line shows the number of nodes and is not relevant
						{
					order_vector.push_back(value); //put the current node in the last position of the order
				}
				line_counter++;
			}
		}
		reading_file.close();
	} else {
		std::cout << "TSP order file '" << path_for_order_file << "' could not be opened." << std::endl;
	}
	return order_vector;
}

//This function solves the given TSP using the systemcall to use the concorde TSP solver. This solver is applied from:
//		http://www.math.uwaterloo.ca/tsp/concorde.html
//First you have to build the Solver in any possible way and if you don't make a ros-package out of it you have to change
//the paths to the right ones. If it is a ros-package the ros::package::getpath() function will find the right path.
//The usage of the solver is: ./concorde [-see below-] [dat_file]
//Navigate to the build Solver and then ./TSP and type ./concorde -h for a short explanation.

//with a given distance matrix
std::vector<int> ConcordeTSPSolver::solveConcordeTSP(const cv::Mat& path_length_Matrix, const int start_Node)
{
	std::vector<int> unsorted_order;
	std::cout << "finding optimal order" << std::endl;
	std::cout << "number of nodes: " << path_length_Matrix.rows << " start node: " << start_Node << std::endl;
	if (path_length_Matrix.rows > 2) //check if the TSP has at least 3 nodes
	{
		//create the TSPlib file
		writeToFile(path_length_Matrix);

		//use concorde to find optimal tour
		std::string cmd = ros::package::getPath("libconcorde_tsp_solver")
				+ "/common/bin/concorde -o " + "$HOME/.ros/TSP_order.txt $HOME/.ros/TSPlib_file.txt";
				//+ ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSP_order.txt "
				//+ ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSPlib_file.txt";
		int result = system(cmd.c_str());
		assert(!result);

		//get order from saving file
		unsorted_order = readFromFile();
	}
	else
	{
		for(int node = 0; node < path_length_Matrix.rows; node++)
		{
			unsorted_order.push_back(node);
		}
	}
	std::cout << "found unsorted order" << std::endl;
	//sort the order with the start_Node at the beginning
	std::vector<int> sorted_order;
	unsigned int start_node_position;

	for (unsigned int i = 0; i < unsorted_order.size(); i++) //find position of the start node in the order
	{
		if (unsorted_order[i] == start_Node)
		{
			start_node_position = i;
		}
	}

	for (unsigned int i = start_node_position; i < unsorted_order.size(); i++) //sort the vector starting at start_Node
	{
		sorted_order.push_back(unsorted_order[i]);
	}
	for (unsigned int i = 0; i < start_node_position; i++)
	{
		sorted_order.push_back(unsorted_order[i]);
	}

	return sorted_order;
}

//compute the distance matrix and maybe return it
std::vector<int> ConcordeTSPSolver::solveConcordeTSP(
		const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution,
		const int start_Node, cv::Mat* distance_matrix) {
	//calculate the distance matrix
	cv::Mat distance_matrix_ref;
	if (distance_matrix != 0)
		distance_matrix_ref = *distance_matrix;
	DistanceMatrix::constructDistanceMatrix(distance_matrix_ref, original_map,
			points, downsampling_factor, robot_radius, map_resolution,
			pathplanner_);

	return (solveConcordeTSP(distance_matrix_ref, start_Node));
}

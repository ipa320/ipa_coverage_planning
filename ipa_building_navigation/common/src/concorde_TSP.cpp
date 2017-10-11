# include <ipa_building_navigation/concorde_TSP.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

//Default constructor
ConcordeTSPSolver::ConcordeTSPSolver()
: abort_computation_(false)
{

}

void ConcordeTSPSolver::distance_matrix_thread(DistanceMatrix& distance_matrix_computation, cv::Mat& distance_matrix,
		const cv::Mat& original_map, const std::vector<cv::Point>& points, double downsampling_factor,
		double robot_radius, double map_resolution, AStarPlanner& path_planner)
{
	distance_matrix_computation.constructDistanceMatrix(distance_matrix, original_map, points, downsampling_factor,
				robot_radius, map_resolution, pathplanner_);
}

void ConcordeTSPSolver::abortComputation()
{
	abort_computation_ = true;

	// kill concorde if running
	std::string pid_cmd = "pidof concorde > concorde_tsp_pid.txt";
	int pid_result = system(pid_cmd.c_str());
	std::ifstream pid_reader("concorde_tsp_pid.txt");
	int value = -1;
	std::string line;
	if (pid_reader.is_open())
	{
		while (getline(pid_reader, line))
		{
			std::istringstream iss(line);
			while (iss >> value)
			{
				std::cout << "PID of concorde: " << value << std::endl;
				std::stringstream ss;
				ss << "kill " << value;
				std::string kill_cmd = ss.str();
				int kill_result = system(kill_cmd.c_str());
				std::cout << "kill result: " << kill_result << std::endl;
			}
		}
		pid_reader.close();
		remove("concorde_tsp_pid.txt");
	}
}

//This function generates a file with the current TSP in TSPlib format. This is neccessary because concorde needs this file
//as input to solve the TSP. See http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/ for documentation.
void ConcordeTSPSolver::writeToFile(const cv::Mat& pathlength_matrix)
{
	std::string path_for_saving_file = "TSPlib_file.txt";//ros::package::getPath("libconcorde_tsp_solver") + "/common/files/TSPlib_file.txt";
	std::ofstream saving_file(path_for_saving_file.c_str());
	if (saving_file.is_open())
	{
		std::cout << "Starting to create the TSPlib file." << std::endl;
		//specify name of the Problem, Type (TSP = symmetrical TSP) and add a comment to the file. Name and Type are neccessary, comment is for better understanding when you open the file.
		// todo: maybe have this name and file names as a parameter
		saving_file << "NAME: ipa-building-navigation" << std::endl
				<< "TYPE: TSP" << std::endl
				<< "COMMENT: This is the TSPlib file for using concorde. See http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/ for documentation."
				<< std::endl;
		saving_file << "DIMENSION: " << pathlength_matrix.cols << std::endl; //Shows the Dimension of the problem --> the number of nodes (Neccessary).
		//Write the distance-matrix into the file as a full-matrix.
		saving_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
		saving_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
		saving_file << "EDGE_WEIGHT_SECTION" << std::endl;

		for (int row = 0; row < pathlength_matrix.rows; row++)
		{
			for (int col = 0; col < pathlength_matrix.cols; col++)
			{
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

	// clear results file
	std::string path_for_order_file = "TSP_order.txt";
	std::ofstream reading_file(path_for_order_file.c_str()); //open file
	if (reading_file.is_open())
	{
		reading_file << "";
		reading_file.close();
	}
	else
	{
		std::cout << "Could not clear results file '" << path_for_order_file << "'." << std::endl;
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

	if (reading_file.is_open())
	{
		//get new line in the file
		while (getline(reading_file, line))
		{
			std::istringstream iss(line);
			while (iss >> value)
			{
				if (line_counter > 0) //first line shows the number of nodes and is not relevant
					order_vector.push_back(value); //put the current node in the last position of the order
				line_counter++;
			}
		}
		reading_file.close();
	}
	else
	{
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
std::vector<int> ConcordeTSPSolver::solveConcordeTSP(const cv::Mat& path_length_matrix, const int start_Node)
{
	std::vector<int> unsorted_order, sorted_order;
	std::cout << "finding optimal order" << std::endl;
	std::cout << "number of nodes: " << path_length_matrix.rows << " start node: " << start_Node << std::endl;
	if (path_length_matrix.rows > 2) //check if the TSP has at least 3 nodes
	{
		//create the TSPlib file
		writeToFile(path_length_matrix);

		//use concorde to find optimal tour
		std::string bin_folder;
		while (bin_folder.length()==0)
		{
			try
			{
				std::string cmd = "rospack libs-only-L libconcorde_tsp_solver > temp_libconcorde_path.txt";
				int result = system(cmd.c_str());
				std::ifstream file("temp_libconcorde_path.txt", std::ifstream::in);
				if (file.is_open())
				{
					file >> bin_folder;
					file.close();
				}
				std::cout << "bin_folder: " << bin_folder << std::endl;
				remove("temp_libconcorde_path.txt");
				//bin_folder = ros::package::command("libs-only-L libconcorde_tsp_solver"); // this command crashes sometimes
				//bin_folder.erase(std::remove(bin_folder.begin(), bin_folder.end(), '\n'));
			}
			catch (...)
			{
				std::cout << "ConcordeTSPSolver::solveConcordeTSP: ERROR: ros::package::command('libs-only-L libconcorde_tsp_solver') failed. Trying again." << std::endl;
			}
		}
		std::string cmd = bin_folder + "/libconcorde_tsp_solver/concorde -o " + "$HOME/.ros/TSP_order.txt $HOME/.ros/TSPlib_file.txt";
		if (abort_computation_==true)
			return sorted_order;
		int result = system(cmd.c_str());
		if (abort_computation_==true)
			return sorted_order;
		assert(!result);

		//get order from saving file
		unsorted_order = readFromFile();
	}
	else
	{
		for(int node = 0; node < path_length_matrix.rows; node++)
		{
			unsorted_order.push_back(node);
		}
	}
	std::cout << "finished TSP" << std::endl;

	// if there is an error, just set unsorted order to 1, 2, 3, ...
	if (unsorted_order.size() != path_length_matrix.rows)
	{
		std::cout << "ConcordeTSPSolver::solveConcordeTSP: Warning: Optimized order invalid, taking standard order 1, 2, 3, ..." << std::endl;
		unsorted_order.clear();
		unsorted_order.resize(path_length_matrix.rows);
		for (int i=0; i<path_length_matrix.rows; ++i)
			unsorted_order[i] = i;
	}

	//sort the order with the start_Node at the beginning
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
std::vector<int> ConcordeTSPSolver::solveConcordeTSP(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, const int start_Node, cv::Mat* distance_matrix)
{
	//calculate the distance matrix
	cv::Mat distance_matrix_ref;
	if (distance_matrix != 0)
		distance_matrix_ref = *distance_matrix;
	DistanceMatrix distance_matrix_computation;
	boost::thread t(boost::bind(&ConcordeTSPSolver::distance_matrix_thread, this, boost::ref(distance_matrix_computation),
			boost::ref(distance_matrix_ref), boost::cref(original_map), boost::cref(points), downsampling_factor,
			robot_radius, map_resolution, boost::ref(pathplanner_)));
	bool finished = false;
	while (finished==false)
	{
		if (abort_computation_==true)
			distance_matrix_computation.abortComputation();
		finished = t.try_join_for(boost::chrono::milliseconds(10));
	}

//	distance_matrix_computation.constructDistanceMatrix(distance_matrix_ref, original_map, points, downsampling_factor,
//			robot_radius, map_resolution, pathplanner_);

	if (abort_computation_==true)
	{
		std::vector<int> sorted_order;
		return sorted_order;
	}

	// todo: check whether distance matrix contains infinite path lenghts and if this is true, create a new distance matrix with maximum size clique of reachable points
	// then solve TSP and re-index points to original indices
	// and do not forget to copy fix to ipa_building_navigation

	return (solveConcordeTSP(distance_matrix_ref, start_Node));
}

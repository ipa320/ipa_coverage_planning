# include <ipa_building_navigation/concorde_TSP.h>

concordeTSPSolver::concordeTSPSolver()
{

}

void concordeTSPSolver::writeToFile(const cv::Mat& pathlength_matrix)
{
	std::string path_for_saving_file = ros::package::getPath("concorde_tsp_solver") + "/common/files/TSPlib_file.txt";
	std::ofstream saving_file(path_for_saving_file.c_str());
	if (saving_file.is_open())
	{
		std::cout << "starting saving pathlength parameters" << std::endl;
		saving_file << "NAME: ipa-building-navigation" << std::endl << "TYPE: TSP" << std::endl << "COMMENT: This is the TSPlib file for using concorde"
		        << std::endl;
		saving_file << "DIMENSION: " << pathlength_matrix.cols << std::endl;
		saving_file << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
		saving_file << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
		saving_file << "EDGE_WEIGHT_SECTION" << std::endl;

		for (int row = 0; row < pathlength_matrix.rows; row++)
		{
			for (int col = 0; col < pathlength_matrix.cols; col++)
			{
				saving_file << " " << (int) pathlength_matrix.at<double>(row, col);
			}
			saving_file << std::endl;
		}

		saving_file << "EOF";

		std::cout << "finished saving" << std::endl;
		saving_file.close();
	}
	else
	{
		std::cout << "nicht geöffnet1" << std::endl;
	}
}

std::vector<int> concordeTSPSolver::readFromFile()
{
	std::string path_for_order_file = ros::package::getPath("concorde_tsp_solver") + "/common/files/TSP_order.txt";
	std::ifstream reading_file(path_for_order_file.c_str());

	std::vector<int> order_vector;

	std::string line;

	int value;

	int line_counter = 0;

	if (reading_file.is_open())
	{
		while (getline(reading_file, line))
		{
			std::istringstream iss(line);
			while (iss >> value)
			{
				if(line_counter > 0)
				{
					order_vector.push_back(value);
				}
				line_counter++;
			}
		}
		reading_file.close();
	}
	else
	{
		std::cout << "nicht geöffnet" << std::endl;
	}
	return order_vector;
}

std::vector<int> concordeTSPSolver::solveConcordeTSP(const cv::Mat& path_length_Matrix, const int start_Node)
{
	writeToFile(path_length_Matrix);

	std::string cmd = ros::package::getPath("concorde_tsp_solver") + "/common/bin/concorde -o" + ros::package::getPath("concorde_tsp_solver") + "/common/files/TSP_order.txt " + ros::package::getPath("concorde_tsp_solver") + "/common/files/TSPlib_file.txt";
	int result = system(cmd.c_str());
	assert(!result);

	std::vector<int> order = readFromFile();

	for(int i = 0; i < order.size(); i++)
	{
		std::cout << order[i] << std::endl;
	}
	std::cout << std::endl;
	return order;
}

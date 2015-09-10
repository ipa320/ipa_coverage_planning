#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <time.h>
#include <sys/time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>
#include <ipa_building_navigation/FindRoomSequenceWithCheckpointsAction.h>
#include <ipa_building_navigation/A_star_pathplanner.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>


struct EvaluationConfig
{
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation
											// 1 = morphological segmentation
											// 2 = distance segmentation
											// 3 = Voronoi segmentation
											// 4 = semantic segmentation
	double max_clique_path_length_;		// max A* path length between two rooms that are assigned to the same clique, in [m]
	int sequence_planning_method_;		// Method for sequence planning
											// 1 = drag trolley if next room is too far away
											// 2 = calculate roomgroups and a trolleyposition for each of it
	int tsp_solver_;					// TSP solver that is used
											// 1 = Nearest Neighbor
											// 2 = Genetic solver
											// 3 = Concorde solver

	EvaluationConfig()
	{
		room_segmentation_algorithm_ = 1;
		max_clique_path_length_ = 12.0;
		sequence_planning_method_ = 2;
		tsp_solver_ = 3;
	}

	EvaluationConfig(const int room_segmentation_algorithm, const double max_clique_path_length, const int sequence_planning_method, const int tsp_solver)
	{
		room_segmentation_algorithm_ = room_segmentation_algorithm;
		max_clique_path_length_ = max_clique_path_length;
		sequence_planning_method_ = sequence_planning_method;
		tsp_solver_ = tsp_solver;
	}

	std::string generateUpperConfigurationFolderString() const
	{
		std::stringstream ss;
		ss << "plmth" << sequence_planning_method_ << "mcl" << std::setprecision(4) <<  max_clique_path_length_;

		return ss.str();
	}

	std::string generateLowerConfigurationFolderString() const
	{
		std::stringstream ss;
		ss << "seg" << room_segmentation_algorithm_ << "tsp" << tsp_solver_;
		return ss.str();
	}

	std::string getConfigurationString() const
	{
		std::stringstream ss;
		ss << "\n==================================================================================================\n" <<
				"Configuration " << generateUpperConfigurationFolderString() << generateLowerConfigurationFolderString() << "\n" <<
				"==================================================================================================\n" <<
				"room_segmentation_algorithm: " << roomSegmentationAlgorithmToString() << " (" << room_segmentation_algorithm_ << ")\n" <<
				"max_clique_path_length [m]: " << max_clique_path_length_ <<
				"\n" << std::endl;

		return ss.str();
	}

	std::string roomSegmentationAlgorithmToString() const
	{
		std::string s = "";
		if (room_segmentation_algorithm_ == 1)
			s = "morphological segmentation";
		else if (room_segmentation_algorithm_ == 2)
			s = "distance segmentation";
		else if (room_segmentation_algorithm_ == 3)
			s = "Voronoi segmentation";
		else if (room_segmentation_algorithm_ == 4)
			s = "semantic segmentation";
		return s;
	}
};

struct EvaluationData
{
	std::string map_name_;		// without file type
	cv::Mat floor_plan_;
	double map_downsampling_factor_;
	float map_resolution_;
	geometry_msgs::Pose map_origin_;
	geometry_msgs::Pose robot_start_position_;
	double robot_radius_;
	std::vector< cv::Point > trash_bin_locations_;

	EvaluationData()
	{
		map_name_ = "";
		floor_plan_ = cv::Mat();
		map_downsampling_factor_ = 0.25;
		map_resolution_ = 0.05;
		robot_radius_ = 0.3;
	}

	EvaluationData(const std::string& map_name, const cv::Mat& floor_plan, const float map_resolution, const double map_downsampling_factor,
			const double robot_radius, const std::vector< cv::Point >& trash_bin_locations)
	{
		map_name_ = map_name;
		floor_plan_ = floor_plan;
		map_downsampling_factor_ = map_downsampling_factor;
		map_resolution_ = map_resolution;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		robot_radius_ = robot_radius;
		trash_bin_locations_ = trash_bin_locations;
		cv::Mat map_eroded;
		cv::erode(floor_plan_, map_eroded, cv::Mat(), cv::Point(-1,-1), robot_radius_/map_resolution_+2);
		cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image
		bool robot_start_coordinate_set = false;
		for (int v=0; v<map_eroded.rows && robot_start_coordinate_set==false; ++v)
			for (int u=0; u<map_eroded.cols && robot_start_coordinate_set==false; ++u)
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<uchar>(v,u) > 15)
				{
					robot_start_position_.position.x = u*map_resolution_ + map_origin_.position.x;
					robot_start_position_.position.y = v*map_resolution_ + map_origin_.position.y;
					robot_start_coordinate_set = true;
				}
	}
};

class Evaluation
{
public:

	// to segment the map only once if it is only one and one segmentation algorithm
	ipa_room_segmentation::MapSegmentationResultConstPtr result_seg_morph;
	bool segmented_morph;
	ipa_room_segmentation::MapSegmentationResultConstPtr result_seg_dist;
	bool segmented_dist;
	ipa_room_segmentation::MapSegmentationResultConstPtr result_seg_vor;
	bool segmented_vor;
	ipa_room_segmentation::MapSegmentationResultConstPtr result_seg_semant;
	bool segmented_semant;

	Evaluation(const std::string& test_map_path, const std::string& data_storage_path, const double robot_radius)
	: robot_radius_(robot_radius)
	{
		segmented_morph = false;
		segmented_dist = false;
		segmented_vor = false;
		segmented_semant = false;
		// prepare relevant floor map data
		std::vector<std::string> map_names;
//		map_names.push_back("lab_ipa"); // done
//		map_names.push_back("Freiburg52_scan"); //done
//		map_names.push_back("Freiburg79_scan"); //done
//		map_names.push_back("lab_c_scan"); //done
//		map_names.push_back("lab_b_scan"); //done
//		map_names.push_back("lab_d_scan"); //done
//		map_names.push_back("lab_intel"); //done
//		map_names.push_back("Freiburg101_scan"); //done
//		map_names.push_back("lab_a_scan"); //done

//		map_names.push_back("lab_f_scan"); //
		map_names.push_back("NLB"); //cl16seg2tsp1+2 done

		//with obstacles:
//		"Freiburg52_scan_furnitures_trashbins"


		for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
		{
			std::string image_filename = test_map_path + map_names[image_index] + ".png";// + "_furnitures_trashbins.png";
			std::cout << "loading image: " << image_filename << std::endl;
			cv::Mat map = cv::imread(image_filename.c_str(), 0);
			//make non-white pixels black
			for (int y = 0; y < map.rows; y++)
			{
				for (int x = 0; x < map.cols; x++)
				{
					if (map.at<unsigned char>(y, x) > 250)
					{
						map.at<unsigned char>(y, x) = 255;
					}
					else //if (map.at<unsigned char>(y, x) != 255)
					{
						map.at<unsigned char>(y, x) = 0;
					}
				}
			}

			// read in trash bin locations
			image_filename = test_map_path + map_names[image_index] + "_trashbins.png";
			cv::Mat temp = cv::imread(image_filename.c_str());
			cv::Vec3b blue(255, 0, 0);
			double map_resoultion_for_evaluation = 0.05;
			int number_of_erosions = (robot_radius_ / map_resoultion_for_evaluation);
			cv::Mat eroded_map;
			cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1,-1), number_of_erosions);
			std::vector< cv::Point > trash_bin_locations;
			for (int y = 0; y < temp.rows; y++)
			{
				for (int x = 0; x < temp.cols; x++)
				{
					//find blue drawn trash bins and check if they are reachable (if one is not reachable the astar planner will give a extremely large path)
					if (temp.at<cv::Vec3b>(y, x) == blue && eroded_map.at<unsigned char>(y, x) != 0)
					{
						trash_bin_locations.push_back(cv::Point(x,y));
						std::cout << "trash: " << x << ", " << y << std::endl;
					}
				}
			}
			//give occupied memory free
//			cv::imshow("given map", eroded_map);
//			cv::waitKey();
			temp.release();
			eroded_map.release();

//			+"_furnitures_trashbins"
			evaluation_data_.push_back(EvaluationData(map_names[image_index], map, map_resoultion_for_evaluation, 0.25, robot_radius_, trash_bin_locations));
		}

		// set configurations
		std::vector< EvaluationConfig > evaluation_configurations;
		setConfigurations(evaluation_configurations);

		// do the evaluation
		for (size_t i=0; i<evaluation_data_.size(); ++i)
		{
			if (evaluateAllConfigs(evaluation_configurations, evaluation_data_[i], data_storage_path) == false)
				return;
			//reset booleans to segment the new map
			segmented_morph = false;
			segmented_dist = false;
			segmented_vor = false;
			segmented_semant = false;
		}

		//read the saved results
//		std::vector<std::vector<double> > map_pathlengths(evaluation_data_.size());
//		std::vector<std::vector<double> > map_cleaning_times(evaluation_data_.size());
//		std::vector<std::vector<double> > sequence_solver_times(evaluation_data_.size());
//		for (size_t i=0; i<evaluation_configurations.size(); ++i)
//		{
//			readFromLocalFiles(evaluation_configurations[i], map_names, data_storage_path, map_pathlengths, map_cleaning_times, sequence_solver_times);
//		}
//
//		std::cout << std::endl << "Pathlengths: " << std::endl;
//		for(size_t i = 0; i < map_pathlengths.size(); ++i)
//		{
//			for(size_t j = 0; j < map_pathlengths[i].size(); ++j)
//			{
//				std::cout << map_pathlengths[i][j] << std::endl;
//			}
//			std::cout << std::endl;
//		}
//
//		std::cout << std::endl << "Cleaningtimes: " << std::endl;
//		for(size_t i = 0; i < map_cleaning_times.size(); ++i)
//		{
//			for(size_t j = 0; j < map_cleaning_times[i].size(); ++j)
//			{
//				std::cout << map_cleaning_times[i][j] << std::endl;
//			}
//			std::cout << std::endl;
//		}
//
//		std::cout << std::endl << "Sequencer times: " << std::endl;
//		for(size_t i = 0; i < sequence_solver_times.size(); ++i)
//		{
//			for(size_t j = 0; j < sequence_solver_times[i].size(); ++j)
//			{
//				std::cout << sequence_solver_times[i][j] << std::endl;
//			}
//			std::cout << std::endl;
//		}
//
//		//write the calculated values to global saving files
//		writeToGlobalFiles(map_names, data_storage_path, map_pathlengths, map_cleaning_times, sequence_solver_times);
	}

	//function to read the saved results out of the files
	void readFromLocalFiles(const EvaluationConfig& evaluation_configuration, const std::vector<std::string>& map_names, const std::string& data_storage_path,
			std::vector<std::vector<double> >& map_pathlengths, std::vector<std::vector<double> >& map_cleaning_times,
			std::vector<std::vector<double> >& sequence_solver_times)
	{
		const std::string upper_folder_name = evaluation_configuration.generateUpperConfigurationFolderString() + "/";
		const std::string path = data_storage_path + upper_folder_name;

		const std::string lower_folder_name = evaluation_configuration.generateLowerConfigurationFolderString() + "/";
		const std::string lower_path = path + lower_folder_name;


		for(size_t map_index = 0; map_index < map_names.size(); ++map_index)
		{
			std::string log_filename = lower_path + map_names[map_index] + "_results.txt";

			std::string line;
			double value, i;
			std::ifstream reading_file(log_filename.c_str());
			if (reading_file.is_open())
			{
				while (getline(reading_file, line))
				{
					std::istringstream iss(line);
					i = 1;
					while (iss >> value)
					{
						if(i == 6) //save pathlength
							map_pathlengths[map_index].push_back(value);
						if(i == 7) //save cleaningtimes
							map_cleaning_times[map_index].push_back(value);
						if(i == 9) //save sequence time
							sequence_solver_times[map_index].push_back(value);
						++i;
					}
				}
				reading_file.close();
			}
		}
	}

	void writeToGlobalFiles(const std::vector<std::string>& map_names, const std::string& data_storage_path, const std::vector<std::vector<double> >& map_pathlengths,
			const std::vector<std::vector<double> >& map_cleaning_times, const std::vector<std::vector<double> >& sequence_solver_times)
	{

		//save the saved parameters in a saving file for each map

		//define the storage path for each planning method
		std::stringstream ss;
		ss << "global";
		const std::string upper_folder_name = ss.str() + "/";
		const std::string path = data_storage_path + upper_folder_name;
		const std::string upper_command = "mkdir -p " + path;
		int return_value = system(upper_command.c_str());

		// outputs to log file
		std::stringstream distance_output[map_names.size()];
		std::stringstream cleaning_time_output[map_names.size()];
		std::stringstream sequence_time_output[map_names.size()];


			//define values to show how much different algorithms has been implemented
			double max_clique_path_length = 4.0;
			int number_of_cliquelenghts = 8;
			int number_of_segmentation_algorithms = 4;
			int number_of_tsp_solver = 3;
			for (int max_length = 0; max_length < number_of_cliquelenghts; ++max_length)
			{
				max_clique_path_length += 2.0;

				int start = number_of_segmentation_algorithms * number_of_tsp_solver * max_length;

				for(size_t map = 0; map < map_names.size(); ++map)
				{
					//show which maximal clique pathlength this is
					distance_output[map] << max_clique_path_length;
					cleaning_time_output[map] << max_clique_path_length;
					sequence_time_output[map] << max_clique_path_length;
					// header
					for(size_t segmentation = 0; segmentation < number_of_segmentation_algorithms; ++segmentation)
					{
						if(segmentation == 0)
						{
							distance_output[map] << " & morph";
							cleaning_time_output[map] << " & morph";
							sequence_time_output[map] << " & morph";
						}
						if(segmentation == 1)
						{
							distance_output[map] << " & dist";
							cleaning_time_output[map] << " & dist";
							sequence_time_output[map] << " & dist";
						}
						if(segmentation == 2)
						{
							distance_output[map] << " & voro";
							cleaning_time_output[map] << " & voro";
							sequence_time_output[map] << " & voro";
						}
						if(segmentation == 3)
						{
							distance_output[map] << " & semant";
							cleaning_time_output[map] << " & semant";
							sequence_time_output[map] << " & semant";
						}

						for(int sequence_planning_method = 0; sequence_planning_method < 2; ++sequence_planning_method)
						{
							int plmth_start = start + number_of_segmentation_algorithms * number_of_tsp_solver * number_of_cliquelenghts * sequence_planning_method;
							int index = plmth_start + number_of_tsp_solver * segmentation;
							for(size_t reading_index = index; reading_index < index+number_of_tsp_solver; ++reading_index)
							{
								distance_output[map] << " & " << map_pathlengths[map][reading_index] ;
								cleaning_time_output[map] << " & " << map_cleaning_times[map][reading_index];
								sequence_time_output[map] << " & " << sequence_solver_times[map][reading_index];
							}
						}

						distance_output[map] << "\\\\" << std::endl;
						cleaning_time_output[map] << "\\\\" << std::endl;
						sequence_time_output[map] << "\\\\" << std::endl;

						if(segmentation < 3)
						{
							distance_output[map] << "\t\\cline{2-8}" << std::endl;
							cleaning_time_output[map] << "\t\\cline{2-8}" << std::endl;
							sequence_time_output[map] << "\t\\cline{2-8}" << std::endl;
						}
					}
					distance_output[map] << "\\hline" << std::endl << "\t\\multicolumn{8}{|c|}{}\\\\ [\\rowheight]" << std::endl << "\\hline" << std::endl;
					cleaning_time_output[map] << "\\hline" << std::endl << "\t\\multicolumn{8}{|c|}{}\\\\ [\\rowheight]" << std::endl << "\\hline" << std::endl;
					sequence_time_output[map] << "\\hline" << std::endl << "\t\\multicolumn{8}{|c|}{}\\\\ [\\rowheight]" << std::endl << "\\hline" << std::endl;
				}
			}
		//write saved parameters to global file
		for(size_t map = 0; map < map_names.size(); ++map)
		{
			std::string pathlength_filename = path + map_names[map] + "_pathlengths.txt";
			std::ofstream pathlength_file(pathlength_filename.c_str(), std::ios::out);
			if (pathlength_file.is_open()==true)
				pathlength_file << distance_output[map].str();
			pathlength_file.close();

			std::string cleaningtime_filename = path + map_names[map] + "_cleaningtimes.txt";
			std::ofstream cleaningtime_file(cleaningtime_filename.c_str(), std::ios::out);
			if (cleaningtime_file.is_open()==true)
				cleaningtime_file << cleaning_time_output[map].str();
			cleaningtime_file.close();

			std::string sequencetime_filename = path + map_names[map] + "_sequencetimes.txt";
			std::ofstream sequencetime_file(sequencetime_filename.c_str(), std::ios::out);
			if (sequencetime_file.is_open()==true)
				sequencetime_file << sequence_time_output[map].str();
			sequencetime_file.close();

		}
	}

	void setConfigurations(std::vector< EvaluationConfig >& evaluation_configurations)
	{
		evaluation_configurations.clear();
		for(int sequence_planning_method = 2; sequence_planning_method <= 2; ++sequence_planning_method)
		{
			for (double max_clique_path_length = 6.; max_clique_path_length <= 20.; max_clique_path_length += 2.0)
			{
				for (int room_segmentation_algorithm=1; room_segmentation_algorithm<=3; ++room_segmentation_algorithm)
				{
					if(room_segmentation_algorithm == 4)
					{
						for(int tsp_solver = 1; tsp_solver <= 3; ++tsp_solver)
								evaluation_configurations.push_back(EvaluationConfig(room_segmentation_algorithm, max_clique_path_length, sequence_planning_method, tsp_solver));
					}
					else
					{
						for(int tsp_solver = 3; tsp_solver <= 3; ++tsp_solver)
							evaluation_configurations.push_back(EvaluationConfig(room_segmentation_algorithm, max_clique_path_length, sequence_planning_method, tsp_solver));
					}
				}
			}
		}
	}

	bool evaluateAllConfigs(const std::vector<EvaluationConfig>& evaluation_configuration_vector, const EvaluationData& evaluation_data,
			const std::string& data_storage_path)
	{
		//go trough each configuration for the given map
		for(size_t config = 0; config < evaluation_configuration_vector.size(); ++config)
		{
			// prepare folders for storing results
			const std::string upper_folder_name = evaluation_configuration_vector[config].generateUpperConfigurationFolderString() + "/";
			const std::string path = data_storage_path + upper_folder_name;
			const std::string upper_command = "mkdir -p " + path;
			int return_value = system(upper_command.c_str());

			const std::string lower_folder_name = evaluation_configuration_vector[config].generateLowerConfigurationFolderString() + "/";
			const std::string lower_path = path + lower_folder_name;
			const std::string lower_command = "mkdir -p " + lower_path;
			return_value = system(lower_command.c_str());

			std::cout << "Current Configuration:" << std::endl << "map: " << evaluation_data.map_name_ << " segmentation algorithm: "
				<< evaluation_configuration_vector[config].room_segmentation_algorithm_
				<< " Maximal Cliquelength: " << evaluation_configuration_vector[config].max_clique_path_length_ << " planning method: "
				<< evaluation_configuration_vector[config].sequence_planning_method_ << " TSP solver: " << evaluation_configuration_vector[config].tsp_solver_ << std::endl;

			AStarPlanner planner;
			//variables for time measurement
			struct timespec t0, t1, t2, t3;

			// 1. retrieve segmentation and check if the map has already been segmented
			clock_gettime(CLOCK_MONOTONIC,  &t0); //set time stamp before the segmentation
			ipa_room_segmentation::MapSegmentationResultConstPtr result_seg;
			if(evaluation_configuration_vector[config].room_segmentation_algorithm_ == 1)
			{
				if(segmented_morph == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_morph) == false)
						return false;
					segmented_morph = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_morph;
			}
			else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 2)
			{
				if(segmented_dist == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_dist) == false)
						return false;
					segmented_dist = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_dist;
			}
			else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 3)
			{
				if(segmented_vor == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vor) == false)
						return false;
					segmented_vor = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_vor;
			}
			else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 4)
			{
				if(segmented_semant == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_semant) == false)
						return false;
					segmented_semant = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_semant;
			}
			clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the segmentation

			//check for accesability of the room centers from start position
			cv::Mat downsampled_map;
			planner.downsampleMap(evaluation_data.floor_plan_, downsampled_map, evaluation_data.map_downsampling_factor_, evaluation_data.robot_radius_, evaluation_data.map_resolution_);
			cv::Point robot_start_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
											(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);

			// get the reachable room centers as cv::Point
			std::cout << "starting to check accessibility of rooms" << std::endl;
			std::vector<cv::Point> room_centers;
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				cv::Point current_center(result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
				if(planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_start_position, evaluation_data.map_downsampling_factor_*current_center, 1., 0., evaluation_data.map_resolution_) < 9000)
					room_centers.push_back(current_center);
			}

			// 2. solve sequence problem
			clock_gettime(CLOCK_MONOTONIC,  &t2); //set time stamp before the sequence planning
			ipa_building_navigation::FindRoomSequenceWithCheckpointsResultConstPtr result_seq;
			if (computeRoomSequence(evaluation_data, evaluation_configuration_vector[config], room_centers, result_seq) == false)
				return false;
			clock_gettime(CLOCK_MONOTONIC,  &t3); //set time stamp after the sequence planning

			// 3. assign trash bins to rooms of the respective segmentation
			std::vector< std::vector<cv::Point> > room_trash_bins(result_seg->room_information_in_pixel.size());
			// converting the map msg in cv format
			cv_bridge::CvImagePtr cv_ptr_obj;
			cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_obj->image;
			for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
			{
				int room_index = segmented_map.at<int>(evaluation_data.trash_bin_locations_[i])-1; //labeling started from 1 --> 0 is for obstacles
				room_trash_bins[room_index].push_back(evaluation_data.trash_bin_locations_[i]);
			}

			// 4. do the movements
			double path_length_robot = 0.;
			double path_length_trolley = 0.;
			double current_pathlength;
//			const double max_clique_path_length_in_pixel = evaluation_configuration.max_clique_path_length_ / evaluation_data.map_resolution_;
			cv::Point robot_position = robot_start_position;
			cv::Point trolley_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
					(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);

			for (size_t clique_index = 0; clique_index<result_seq->checkpoints.size(); ++clique_index)
			{
				std::cout << "cleaning new clique" << std::endl;
				// move trolley
				//		i) robot to trolley
				current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
				if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
					current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, robot_position, trolley_position, 1., 0., evaluation_data.map_resolution_);
				path_length_robot += current_pathlength;
				// 		ii) trolley to next trolley goal
				cv::Point trolley_goal_position(result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.x, result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.y);
				current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*trolley_position, evaluation_data.map_downsampling_factor_*trolley_goal_position, 1., 0., evaluation_data.map_resolution_);
				if(current_pathlength > 9000)  //if no path can be found try with the not downsampled map
					current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, trolley_position, trolley_goal_position, 1., 0., evaluation_data.map_resolution_);
				path_length_trolley += current_pathlength;
				trolley_position = trolley_goal_position;
				robot_position = trolley_goal_position;
				std::cout << "moved trolley" << std::endl;

				// move robot to rooms
				for(size_t room = 0; room < result_seq->checkpoints[clique_index].room_indices.size(); ++room)
				{
					// get next room in sequence
					const int room_index = result_seq->checkpoints[clique_index].room_indices[room];
					cv::Point current_roomcenter = room_centers[room_index];
					// drive to next room
					current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*current_roomcenter, 1., 0., evaluation_data.map_resolution_);
					if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
						current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, robot_position, current_roomcenter, 1., 0., evaluation_data.map_resolution_);
					path_length_robot += current_pathlength;
					robot_position = current_roomcenter;
					// clear all trash bins: go to trash bin, go back to trolley to empty trash and then drive back to trash bin
					std::cout << "starting to clean the trash bins" << std::endl;
					for (size_t t=0; t<room_trash_bins[room_index].size(); ++t)
					{
						// drive robot to trash bin
						current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*room_trash_bins[room_index][t], 1., 0., evaluation_data.map_resolution_);
						if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
							current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, robot_position, room_trash_bins[room_index][t], 1., 0., evaluation_data.map_resolution_);
						path_length_robot += current_pathlength;
						// drive trash bin to trolley and back
						current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*room_trash_bins[room_index][t], evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
						if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
							current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, room_trash_bins[room_index][t], trolley_position, 1., 0., evaluation_data.map_resolution_);
						path_length_robot += 2. * current_pathlength;
						robot_position = room_trash_bins[room_index][t];
					}
				}
				std::cout << "cleaned all rooms and trash bins" << std::endl;
			}
			// finally go back to trolley
			current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*robot_position, evaluation_data.map_downsampling_factor_*trolley_position, 1., 0., evaluation_data.map_resolution_);
			if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
				current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, robot_position, trolley_position, 1., 0., evaluation_data.map_resolution_);
			path_length_robot += current_pathlength;
			// and back to start position
			current_pathlength = planner.planPath(downsampled_map, evaluation_data.map_downsampling_factor_*trolley_position, evaluation_data.map_downsampling_factor_*robot_start_position, 1., 0., evaluation_data.map_resolution_);
			if(current_pathlength > 9000) //if no path can be found try with the not downsampled map
				current_pathlength = evaluation_data.map_downsampling_factor_ * planner.planPath(evaluation_data.floor_plan_, trolley_position, robot_start_position, 1., 0., evaluation_data.map_resolution_);
			path_length_trolley += current_pathlength;

			// evaluation
			double path_length_robot_in_meter = (path_length_robot / evaluation_data.map_downsampling_factor_) * evaluation_data.map_resolution_;
			double path_length_trolley_in_meter = (path_length_trolley / evaluation_data.map_downsampling_factor_) * evaluation_data.map_resolution_;
			double path_length_total_in_meter = path_length_robot_in_meter + path_length_trolley_in_meter;
			double robot_speed_without_trolley = 0.3;		// [m/s]
			double robot_speed_with_trolley = 0.2;			// [m/s]
			double time_for_trashbin_manipulation = 150;	// [s], without driving
			double time_for_trolley_manipulation = 90;		// [s], without driving
			double time = path_length_robot_in_meter / robot_speed_without_trolley
						+ path_length_trolley_in_meter / robot_speed_with_trolley
						+ time_for_trashbin_manipulation * evaluation_data.trash_bin_locations_.size()
						+ time_for_trolley_manipulation * (result_seq->checkpoints.size()+1);

			//get the runtimes for the segmentation and the sequence planner
			double segmentation_time = (t1.tv_sec - t0.tv_sec) + (double) (t1.tv_nsec - t0.tv_nsec) * 1e-9;
			double sequence_time = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;

			// write log file
			std::stringstream output;
			// header
			output << evaluation_configuration_vector[config].getConfigurationString();
			output << "robot speed" << "\t"<< "trash bin handling time" << "\t" << "number of trash bins" << "\t"
					<< "trolley handling time" << "\t" << "trolley handling count" << "\t" << "pathlength [m]" << "\t"
					<< "cleaning time [s]" << "\t" << "calculation time segmentation[s]" << "\t" << "calculation time sequencer[s]" << "\t" << std::endl;
			output << robot_speed_without_trolley << "\t" << time_for_trashbin_manipulation << "\t" << evaluation_data.trash_bin_locations_.size() << "\t"
					<< time_for_trolley_manipulation << "\t" << (result_seq->checkpoints.size()+1) << "\t" << path_length_total_in_meter << "\t"
					<< time << "\t" << segmentation_time << "\t" << sequence_time;

			std::string log_filename = lower_path + evaluation_data.map_name_ + "_results.txt";
			std::ofstream file(log_filename.c_str(), std::ios::out);
			if (file.is_open()==true)
				file << output.str();
			file.close();

			// images: segmented_map, sequence_map
			std::string segmented_map_filename = lower_path + evaluation_data.map_name_ + "_segmented.png";
			cv::Mat colour_segmented_map = segmented_map.clone();
			colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
			cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
			for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
			{
				//choose random colour for each room
				int blue = (rand() % 250) + 1;
				int green = (rand() % 250) + 1;
				int red = (rand() % 250) + 1;
				for(size_t u = 0; u < segmented_map.rows; ++u)
				{
					for(size_t v = 0; v < segmented_map.cols; ++v)
					{
						if(segmented_map.at<int>(u,v) == i)
						{
							colour_segmented_map.at<cv::Vec3b>(u,v)[0] = blue;
							colour_segmented_map.at<cv::Vec3b>(u,v)[1] = green;
							colour_segmented_map.at<cv::Vec3b>(u,v)[2] = red;
						}
					}
				}
			}
			//draw the roomcenters into the map
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				cv::Point current_center (result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
				cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,0), CV_FILLED);
			}
			//colour image in unique colour to show the segmentation
			cv::imwrite(segmented_map_filename.c_str(), colour_segmented_map);

			std::string sequence_map_filename = lower_path + evaluation_data.map_name_ + "_sequence.png";
			cv_bridge::CvImagePtr cv_ptr_seq;
			cv_ptr_seq = cv_bridge::toCvCopy(result_seq->sequence_map, sensor_msgs::image_encodings::BGR8);
			cv::Mat sequence_map = cv_ptr_seq->image;
			cv::imwrite(sequence_map_filename.c_str(), sequence_map);
		}

		return true;
	}

	bool segmentFloorPlan(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			ipa_room_segmentation::MapSegmentationResultConstPtr& result_seg)
	{
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = evaluation_data.floor_plan_;
		cv_image.toImageMsg(map_msg);
		actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> ac_seg("/room_segmentation/room_segmentation_server", true);
		ROS_INFO("Waiting for action server '/room_segmentation/room_segmentation_server' to start.");
		ac_seg.waitForServer(); // wait for the action server to start, will wait for infinite time

		std::cout << "Action server started, sending goal_seg." << std::endl;
		// send a goal to the action
		ipa_room_segmentation::MapSegmentationGoal goal_seg;
		goal_seg.input_map = map_msg;
		goal_seg.map_origin = evaluation_data.map_origin_;
		goal_seg.map_resolution = evaluation_data.map_resolution_;
		goal_seg.return_format_in_meter = false;
		goal_seg.return_format_in_pixel = true;
		goal_seg.room_segmentation_algorithm = evaluation_configuration.room_segmentation_algorithm_;
		ac_seg.sendGoal(goal_seg);

		//wait for the action to return
		bool finished_before_timeout = ac_seg.waitForResult();//ros::Duration(900.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room segmentation.");
			return false;
		}
		result_seg = ac_seg.getResult();
		std::cout << "Finished segmentation successfully!" << std::endl;

		return true;
	}

	bool computeRoomSequence(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			const std::vector<cv::Point>& reachable_roomcenters,
			ipa_building_navigation::FindRoomSequenceWithCheckpointsResultConstPtr& result_seq)
	{
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = evaluation_data.floor_plan_;
		cv_image.toImageMsg(map_msg);

		actionlib::SimpleActionClient<ipa_building_navigation::FindRoomSequenceWithCheckpointsAction> ac_seq("/room_sequence_planning/room_sequence_planning_server", true);
		ROS_INFO("Waiting for action server '/room_sequence_planning/room_sequence_planning_server' to start.");
		// wait for the action server to start
		ac_seq.waitForServer(); //will wait for infinite time

		//put the vector<Point> format in the msg format
		std::vector<ipa_room_segmentation::RoomInformation> roomcenters_for_sequence_planning(reachable_roomcenters.size());
		for(size_t room = 0; room < reachable_roomcenters.size(); ++room)
		{
			roomcenters_for_sequence_planning[room].room_center.x = reachable_roomcenters[room].x;
			roomcenters_for_sequence_planning[room].room_center.y = reachable_roomcenters[room].y;
		}

		std::cout << "Action server started, sending goal_seq." << std::endl;
		// send a goal_seg to the action
		ipa_building_navigation::FindRoomSequenceWithCheckpointsGoal goal_seq;
		goal_seq.input_map = map_msg;
		goal_seq.map_resolution = evaluation_data.map_resolution_;
		goal_seq.map_origin = evaluation_data.map_origin_;
		goal_seq.room_information_in_pixel = roomcenters_for_sequence_planning;
		goal_seq.max_clique_path_length = evaluation_configuration.max_clique_path_length_;
		goal_seq.map_downsampling_factor = evaluation_data.map_downsampling_factor_;
		goal_seq.robot_radius = robot_radius_;
		goal_seq.robot_start_coordinate = evaluation_data.robot_start_position_;
		goal_seq.planning_method = evaluation_configuration.sequence_planning_method_;
		goal_seq.tsp_solver = evaluation_configuration.tsp_solver_;
		goal_seq.return_sequence_map = true;
		goal_seq.check_accessibility_of_rooms = false;
		ac_seq.sendGoal(goal_seq);

		//wait for the action to return
		bool finished_before_timeout = ac_seq.waitForResult();//ros::Duration(600.0));
		if (finished_before_timeout == false)
		{
			ROS_ERROR("Timeout on room sequence planning.");
			return false;
		}
		result_seq = ac_seq.getResult();
		std::cout << "Finished sequence planning successfully!" << std::endl;

		return true;
	}

private:

	std::vector< EvaluationData > evaluation_data_;

	const double robot_radius_;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_sequence_planning_client");

	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string data_storage_path = "room_sequence_planning/";
	Evaluation ev(test_map_path, data_storage_path, 0.3);

	//exit
	return 0;
}

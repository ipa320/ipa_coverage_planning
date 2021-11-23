#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <time.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_building_msgs/MapSegmentationAction.h>
#include <ipa_building_msgs/FindRoomSequenceWithCheckpointsAction.h>
#include <ipa_building_navigation/A_star_pathplanner.h>

#include <ipa_building_navigation/timer.h>
#include <ipa_building_navigation/dynamic_reconfigure_client.h>
#include <ipa_building_navigation/contains.h>
#include <ipa_building_navigation/tsp_solver_defines.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/image_encodings.h>

#include <map>
#include <vector>
#include <set>


struct EvaluationConfig
{
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation
											// 1 = morphological segmentation
											// 2 = distance segmentation
											// 3 = Voronoi segmentation
											// 4 = semantic segmentation
											// 5 = Voronoi random field segmentation
	double max_clique_path_length_;		// max A* path length between two rooms that are assigned to the same clique, in [m]
	int sequence_planning_method_;		// Method for sequence planning
											// 1 = drag trolley if next room is too far away
											// 2 = calculate roomgroups and a trolleyposition for each of it
	int tsp_solver_;					// TSP solver that is used
											// TSP_NEAREST_NEIGHBOR=1 = Nearest Neighbor
											// TSP_GENETIC=2 = Genetic solver
											// TSP_CONCORDE=3 = Concorde solver
	int trashbins_per_trolley_;			// variable that shows how many trashbins can be emptied into one trolley

	EvaluationConfig()
	{
		room_segmentation_algorithm_ = 1;
		max_clique_path_length_ = 12.0;
		sequence_planning_method_ = 2;
		tsp_solver_ = TSP_CONCORDE;
		trashbins_per_trolley_ = 9001;
	}

	EvaluationConfig(const int room_segmentation_algorithm, const double max_clique_path_length, const int sequence_planning_method, const int tsp_solver)
	{
		room_segmentation_algorithm_ = room_segmentation_algorithm;
		max_clique_path_length_ = max_clique_path_length;
		sequence_planning_method_ = sequence_planning_method;
		tsp_solver_ = tsp_solver;
		trashbins_per_trolley_ = 9001;
	}

	EvaluationConfig(const int room_segmentation_algorithm, const double max_clique_path_length, const int sequence_planning_method, const int tsp_solver, const double trashbins_per_trolley)
	{
		room_segmentation_algorithm_ = room_segmentation_algorithm;
		max_clique_path_length_ = max_clique_path_length;
		sequence_planning_method_ = sequence_planning_method;
		tsp_solver_ = tsp_solver;
		trashbins_per_trolley_ = trashbins_per_trolley;
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
		else if (room_segmentation_algorithm_ == 5)
			s = "Voronoi random field segmentation";
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
	cv::Point central_trolley_park_;

	EvaluationData()
	{
		map_name_ = "";
		floor_plan_ = cv::Mat();
		map_downsampling_factor_ = 0.25;
		map_resolution_ = 0.05;
		robot_radius_ = 0.3;
	}

	EvaluationData(const std::string& map_name, const cv::Mat& floor_plan, const float map_resolution, const double map_downsampling_factor,
			const double robot_radius, const std::vector< cv::Point >& trash_bin_locations, const cv::Point trolley_park_position = cv::Point(-1, -1))
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
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<uchar>(v,u) > 20)			// todo: parameter
				{
					robot_start_position_.position.x = u*map_resolution_ + map_origin_.position.x;
					robot_start_position_.position.y = v*map_resolution_ + map_origin_.position.y;
					robot_start_coordinate_set = true;
				}
		central_trolley_park_ = trolley_park_position;
	}
};

struct StatisticsItem
{
	double robot_speed_without_trolley;	// in [m/s]
	double robot_speed_with_trolley;	// in [m/s]
	double time_for_trashbin_manipulation;	// in [s]
	double time_for_trolley_manipulation;	// in [s]
	int number_trash_bins;
	int number_trolley_movements;
	double path_length_robot;	// in [m]
	double path_length_trolley;	// in [m]
	double path_length_trash_bins;	// in [m]
	double pathlength;	// in [m]
	double cleaning_time;	// in [s]
	double calculation_time_segmentation;	// in [s]
	double calculation_time_sequencer;	// in [s]
	double human_way; // in [m]

	StatisticsItem()
	{
		robot_speed_without_trolley=0.;	// in [m/s]
		robot_speed_with_trolley=0.;	// in [m/s]
		time_for_trashbin_manipulation=0.;	// in [s]
		time_for_trolley_manipulation=0.;	// in [s]
		number_trash_bins=0;
		number_trolley_movements=0;
		path_length_robot=0.;	// in [m]
		path_length_trolley=0.;	// in [m]
		path_length_trash_bins=0.;	// in [m]
		pathlength=1e10;	// in [m]
		cleaning_time=1e10;	// in [s]
		calculation_time_segmentation=0.;	// in [s]
		calculation_time_sequencer=0.;
		human_way=0.;
	}
};

class Evaluation
{
public:

	// to segment the map only once for each segmentation algorithm
	ipa_building_msgs::MapSegmentationResultConstPtr result_seg_morph;
	bool segmented_morph;
	ipa_building_msgs::MapSegmentationResultConstPtr result_seg_dist;
	bool segmented_dist;
	ipa_building_msgs::MapSegmentationResultConstPtr result_seg_vor;
	bool segmented_vor;
	ipa_building_msgs::MapSegmentationResultConstPtr result_seg_semant;
	bool segmented_semant;
	ipa_building_msgs::MapSegmentationResultConstPtr result_seg_vrf;
	bool segmented_vrf;

	Evaluation(ros::NodeHandle& nh, const std::string& test_map_path, const std::string& data_storage_path, const double robot_radius)
	: node_handle_(nh), robot_radius_(robot_radius)
	{
		segmented_morph = false;
		segmented_dist = false;
		segmented_vor = false;
		segmented_semant = false;
		segmented_vrf = false;
		// prepare relevant floor map data
//		std::vector<std::string> map_names;
////	map_names.push_back("lab_ipa"); // done
////	map_names.push_back("Freiburg52_scan"); //done
////	map_names.push_back("Freiburg79_scan"); //done
////	map_names.push_back("lab_c_scan"); //done
////	map_names.push_back("lab_b_scan"); //done
////	map_names.push_back("lab_d_scan"); //done
////	map_names.push_back("lab_intel"); //done
////	map_names.push_back("Freiburg101_scan"); //done
////	map_names.push_back("lab_a_scan"); //done
//		map_names.push_back("lab_f_scan"); //done
////	map_names.push_back("NLB"); //done

		//with obstacles:
//		"Freiburg52_scan_furnitures_trashbins"
//		map_names.push_back("lab_ipa_furnitures");

		std::vector< std::string > map_names;
		map_names.push_back("lab_ipa");
		map_names.push_back("lab_c_scan");
		map_names.push_back("Freiburg52_scan");
		map_names.push_back("Freiburg79_scan");
		map_names.push_back("lab_b_scan");
		map_names.push_back("lab_intel");
		map_names.push_back("Freiburg101_scan");
		map_names.push_back("lab_d_scan");
		map_names.push_back("lab_f_scan");
		map_names.push_back("lab_a_scan");
		map_names.push_back("NLB");
		map_names.push_back("office_a");
		map_names.push_back("office_b");
		map_names.push_back("office_c");
		map_names.push_back("office_d");
		map_names.push_back("office_e");
		map_names.push_back("office_f");
		map_names.push_back("office_g");
		map_names.push_back("office_h");
		map_names.push_back("office_i");
		map_names.push_back("lab_ipa_furnitures");
		map_names.push_back("lab_c_scan_furnitures");
		map_names.push_back("Freiburg52_scan_furnitures");
		map_names.push_back("Freiburg79_scan_furnitures");
		map_names.push_back("lab_b_scan_furnitures");
		map_names.push_back("lab_intel_furnitures");
		map_names.push_back("Freiburg101_scan_furnitures");
		map_names.push_back("lab_d_scan_furnitures");
		map_names.push_back("lab_f_scan_furnitures");
		map_names.push_back("lab_a_scan_furnitures");
		map_names.push_back("NLB_furnitures");
		map_names.push_back("office_a_furnitures");
		map_names.push_back("office_b_furnitures");
		map_names.push_back("office_c_furnitures");
		map_names.push_back("office_d_furnitures");
		map_names.push_back("office_e_furnitures");
		map_names.push_back("office_f_furnitures");
		map_names.push_back("office_g_furnitures");
		map_names.push_back("office_h_furnitures");
		map_names.push_back("office_i_furnitures");


		// prepare image data for evaluation
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
			std::string map_name_basic = map_names[image_index];
			std::size_t pos = map_names[image_index].find("_furnitures");
			if (pos != std::string::npos)
				map_name_basic = map_names[image_index].substr(0, pos);
			image_filename = test_map_path + map_name_basic + "_trashbins.png";
			cv::Mat temp = cv::imread(image_filename.c_str());
			cv::Vec3b blue(255, 0, 0);
			cv::Vec3b red(0, 0, 255);
			double map_resoultion_for_evaluation = 0.05;
			int number_of_erosions = (robot_radius_ / map_resoultion_for_evaluation);
			cv::Mat eroded_map;
			cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1,-1), number_of_erosions);
			std::vector< cv::Point > trash_bin_locations;
			cv::Point central_trolley_park;
			for (int y = 0; y < temp.rows; y++)
			{
				for (int x = 0; x < temp.cols; x++)
				{
					//find blue drawn trash bins and check if they are reachable (if one is not reachable the a star planner will give an extremely large path)
					if (temp.at<cv::Vec3b>(y, x) == blue && eroded_map.at<unsigned char>(y, x) != 0)
					{
						trash_bin_locations.push_back(cv::Point(x,y));
						std::cout << "trash: " << x << ", " << y << std::endl;
					}
					//find red drawn trolley park and check if they are reachable (if one is not reachable the a star planner will give an extremely large path)
					if (temp.at<cv::Vec3b>(y, x) == red && eroded_map.at<unsigned char>(y, x) != 0)
					{
						central_trolley_park = cv::Point(x, y);
						std::cout << "trolley park: " << x << ", " << y << std::endl;
					}
				}
			}
			//give occupied memory free
			temp.release();
			eroded_map.release();

//			+"_furnitures_trashbins"
//			evaluation_data_.push_back(EvaluationData(map_names[image_index], map, map_resoultion_for_evaluation, 0.25, robot_radius_, trash_bin_locations));
			evaluation_data_.push_back(EvaluationData(map_names[image_index], map, map_resoultion_for_evaluation, 0.25, robot_radius_, trash_bin_locations, central_trolley_park));
		}

		// set configurations
		std::vector<EvaluationConfig> evaluation_configurations;
		setConfigurations(evaluation_configurations);

		// do the evaluation
		std::string bugfile = data_storage_path + "bugfile.txt";
		std::ofstream failed_maps(bugfile.c_str(), std::ios::out);
		if(failed_maps.is_open())
			failed_maps << "maps that had a bug during the simulation and couldn't be finished: " << std::endl;
		for (size_t i=0; i<evaluation_data_.size(); ++i)
		{
//			if (evaluateAllConfigs0815(evaluation_configurations, evaluation_data_[i], data_storage_path) == false)
//			{
//				std::cout << "failed to simulate map " << evaluation_data_[i].map_name_ << std::endl;
//				if(failed_maps.is_open())
//					failed_maps << evaluation_data_[i].map_name_ << std::endl;
//			}
//			if (evaluateAllConfigsVer1(evaluation_configurations, evaluation_data_[i], data_storage_path) == false)
//			{
//				std::cout << "failed to simulate map " << evaluation_data_[i].map_name_ << std::endl;
//				if(failed_maps.is_open())
//					failed_maps << evaluation_data_[i].map_name_ << std::endl;
//			}
			if (evaluateAllConfigsVer2(evaluation_configurations, evaluation_data_[i], data_storage_path) == false)
			{
				std::cout << "failed to simulate map " << evaluation_data_[i].map_name_ << std::endl;
				if(failed_maps.is_open())
					failed_maps << evaluation_data_[i].map_name_ << std::endl;
			}
			//reset booleans to segment the new map
			segmented_morph = false;
			segmented_dist = false;
			segmented_vor = false;
			segmented_semant = false;
			segmented_vrf = false;
		}
		failed_maps.close();

		//read the saved results
		std::map<std::string, std::vector<StatisticsItem> > results;	// maps from [map_name]->StatisticsItems[evaluation_configurations.size()]
		readFromLocalFiles(evaluation_configurations, map_names, data_storage_path, results);

		//write the calculated values to global saving files
		writeGlobalStatistics(evaluation_configurations, map_names, data_storage_path, results);
//		writeToGlobalFiles(map_names, data_storage_path, map_pathlengths, map_cleaning_times, sequence_solver_times);
	}

	//function to read the saved results out of the files
	void readFromLocalFiles(const std::vector<EvaluationConfig>& evaluation_configurations, const std::vector<std::string>& map_names, const std::string& data_storage_path,
			std::map<std::string, std::vector<StatisticsItem> >& results)
	{
		for (size_t i=0; i<evaluation_configurations.size(); ++i)
		{
			const std::string upper_folder_name = evaluation_configurations[i].generateUpperConfigurationFolderString() + "/";
			const std::string path = data_storage_path + upper_folder_name;
			const std::string lower_folder_name = evaluation_configurations[i].generateLowerConfigurationFolderString() + "/";
			const std::string lower_path = path + lower_folder_name;

			for(size_t map_index = 0; map_index < map_names.size(); ++map_index)
			{
				std::string log_filename = lower_path + map_names[map_index] + "_results.txt";

				StatisticsItem stats;
				std::ifstream reading_file(log_filename.c_str());
				if (reading_file.is_open())
				{
					std::string line;
					for (int k=0; k<8; ++k)
						getline(reading_file, line);
					getline(reading_file, line);
					std::istringstream iss(line);
					iss >> stats.robot_speed_without_trolley;	// in [m/s]
					iss >> stats.robot_speed_with_trolley;	// in [m/s]
					iss >> stats.time_for_trashbin_manipulation;	// in [s]
					iss >> stats.time_for_trolley_manipulation;	// in [s]
					iss >> stats.number_trash_bins;
					iss >> stats.number_trolley_movements;
					iss >> stats.path_length_robot;	// in [m]
					iss >> stats.path_length_trolley;	// in [m]
					iss >> stats.path_length_trash_bins;	// in [m]
					iss >> stats.pathlength;	// in [m]
					iss >> stats.cleaning_time;	// in [s]
					iss >> stats.calculation_time_segmentation;	// in [s]
					iss >> stats.calculation_time_sequencer;	// in [s]
					reading_file.close();
				}
				else
				{
					std::cout << "missing data: " << log_filename << std::endl;
				}

				std::string human_filename = data_storage_path + "human_way/" + upper_folder_name + lower_folder_name + map_names[map_index] + "_human.txt";
				std::ifstream human_file(human_filename.c_str());

				if(human_file.is_open())
				{
					std::string line;
					getline(human_file, line); // first line is text
					getline(human_file, line);
					std::istringstream iss(line);
					iss >> stats.human_way; // in [m]
					human_file.close();
				}
				results[map_names[map_index]].push_back(stats);
			}
		}
	}

	void writeGlobalStatistics(const std::vector<EvaluationConfig>& evaluation_configurations, const std::vector<std::string>& map_names, const std::string& data_storage_path,
			std::map<std::string, std::vector<StatisticsItem> >& results)
	{
		//define the storage path for each planning method
		const std::string path = data_storage_path + "global/";
		const std::string upper_command = "mkdir -p " + path;
		int return_value = system(upper_command.c_str());

		// prepare files for different evaluation criteria
		std::vector<std::string> filenames;
		filenames.push_back("pathlength");
		filenames.push_back("cleaning_time");
		filenames.push_back("number_trash_bins");
		filenames.push_back("number_trolley_movements");
		filenames.push_back("calculation_time_segmentation");
		filenames.push_back("calculation_time_sequencer");
		filenames.push_back("path_length_robot");
		filenames.push_back("path_length_trolley");
		filenames.push_back("path_length_trash_bins");
		filenames.push_back("human_way");

		for (std::vector<std::string>::iterator it_filename=filenames.begin(); it_filename!=filenames.end(); ++it_filename)
		{
			// collect column data
			std::set<double> max_clique_lengths;
			for (size_t i=0; i<evaluation_configurations.size(); ++i)
				max_clique_lengths.insert(evaluation_configurations[i].max_clique_path_length_);

			// prepare output matrix of strings
			const int data_columns = (int)evaluation_configurations.size()/max_clique_lengths.size();
			std::vector<std::vector<std::string> > output_matrix(1+data_columns); // [column index][row index] !
			for (size_t i=0; i<output_matrix.size(); ++i)
				output_matrix[i].resize(1+max_clique_lengths.size());

			std::stringstream output_stream;
			for (std::vector<std::string>::const_iterator it=map_names.begin(); it!=map_names.end(); it++)
			{
				// prepare first column of output
				output_matrix[0][0] = *it;
				int r=1;
				for (std::set<double>::iterator mcl_it = max_clique_lengths.begin(); mcl_it != max_clique_lengths.end(); mcl_it++, r++)
				{
					std::stringstream ss;
					ss << "mcl" << *mcl_it;
					output_matrix[0][r] = ss.str();
				}

				// write remaining columns of output
				for (int i=0; i<data_columns; ++i)
				{
					int base_index = i*max_clique_lengths.size();
					std::stringstream ss;
					ss << "seg" << evaluation_configurations[base_index].room_segmentation_algorithm_ << "plmth" << evaluation_configurations[base_index].sequence_planning_method_ << "tsp" << evaluation_configurations[base_index].tsp_solver_;
					output_matrix[1+i][0] = ss.str();
					for (size_t k=0; k<max_clique_lengths.size(); ++k)
					{
						std::stringstream sss;
						if (it_filename->compare("pathlength") == 0)
							sss << results[*it][base_index+k].pathlength;
						else if (it_filename->compare("cleaning_time") == 0)
							sss << results[*it][base_index+k].cleaning_time;
						else if (it_filename->compare("number_trash_bins") == 0)
							sss << results[*it][base_index+k].number_trash_bins;
						else if (it_filename->compare("number_trolley_movements") == 0)
							sss << results[*it][base_index+k].number_trolley_movements;
						else if (it_filename->compare("calculation_time_segmentation") == 0)
							sss << results[*it][base_index+k].calculation_time_segmentation;
						else if (it_filename->compare("calculation_time_sequencer") == 0)
							sss << results[*it][base_index+k].calculation_time_sequencer;
						else if (it_filename->compare("path_length_robot") == 0)
							sss << results[*it][base_index+k].path_length_robot;
						else if (it_filename->compare("path_length_trolley") == 0)
							sss << results[*it][base_index+k].path_length_trolley;
						else if (it_filename->compare("path_length_trash_bins") == 0)
							sss << results[*it][base_index+k].path_length_trash_bins;
						else if (it_filename->compare("human_way") == 0)
							sss << results[*it][base_index+k].human_way;
						output_matrix[1+i][1+k] = sss.str();
					}
				}
				// write output string
				for (size_t row=0; row<output_matrix[0].size(); ++row)
				{
					for (size_t col=0; col<output_matrix.size(); ++col)
						output_stream << output_matrix[col][row] << "\t";
					output_stream << "\n";
				}
			}

			// write output data to file
			std::string filename = path + *it_filename + ".txt";
			std::ofstream file(filename.c_str(), std::ofstream::out);
			if (file.is_open())
			{
				file << output_stream.str();
			}
			else
				ROS_ERROR("Could not write to file '%s'.", filename.c_str());
			file.close();
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

		const std::string cleaning_time_folder = "cleaningtime/";
		std::string lower_command = "mkdir -p " + path + cleaning_time_folder;
		return_value = system(lower_command.c_str());
		const std::string path_length_folder = "pathlengths/";
		lower_command = "mkdir -p " + path + path_length_folder;
		return_value = system(lower_command.c_str());
		const std::string computation_time_folder = "comp_time/";
		lower_command = "mkdir -p " + path + computation_time_folder;
		return_value = system(lower_command.c_str());

		// outputs to log file
		std::stringstream distance_output[map_names.size()];
		std::stringstream cleaning_time_output[map_names.size()];
		std::stringstream sequence_time_output[map_names.size()];


			//define values to show how much different algorithms has been implemented
			double max_clique_path_length = 4.0;
			int number_of_cliquelenghts = 8;
			int number_of_segmentation_algorithms = 5;
			int number_of_tsp_solver = 3;
			for(size_t i = 0; i < map_names.size(); ++i)
			{
				distance_output[i] << "\t" << "Nachziehmethode" << "\t" << "\t" << "Trolley-Gruppen" << std::endl
						<< "max. Fahrdistanz" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << std::endl;
				cleaning_time_output[i] << "\t" << "\t" << "Nachziehmethode" << "\t" << "\t" << "Trolley-Gruppen" << std::endl
						<< "max. Fahrdistanz" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << std::endl;
				sequence_time_output[i] << "\t" << "\t" << "Nachziehmethode" << "\t" << "\t" << "Trolley-Gruppen" << std::endl
						<< "max. Fahrdistanz" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << "\t" << "nearest" << "\t" << "genetic" << "\t" << "concorde" << std::endl;
			}
			for (int max_length = 0; max_length < number_of_cliquelenghts; ++max_length)
			{
				max_clique_path_length += 2.0;

				int start = number_of_segmentation_algorithms * number_of_tsp_solver * max_length;

				for(size_t map = 0; map < map_names.size(); ++map)
				{
					//show which maximal clique pathlength this is
					distance_output[map] << std::endl << max_clique_path_length;
					cleaning_time_output[map] << std::endl << max_clique_path_length;
					sequence_time_output[map] << std::endl << max_clique_path_length;
					// header
					for(size_t segmentation = 0; segmentation < number_of_segmentation_algorithms; ++segmentation)
					{
						if(segmentation > 0)
						{
							distance_output[map] << 0;
							cleaning_time_output[map] << 0;
							sequence_time_output[map] << 0;
						}
						for(int sequence_planning_method = 0; sequence_planning_method < 2; ++sequence_planning_method)
						{
							int plmth_start = start + number_of_segmentation_algorithms * number_of_tsp_solver * number_of_cliquelenghts * sequence_planning_method;
							int index = plmth_start + number_of_tsp_solver * segmentation;
							for(size_t reading_index = index; reading_index < index+number_of_tsp_solver; ++reading_index)
							{
								distance_output[map] << " " << map_pathlengths[map][reading_index] ;
								cleaning_time_output[map] << " " << map_cleaning_times[map][reading_index];
								sequence_time_output[map] << " " << sequence_solver_times[map][reading_index];
							}
						}

						distance_output[map] << std::endl;
						cleaning_time_output[map] << std::endl;
						sequence_time_output[map] << std::endl;
					}
				}
			}
		//write saved parameters to global file
		for(size_t map = 0; map < map_names.size(); ++map)
		{
			std::string pathlength_filename = path + path_length_folder + map_names[map] + "_pathlengths.txt";
			std::ofstream pathlength_file(pathlength_filename.c_str(), std::ios::out);
			if (pathlength_file.is_open()==true)
				pathlength_file << distance_output[map].str();
			pathlength_file.close();

			std::string cleaningtime_filename = path + cleaning_time_folder + map_names[map] + "_cleaningtimes.txt";
			std::ofstream cleaningtime_file(cleaningtime_filename.c_str(), std::ios::out);
			if (cleaningtime_file.is_open()==true)
				cleaningtime_file << cleaning_time_output[map].str();
			cleaningtime_file.close();

			std::string sequencetime_filename = path + computation_time_folder + map_names[map] + "_sequencetimes.txt";
			std::ofstream sequencetime_file(sequencetime_filename.c_str(), std::ios::out);
			if (sequencetime_file.is_open()==true)
				sequencetime_file << sequence_time_output[map].str();
			sequencetime_file.close();

		}
	}

	void setConfigurations(std::vector< EvaluationConfig >& evaluation_configurations)
	{
		evaluation_configurations.clear();
		for (int room_segmentation_algorithm=1; room_segmentation_algorithm<=5; ++room_segmentation_algorithm)
		{
			for(int sequence_planning_method = 2; sequence_planning_method <= 2; ++sequence_planning_method)
			{
				for(int tsp_solver = 1; tsp_solver <= 3; ++tsp_solver)
				{
					cv::Mat max_clique_lengths = (cv::Mat_<double>(1,11) << 6., 8., 10., 12., 14., 16., 18., 20., 25., 30., 50.);
					//for (double max_clique_path_length = 20.; max_clique_path_length <= 20.; max_clique_path_length += 2.0)
					for (int i=0; i<max_clique_lengths.cols; ++i)
						evaluation_configurations.push_back(EvaluationConfig(room_segmentation_algorithm, max_clique_lengths.at<double>(0,i), sequence_planning_method, tsp_solver, 10));
				}
			}
		}
	}

	bool evaluateAllConfigs0815(const std::vector<EvaluationConfig>& evaluation_configuration_vector, const EvaluationData& evaluation_data, const std::string& data_storage_path)
	{
		// go through each configuration for the given map
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

			std::cout << "\nCurrent Configuration:" << std::endl << "map: " << evaluation_data.map_name_ << "\tsegmentation algorithm: "
				<< evaluation_configuration_vector[config].room_segmentation_algorithm_
				<< "\tplanning method: " << evaluation_configuration_vector[config].sequence_planning_method_
				<< "\tTSP solver: " << evaluation_configuration_vector[config].tsp_solver_
				<< "\tmaximal clique length: " << evaluation_configuration_vector[config].max_clique_path_length_ << std::endl;

			AStarPlanner planner;
			//variables for time measurement
			struct timespec t0, t1, t2, t3;

			// 1. retrieve segmentation and check if the map has already been segmented
			ipa_building_msgs::MapSegmentationResultConstPtr result_seg;
			clock_gettime(CLOCK_MONOTONIC,  &t0); //set time stamp before the segmentation
			if(evaluation_configuration_vector[config].room_segmentation_algorithm_ == 1)
			{
				if(segmented_morph == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_morph, t0) == false)
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
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_dist, t0) == false)
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
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vor, t0) == false)
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
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_semant, t0) == false)
						return false;
					segmented_semant = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_semant;
			}
			else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 5)
			{
				if(segmented_vrf == false)
				{
					if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vrf, t0) == false)
						return false;
					segmented_vrf = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_vrf;
			}
			clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the segmentation
			std::cout << "Segmentation computed " << result_seg->room_information_in_pixel.size() << " rooms." << std::endl;

			//check for accessibility of the room centers from start position
			cv::Mat downsampled_map;
			Timer tim;
			planner.downsampleMap(evaluation_data.floor_plan_, downsampled_map, evaluation_data.map_downsampling_factor_, evaluation_data.robot_radius_, evaluation_data.map_resolution_);
			std::cout << "downsampling map: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
			cv::Point robot_start_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
											(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);

			// get the reachable room centers as cv::Point
			tim.start();
			std::cout << "Starting to check accessibility of rooms. Start position: " << robot_start_position << std::endl;
			std::vector<cv::Point> room_centers;
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				cv::Point current_center(result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
				double length = planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_start_position, current_center, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_);
				if(length < 1e9)
					room_centers.push_back(current_center);
				else
					std::cout << "room " << i << " not accessible, center: " << current_center << std::endl;
			}
			std::cout << "room centers computed: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
			std::cout << "Number of accessible room centers: " << room_centers.size() << std::endl;

			if(room_centers.size() == 0) //no room center is reachable for the given start position --> needs to be looked at separately
			{
				std::cout << "++++++++++ no roomcenter reachable from given startposition ++++++++++++" << std::endl;
				return false;
			}

			// 2. solve sequence problem
			std::cout << "Starting to solve sequence problem." << std::endl;
			tim.start();
			ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_seq;
			clock_gettime(CLOCK_MONOTONIC,  &t2); //set time stamp before the sequence planning
			if (computeRoomSequence(evaluation_data, evaluation_configuration_vector[config], room_centers, result_seq, t2) == false)
			{
				std::cout << "++++++++++ computeRoomSequence failed ++++++++++++" << std::endl;
				return false;
			}
			std::cout << "sequence problem solved: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
			clock_gettime(CLOCK_MONOTONIC,  &t3); //set time stamp after the sequence planning

			// 3. assign trash bins to rooms of the respective segmentation
			std::vector< std::vector<cv::Point> > room_trash_bins(result_seg->room_information_in_pixel.size());
			// converting the map msg in cv format
			cv_bridge::CvImagePtr cv_ptr_obj;
			cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_obj->image;
			for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
			{
				int trash_bin_index = segmented_map.at<int>(evaluation_data.trash_bin_locations_[i]); //labeling started from 1 --> 0 is for obstacles
				int room_index = -1;
				for (size_t r=0; r<room_centers.size(); ++r)
				{
					if (segmented_map.at<int>(room_centers[r]) == trash_bin_index)
					{
						room_index = r;
						break;
					}
				}
				if (room_index != -1)
				{
					room_trash_bins[room_index].push_back(evaluation_data.trash_bin_locations_[i]);
					std::cout << "trash bin " << evaluation_data.trash_bin_locations_[i] << "   at room center[" << room_index << "] " << room_centers[room_index] << std::endl;
				}
				else
					std::cout << "########## trash bin " << evaluation_data.trash_bin_locations_[i] << " does not match any room." << std::endl;
			}

			// 4. do the movements
			cv::Mat draw_path_map, draw_path_map2;
			cv::cvtColor(evaluation_data.floor_plan_, draw_path_map, CV_GRAY2BGR);
			draw_path_map2 = draw_path_map.clone();
			double path_length_robot = 0.;
			double path_length_trolley = 0.;
			double path_length_trash_bins = 0.;
//			const double max_clique_path_length_in_pixel = evaluation_configuration.max_clique_path_length_ / evaluation_data.map_resolution_;
			cv::Point robot_position = robot_start_position;
			cv::Point trolley_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
					(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);
			cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
			cv::circle(draw_path_map, trolley_position, 5, CV_RGB(0,0,255), -1);
			cv::circle(draw_path_map2, trolley_position, 5, CV_RGB(0,0,255), -1);

			std::stringstream screenoutput;
			for (size_t clique_index = 0; clique_index<result_seq->checkpoints.size(); ++clique_index)
			{
				std::cout << "cleaning new clique" << std::endl; screenoutput << "cleaning new clique" << std::endl;
				// move trolley
				//		i) robot to trolley
				path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
				// 		ii) trolley to next trolley goal
				cv::Point trolley_goal_position(result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.x, result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.y);
				path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, trolley_goal_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
				trolley_position = trolley_goal_position;
				robot_position = trolley_goal_position;
				cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
				cv::circle(draw_path_map, trolley_position, 5, CV_RGB(0,0,255), -1);
				cv::circle(draw_path_map2, trolley_position, 5, CV_RGB(0,0,255), -1);
				std::cout << "moved trolley to " << trolley_position << std::endl; screenoutput << "moved trolley to " << trolley_position << std::endl;

				// move robot to rooms
				for(size_t room = 0; room < result_seq->checkpoints[clique_index].room_indices.size(); ++room)
				{
					// get next room in sequence
					const int room_index = result_seq->checkpoints[clique_index].room_indices[room];
					cv::Point current_roomcenter = room_centers[room_index];
					// drive to next room
					path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, current_roomcenter, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
					robot_position = current_roomcenter;
					cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
					// clear all trash bins: go to trash bin, go back to trolley to empty trash and then drive back to trash bin
					std::cout << " arrived in room " << current_roomcenter << "\n starting to clean the trash bins" << std::endl; screenoutput << " arrived in room " << current_roomcenter << "\n starting to clean the trash bins" << std::endl;
					ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_trash_bin_seq;
					std::vector<cv::Point> trash_bin_sequence_in_this_room;
					if (room_trash_bins[room_index].size()>1 && computeTrashBinSequence(evaluation_data, evaluation_configuration_vector[config], room_trash_bins[room_index], robot_position, result_trash_bin_seq) == true)
					{
						for (size_t cc=0; cc<result_trash_bin_seq->checkpoints.size(); ++cc)
							for (size_t tt = 0; tt < result_trash_bin_seq->checkpoints[cc].room_indices.size(); ++tt)
								trash_bin_sequence_in_this_room.push_back(room_trash_bins[room_index][result_trash_bin_seq->checkpoints[cc].room_indices[tt]]);
					}
					else
					{
						trash_bin_sequence_in_this_room = room_trash_bins[room_index];
					}
					for (size_t t=0; t<trash_bin_sequence_in_this_room.size(); ++t)
					{
						// drive robot to trash bin
						double trash_bin_dist1 = planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trash_bin_sequence_in_this_room[t], evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
						path_length_robot += trash_bin_dist1;
						cv::circle(draw_path_map, trash_bin_sequence_in_this_room[t], 2, CV_RGB(128,0,255), -1);
						cv::circle(draw_path_map2, trash_bin_sequence_in_this_room[t], 2, CV_RGB(128,0,255), -1);
						// drive trash bin to trolley and back
						double trash_bin_dist2 = 2. * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trash_bin_sequence_in_this_room[t], trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map2);
						path_length_robot += trash_bin_dist2;
						path_length_trash_bins += trash_bin_dist1 + trash_bin_dist2;
						std::cout << "  room: " << room_index << "\ttrash bin: " << t << " at " << trash_bin_sequence_in_this_room[t] << "\ttraveling distance: " << (trash_bin_dist1 + trash_bin_dist2) * evaluation_data.map_resolution_ << " m" << std::endl;
						screenoutput << "  room: " << room_index << "\ttrash bin: " << t << " at " << trash_bin_sequence_in_this_room[t] << "\ttraveling distance: " << (trash_bin_dist1 + trash_bin_dist2) * evaluation_data.map_resolution_ << " m" << std::endl;
						robot_position = trash_bin_sequence_in_this_room[t];
					}
				}
				std::cout << " cleaned all rooms and trash bins in current clique" << std::endl; screenoutput << " cleaned all rooms and trash bins in current clique";
			}
			// finally go back to trolley
			path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
			// and back to start position
			path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, robot_start_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);

			// evaluation
			double path_length_robot_in_meter = path_length_robot * evaluation_data.map_resolution_;
			double path_length_trolley_in_meter = path_length_trolley * evaluation_data.map_resolution_;
			double path_length_total_in_meter = path_length_robot_in_meter + path_length_trolley_in_meter;
			double path_length_trash_bins_in_meter = path_length_trash_bins * evaluation_data.map_resolution_;
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
			output << "robot_speed_without_trolley" << "\t" << "robot_speed_with_trolley" << "\t" << "time_for_trashbin_manipulation" << "\t"
					<< "time_for_trolley_manipulation" << "\t" << "number_of_trash_bins" << "\t" << "number_trolley_movements" << "\t"
					<< "path_length_robot_in_meter" << "\t" << "path_length_trolley_in_meter" << "\t" << "path_length_trash_bins_in_meter\t"
					<< "pathlength [m]" << "\t"
					<< "cleaning time [s]" << "\t" << "calculation time segmentation [s]" << "\t" << "calculation time sequencer [s]" << "\t" << std::endl;
			output << robot_speed_without_trolley << "\t" << robot_speed_with_trolley << "\t" << time_for_trashbin_manipulation << "\t" << time_for_trolley_manipulation << "\t"
					<< evaluation_data.trash_bin_locations_.size() << "\t"
					<< (result_seq->checkpoints.size()+1) << "\t" << path_length_robot_in_meter << "\t" << path_length_trolley_in_meter << "\t" << path_length_trash_bins_in_meter << "\t"
					<< path_length_total_in_meter << "\t"
					<< time << "\t" << segmentation_time << "\t" << sequence_time;
			output << "\n\n\n" << screenoutput.str() << std::endl;

			std::string log_filename = lower_path + evaluation_data.map_name_ + "_results.txt";
			std::ofstream file(log_filename.c_str(), std::ios::out);
			if (file.is_open()==true)
				file << output.str();
			else
				ROS_ERROR("Error on writing file '%s'", log_filename.c_str());
			file.close();

			// images: segmented_map, sequence_map
			std::string segmented_map_filename = lower_path + evaluation_data.map_name_ + "_segmented.png";
			cv::Mat colour_segmented_map = segmented_map.clone();
			colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
			cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
			for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
			{
				// choose random color for each room
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
			// draw the room centers into the map
			for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
			{
				cv::Point current_center (result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
#if CV_MAJOR_VERSION<=3
				cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), CV_FILLED);
#else
				cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), cv::FILLED);
#endif
			}
			// color image in unique colour to show the segmentation
			if (cv::imwrite(segmented_map_filename.c_str(), colour_segmented_map) == false)
				ROS_ERROR("Error on writing file '%s'", segmented_map_filename.c_str());

			std::string sequence_map_filename = lower_path + evaluation_data.map_name_ + "_sequence.png";
			cv_bridge::CvImagePtr cv_ptr_seq;
			cv_ptr_seq = cv_bridge::toCvCopy(result_seq->sequence_map, sensor_msgs::image_encodings::BGR8);
			cv::Mat sequence_map = cv_ptr_seq->image;
			// draw in trash bins
			for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
#if CV_MAJOR_VERSION<=3
				cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), CV_FILLED);
#else
				cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), cv::FILLED);
#endif
			if (cv::imwrite(sequence_map_filename.c_str(), sequence_map) == false)
				ROS_ERROR("Error on writing file '%s'", sequence_map_filename.c_str());
			std::string sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail.png";
			if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map) == false)
				ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
			sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail2.png";
			if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map2) == false)
				ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
		}

		return true;
	}

	// variation 1: drive trolley back to a central trolley-park if the max. numbers of trashbins have been emptied in it.
	bool evaluateAllConfigsVer1(const std::vector<EvaluationConfig>& evaluation_configuration_vector, const EvaluationData& evaluation_data, const std::string& data_storage_path)
		{
			// go through each configuration for the given map
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

				std::cout << "\nCurrent Configuration:" << std::endl << "map: " << evaluation_data.map_name_ << "\tsegmentation algorithm: "
					<< evaluation_configuration_vector[config].room_segmentation_algorithm_
					<< "\tplanning method: " << evaluation_configuration_vector[config].sequence_planning_method_
					<< "\tTSP solver: " << evaluation_configuration_vector[config].tsp_solver_
					<< "\tmaximal clique length: " << evaluation_configuration_vector[config].max_clique_path_length_ << std::endl;

				AStarPlanner planner;
				//variables for time measurement
				struct timespec t0, t1, t2, t3;

				// 1. retrieve segmentation and check if the map has already been segmented
				ipa_building_msgs::MapSegmentationResultConstPtr result_seg;
				clock_gettime(CLOCK_MONOTONIC,  &t0); //set time stamp before the segmentation
				if(evaluation_configuration_vector[config].room_segmentation_algorithm_ == 1)
				{
					if(segmented_morph == false)
					{
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_morph, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_dist, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vor, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_semant, t0) == false)
							return false;
						segmented_semant = true;
					}
					else
						std::cout << "map has already been segmented" << std::endl;
					result_seg = result_seg_semant;
				}
				else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 5)
				{
					if(segmented_vrf == false)
					{
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vrf, t0) == false)
							return false;
						segmented_vrf = true;
					}
					else
						std::cout << "map has already been segmented" << std::endl;
					result_seg = result_seg_vrf;
				}
				clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the segmentation
				std::cout << "Segmentation computed " << result_seg->room_information_in_pixel.size() << " rooms." << std::endl;

				//check for accessibility of the room centers from start position
				cv::Mat downsampled_map;
				Timer tim;
				planner.downsampleMap(evaluation_data.floor_plan_, downsampled_map, evaluation_data.map_downsampling_factor_, evaluation_data.robot_radius_, evaluation_data.map_resolution_);
				std::cout << "downsampling map: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
				cv::Point robot_start_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
												(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);

				// get the reachable room centers as cv::Point
				tim.start();
				std::cout << "Starting to check accessibility of rooms. Start position: " << robot_start_position << std::endl;
				std::vector<cv::Point> room_centers;
				for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
				{
					cv::Point current_center(result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
					double length = planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_start_position, current_center, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_);
					if(length < 1e9)
						room_centers.push_back(current_center);
					else
						std::cout << "room " << i << " not accessible, center: " << current_center << std::endl;
				}
				std::cout << "room centers computed: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
				std::cout << "Number of accessible room centers: " << room_centers.size() << std::endl;

				if(room_centers.size() == 0) //no room center is reachable for the given start position --> needs to be looked at separately
				{
					std::cout << "++++++++++ no roomcenter reachable from given startposition ++++++++++++" << std::endl;
					return false;
				}

				// 2. solve sequence problem
				std::cout << "Starting to solve sequence problem." << std::endl;
				tim.start();
				ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_seq;
				clock_gettime(CLOCK_MONOTONIC,  &t2); //set time stamp before the sequence planning
				if (computeRoomSequence(evaluation_data, evaluation_configuration_vector[config], room_centers, result_seq, t2) == false)
				{
					std::cout << "++++++++++ computeRoomSequence failed ++++++++++++" << std::endl;
					return false;
				}
				std::cout << "sequence problem solved: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
				clock_gettime(CLOCK_MONOTONIC,  &t3); //set time stamp after the sequence planning

				// 3. assign trash bins to rooms of the respective segmentation
				std::vector< std::vector<cv::Point> > room_trash_bins(result_seg->room_information_in_pixel.size());
				// converting the map msg in cv format
				cv_bridge::CvImagePtr cv_ptr_obj;
				cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
				cv::Mat segmented_map = cv_ptr_obj->image;
				for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
				{
					int trash_bin_index = segmented_map.at<int>(evaluation_data.trash_bin_locations_[i]); //labeling started from 1 --> 0 is for obstacles
					int room_index = -1;
					for (size_t r=0; r<room_centers.size(); ++r)
					{
						if (segmented_map.at<int>(room_centers[r]) == trash_bin_index)
						{
							room_index = r;
							break;
						}
					}
					if (room_index != -1)
					{
						room_trash_bins[room_index].push_back(evaluation_data.trash_bin_locations_[i]);
						std::cout << "trash bin " << evaluation_data.trash_bin_locations_[i] << "   at room center[" << room_index << "] " << room_centers[room_index] << std::endl;
					}
					else
						std::cout << "########## trash bin " << evaluation_data.trash_bin_locations_[i] << " does not match any room." << std::endl;
				}

				// 4. do the movements
				cv::Mat draw_path_map, draw_path_map2;
				cv::cvtColor(evaluation_data.floor_plan_, draw_path_map, CV_GRAY2BGR);
				draw_path_map2 = draw_path_map.clone();
				double path_length_robot = 0.;
				double path_length_trolley = 0.;
				double path_length_trash_bins = 0.;
				int current_emptied_trashbins = 0;
				int switching_trolley_handling = 0; // int that shows how often the trolley has been handled during the switching procedure
	//			const double max_clique_path_length_in_pixel = evaluation_configuration.max_clique_path_length_ / evaluation_data.map_resolution_;
				cv::Point robot_position = robot_start_position;
				cv::Point trolley_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
						(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);
				cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
				cv::circle(draw_path_map, trolley_position, 5, CV_RGB(0,0,255), -1);
				cv::circle(draw_path_map2, trolley_position, 5, CV_RGB(0,0,255), -1);

				std::stringstream screenoutput;
				for (size_t clique_index = 0; clique_index<result_seq->checkpoints.size(); ++clique_index)
				{
					std::cout << "cleaning new clique" << std::endl; screenoutput << "cleaning new clique" << std::endl;
					// move trolley
					//		i) robot to trolley
					path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
					// 		ii) trolley to next trolley goal
					cv::Point trolley_goal_position(result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.x, result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.y);
					if(current_emptied_trashbins == evaluation_configuration_vector[config].trashbins_per_trolley_)
					{
						// get new trolley before going to the next trolley position
						// i) from current trolley position to central trolley park
						path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, evaluation_data.central_trolley_park_, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
						// ii) from central trolley park to new trolley position
						path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, evaluation_data.central_trolley_park_, trolley_goal_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
						// increase int that shows how often the trolley has been handled during switching by one
						++switching_trolley_handling;
						// reset number of emptied trashbins
						current_emptied_trashbins = 0;
					}
					else
					{
						// just drive the trolley
						path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, trolley_goal_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
					}
					trolley_position = trolley_goal_position;
					robot_position = trolley_goal_position;
					cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
					cv::circle(draw_path_map, trolley_position, 5, CV_RGB(0,0,255), -1);
					cv::circle(draw_path_map2, trolley_position, 5, CV_RGB(0,0,255), -1);
					std::cout << "moved trolley to " << trolley_position << std::endl; screenoutput << "moved trolley to " << trolley_position << std::endl;

					// move robot to rooms
					for(size_t room = 0; room < result_seq->checkpoints[clique_index].room_indices.size(); ++room)
					{
						// check if the trolley needs to be changed
						if(current_emptied_trashbins == evaluation_configuration_vector[config].trashbins_per_trolley_)
						{
							// if so drive the robot to the trolley and the trolley back to the park and the other way around
							path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
							path_length_trolley += 2.0 * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, evaluation_data.central_trolley_park_, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
							robot_position = trolley_position;
							// increase handling of the trolley by two (grasping the robot before and during switching)
							switching_trolley_handling += 2;
							// reset number of emptied trashbins
							current_emptied_trashbins = 0;
						}
						// get next room in sequence
						const int room_index = result_seq->checkpoints[clique_index].room_indices[room];
						cv::Point current_roomcenter = room_centers[room_index];
						// drive to next room
						path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, current_roomcenter, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
						robot_position = current_roomcenter;
						cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);
						// clear all trash bins: go to trash bin, go back to trolley to empty trash and then drive back to trash bin
						std::cout << " arrived in room " << current_roomcenter << "\n starting to plan the trash bins" << std::endl; screenoutput << " arrived in room " << current_roomcenter << "\n starting to clean the trash bins" << std::endl;
						ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_trash_bin_seq;
						std::vector<cv::Point> trash_bin_sequence_in_this_room;
						if (room_trash_bins[room_index].size()>1)
						{
							bool result_trash = computeTrashBinSequence(evaluation_data, evaluation_configuration_vector[config], room_trash_bins[room_index], robot_position, result_trash_bin_seq);
							if(result_trash == true)
							{
								for (size_t cc=0; cc<result_trash_bin_seq->checkpoints.size(); ++cc)
									for (size_t tt = 0; tt < result_trash_bin_seq->checkpoints[cc].room_indices.size(); ++tt)
										trash_bin_sequence_in_this_room.push_back(room_trash_bins[room_index][result_trash_bin_seq->checkpoints[cc].room_indices[tt]]);
							}
						}
						else
						{
							trash_bin_sequence_in_this_room = room_trash_bins[room_index];
						}
						std::cout << "starting to clean the trashbins. Size of sequence: " << trash_bin_sequence_in_this_room.size() << std::endl;
						for (size_t t=0; t<trash_bin_sequence_in_this_room.size(); ++t)
						{
							// check if the trolley needs to be changed
							if(current_emptied_trashbins >= evaluation_configuration_vector[config].trashbins_per_trolley_)
							{
								std::cout << "changing trolley " << std::endl;
								// if so drive the robot to the trolley and the trolley back to the park and the other way around
								path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
								path_length_trolley += 2.0 * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, evaluation_data.central_trolley_park_, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
								robot_position = trolley_position;
								// increase handling of the trolley by two (grasping the robot before and during switching)
								switching_trolley_handling += 2;
								// reset number of emptied trashbins
								current_emptied_trashbins = 0;
								std::cout << "changed trolley" << std::endl;
							}
							// drive robot to trash bin
							double trash_bin_dist1 = planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trash_bin_sequence_in_this_room[t], evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
							path_length_robot += trash_bin_dist1;
							cv::circle(draw_path_map, trash_bin_sequence_in_this_room[t], 2, CV_RGB(128,0,255), -1);
							cv::circle(draw_path_map2, trash_bin_sequence_in_this_room[t], 2, CV_RGB(128,0,255), -1);
							std::cout << "driven robot to trashbin" << std::endl;
							// drive trash bin to trolley and back
							double trash_bin_dist2 = 2. * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trash_bin_sequence_in_this_room[t], trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map2);
							path_length_robot += trash_bin_dist2;
							path_length_trash_bins += trash_bin_dist1 + trash_bin_dist2;
							std::cout << "  room: " << room_index << "\ttrash bin: " << t << " at " << trash_bin_sequence_in_this_room[t] << "\ttraveling distance: " << (trash_bin_dist1 + trash_bin_dist2) * evaluation_data.map_resolution_ << " m" << std::endl;
							screenoutput << "  room: " << room_index << "\ttrash bin: " << t << " at " << trash_bin_sequence_in_this_room[t] << "\ttraveling distance: " << (trash_bin_dist1 + trash_bin_dist2) * evaluation_data.map_resolution_ << " m" << std::endl;
							robot_position = trash_bin_sequence_in_this_room[t];
							// increase number of emptied trashbins by one
							++current_emptied_trashbins;
						}
					}
					std::cout << " cleaned all rooms and trash bins in current clique" << std::endl; screenoutput << " cleaned all rooms and trash bins in current clique";
				}
				// finally go back to trolley
				path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
				// and back to start position
				path_length_trolley += planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_position, robot_start_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);

				// evaluation
				double path_length_robot_in_meter = path_length_robot * evaluation_data.map_resolution_;
				double path_length_trolley_in_meter = path_length_trolley * evaluation_data.map_resolution_;
				double path_length_total_in_meter = path_length_robot_in_meter + path_length_trolley_in_meter;
				double path_length_trash_bins_in_meter = path_length_trash_bins * evaluation_data.map_resolution_;
				double robot_speed_without_trolley = 0.3;		// [m/s]
				double robot_speed_with_trolley = 0.2;			// [m/s]
				double time_for_trashbin_manipulation = 150;	// [s], without driving
				double time_for_trolley_manipulation = 90;		// [s], without driving
				double time = path_length_robot_in_meter / robot_speed_without_trolley
							+ path_length_trolley_in_meter / robot_speed_with_trolley
							+ time_for_trashbin_manipulation * evaluation_data.trash_bin_locations_.size()
							+ time_for_trolley_manipulation * (result_seq->checkpoints.size()+1)
							+ switching_trolley_handling * time_for_trolley_manipulation;

				//get the runtimes for the segmentation and the sequence planner
				double segmentation_time = (t1.tv_sec - t0.tv_sec) + (double) (t1.tv_nsec - t0.tv_nsec) * 1e-9;
				double sequence_time = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;

				// write log file
				std::stringstream output;
				// header
				output << evaluation_configuration_vector[config].getConfigurationString();
				output << "robot_speed_without_trolley" << "\t" << "robot_speed_with_trolley" << "\t" << "time_for_trashbin_manipulation" << "\t"
						<< "time_for_trolley_manipulation" << "\t" << "number_of_trash_bins" << "\t" << "number_trolley_movements" << "\t"
						<< "path_length_robot_in_meter" << "\t" << "path_length_trolley_in_meter" << "\t" << "path_length_trash_bins_in_meter\t"
						<< "pathlength [m]" << "\t"
						<< "cleaning time [s]" << "\t" << "calculation time segmentation [s]" << "\t" << "calculation time sequencer [s]" << "\t" << std::endl;
				output << robot_speed_without_trolley << "\t" << robot_speed_with_trolley << "\t" << time_for_trashbin_manipulation << "\t" << time_for_trolley_manipulation << "\t"
						<< evaluation_data.trash_bin_locations_.size() << "\t"
						<< (result_seq->checkpoints.size()+1) << "\t" << path_length_robot_in_meter << "\t" << path_length_trolley_in_meter << "\t" << path_length_trash_bins_in_meter << "\t"
						<< path_length_total_in_meter << "\t"
						<< time << "\t" << segmentation_time << "\t" << sequence_time;
				output << "\n\n\n" << screenoutput.str() << std::endl;

				std::string log_filename = lower_path + evaluation_data.map_name_ + "_results.txt";
				std::ofstream file(log_filename.c_str(), std::ios::out);
				if (file.is_open()==true)
					file << output.str();
				else
					ROS_ERROR("Error on writing file '%s'", log_filename.c_str());
				file.close();

				// images: segmented_map, sequence_map
				std::string segmented_map_filename = lower_path + evaluation_data.map_name_ + "_segmented.png";
				cv::Mat colour_segmented_map = segmented_map.clone();
				colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
				cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
				for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
				{
					// choose random color for each room
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
				// draw the room centers into the map
				for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
				{
					cv::Point current_center (result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
#if CV_MAJOR_VERSION<=3
					cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), CV_FILLED);
#else
					cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), cv::FILLED);
#endif
				}
				// color image in unique colour to show the segmentation
				if (cv::imwrite(segmented_map_filename.c_str(), colour_segmented_map) == false)
					ROS_ERROR("Error on writing file '%s'", segmented_map_filename.c_str());

				std::string sequence_map_filename = lower_path + evaluation_data.map_name_ + "_sequence.png";
				cv_bridge::CvImagePtr cv_ptr_seq;
				cv_ptr_seq = cv_bridge::toCvCopy(result_seq->sequence_map, sensor_msgs::image_encodings::BGR8);
				cv::Mat sequence_map = cv_ptr_seq->image;
				// draw in trash bins
				for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
#if CV_MAJOR_VERSION<=3
					cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), CV_FILLED);
#else
					cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), cv::FILLED);
#endif
				if (cv::imwrite(sequence_map_filename.c_str(), sequence_map) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_map_filename.c_str());
				std::string sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail.png";
				if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
				sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail2.png";
				if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map2) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
			}

			return true;
		}

	// variation 2: use trashbins to plan cliques and limit the size of the cliques to a defined value
	bool evaluateAllConfigsVer2(const std::vector<EvaluationConfig>& evaluation_configuration_vector, const EvaluationData& evaluation_data, const std::string& data_storage_path)
		{
			// go through each configuration for the given map
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

				std::cout << "\nCurrent Configuration:" << std::endl << "map: " << evaluation_data.map_name_ << "\tsegmentation algorithm: "
					<< evaluation_configuration_vector[config].room_segmentation_algorithm_
					<< "\tplanning method: " << evaluation_configuration_vector[config].sequence_planning_method_
					<< "\tTSP solver: " << evaluation_configuration_vector[config].tsp_solver_
					<< "\tmaximal clique length: " << evaluation_configuration_vector[config].max_clique_path_length_ << std::endl;

				DynamicReconfigureClient drc(node_handle_, "/room_sequence_planning/room_sequence_planning_server/set_parameters", "/room_sequence_planning/room_sequence_planning_server/parameter_updates");
				drc.setConfig("maximum_clique_size", 10);

				AStarPlanner planner;
				//variables for time measurement
				struct timespec t0, t1, t2, t3;

				// 1. retrieve segmentation and check if the map has already been segmented
				ipa_building_msgs::MapSegmentationResultConstPtr result_seg;
				clock_gettime(CLOCK_MONOTONIC,  &t0); //set time stamp before the segmentation
				if(evaluation_configuration_vector[config].room_segmentation_algorithm_ == 1)
				{
					if(segmented_morph == false)
					{
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_morph, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_dist, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vor, t0) == false)
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
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_semant, t0) == false)
							return false;
						segmented_semant = true;
					}
					else
						std::cout << "map has already been segmented" << std::endl;
					result_seg = result_seg_semant;
				}
				else if (evaluation_configuration_vector[config].room_segmentation_algorithm_ == 5)
				{
					if(segmented_vrf == false)
					{
						if (segmentFloorPlan(evaluation_data, evaluation_configuration_vector[config], result_seg_vrf, t0) == false)
							return false;
						segmented_vrf = true;
					}
					else
						std::cout << "map has already been segmented" << std::endl;
					result_seg = result_seg_vrf;
				}
				clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the segmentation
				std::cout << "Segmentation computed " << result_seg->room_information_in_pixel.size() << " rooms." << std::endl;

				//check for accessibility of the room centers from start position
				cv::Mat downsampled_map;
				Timer tim;
				planner.downsampleMap(evaluation_data.floor_plan_, downsampled_map, evaluation_data.map_downsampling_factor_, evaluation_data.robot_radius_, evaluation_data.map_resolution_);
				std::cout << "downsampling map: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
				cv::Point robot_start_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
												(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);

				// get the reachable room centers as cv::Point
				tim.start();
				std::vector<cv::Point> room_centers;
				for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
				{
					cv::Point current_center(result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
					double length = planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_start_position, current_center, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_);
					if(length < 1e9)
						room_centers.push_back(current_center);
					else
						std::cout << "room " << i << " not accessible, center: " << current_center << std::endl;
				}
				std::cout << "room centers computed: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;

				if(room_centers.size() == 0) //no room center is reachable for the given start position --> needs to be looked at separately
				{
					std::cout << "++++++++++ no roomcenter reachable from given startposition ++++++++++++" << std::endl;
					return false;
				}

				// 2. solve sequence problem
				std::cout << "Starting to solve sequence problem." << std::endl;
				tim.start();
				ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_seq;
				clock_gettime(CLOCK_MONOTONIC,  &t2); //set time stamp before the sequence planning
				if (computeRoomSequence(evaluation_data, evaluation_configuration_vector[config], evaluation_data.trash_bin_locations_, result_seq, t2) == false)
				{
					std::cout << "++++++++++ computeRoomSequence failed ++++++++++++" << std::endl;
					return false;
				}
				std::cout << "sequence problem solved: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
				clock_gettime(CLOCK_MONOTONIC,  &t3); //set time stamp after the sequence planning

				// converting the map msg in cv format
				cv_bridge::CvImagePtr cv_ptr_obj;
				cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
				cv::Mat segmented_map = cv_ptr_obj->image;

				// 3. find rooms that belonging to a trolley position
				std::vector<int> assigned_rooms;
				std::vector<std::vector<cv::Point> > rooms_for_trolley_position(result_seq->checkpoints.size());

				for(size_t checkpoint = 0; checkpoint < result_seq->checkpoints.size(); ++checkpoint)
				{
					std::vector<uint> current_trashbins = result_seq->checkpoints[checkpoint].room_indices;

					for(size_t trash = 0; trash < current_trashbins.size(); ++trash)
					{
						int current_room_index = segmented_map.at<int>(evaluation_data.trash_bin_locations_[current_trashbins[trash]]);

						// find romm center that is assigned with this room index
						int room_vector_index = -1;
						for(size_t room = 0; room < room_centers.size(); ++room)
						{
							if (segmented_map.at<int>(room_centers[room]) == current_room_index)
							{
								room_vector_index = room;
								break;
							}
						}

						// only save room centers that are reachable and haven't been saved yet
						if(room_vector_index != -1)
						{
							if(contains(rooms_for_trolley_position[checkpoint], room_centers[room_vector_index]) == false)
								rooms_for_trolley_position[checkpoint].push_back(room_centers[room_vector_index]);

							// save room as assigned, segmented map starts with index 1 for rooms
							if(contains(assigned_rooms, current_room_index) == false)
								assigned_rooms.push_back(current_room_index);
						}
					}
				}

				// 4. check if a room has no trashbins in it, if so it hasn't been assigned to a trolley position yet and needs
				//	  to be assigned now
				for(size_t room = 0; room < room_centers.size(); ++room)
				{
					int current_room_index = segmented_map.at<int>(room_centers[room]);

					if(contains(assigned_rooms, current_room_index) == false)
					{
						// find the trolley which is nearest to this center and assign the room to this trolley position
						double smallest_distance = 1e6;
						size_t trolley_index = 0;
						for(size_t checkpoint = 0; checkpoint < result_seq->checkpoints.size(); ++checkpoint)
						{
							cv::Point trolley_position = cv::Point(result_seq->checkpoints[checkpoint].checkpoint_position_in_pixel.x, result_seq->checkpoints[checkpoint].checkpoint_position_in_pixel.y);
							double current_distance = planner.planPath(evaluation_data.floor_plan_, downsampled_map, room_centers[room], trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_);;

							if(current_distance < smallest_distance)
							{
								smallest_distance = current_distance;
								trolley_index = checkpoint;
							}
						}

						// assign the room to the trolley
						rooms_for_trolley_position[trolley_index].push_back(room_centers[room]);
					}
				}

				// 5. do the movements
				drc.setConfig("maximum_clique_size", 9001);
				cv::Mat draw_path_map, draw_path_map2;
				cv::cvtColor(evaluation_data.floor_plan_, draw_path_map, CV_GRAY2BGR);
				draw_path_map2 = draw_path_map.clone();
				double path_length_robot = 0.;
				double path_length_trolley = 0.;
				double path_length_trash_bins = 0.;
	//			const double max_clique_path_length_in_pixel = evaluation_configuration.max_clique_path_length_ / evaluation_data.map_resolution_;
				cv::Point robot_position = robot_start_position;
				cv::Point trolley_position((evaluation_data.robot_start_position_.position.x - evaluation_data.map_origin_.position.x)/evaluation_data.map_resolution_,
						(evaluation_data.robot_start_position_.position.y - evaluation_data.map_origin_.position.y)/evaluation_data.map_resolution_);
				cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,0,255), -1);

				std::stringstream screenoutput;
				std::vector<int> done_rooms;
				for (size_t clique_index = 0; clique_index<result_seq->checkpoints.size(); ++clique_index)
				{
					std::cout << "cleaning new clique" << std::endl; screenoutput << "cleaning new clique" << std::endl;
					// mark new trolley position to empty the trashbins
					trolley_position = cv::Point(result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.x, result_seq->checkpoints[clique_index].checkpoint_position_in_pixel.y);
					cv::circle(draw_path_map, trolley_position, 3, CV_RGB(0,0,255), -1);
					// compute optimal room visiting order
					std::vector<cv::Point> current_room_order;
					ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr result_room_seq;
					if(computeTrashBinSequence(evaluation_data, evaluation_configuration_vector[config], rooms_for_trolley_position[clique_index], robot_position, result_room_seq) == true)
					{
						for (size_t cc=0; cc<result_room_seq->checkpoints.size(); ++cc)
							for (size_t tt = 0; tt < result_room_seq->checkpoints[cc].room_indices.size(); ++tt)
								current_room_order.push_back(rooms_for_trolley_position[clique_index][result_room_seq->checkpoints[cc].room_indices[tt]]);

					}


					// move robot to rooms if they haven't been cleaned yet
					for(size_t room = 0; room < current_room_order.size(); ++room)
					{
						cv::Point current_roomcenter = current_room_order[room];

						// find room that contains this trashbin
						int room_index = segmented_map.at<int>(current_roomcenter);
						std::cout << "room index: " << room_index << std::endl;

						// check if room has been visited already, if so only empty trashbins in this room
						if(contains(done_rooms, room_index) == false)
						{
							// room has not been cleaned yet
							path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, current_roomcenter, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
							robot_position = current_roomcenter;
							cv::circle(draw_path_map, robot_position, 3, CV_RGB(0,255,0), -1);

							// mark room as cleaned
							done_rooms.push_back(room_index);
						}

						// find trashbins in the current room that belong to the current trolley checkpoint
						// note: when iterating from the start of the optimal trashbin sequence of one trolleyposition the
						// order of the trashbins after this asignment should be optimal
						std::vector<cv::Point> trashbins_for_room;
						for(size_t trash = 0; trash < result_seq->checkpoints[clique_index].room_indices.size(); ++trash)
						{
							cv::Point trashbin = evaluation_data.trash_bin_locations_[result_seq->checkpoints[clique_index].room_indices[trash]];
							if(room_index == segmented_map.at<int>(trashbin))
							{
								// empty trashbin if it belongs to this room
//								trashbins_for_room.push_back(trashbin);
								path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, trashbin, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
								path_length_trash_bins += 2.0 * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trashbin, trolley_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);
								robot_position = trashbin;
								cv::circle(draw_path_map, trashbin, 2, CV_RGB(128,0,255), -1);
							}
						}
					}
					std::cout << " cleaned all rooms and trash bins in current clique" << std::endl; screenoutput << " cleaned all rooms and trash bins in current clique";
				}
				// finally go back to start position
				path_length_robot += planner.planPath(evaluation_data.floor_plan_, downsampled_map, robot_position, robot_start_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1, &draw_path_map);

				// evaluation
				double path_length_robot_in_meter = path_length_robot * evaluation_data.map_resolution_;
				double path_length_trolley_in_meter = path_length_trolley * evaluation_data.map_resolution_;
				double path_length_total_in_meter = path_length_robot_in_meter;
				double path_length_trash_bins_in_meter = path_length_trash_bins * evaluation_data.map_resolution_;
				double robot_speed_without_trolley = 0.3;		// [m/s]
				double robot_speed_with_trolley = 0.2;			// [m/s]
				double time_for_trashbin_manipulation = 150;	// [s], without driving
				double time_for_trolley_manipulation = 90;		// [s], without driving
				double time = path_length_robot_in_meter / robot_speed_without_trolley
							+ time_for_trashbin_manipulation * evaluation_data.trash_bin_locations_.size();

				//get the runtimes for the segmentation and the sequence planner
				double segmentation_time = (t1.tv_sec - t0.tv_sec) + (double) (t1.tv_nsec - t0.tv_nsec) * 1e-9;
				double sequence_time = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;

				// write log file
				std::stringstream output;
				// header
				output << evaluation_configuration_vector[config].getConfigurationString();
				output << "robot_speed_without_trolley" << "\t" << "robot_speed_with_trolley" << "\t" << "time_for_trashbin_manipulation" << "\t"
						<< "time_for_trolley_manipulation" << "\t" << "number_of_trash_bins" << "\t" << "number_trolley_movements" << "\t"
						<< "path_length_robot_in_meter" << "\t" << "path_length_trolley_in_meter" << "\t" << "path_length_trash_bins_in_meter\t"
						<< "pathlength [m]" << "\t"
						<< "cleaning time [s]" << "\t" << "calculation time segmentation [s]" << "\t" << "calculation time sequencer [s]" << "\t" << std::endl;
				output << robot_speed_without_trolley << "\t" << robot_speed_with_trolley << "\t" << time_for_trashbin_manipulation << "\t" << time_for_trolley_manipulation << "\t"
						<< evaluation_data.trash_bin_locations_.size() << "\t"
						<< 0 << "\t" << path_length_robot_in_meter << "\t" << path_length_trolley_in_meter << "\t" << path_length_trash_bins_in_meter << "\t"
						<< path_length_total_in_meter << "\t"
						<< time << "\t" << segmentation_time << "\t" << sequence_time;
				output << "\n\n\n" << screenoutput.str() << std::endl;

				std::string log_filename = lower_path + evaluation_data.map_name_ + "_results.txt";
				std::ofstream file(log_filename.c_str(), std::ios::out);
				if (file.is_open()==true)
					file << output.str();
				else
					ROS_ERROR("Error on writing file '%s'", log_filename.c_str());
				file.close();

				// compute travel distance for humans and save them
				double travel_distance_human = 0.0;
				for(size_t pos = 0; pos < result_seq->checkpoints.size(); ++pos)
				{
					cv::Point trolley_placing_position = cv::Point(result_seq->checkpoints[pos].checkpoint_position_in_pixel.x, result_seq->checkpoints[pos].checkpoint_position_in_pixel.y);
					travel_distance_human += 2.0 * planner.planPath(evaluation_data.floor_plan_, downsampled_map, trolley_placing_position, robot_start_position, evaluation_data.map_downsampling_factor_, 0., evaluation_data.map_resolution_, 1);
				}
				travel_distance_human = travel_distance_human * evaluation_data.map_resolution_;
				std::string storage_path_human = data_storage_path + "human_way/" + upper_folder_name + evaluation_configuration_vector[config].generateLowerConfigurationFolderString() + "/";
				const std::string upper_human_command = "mkdir -p " + storage_path_human;
				return_value = system(upper_human_command.c_str());
				storage_path_human = storage_path_human + evaluation_data.map_name_ + "_human.txt";
				std::stringstream human_distance_output;
				human_distance_output << "dist to place all trolleys [m]: " << std::endl << travel_distance_human;
				std::ofstream human_file(storage_path_human.c_str(), std::ios::out);
				if (human_file.is_open()==true)
					human_file << human_distance_output.str();
				else
					ROS_ERROR("Error on writing file '%s'", storage_path_human.c_str());
				human_file.close();

				// images: segmented_map, sequence_map
				std::string segmented_map_filename = lower_path + evaluation_data.map_name_ + "_segmented.png";
				cv::Mat colour_segmented_map = segmented_map.clone();
				colour_segmented_map.convertTo(colour_segmented_map, CV_8U);
				cv::cvtColor(colour_segmented_map, colour_segmented_map, CV_GRAY2BGR);
				for(size_t i = 1; i <= result_seg->room_information_in_pixel.size(); ++i)
				{
					// choose random color for each room
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
				// draw the room centers into the map
				for(size_t i = 0; i < result_seg->room_information_in_pixel.size(); ++i)
				{
					cv::Point current_center (result_seg->room_information_in_pixel[i].room_center.x, result_seg->room_information_in_pixel[i].room_center.y);
#if CV_MAJOR_VERSION<=3
					cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), CV_FILLED);
#else
					cv::circle(colour_segmented_map, current_center, 2, CV_RGB(0,0,255), cv::FILLED);
#endif
				}
				// color image in unique colour to show the segmentation
				if (cv::imwrite(segmented_map_filename.c_str(), colour_segmented_map) == false)
					ROS_ERROR("Error on writing file '%s'", segmented_map_filename.c_str());

				std::string sequence_map_filename = lower_path + evaluation_data.map_name_ + "_sequence.png";
				cv_bridge::CvImagePtr cv_ptr_seq;
				cv_ptr_seq = cv_bridge::toCvCopy(result_seq->sequence_map, sensor_msgs::image_encodings::BGR8);
				cv::Mat sequence_map = cv_ptr_seq->image;
				// draw in trash bins
				for (size_t i=0; i<evaluation_data.trash_bin_locations_.size(); ++i)
#if CV_MAJOR_VERSION<=3
					cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), CV_FILLED);
#else
					cv::circle(sequence_map, evaluation_data.trash_bin_locations_[i], 2, CV_RGB(128,0,255), cv::FILLED);
#endif
				if (cv::imwrite(sequence_map_filename.c_str(), sequence_map) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_map_filename.c_str());
				std::string sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail.png";
				if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
				sequence_detail_map_filename = lower_path + evaluation_data.map_name_ + "_sequence_detail2.png";
				if (cv::imwrite(sequence_detail_map_filename.c_str(), draw_path_map2) == false)
					ROS_ERROR("Error on writing file '%s'", sequence_detail_map_filename.c_str());
			}

			return true;
		}

	bool segmentFloorPlan(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			ipa_building_msgs::MapSegmentationResultConstPtr& result_seg, struct timespec& t0)
	{
		int loopcounter = 0;
		bool segmented = false;
		do
		{
			clock_gettime(CLOCK_MONOTONIC, &t0); //set time stamp before the segmentation
			sensor_msgs::Image map_msg;
			cv_bridge::CvImage cv_image;
			cv_image.encoding = "mono8";
			cv_image.image = evaluation_data.floor_plan_;
			cv_image.toImageMsg(map_msg);
			actionlib::SimpleActionClient<ipa_building_msgs::MapSegmentationAction> ac_seg("/room_segmentation/room_segmentation_server", true);
			ROS_INFO("Waiting for action server '/room_segmentation/room_segmentation_server' to start.");
			ac_seg.waitForServer(ros::Duration(60)); // wait for the action server to start, will wait for infinite time
			std::cout << "Action server started, sending goal_seg." << std::endl;
			ros::Duration s(0.5);
			s.sleep();

			// send dynamic reconfigure config
			ROS_INFO("Trying to connect to dynamic reconfigure server.");
			DynamicReconfigureClient drc(node_handle_, "/room_segmentation/room_segmentation_server/set_parameters", "/room_segmentation/room_segmentation_server/parameter_updates");
			ROS_INFO("Done connecting to the dynamic reconfigure server.");
			const int room_segmentation_algorithm = evaluation_configuration.room_segmentation_algorithm_;
			drc.setConfig("room_segmentation_algorithm", room_segmentation_algorithm);
			if(room_segmentation_algorithm == 1) //morpho
			{
				drc.setConfig("room_area_factor_lower_limit_morphological", 0.8);
				drc.setConfig("room_area_factor_upper_limit_morphological", 47.0);
				ROS_INFO("You have chosen the morphological segmentation.");
			}
			if(room_segmentation_algorithm == 2) //distance
			{
				drc.setConfig("room_area_factor_lower_limit_distance", 0.35);
				drc.setConfig("room_area_factor_upper_limit_distance", 163.0);
				ROS_INFO("You have chosen the distance segmentation.");
			}
			if(room_segmentation_algorithm == 3) //voronoi
			{
				drc.setConfig("room_area_factor_lower_limit_voronoi", 0.1);	//1.53;
				drc.setConfig("room_area_factor_upper_limit_voronoi", 1000000.);	//120.0;
				drc.setConfig("voronoi_neighborhood_index", 280);
				drc.setConfig("max_iterations", 150);
				drc.setConfig("min_critical_point_distance_factor", 0.5); //1.6;
				drc.setConfig("max_area_for_merging", 12.5);
				ROS_INFO("You have chosen the Voronoi segmentation");
			}
			if(room_segmentation_algorithm == 4) //semantic
			{
				drc.setConfig("room_area_factor_lower_limit_semantic", 1.0);
				drc.setConfig("room_area_factor_upper_limit_semantic", 1000000.);//23.0;
				ROS_INFO("You have chosen the semantic segmentation.");
			}
			if(room_segmentation_algorithm == 5) //voronoi random field
			{
				drc.setConfig("room_area_lower_limit_voronoi_random", 1.53); //1.53
				drc.setConfig("room_area_upper_limit_voronoi_random", 1000000.); //1000000.0
				drc.setConfig("max_iterations", 150);
				drc.setConfig("voronoi_random_field_epsilon_for_neighborhood", 7);
				drc.setConfig("min_neighborhood_size", 5);
				drc.setConfig("min_voronoi_random_field_node_distance", 7.0); // [pixel]
				drc.setConfig("max_voronoi_random_field_inference_iterations", 9000);
				drc.setConfig("max_area_for_merging", 12.5);
				ROS_INFO("You have chosen the Voronoi random field segmentation.");
			}
			drc.setConfig("display_segmented_map", false);

			// send a goal to the action
			ipa_building_msgs::MapSegmentationGoal goal_seg;
			goal_seg.input_map = map_msg;
			goal_seg.map_origin = evaluation_data.map_origin_;
			goal_seg.map_resolution = evaluation_data.map_resolution_;
			goal_seg.return_format_in_meter = false;
			goal_seg.return_format_in_pixel = true;
			//goal_seg.room_segmentation_algorithm = evaluation_configuration.room_segmentation_algorithm_;
			goal_seg.robot_radius = evaluation_data.robot_radius_;
			ac_seg.sendGoal(goal_seg);

			//wait for the action to return
			bool finished_before_timeout = ac_seg.waitForResult(ros::Duration(600.0 + loopcounter * 100.0));
			if (finished_before_timeout == false) //if it takes too long the server should be killed and restarted
			{
				std::cout << "action server took too long" << std::endl;
				std::string pid_cmd = "pidof room_segmentation_server > room_sequence_planning/seg_srv_pid.txt";
				int pid_result = system(pid_cmd.c_str());
				std::ifstream pid_reader("room_sequence_planning/seg_srv_pid.txt");
				int value;
				std::string line;
				if (pid_reader.is_open())
				{
					while (getline(pid_reader, line))
					{
						std::istringstream iss(line);
						while (iss >> value)
						{
							std::cout << "PID of room_segmentation_server: " << value << std::endl;
							std::stringstream ss;
							ss << "kill " << value;
							std::string kill_cmd = ss.str();
							int kill_result = system(kill_cmd.c_str());
							std::cout << "kill result: " << kill_result << std::endl;
						}
					}
					pid_reader.close();
					remove("room_sequence_planning/seg_srv_pid.txt");
				}
				else
				{
					std::cout << "missing logfile" << std::endl;
				}
			}
			else // segmentation finished while given time --> return result
			{
				result_seg = ac_seg.getResult();
				segmented = true;
				std::cout << "Finished segmentation successfully!" << std::endl;
			}
			++loopcounter; //enlarge the loop counter so the client will wait longer for the server to start
		}while(segmented == false && loopcounter <= 6);

		if(loopcounter > 6)
			return false;

		return true;
	}

	bool computeTrashBinSequence(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			const std::vector<cv::Point>& reachable_roomcenters, const cv::Point& robot_start_position,
			ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr& result_seq)
	{
		bool planned = false;

		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = evaluation_data.floor_plan_;
		cv_image.toImageMsg(map_msg);

		actionlib::SimpleActionClient<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction> ac_seq("/room_sequence_planning/room_sequence_planning_server", true);
		ROS_INFO("Waiting for action server '/room_sequence_planning/room_sequence_planning_server' to start.");
		// wait for the action server to start
		ac_seq.waitForServer(ros::Duration(3.0)); //will wait for infinite time
		ROS_INFO("Action server for trashbin sequence planning found.");
		ros::Duration s(0.5);
		s.sleep();

		//put the vector<Point> format in the msg format
		std::vector<ipa_building_msgs::RoomInformation> roomcenters_for_sequence_planning(reachable_roomcenters.size());
		for(size_t room = 0; room < reachable_roomcenters.size(); ++room)
		{
			roomcenters_for_sequence_planning[room].room_center.x = reachable_roomcenters[room].x;
			roomcenters_for_sequence_planning[room].room_center.y = reachable_roomcenters[room].y;
		}

		// set algorithm parameters
		ROS_INFO("Trying to connect to dynamic reconfigure server.");
		DynamicReconfigureClient drc(node_handle_, "/room_sequence_planning/room_sequence_planning_server/set_parameters", "/room_sequence_planning/room_sequence_planning_server/parameter_updates");
		ROS_INFO("Done connecting to the dynamic reconfigure server.");
		drc.setConfig("max_clique_path_length", 1e9);
		drc.setConfig("map_downsampling_factor", evaluation_data.map_downsampling_factor_);
		drc.setConfig("planning_method", 1);
		drc.setConfig("tsp_solver", evaluation_configuration.tsp_solver_);
		drc.setConfig("return_sequence_map", false);
		drc.setConfig("check_accessibility_of_rooms", false);

//		std::cout << "Action server started, sending goal_seq." << std::endl;
		// send a goal_seg to the action
		ipa_building_msgs::FindRoomSequenceWithCheckpointsGoal goal_seq;
		goal_seq.input_map = map_msg;
		goal_seq.map_resolution = evaluation_data.map_resolution_;
		goal_seq.map_origin = evaluation_data.map_origin_;
		goal_seq.room_information_in_pixel = roomcenters_for_sequence_planning;
		goal_seq.robot_radius = 0.;
		goal_seq.robot_start_coordinate.position.x = robot_start_position.x*evaluation_data.map_resolution_ + evaluation_data.map_origin_.position.x;
		goal_seq.robot_start_coordinate.position.y = robot_start_position.y*evaluation_data.map_resolution_ + evaluation_data.map_origin_.position.y;
		ac_seq.sendGoal(goal_seq);

		//wait for the action to return
		bool finished_before_timeout = ac_seq.waitForResult(ros::Duration(30.0 ));
		if (finished_before_timeout == true) //if it takes too long the server should be killed and restarted
		{
			result_seq = ac_seq.getResult();
//			std::cout << "Finished trash bin sequence planning successfully!" << std::endl;
			planned = true;
		}

		return planned;
	}

	bool computeRoomSequence(const EvaluationData& evaluation_data, const EvaluationConfig& evaluation_configuration,
			const std::vector<cv::Point>& reachable_roomcenters,
			ipa_building_msgs::FindRoomSequenceWithCheckpointsResultConstPtr& result_seq, struct timespec& t2)
	{
		bool planned = false;
		int loopcounter = 0;
		do
		{
			clock_gettime(CLOCK_MONOTONIC,  &t2);
			sensor_msgs::Image map_msg;
			cv_bridge::CvImage cv_image;
			cv_image.encoding = "mono8";
			cv_image.image = evaluation_data.floor_plan_;
			cv_image.toImageMsg(map_msg);

			actionlib::SimpleActionClient<ipa_building_msgs::FindRoomSequenceWithCheckpointsAction> ac_seq("/room_sequence_planning/room_sequence_planning_server", true);
			ROS_INFO("Waiting for action server '/room_sequence_planning/room_sequence_planning_server' to start.");
			// wait for the action server to start
			ac_seq.waitForServer(ros::Duration(60)); //will wait for infinite time
			ros::Duration s(0.5);
			s.sleep();

			//put the vector<Point> format in the msg format
			std::vector<ipa_building_msgs::RoomInformation> roomcenters_for_sequence_planning(reachable_roomcenters.size());
			for(size_t room = 0; room < reachable_roomcenters.size(); ++room)
			{
				roomcenters_for_sequence_planning[room].room_center.x = reachable_roomcenters[room].x;
				roomcenters_for_sequence_planning[room].room_center.y = reachable_roomcenters[room].y;
			}

			// set algorithm parameters
			ROS_INFO("Trying to connect to dynamic reconfigure server.");
			DynamicReconfigureClient drc(node_handle_, "/room_sequence_planning/room_sequence_planning_server/set_parameters", "/room_sequence_planning/room_sequence_planning_server/parameter_updates");
			ROS_INFO("Done conencting to the dynamic reconfigure server.");
			drc.setConfig("max_clique_path_length", evaluation_configuration.max_clique_path_length_);
			drc.setConfig("map_downsampling_factor", evaluation_data.map_downsampling_factor_);
			drc.setConfig("planning_method", evaluation_configuration.sequence_planning_method_);
			drc.setConfig("tsp_solver", evaluation_configuration.tsp_solver_);
			drc.setConfig("return_sequence_map", true);
			drc.setConfig("check_accessibility_of_rooms", false);

			std::cout << "Action server started, sending goal_seq." << std::endl;
			// send a goal_seg to the action
			ipa_building_msgs::FindRoomSequenceWithCheckpointsGoal goal_seq;
			goal_seq.input_map = map_msg;
			goal_seq.map_resolution = evaluation_data.map_resolution_;
			goal_seq.map_origin = evaluation_data.map_origin_;
			goal_seq.room_information_in_pixel = roomcenters_for_sequence_planning;
			goal_seq.robot_radius = robot_radius_;
			goal_seq.robot_start_coordinate = evaluation_data.robot_start_position_;
			ac_seq.sendGoal(goal_seq);

			//wait for the action to return
			bool finished_before_timeout = ac_seq.waitForResult(ros::Duration(2400.0 + 200 * loopcounter));
			if (finished_before_timeout == false) //if it takes too long the server should be killed and restarted
			{
				std::cout << "action server took too long" << std::endl;
				std::string pid_cmd = "pidof room_sequence_planning_server > room_sequence_planning/seq_srv_pid.txt";
				int pid_result = system(pid_cmd.c_str());
				std::ifstream pid_reader("room_sequence_planning/seq_srv_pid.txt");
				int value;
				std::string line;
				if (pid_reader.is_open())
				{
					while (getline(pid_reader, line))
					{
						std::istringstream iss(line);
						while (iss >> value)
						{
							std::cout << "PID of room_sequence_planning_server: " << value << std::endl;
							std::stringstream ss;
							ss << "kill " << value;
							std::string kill_cmd = ss.str();
							int kill_result = system(kill_cmd.c_str());
							std::cout << "kill result: " << kill_result << std::endl;
						}
					}
					pid_reader.close();
					remove("room_sequence_planning/seq_srv_pid.txt");
				}
				else
				{
					std::cout << "missing logfile" << std::endl;
				}
			}
			else
			{
				result_seq = ac_seq.getResult();
				std::cout << "Finished sequence planning successfully!" << std::endl;
				planned = true;
			}
			++loopcounter;
		}while(planned == false && loopcounter <= 6);

		if(loopcounter > 6)
			return false;

		return true;
	}


private:

	ros::NodeHandle node_handle_;

	std::vector< EvaluationData > evaluation_data_;

	const double robot_radius_;
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_sequence_planning_client");
	ros::NodeHandle nh;

	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string data_storage_path = "room_sequence_planning/";
	Evaluation ev(nh, test_map_path, data_storage_path, 0.3);
	ros::shutdown();

	//exit
	return 0;
}

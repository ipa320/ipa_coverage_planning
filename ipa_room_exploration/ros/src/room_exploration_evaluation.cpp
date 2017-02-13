#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <numeric>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_building_msgs/MapSegmentationAction.h>
#include <ipa_building_msgs/RoomExplorationAction.h>

#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_room_exploration/A_star_pathplanner.h>

#include <time.h>
#include <sys/time.h>
#include <ipa_room_exploration/timer.h>

#include <Eigen/Dense>

#include <boost/regex.hpp>

// Overload of << operator for geometry_msgs::Pose2D to wanted format.
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose2D& obj)
{
	std::stringstream ss;
	ss <<  "[" << obj.x << ", " << obj.y << ", " << obj.theta << "]";
	os << ss.rdbuf();
    return os;
}


// Struct used to define, which segmentation algorithm together with which exploration algorithm should be used. Also a function is
// provided, that returns a string showing the current configuration --> used to save the results.
struct explorationConfig
{
	int exploration_algorithm_;	// this variable selects, which exploration algorithm should be used
									// 1: grid point explorator
									// 2: boustrophedon explorator
									// 3: neural network explorator
									// 4: convexSPP explorator
									// 5: flowNetwork explorator
									// 6: energyFunctional explorator

	// default values --> best ones?
	explorationConfig()
	{
		exploration_algorithm_ = 2;
	}

	// create one configuration
	explorationConfig(const int exploration_algorithm)
	{
		exploration_algorithm_ = exploration_algorithm;
	}

	// function that returns the current configuration as string
	std::string generateConfigurationFolderString() const
	{
		std::stringstream ss;
		ss << "expl" << exploration_algorithm_;
		return ss.str();
	}

	// function that returns the name of the chosen exploration algorithm
	std::string roomExplorationAlgorithmToString() const
	{
		std::string s = "";
		if (exploration_algorithm_ == 1)
			s = "grid point exploration";
		else if (exploration_algorithm_ == 2)
			s = "boustrophedon exploration";
		else if (exploration_algorithm_ == 3)
			s = "neural network exploration";
		else if (exploration_algorithm_ == 4)
			s = "convex SPP exploration";
		else if (exploration_algorithm_ == 5)
			s = "flow network exploration";
		else if (exploration_algorithm_ == 6)
			s = "energy functional exploration";

		return s;
	}
};

// Struct that carries several parameters for the action servers
struct explorationData
{
	std::string map_name_;		// without file type
	cv::Mat floor_plan_;
	float map_resolution_;
	geometry_msgs::Pose map_origin_;
	geometry_msgs::Pose2D robot_start_position_;
	double robot_radius_;
	double robot_speed_; // [m/s]
	double rotation_speed_; // [rad/s]
	std::vector<geometry_msgs::Point32> fow_points_;

	// empty values as default
	explorationData()
	{
		map_name_ = "";
		floor_plan_ = cv::Mat();
		map_resolution_ = 0.05;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		robot_radius_ = 0.8;
		robot_speed_ = 0.3;
		rotation_speed_ = 0.1;
	}

	// set data used in this evaluation
	explorationData(const std::string map_name, const cv::Mat floor_plan, const float map_resolution, const double robot_radius,
			const std::vector<geometry_msgs::Point32>& fow_points)
	{
		map_name_ = map_name;
		floor_plan_ = floor_plan;
		map_resolution_ = map_resolution;
		robot_radius_ = robot_radius;
		robot_speed_ = 0.3;
		rotation_speed_ = 0.1;
		fow_points_ = fow_points;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		cv::Mat map_eroded;
		cv::erode(floor_plan_, map_eroded, cv::Mat(), cv::Point(-1,-1), robot_radius_/map_resolution_+2);
		cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image
		bool robot_start_coordinate_set = false;
		for (int v=0; v<map_eroded.rows && robot_start_coordinate_set==false; ++v)
			for (int u=0; u<map_eroded.cols && robot_start_coordinate_set==false; ++u)
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<uchar>(v,u) > 20)			// TODO: parameter
				{
					robot_start_position_.x = u*map_resolution_ + map_origin_.position.x;
					robot_start_position_.y = v*map_resolution_ + map_origin_.position.y;
					robot_start_coordinate_set = true;
				}
	}
};

struct evaluationResults
{
	double calculation_time;
	double path_length;
	double execution_time;
	cv::Mat room_map, coverage_map;
	int number_of_turns;
};

// class that segments the wanted maps, finds for each resulting room a coverage path and saves these paths
class explorationEvaluation
{
protected:

	// function that creates configurations to get all possible combinations of segmentations and exploration algorithms
	void setconfigurations(std::vector<explorationConfig>& configurations, const std::vector<int>& exploration_algorithms)
	{
		for(std::vector<int>::const_iterator expl=exploration_algorithms.begin(); expl!=exploration_algorithms.end(); ++expl)
		{
			explorationConfig current_config(*expl);
			configurations.push_back(current_config);
		}
	}

public:

	ros::NodeHandle node_handle_;

	explorationEvaluation(ros::NodeHandle& nh, const std::string& test_map_path, const std::string& data_storage_path,
			const double robot_radius, const std::vector<int>& exploration_algorithms, const std::vector<geometry_msgs::Point32>& fow_points)
	{
		// set node-handle
		node_handle_ = nh;

		// prepare relevant floor map data
		std::vector< std::string > map_names;
		map_names.push_back("lab_ipa");
//		map_names.push_back("lab_c_scan");
//		map_names.push_back("Freiburg52_scan");
//		map_names.push_back("Freiburg79_scan");
//		map_names.push_back("lab_b_scan");
//		map_names.push_back("lab_intel");
//		map_names.push_back("Freiburg101_scan");
//		map_names.push_back("lab_d_scan");
//		map_names.push_back("lab_f_scan");
//		map_names.push_back("lab_a_scan");
//		map_names.push_back("NLB");
//		map_names.push_back("office_a");
//		map_names.push_back("office_b");
//		map_names.push_back("office_c");
//		map_names.push_back("office_d");
//		map_names.push_back("office_e");
//		map_names.push_back("office_f");
//		map_names.push_back("office_g");
//		map_names.push_back("office_h");
//		map_names.push_back("office_i");
//		map_names.push_back("lab_ipa_furnitures");
//		map_names.push_back("lab_c_scan_furnitures");
//		map_names.push_back("Freiburg52_scan_furnitures");
//		map_names.push_back("Freiburg79_scan_furnitures");
//		map_names.push_back("lab_b_scan_furnitures");
//		map_names.push_back("lab_intel_furnitures");
//		map_names.push_back("Freiburg101_scan_furnitures");
//		map_names.push_back("lab_d_scan_furnitures");
//		map_names.push_back("lab_f_scan_furnitures");
//		map_names.push_back("lab_a_scan_furnitures");
//		map_names.push_back("NLB_furnitures");
//		map_names.push_back("office_a_furnitures");
//		map_names.push_back("office_b_furnitures");
//		map_names.push_back("office_c_furnitures");
//		map_names.push_back("office_d_furnitures");
//		map_names.push_back("office_e_furnitures");
//		map_names.push_back("office_f_furnitures");
//		map_names.push_back("office_g_furnitures");
//		map_names.push_back("office_h_furnitures");
//		map_names.push_back("office_i_furnitures");

		// create all needed configurations
		std::vector<explorationConfig> configs;
		setconfigurations(configs, exploration_algorithms);

		// prepare images and evaluation datas
		std::vector<explorationData> evaluation_datas;
		for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
		{
			std::string image_filename = test_map_path + map_names[image_index] + ".png";// + "_furnitures_trashbins.png";
			std::cout << "loading image: " << image_filename << std::endl;
			cv::Mat map = cv::imread(image_filename.c_str(), 0);
			//make non-white pixels black
			for (int y=0; y<map.rows; y++)
			{
				for (int x=0; x<map.cols; x++)
				{
					if (map.at<unsigned char>(y, x)>250)
					{
						map.at<unsigned char>(y, x)=255;
					}
					else //if (map.at<unsigned char>(y, x) != 255)
					{
						map.at<unsigned char>(y, x)=0;
					}
				}
			}

			// create evaluation data
			evaluation_datas.push_back(explorationData(map_names[image_index], map, 0.05, robot_radius, fow_points));
		}

		// do the evaluation
//		std::string bugfile = data_storage_path + "bugfile.txt";
//		std::ofstream failed_maps(bugfile.c_str(), std::ios::out);
//		if(failed_maps.is_open())
//			failed_maps << "maps that had a bug during the simulation and couldn't be finished: " << std::endl;
//		ROS_INFO("Evaluating the maps.");
//		for (size_t i=0; i<evaluation_datas.size(); ++i)
//		{
//			if (planCoveragePaths(configs, evaluation_datas[i], data_storage_path)==false)
//			{
//				std::cout << "failed to simulate map " << evaluation_datas[i].map_name_ << std::endl;
//				if(failed_maps.is_open())
//					failed_maps << evaluation_datas[i].map_name_ << std::endl;
//			}
//		}
//		failed_maps.close();

		// read out the computed paths and calculate the evaluation values
		ROS_INFO("Reading out all saved paths.");
		// TODO: finish
		std::vector<evaluationResults> results;
		for (size_t i=0; i<evaluation_datas.size(); ++i)
		{
			evaluateCoveragePaths(configs, evaluation_datas[i], results, data_storage_path);
		}

	}

	// function that does the whole evaluation for all configs
	bool planCoveragePaths(const std::vector<explorationConfig>& configs, const explorationData& datas, const std::string data_storage_path)
	{
		// go trough all configs and do the evaluations
		for(std::vector<explorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			// create a folder for the log directory
			std::string folder_path = config->generateConfigurationFolderString() + "/";
			const std::string upper_command = "mkdir -p " + data_storage_path + folder_path;
			int return_value = system(upper_command.c_str());

			std::cout << "expl: " << config->exploration_algorithm_ << std::endl;
			//variables for time measurement
			struct timespec t0, t1;

			// 1. read out the ground truth map
			std::string map_name_basic = datas.map_name_;
			std::size_t pos = datas.map_name_.find("_furnitures");
			if (pos != std::string::npos)
				map_name_basic = datas.map_name_.substr(0, pos);
			std::string gt_image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name_basic + "_gt_segmentation.png";
			std::cout << "Loading ground truth segmentation from: " << gt_image_filename << std::endl;
			cv::Mat gt_map = cv::imread(gt_image_filename.c_str(), CV_8U);
			cv::threshold(gt_map, gt_map, 250, 255, CV_THRESH_BINARY);


			// 2. retrieve the rooms for each ground truth map and get the maps that show only one room each
			int label = 1;
			std::vector<cv::Rect> bounding_boxes;
			cv::Mat labeled_map;
			gt_map.convertTo(labeled_map, CV_32SC1);
			for (int y = 0; y < gt_map.rows; y++)
			{
				for (int x = 0; x < gt_map.cols; x++)
				{
					if (gt_map.at<uchar>(y,x)!=255 || labeled_map.at<int>(y,x)!=255)
						continue;

					// fill each room area with a unique id
					cv::Rect rect;
					cv::floodFill(labeled_map, cv::Point(x,y), label, &rect, 0, 0, 8);

					// save the bounding box to retrieve the min/max coordinates
					bounding_boxes.push_back(rect);

					++label;
				}
			}
			std::vector<cv::Mat> room_maps;
			std::vector<cv::Rect> chosen_bb;
			for(int room=1; room<label; ++room)
			{
				cv::Mat room_map = cv::Mat(labeled_map.rows, labeled_map.cols, CV_8U, cv::Scalar(0));
				// go trough pixels and make pixels belonging to room white and not belonging pixels black
				for(size_t y=0; y<room_map.rows; ++y)
					for(size_t x=0; x<room_map.cols; ++x)
						if(labeled_map.at<int>(y,x)==room)
							room_map.at<uchar>(y,x) = 255;

				// check for the eroded map (the map that shows the in reality reachable areas) to have enough free pixels
				cv::Mat eroded_map;
				int robot_radius_in_pixel = (datas.robot_radius_ / datas.map_resolution_);
				cv::erode(room_map, eroded_map, cv::Mat(), cv::Point(-1, -1), robot_radius_in_pixel);
				int number_of_pixels = 0;
				for(size_t y=0; y<eroded_map.rows; ++y)
					for(size_t x=0; x<eroded_map.cols; ++x)
						if(eroded_map.at<uchar>(y,x)==255)
							++number_of_pixels;

				// save room map, if region is big enough
				if(number_of_pixels>0)
				{
					room_maps.push_back(room_map);
					chosen_bb.push_back(bounding_boxes[room-1]);
//					cv::rectangle(room_map, bounding_boxes[room-1], cv::Scalar(127), 2);
//					cv::imshow("room", room_map);
//					cv::waitKey();
				}
			}

			// 3. go trough all rooms and find the coverage path trough it
			geometry_msgs::Polygon region_of_interest; // always assume full map in this case
			geometry_msgs::Point32 edge_point;
			edge_point.x = 0;
			edge_point.y = 0;
			region_of_interest.points.push_back(edge_point);
			edge_point.x = gt_map.cols;
			edge_point.y = gt_map.rows;
			region_of_interest.points.push_back(edge_point);
			std::stringstream output;
			for(size_t room_index=0; room_index<room_maps.size(); ++room_index)
			{
				cv::Mat room_map = room_maps[room_index];

				// find min/max coordinates for this room by using the saved bounding box
//				int min_y = 1e5, max_y = 0, min_x = 1e5, max_x = 0;
//				for (int y = 0; y < room_map.rows; y++)
//				{
//					for (int x = 0; x < room_map.cols; x++)
//					{
//						//only check white pixels
//						if(room_map.at<uchar>(y,x)==255)
//						{
//							if(y < min_y)
//								min_y = y;
//							if(y > max_y)
//								max_y = y;
//							if(x < min_x)
//								min_x = x;
//							if(x > max_x)
//								max_x = x;
//						}
//					}
//				}
				int min_y = chosen_bb[room_index].y;
				int max_y = chosen_bb[room_index].y+chosen_bb[room_index].height;
				int min_x = chosen_bb[room_index].x;
				int max_x = chosen_bb[room_index].x+chosen_bb[room_index].width;
				min_y -= 1;
				min_x -= 1;
				max_y += 1;
				max_x += 1;
				std::cout << "min coordinates: " << min_y << ":" << max_y << "(y), " << min_x << ":" << max_x << "(x)" << std::endl;
				geometry_msgs::Polygon min_max_points;
				geometry_msgs::Point32 min_point, max_point;
				min_point.x = min_x;
				min_point.y = min_y;
				max_point.x = max_x;
				max_point.y = max_y;
				min_max_points.points;
				min_max_points.points.push_back(min_point);
				min_max_points.points.push_back(max_point);

				// send the exploration goal
				ipa_building_msgs::RoomExplorationResultConstPtr result_expl;
				if(planCoveragePath(room_map, datas, *config, result_expl, t0, datas.robot_start_position_, min_max_points, region_of_interest)==false)
				{
					output << "room " << room_index << " exceeded the time limitation of 3 hours" << std::endl << std::endl;
					continue;
				}
				clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the path planning

				// retrieve the solution and save the found results
				double calculation_time = (t1.tv_sec - t0.tv_sec) + (double) (t1.tv_nsec - t0.tv_nsec) * 1e-9;
				std::vector<geometry_msgs::Pose2D> coverage_path = result_expl->coverage_path;
				// transform path to map coordinates
				std::cout << "length of path: " << coverage_path.size() << std::endl;
				if(coverage_path.size()==0)
				{
					output << "room " << room_index << " had a bug" << std::endl << std::endl;
					continue;
				}
				for(size_t point=0; point<coverage_path.size(); ++point)
				{
					coverage_path[point].x = (coverage_path[point].x-datas.map_origin_.position.x)/datas.map_resolution_;
					coverage_path[point].y = (coverage_path[point].y-datas.map_origin_.position.y)/datas.map_resolution_;
				}
				output << "calculation time: " << calculation_time << "s" << std::endl;
				for(size_t point=0; point<coverage_path.size(); ++point)
					output << coverage_path[point] << std::endl;
				output << std::endl;
			}
			std::string log_filename = data_storage_path + folder_path + datas.map_name_ + "_results.txt";
			std::cout << log_filename << std::endl;
			std::ofstream file(log_filename.c_str(), std::ios::out);
			if (file.is_open()==true)
				file << output.str();
			else
				ROS_ERROR("Error on writing file '%s'", log_filename.c_str());
			file.close();
		}

		// if all configurations finished, return a boolean showing success
		return true;
	}

	// function that reads out the calculated paths and does the evaluation of the calculated these
	void evaluateCoveragePaths(const std::vector<explorationConfig>& configs, const explorationData& datas,
			std::vector<evaluationResults>& results, const std::string data_storage_path)
	{
		for(std::vector<explorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			// 1. get the location of the results and open this file
			std::string folder_path = config->generateConfigurationFolderString() + "/";
			std::cout << folder_path << datas.map_name_ << std::endl;
			std::string log_filename = data_storage_path + folder_path + datas.map_name_ + "_results.txt";
			std::ifstream reading_file(log_filename.c_str(), std::ios::in);

			// 2. if the file could be opened, read out the given paths for all rooms
			std::vector<std::vector<geometry_msgs::Pose2D> > paths;
			std::vector<std::vector<double> > calculation_times;
			std::vector<geometry_msgs::Pose2D> current_path;
			std::vector<double> current_calculation_times;
			if (reading_file.is_open()==true)
			{
				std::string line;
				bool initial = true;
				while(getline(reading_file, line))
				{
					double calculation_time;
					double x, y, theta;

					// check if the current line is empty --> shows a new room
					if(line.empty()==true)
					{
						// save the previously found calculation times and paths
						paths.push_back(current_path);
						calculation_times.push_back(current_calculation_times);

						// reset temporary vectors
						current_path.clear();
						current_calculation_times.clear();

						// set the flag to a new room
						initial = true;

						// ignore the empty line
						continue;
					}

					// if the new line is the first after an empty line, it contains the calculation time
					if(initial==true)
					{
						// if the time limit was exceeded or a bug appeared, save a -1
						if(line.find("exceeded")!=std::string::npos || line.find("bug")!=std::string::npos)
						{
//							std::cout << "exceeded calculation time" << std::endl;
							calculation_time = -1;
							// save a invalid pose, to show that this room has no coverage path
							geometry_msgs::Pose2D false_pose;
							false_pose.x = -1;
							false_pose.y = -1;
							current_path.push_back(false_pose);
						}
						else
						{
							const char* str = line.c_str();
							sscanf(str, "%*[^0-9]%lf", &calculation_time);
//							std::cout << calculation_time << std::endl;
						}
						current_calculation_times.push_back(calculation_time);
						initial = false;
					}
					// else read out x,y and theta and create a new Pose
					else
					{
						std::istringstream iss(line);
						int pos_counter = 0;
						std::string buffer;

						// get saved output until comma or the end of the line is reached
						while(getline(iss, buffer, ','))
						{
							// remove brackets
							buffer.erase(std::remove(buffer.begin(), buffer.end(), '['), buffer.end());
							buffer.erase(std::remove(buffer.begin(), buffer.end(), ']'), buffer.end());

							// save value regarding the position of it
							if(pos_counter==0)
								x = atof(buffer.c_str());
							else if(pos_counter==1)
								y = atof(buffer.c_str());
							else if(pos_counter==2)
								theta = atof(buffer.c_str());

							// increase position counter
							++pos_counter;

						}
//						std::cout << "x: " << x << ", y: " << y << ", theta: " << theta << std::endl;

						// save the found pose
						if(x>0 && y>0)
						{
							geometry_msgs::Pose2D current_pose;
							current_pose.x = x;
							current_pose.y = y;
							current_pose.theta = theta;
							current_path.push_back(current_pose);
						}
					}
				}
			}
			else
				ROS_WARN("Error on reading file '%s'", log_filename.c_str());
			reading_file.close();

			// 3. do the evaluations
			AStarPlanner path_planner;
			// 3.1 overall, average pathlength and variance of it for the calculated paths and get the numbers of the turns
			cv::Mat map = datas.floor_plan_.clone();
			cv::Mat eroded_map;
			int robot_radius_in_pixel = (datas.robot_radius_ / datas.map_resolution_);
			cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1, -1), robot_radius_in_pixel);
			std::vector<double> pathlengths_for_map;
			std::vector<std::vector<cv::Point> > interpolated_paths; // variable that stores the path points and the points between them
			int nonzero_paths = 0;
			std::vector<double> rotation_values;
			std::vector<int> number_of_rotations = 0;
			for(size_t room=0; room<paths.size(); ++room)
			{
//				std::cout << "room " << room << ", size of path: " << paths[room].size() << std::endl;

				if(paths[room].size()==0)
					continue;
				else
					++nonzero_paths;

				double current_pathlength = 0.0;
				std::vector<cv::Point> current_interpolated_path;
				double previous_angle = 0.0;
				double current_rotation_abs = 0.0;
				int current_number_of_rotations = 0;
				for(std::vector<geometry_msgs::Pose2D>::iterator pose=paths[room].begin(); pose!=paths[room].end()-1; ++pose)
				{
					// if a false pose has been saved, ignore it
					if(pose->x==-1 && pose->y==-1)
						continue;

					// get the angle and check if it the same as before, if not add the rotation
					if(pose-paths[room].begin()!=1) // not for start pose
					{
						double angle_difference = previous_angle-pose->theta;

						if(angle_difference!=0.0)
						{
							++current_number_of_rotations;
							current_rotation_abs += std::abs(angle_difference);
						}
					}
					// save current angle of pose
					previous_angle = pose->theta;

					// create output map to show path
					cv::circle(eroded_map, cv::Point(pose->x, pose->y), 2, cv::Scalar(127), CV_FILLED);
					cv::line(eroded_map, cv::Point(pose->x, pose->y), cv::Point((pose+1)->x, (pose+1)->y), cv::Scalar(100));

					// find pathlength and path between two consecutive poses
					current_pathlength += path_planner.planPath(map, cv::Point(pose->x, pose->y), cv::Point((pose+1)->x, (pose+1)->y), 1.0, 0.0, datas.map_resolution_, 0, &current_interpolated_path);
				}

				// save rotation values
				rotation_values.push_back(current_rotation_abs);
				number_of_rotations.push_back(current_number_of_rotations);

				// save the interpolated path between
				interpolated_paths.push_back(current_interpolated_path);

				// transform the pixel length to meter
				current_pathlength *= datas.map_resolution_;
//				std::cout << "length: " << current_pathlength << "m" << std::endl;
				pathlengths_for_map.push_back(current_pathlength);
//				cv::imshow("room paths", eroded_map);
//				cv::waitKey();
			}

			// save the map with the drawn in coverage paths
			std::stringstream map_filename;
			std::string image_path = data_storage_path + folder_path + datas.map_name_ + "_paths.png";
//			std::cout << image_path << std::endl;
			cv::imwrite(image_path.c_str(), eroded_map);
//			cv::imshow("room paths", room_map);
//			cv::waitKey();

			// calculate the overall pathlength, the average and the variance
			double overall_pathlength = std::accumulate(pathlengths_for_map.begin(), pathlengths_for_map.end(), 0);
			double average_pathlength = overall_pathlength/nonzero_paths;
			double pathlength_variance_squared = 0;
			std::vector<double> travel_times_in_rooms;
			for(std::vector<double>::iterator length=pathlengths_for_map.begin(); length!=pathlengths_for_map.end(); ++length)
			{
				pathlength_variance_squared += std::pow(*length-average_pathlength, 2.0);
				travel_times_in_rooms.push_back(*length/datas.robot_speed_);
			}
			pathlength_variance_squared /= nonzero_paths;

			// 2. calculate the execution time by using the robot speed and the rotation speed
			double average_execution_time = 0.0;
			double execution_time_squared_variance = 0.0;
			double overall_execution_time = overall_pathlength/datas.robot_speed_; // travel time
			std::vector<double> rotation_times_in_rooms;
			for(std::vector<double>::iterator rotation=rotation_values.begin(); rotation!=rotation_values.end(); ++rotation)
			{
				overall_execution_time += *rotation/datas.rotation_speed_;
				rotation_times_in_rooms.push_back(*rotation/datas.rotation_speed_);
			}
			average_execution_time = overall_execution_time/nonzero_paths;
			// compute variance
			for(size_t room=0; room<paths.size(); ++room)
			{
				if(paths[room].size()==0)
					continue;

				execution_time_squared_variance += std::pow(rotation_times_in_rooms[room]+travel_times_in_rooms[room]-average_execution_time, 2.0);
			}

			// TODO: average initial coverage + deviation --> return room maps?
			// 3. coverage when executing the coverage paths
			std::vector<double> area_covered_percentages;

			// TODO: average calculation time + deviation

			// TODO: parallelity?

			// TODO: number of rooms
		}
	}

	// function that plans one coverage path for the given room map
	bool planCoveragePath(const cv::Mat& room_map, const explorationData& evaluation_data, const explorationConfig& evaluation_configuration,
				ipa_building_msgs::RoomExplorationResultConstPtr& result_expl, struct timespec& t0,
				const geometry_msgs::Pose2D& starting_position, const geometry_msgs::Polygon& min_max_points,
				const geometry_msgs::Polygon& region_of_interest)
	{
		clock_gettime(CLOCK_MONOTONIC, &t0); //set time stamp before the path planning
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = room_map;
		cv_image.toImageMsg(map_msg);

		actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac_exp("room_exploration_server", true);
		ROS_INFO("Waiting for action server to start.");
		ac_exp.waitForServer(); //will wait for infinite time
		ROS_INFO("Action server started.");

		ROS_INFO("Trying to connect to dynamic reconfigure server.");
		DynamicReconfigureClient drc_exp(node_handle_, "room_exploration_server/set_parameters", "room_exploration_server/parameter_updates");
		ROS_INFO("Done connecting to the dynamic reconfigure server.");
		if (evaluation_configuration.exploration_algorithm_==1)
		{
			drc_exp.setConfig("room_exploration_algorithm", 1);
			ROS_INFO("You have chosen the grid exploration method.");
		}
		else if(evaluation_configuration.exploration_algorithm_==2)
		{
			drc_exp.setConfig("room_exploration_algorithm", 2);
			ROS_INFO("You have chosen the boustrophedon exploration method.");
		}
		else if(evaluation_configuration.exploration_algorithm_==3)
		{
			drc_exp.setConfig("room_exploration_algorithm", 3);
			ROS_INFO("You have chosen the neural network exploration method.");
		}
		else if(evaluation_configuration.exploration_algorithm_==4)
		{
			drc_exp.setConfig("room_exploration_algorithm", 4);
			ROS_INFO("You have chosen the convexSPP exploration method.");
		}
		else if(evaluation_configuration.exploration_algorithm_==5)
		{
			drc_exp.setConfig("room_exploration_algorithm", 5);
			ROS_INFO("You have chosen the flow network exploration method.");
		}
		else if(evaluation_configuration.exploration_algorithm_==6)
		{
			drc_exp.setConfig("room_exploration_algorithm", 6);
			ROS_INFO("You have chosen the energy functional exploration method.");
		}

		ipa_building_msgs::RoomExplorationGoal goal;
		goal.input_map = map_msg;
		geometry_msgs::Pose2D map_origin;
		map_origin.x = evaluation_data.map_origin_.position.x;
		map_origin.y = evaluation_data.map_origin_.position.y;
		goal.map_origin = map_origin;
		goal.starting_position = evaluation_data.robot_start_position_;
		goal.map_resolution = evaluation_data.map_resolution_;
		goal.robot_radius = evaluation_data.robot_radius_;
		goal.room_min_max = min_max_points;
		goal.field_of_view = evaluation_data.fow_points_;
		goal.region_of_interest_coordinates = region_of_interest;
		goal.return_path = true;
		goal.execute_path = false;
		ac_exp.sendGoal(goal);

		// wait for results for 3 hours
		bool finished;
		// higher timeout for the flowNetworkExplorator, because much slower than the others
		if(evaluation_configuration.exploration_algorithm_==5)
			finished = ac_exp.waitForResult(ros::Duration(10800));
		else
			finished = ac_exp.waitForResult(ros::Duration(1800));


		if (finished == false) //if it takes too long the server should be killed and restarted
		{
			std::cout << "action server took too long" << std::endl;
			std::string pid_cmd = "pidof room_exploration_server > room_exploration_evaluation/expl_srv_pid.txt";
			int pid_result = system(pid_cmd.c_str());
			std::ifstream pid_reader("room_exploration_evaluation/expl_srv_pid.txt");
			int value;
			std::string line;
			if (pid_reader.is_open())
			{
				while (getline(pid_reader, line))
				{
					std::istringstream iss(line);
					while (iss >> value)
					{
						std::cout << "PID of room_exploration_server: " << value << std::endl;
						std::stringstream ss;
						ss << "kill " << value;
						std::string kill_cmd = ss.str();
						int kill_result = system(kill_cmd.c_str());
						std::cout << "kill result: " << kill_result << std::endl;
					}
				}
				pid_reader.close();
				remove("rroom_exploration_evaluation/expl_srv_pid.txt");
			}
			else
			{
				std::cout << "missing logfile" << std::endl;
			}
			return false;
		}
		else
		{
			// retrieve solution
			result_expl = ac_exp.getResult();
			std::cout << "Finished coverage planning successfully!" << std::endl;

			// show success
			return true;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_evaluation");
	ros::NodeHandle nh;

	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string data_storage_path = "room_exploration_evaluation/";
	//explorationEvaluation(ros::NodeHandle& nh, const std::string& test_map_path, const std::string& data_storage_path,
//	const double robot_radius, const std::vector<int>& segmentation_algorithms, const std::vector<int>& exploration_algorithms,
//	const std::vector<geometry_msgs::Point32>& fow_points)
	std::vector<int> exploration_algorithms;
	for(int i=1; i<=6; ++i)
	{
		// choose which algorithms not to evaluate
		if(i==5)
			continue;

		exploration_algorithms.push_back(i);
	}
	geometry_msgs::Point32 fow_point_1;// geometry_msgs::Point32(0.3, 0.3);
	fow_point_1.x = 0.1;
	fow_point_1.y = 0.5;
	geometry_msgs::Point32 fow_point_2;// = geometry_msgs::Point32(0.3, -0.3);
	fow_point_2.x = 0.1;
	fow_point_2.y = -0.5;
	geometry_msgs::Point32 fow_point_3;// = geometry_msgs::Point32(0.7, 0.7);
	fow_point_3.x = 0.6;
	fow_point_3.y = -0.75;
	geometry_msgs::Point32 fow_point_4;// = geometry_msgs::Point32(0.7, -0.7);
	fow_point_4.x = 0.6;
	fow_point_4.y = 0.75;
	std::vector<geometry_msgs::Point32> fow_points(4);
	fow_points[0] = fow_point_1;
	fow_points[1] = fow_point_2;
	fow_points[2] = fow_point_3;
	fow_points[3] = fow_point_4;
	// radius=0.6m
	explorationEvaluation ev(nh, test_map_path, data_storage_path, 0.6, exploration_algorithms, fow_points);
	ros::shutdown();

	//exit
	return 0;
}

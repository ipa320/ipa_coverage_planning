#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

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

#include <time.h>
#include <sys/time.h>
#include <ipa_room_exploration/timer.h>

#include <Eigen/Dense>

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
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation
											// 1 = morphological segmentation
											// 2 = distance segmentation
											// 3 = Voronoi segmentation
											// 4 = semantic segmentation
											// 5 = Voronoi random field segmentation

	// default values --> best ones?
	explorationConfig()
	{
		exploration_algorithm_ = 2;
		room_segmentation_algorithm_ = 3;
	}

	// create one configuration
	explorationConfig(const int exploration_algorithm, const int room_segmentation_algorithm)
	{
		exploration_algorithm_ = exploration_algorithm;
		room_segmentation_algorithm_ = room_segmentation_algorithm;
	}

	// function that returns the current configuration as string
	std::string generateConfigurationFolderString() const
	{
		std::stringstream ss;
		ss << "seg" << room_segmentation_algorithm_ << "expl" << exploration_algorithm_;
		return ss.str();
	}

	// function that returns the name of the chosen segmentation algorithm
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
	}

	// set data used in this evaluation
	explorationData(const std::string map_name, const cv::Mat floor_plan, const float map_resolution, const double robot_radius,
			const std::vector<geometry_msgs::Point32>& fow_points)
	{
		map_name_ = map_name;
		floor_plan_ = floor_plan;
		map_resolution_ = map_resolution;
		robot_radius_ = robot_radius;
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


// class that segments the wanted maps, finds for each resulting room a coverage path and saves these paths
class explorationEvaluation
{
protected:

	// function that creates configurations to get all possible combinations of segmentations and exploration algorithms
	void setconfigurations(std::vector<explorationConfig>& configurations, const std::vector<int>& segmentation_algorithms,
			const std::vector<int>& exploration_algorithms)
	{
		for(std::vector<int>::const_iterator seg=segmentation_algorithms.begin(); seg!=segmentation_algorithms.end(); ++seg)
		{
			for(std::vector<int>::const_iterator expl=exploration_algorithms.begin(); expl!=exploration_algorithms.end(); ++expl)
			{
				explorationConfig current_config(*expl, *seg);
				configurations.push_back(current_config);
			}
		}
	}

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

	ros::NodeHandle node_handle_;

	explorationEvaluation(ros::NodeHandle& nh, const std::string& test_map_path, const std::string& data_storage_path,
			const double robot_radius, const std::vector<int>& segmentation_algorithms, const std::vector<int>& exploration_algorithms,
			const std::vector<geometry_msgs::Point32>& fow_points)
	{
		segmented_morph = false;
		segmented_dist = false;
		segmented_vor = false;
		segmented_semant = false;
		segmented_vrf = false;

		// set node-handle
		node_handle_ = nh;

		// prepare relevant floor map data
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
		setconfigurations(configs, segmentation_algorithms, exploration_algorithms);

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
		std::string bugfile = data_storage_path + "bugfile.txt";
		std::ofstream failed_maps(bugfile.c_str(), std::ios::out);
		if(failed_maps.is_open())
			failed_maps << "maps that had a bug during the simulation and couldn't be finished: " << std::endl;
		std::cout << "evaluating configs" << std::endl;
		for (size_t i=0; i<evaluation_datas.size(); ++i)
		{
			if (evaluateAllConfigs(configs, evaluation_datas[i], data_storage_path)==false)
			{
				std::cout << "failed to simulate map " << evaluation_datas[i].map_name_ << std::endl;
				if(failed_maps.is_open())
					failed_maps << evaluation_datas[i].map_name_ << std::endl;
			}
			//reset booleans to segment the new map
			segmented_morph = false;
			segmented_dist = false;
			segmented_vor = false;
			segmented_semant = false;
			segmented_vrf = false;
		}
		failed_maps.close();
	}

	// function that does the whole evaluation for all configs
	bool evaluateAllConfigs(const std::vector<explorationConfig>& configs, const explorationData& datas, const std::string data_storage_path)
	{
		// go trough all configs and do the evaluations
		for(std::vector<explorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			std::cout << "config: seg" << config->room_segmentation_algorithm_  << ", expl: " << config->exploration_algorithm_ << std::endl;
			//variables for time measurement
			struct timespec t0, t1, t2, t3;

			// 1. retrieve segmentation and check if the map has already been segmented
			ipa_building_msgs::MapSegmentationResultConstPtr result_seg;
			clock_gettime(CLOCK_MONOTONIC,  &t0); //set time stamp before the segmentation
			if(config->room_segmentation_algorithm_ == 1)
			{
				if(segmented_morph == false)
				{
					if (segmentFloorPlan(datas, *config, result_seg_morph, t0) == false)
						return false;
					segmented_morph = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_morph;
			}
			else if (config->room_segmentation_algorithm_ == 2)
			{
				if(segmented_dist == false)
				{
					if (segmentFloorPlan(datas, *config, result_seg_dist, t0) == false)
						return false;
					segmented_dist = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_dist;
			}
			else if (config->room_segmentation_algorithm_ == 3)
			{
				if(segmented_vor == false)
				{
					if (segmentFloorPlan(datas, *config, result_seg_vor, t0) == false)
						return false;
					segmented_vor = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_vor;
			}
			else if (config->room_segmentation_algorithm_ == 4)
			{
				if(segmented_semant == false)
				{
					if (segmentFloorPlan(datas, *config, result_seg_semant, t0) == false)
						return false;
					segmented_semant = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_semant;
			}
			else if (config->room_segmentation_algorithm_ == 5)
			{
				if(segmented_vrf == false)
				{
					if (segmentFloorPlan(datas, *config, result_seg_vrf, t0) == false)
						return false;
					segmented_vrf = true;
				}
				else
					std::cout << "map has already been segmented" << std::endl;
				result_seg = result_seg_vrf;
			}
			clock_gettime(CLOCK_MONOTONIC,  &t1); //set time stamp after the segmentation
			std::cout << "Segmentation computed " << result_seg->room_information_in_pixel.size() << " rooms." << std::endl;

			// 2. retrieve the segmented map and get the maps that show only one room each
			cv_bridge::CvImagePtr cv_ptr_obj;
			cv_ptr_obj = cv_bridge::toCvCopy(result_seg->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_obj->image;
			std::vector<cv::Mat> room_maps;
			for(int room=1; room<=result_seg->room_information_in_pixel.size(); ++room)
			{
				cv::Mat room_map = cv::Mat(segmented_map.rows, segmented_map.cols, CV_8U, cv::Scalar(0));
				// go trough pixels and make pixels belonging to room white and not belonging pixels black
				for(size_t y=0; y<room_map.rows; ++y)
				{
					for(size_t x=0; x<room_map.cols; ++x)
					{
						if(segmented_map.at<int>(y,x)==room)
							room_map.at<uchar>(y,x) = 255;
						else
							room_map.at<uchar>(y,x) = 0;
					}
				}

				// save room map
				room_maps.push_back(room_map);
			}

			// 3. go trough all rooms and find the coverage path trough it
			geometry_msgs::Polygon region_of_interest; // always assume full map in this case
			geometry_msgs::Point32 edge_point;
			edge_point.x = 0;
			edge_point.y = 0;
			region_of_interest.points.push_back(edge_point);
			edge_point.x = segmented_map.cols;
			edge_point.y = segmented_map.rows;
			region_of_interest.points.push_back(edge_point);
			std::stringstream output;
			for(size_t room_index=0; room_index<room_maps.size(); ++room_index)
			{
				cv::Mat room_map = room_maps[room_index];

				// find min/max coordinates for this room
				int min_y = 1e5, max_y = 0, min_x = 1e5, max_x = 0;
				for (int y = 0; y < room_map.rows; y++)
				{
					for (int x = 0; x < room_map.cols; x++)
					{
						//only check white pixels
						if(room_map.at<uchar>(y,x)==255)
						{
							if(y < min_y)
								min_y = y;
							if(y > max_y)
								max_y = y;
							if(x < min_x)
								min_x = x;
							if(x > max_x)
								max_x = x;
						}
					}
				}
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
				if(planCoveragePath(room_map, datas, *config, result_expl, t2, datas.robot_start_position_, min_max_points, region_of_interest)==false)
					return false;
				clock_gettime(CLOCK_MONOTONIC,  &t3); //set time stamp after the path planning

				// retrieve the solution and save the found results
				double calculation_time = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
				std::vector<geometry_msgs::Pose2D> coverage_path = result_expl->coverage_path;
				// transform path to map coordinates
				std::cout << "length of path: " << coverage_path.size() << std::endl;
				if(coverage_path.size()==0)
					return false;
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
			std::string folder_path = config->generateConfigurationFolderString() + "/";
			std::string log_filename = data_storage_path + folder_path + datas.map_name_ + "_results.txt";
			const std::string upper_command = "mkdir -p " + data_storage_path + folder_path;
			int return_value = system(upper_command.c_str());
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

	// function that segments the given floor plan
	bool segmentFloorPlan(const explorationData& evaluation_data, const explorationConfig& evaluation_configuration,
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
				std::ifstream pid_reader("room_exploration/seg_srv_pid.txt");
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
					remove("room_exploration/seg_srv_pid.txt");
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

	// function that segments the current floor plan
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

		// wait for results
		bool finished = ac_exp.waitForResult();

		// if an error occurred, return a boolean showing failure
		if(finished==false)
			return false;

		// retrieve solution
		result_expl = ac_exp.getResult();

		// show success
		return true;
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
	std::vector<int> segmentation_algorithms(5);
	for(int i=1; i<=5; ++i)
		segmentation_algorithms[i-1] = i;
	std::vector<int> exploration_algorithms(6);
	for(int i=1; i<=6; ++i)
		exploration_algorithms[i-1] = i;
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
	// radius=0.7m
	explorationEvaluation ev(nh, test_map_path, data_storage_path, 0.7, segmentation_algorithms, exploration_algorithms, fow_points);
	ros::shutdown();

	//exit
	return 0;
}

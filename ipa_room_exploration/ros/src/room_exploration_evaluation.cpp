/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan, Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 03.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <ros/ros.h>
#include <ros/package.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <angles/angles.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_building_msgs/MapSegmentationAction.h>
#include <ipa_building_msgs/RoomExplorationAction.h>
#include <cob_map_accessibility_analysis/CheckPerimeterAccessibility.h>

#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_building_msgs/CheckCoverage.h>
#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/coverage_check_server.h>

#include <time.h>
#include <sys/time.h>
#include <ipa_room_exploration/timer.h>

#include <Eigen/Dense>

#include <boost/regex.hpp>

#include <cob_map_accessibility_analysis/map_accessibility_analysis.h>

#define PI 3.14159265359

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
struct ExplorationConfig
{
	int exploration_algorithm_;	// this variable selects, which exploration algorithm should be used
									// 1: grid point explorator
									// 2: boustrophedon explorator
									// 3: neural network explorator
									// 4: convexSPP explorator
									// 5: flowNetwork explorator
									// 6: energyFunctional explorator
									// 7: Voronoi explorator

	// default values
	ExplorationConfig()
	{
		exploration_algorithm_ = 2;
	}

	// create one configuration
	ExplorationConfig(const int exploration_algorithm)
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
		else if (exploration_algorithm_ == 7)
			s = "voronoi exploration";

		return s;
	}
};

// Struct that carries several parameters for the action servers
enum PlanningMode {FOOTPRINT=1, FIELD_OF_VIEW=2};
struct ExplorationData
{
	std::string map_name_;		// without file type
	cv::Mat floor_plan_;
	std::vector<cv::Mat> room_maps_;
	std::vector<cv::Rect> bounding_boxes_;
	float map_resolution_;	// [m/pixel]
	geometry_msgs::Pose map_origin_;	// [m]
	geometry_msgs::Pose2D robot_start_position_;
	double robot_radius_;	// [m], effective robot radius, taking the enlargement of the costmap into account, in [meter]
	double coverage_radius_;	// [m], radius that is used to plan the coverage planning for the robot and not the field of view, assuming that the part that needs to cover everything (e.g. the cleaning part) can be represented by a fitting circle (e.g. smaller than the actual part to ensure coverage), in [meter]
	std::vector<geometry_msgs::Point32> fov_points_;	// [m], the points that define the field of view of the robot, relative to the robot center (x-axis points to robot's front side, y-axis points to robot's left side, z-axis upwards), in [meter]
	geometry_msgs::Point32 fov_origin_;		// [m], the mounting position of the camera spanning the field of view, relative to the robot center (x-axis points to robot's front side, y-axis points to robot's left side, z-axis upwards), in [meter]
	enum PlanningMode planning_mode_;	// 1 = plans a path for coverage with the robot footprint, 2 = plans a path for coverage with the robot's field of view
	double robot_speed_; // [m/s]
	double robot_rotation_speed_; // [rad/s]

	// empty values as default
	ExplorationData()
	{
		map_name_ = "";
		floor_plan_ = cv::Mat();
		map_resolution_ = 0.05;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		robot_radius_ = 0.35;
		coverage_radius_ = 0.35;
		planning_mode_ = FOOTPRINT;
		robot_speed_ = 0.3;
		robot_rotation_speed_ = 0.1;
	}

	// set data used in this evaluation
	ExplorationData(const std::string map_name, const cv::Mat floor_plan, const float map_resolution, const double robot_radius,
			const double coverage_radius, const std::vector<geometry_msgs::Point32>& fov_points, const geometry_msgs::Point32& fov_origin,
			const int planning_mode, const double robot_speed, const double robot_rotation_speed)
	{
		map_name_ = map_name;
		floor_plan_ = floor_plan;
		map_resolution_ = map_resolution;
		map_origin_.position.x = 0;
		map_origin_.position.y = 0;
		robot_radius_ = robot_radius;
		coverage_radius_ = coverage_radius;
		fov_points_ = fov_points;
		fov_origin_ = fov_origin;
		planning_mode_ = (PlanningMode)planning_mode;
		robot_speed_ = robot_speed;
		robot_rotation_speed_ = robot_rotation_speed;
		cv::Mat map_eroded;
		cv::erode(floor_plan_, map_eroded, cv::Mat(), cv::Point(-1,-1), robot_radius_/map_resolution_+2);
		cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(map_eroded, distance_map, CV_DIST_L2, 5);
		cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image
		float max_distance = 0;
		for (int v=0; v<map_eroded.rows; ++v)
			for (int u=0; u<map_eroded.cols; ++u)
				if (map_eroded.at<uchar>(v,u) != 0 && distance_map.at<float>(v,u) > max_distance)
				{
					max_distance = distance_map.at<float>(v,u);
					robot_start_position_.x = u*map_resolution_ + map_origin_.position.x;
					robot_start_position_.y = v*map_resolution_ + map_origin_.position.y;
				}
	}
};


// class that segments the wanted maps, finds for each resulting room a coverage path and saves these paths
class ExplorationEvaluation
{
protected:

	// function that creates configurations to get all possible combinations of exploration algorithms
	void setConfigurations(std::vector<ExplorationConfig>& configurations, const std::vector<int>& exploration_algorithms)
	{
		for(std::vector<int>::const_iterator expl=exploration_algorithms.begin(); expl!=exploration_algorithms.end(); ++expl)
		{
			ExplorationConfig current_config(*expl);
			configurations.push_back(current_config);
		}
	}

	template <typename T>
	double stddev(const std::vector<T>& values, const double mean)
	{
		if (values.size() < 2)
			return 0.;
		double stddev = 0.;
		for (typename std::vector<T>::const_iterator val=values.begin(); val!=values.end(); ++val)
			stddev += ((double)*val-mean)*((double)*val-mean);
		return sqrt(stddev)/(values.size()-1);
	}

	ros::NodeHandle node_handle_;

public:


	ExplorationEvaluation(ros::NodeHandle& nh, const std::string& test_map_path, const std::vector<std::string>& map_names, const float map_resolution,
			const std::string& data_storage_path, const double robot_radius, const double coverage_radius,
			const std::vector<geometry_msgs::Point32>& fov_points, const geometry_msgs::Point32& fov_origin, const int planning_mode,
			const std::vector<int>& exploration_algorithms, const double robot_speed, const double robot_rotation_speed, bool do_path_planning=true, bool do_evaluation=true)
	: node_handle_(nh)
	{
		// 1. create all needed configurations
		std::vector<ExplorationConfig> configs;
		setConfigurations(configs, exploration_algorithms);

		// 2. prepare images and evaluation data
		std::vector<ExplorationData> evaluation_data;
		for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
		{
			std::string image_filename = test_map_path + map_names[image_index] + ".png";
			std::cout << "loading image: " << image_filename << std::endl;
			cv::Mat map = cv::imread(image_filename.c_str(), 0);
			//make non-white pixels black
			for (int y=0; y<map.rows; y++)
			{
				for (int x=0; x<map.cols; x++)
				{
					if (map.at<unsigned char>(y, x)>250)
						map.at<unsigned char>(y, x)=255;
					else //if (map.at<unsigned char>(y, x) != 255)
						map.at<unsigned char>(y, x)=0;
				}
			}

			// create evaluation data
			evaluation_data.push_back(ExplorationData(map_names[image_index], map, map_resolution, robot_radius, coverage_radius, fov_points, fov_origin,
					planning_mode, robot_speed, robot_rotation_speed));
		}
		// get the room maps for each evaluation data
		getRoomMaps(evaluation_data);

		// 3. compute exploration paths for each room in the maps
		if (do_path_planning == true)
		{
			std::string bugfile = data_storage_path + "bugfile.txt";
			std::ofstream failed_maps(bugfile.c_str(), std::ios::out);
			if (failed_maps.is_open())
				failed_maps << "Maps that had a bug during the simulation and couldn't be finished: " << std::endl;
			ROS_INFO("Evaluating the maps.");
			for (size_t i=0; i<evaluation_data.size(); ++i)
			{
				std::stringstream error_output;
				if (planCoveragePaths(evaluation_data[i], configs, data_storage_path, error_output)==false)
				{
					std::cout << "failed to compute exploration path for map " << evaluation_data[i].map_name_ << std::endl;
					if (failed_maps.is_open())
						failed_maps << evaluation_data[i].map_name_ << std::endl;
				}
				if (failed_maps.is_open() && error_output.str().length()>1)
					failed_maps << evaluation_data[i].map_name_ << std::endl << error_output.str();
			}
			if (failed_maps.is_open())
				failed_maps.close();
		}

		// 4. evaluate the generated paths
		// read out the computed paths and calculate the evaluation values
		if (do_evaluation == true)
		{
			ROS_INFO("Reading out all saved paths.");
			for (size_t i=0; i<evaluation_data.size(); ++i)
				evaluateCoveragePaths(evaluation_data[i], configs, data_storage_path);
			// accumulate all statistics in one file
			writeCumulativeStatistics(evaluation_data, configs, data_storage_path);
		}
	}

	void getRoomMaps(std::vector<ExplorationData>& data_saver)
	{
		for(std::vector<ExplorationData>::iterator data=data_saver.begin(); data!=data_saver.end(); ++data)
		{
			// 1. read out the ground truth map
			std::string map_name_basic = data->map_name_;
			std::size_t pos = data->map_name_.find("_furnitures");
			if (pos != std::string::npos)
				map_name_basic = data->map_name_.substr(0, pos);
			std::string gt_image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name_basic + "_gt_segmentation.png";
			std::cout << "Loading ground truth segmentation from: " << gt_image_filename << std::endl;
			cv::Mat gt_map = cv::imread(gt_image_filename.c_str(), CV_8U);
			cv::threshold(gt_map, gt_map, 250, 255, CV_THRESH_BINARY);

			// load the original map (without furniture if applicable)
			std::string original_image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name_basic + ".png";
			std::cout << "Loading original image from: " << original_image_filename << std::endl;
			cv::Mat original_map = cv::imread(original_image_filename.c_str(), CV_8U);

			// combine real floor plan (but not its furniture) and gt_map
			for (int y = 0; y < gt_map.rows; y++)
				for (int x = 0; x < gt_map.cols; x++)
					if (original_map.at<uchar>(y,x) <= 250)
						gt_map.at<uchar>(y,x) = 0;

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
				// go trough pixels and make pixels belonging to room white and not belonging pixels black (and consider furniture this time)
				for(size_t y=0; y<room_map.rows; ++y)
					for(size_t x=0; x<room_map.cols; ++x)
						if(labeled_map.at<int>(y,x)==room && data->floor_plan_.at<uchar>(y,x)==255)
							room_map.at<uchar>(y,x) = 255;

				// check for the eroded map (the map that shows the in reality reachable areas) to have enough free pixels
				cv::Mat eroded_map;
				const int robot_radius_in_pixel = (data->robot_radius_ / data->map_resolution_);
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

			// combine real floor plan (now including its furniture) and gt_map
			for (int y = 0; y < gt_map.rows; y++)
				for (int x = 0; x < gt_map.cols; x++)
					if (data->floor_plan_.at<uchar>(y,x) != 255)
						gt_map.at<uchar>(y,x) = 0;

			// save the found room maps and bounding boxes
			data->floor_plan_ = gt_map;
			data->room_maps_ = room_maps;
			data->bounding_boxes_ = chosen_bb;
		}
	}

	// function that does the whole evaluation for all configs on all rooms of one map
	bool planCoveragePaths(ExplorationData& data, const std::vector<ExplorationConfig>& configs, const std::string data_storage_path, std::stringstream& error_output)
	{
		// go trough all configs and do the evaluations
		for(std::vector<ExplorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			// create a folder for the log directory
			const std::string configuration_folder_name = config->generateConfigurationFolderString() + "/";
			const std::string upper_command = "mkdir -p " + data_storage_path + configuration_folder_name;
			int return_value = system(upper_command.c_str());

			std::cout << "Exploration algorithm: " << config->exploration_algorithm_ << std::endl;
			//variables for time measurement
			struct timespec t0, t1;

			// go trough all rooms and find the coverage path trough it
			std::stringstream output;
			cv::Mat path_map = data.floor_plan_.clone();
			for(size_t room_index=0; room_index<data.room_maps_.size(); ++room_index)
			{
				cv::Mat room_map = data.room_maps_[room_index];

				// send the exploration goal
				ipa_building_msgs::RoomExplorationResultConstPtr result_expl;
				clock_gettime(CLOCK_MONOTONIC, &t0); //set time stamp before the path planning
				if(planCoveragePath(room_map, data, *config, result_expl)==false)
				{
					output << "room " << room_index << " exceeded the time limitation for computation" << std::endl << std::endl;
					error_output << "  room " << room_index << " exceeded the time limitation for computation" << std::endl;
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
					output << "room " << room_index << " has zero length path" << std::endl << std::endl;
					error_output << "  room " << room_index << " has zero length path" << std::endl;
					continue;
				}
				const double map_resolution_inv = 1.0/data.map_resolution_;
				for(size_t point=0; point<coverage_path.size(); ++point)
				{
					coverage_path[point].x = (coverage_path[point].x-data.map_origin_.position.x)*map_resolution_inv;
					coverage_path[point].y = (coverage_path[point].y-data.map_origin_.position.y)*map_resolution_inv;
				}
				output << "calculation time: " << calculation_time << "s" << std::endl;
				for(size_t point=0; point<coverage_path.size(); ++point)
					output << coverage_path[point] << std::endl;
				output << std::endl;

				// display path
				//cv::Mat path_map = room_map.clone();
				for (size_t point=0; point<coverage_path.size(); ++point)
				{
					cv::circle(path_map, cv::Point(coverage_path[point].x, coverage_path[point].y), 1, cv::Scalar(196), -1);
					if (point > 0)
						cv::line(path_map, cv::Point(coverage_path[point].x, coverage_path[point].y), cv::Point(coverage_path[point-1].x, coverage_path[point-1].y), cv::Scalar(128), 1);
//					std::cout << "coverage_path[" << point << "]: x=" << coverage_path[point].x << ", y=" << coverage_path[point].y << ", theta=" << coverage_path[point].theta << std::endl;
//					cv::imshow("path", path_map);
//					cv::waitKey();
				}
//				cv::imshow("path", path_map);
//				cv::waitKey();
			}
			const std::string img_filename = data_storage_path + configuration_folder_name + data.map_name_ + "_paths.png";
			cv::imwrite(img_filename.c_str(), path_map);

			const std::string log_filename = data_storage_path + configuration_folder_name + data.map_name_ + "_results.txt";
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

	// function that plans one coverage path for the given room map
	bool planCoveragePath(const cv::Mat& room_map, const ExplorationData& evaluation_data, const ExplorationConfig& evaluation_configuration,
				ipa_building_msgs::RoomExplorationResultConstPtr& result_expl)
	{
		// convert image to message
		sensor_msgs::Image map_msg;
		cv_bridge::CvImage cv_image;
		cv_image.encoding = "mono8";
		cv_image.image = room_map;
		cv_image.toImageMsg(map_msg);

		// initialize action server for room exploration
		actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac_exp("room_exploration_server", true);
		ROS_INFO("Waiting for action server to start.");
		ac_exp.waitForServer(); //will wait for infinite time
		ROS_INFO("Action server started.");

		// connect to dynamic reconfigure and set planning algorithm
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
		else if(evaluation_configuration.exploration_algorithm_==7)
		{
			drc_exp.setConfig("room_exploration_algorithm", 7);
			ROS_INFO("You have chosen the Voronoi exploration method.");
		}

		// prepare and send the action message
		ipa_building_msgs::RoomExplorationGoal goal;
		goal.input_map = map_msg;
		goal.map_resolution = evaluation_data.map_resolution_;
		goal.map_origin = evaluation_data.map_origin_;
		goal.robot_radius = evaluation_data.robot_radius_;
		goal.coverage_radius = evaluation_data.coverage_radius_;
		goal.field_of_view = evaluation_data.fov_points_;
		goal.field_of_view_origin = evaluation_data.fov_origin_;
		goal.planning_mode = evaluation_data.planning_mode_;
		goal.starting_position = evaluation_data.robot_start_position_;
		ac_exp.sendGoal(goal);

		// wait for results for some time
		bool finished = false;
		if(evaluation_configuration.exploration_algorithm_==5)	// different timeout for the flowNetworkExplorator, because it can be much slower than the others
			finished = ac_exp.waitForResult(ros::Duration(600));		// todo: adapt if necessary
		else
			finished = ac_exp.waitForResult(ros::Duration(36000));

		//if it takes too long the server should be killed and restarted
		if (finished == false)
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
				remove("room_exploration_evaluation/expl_srv_pid.txt");
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

	// function that reads out the calculated paths and does the evaluation
	void evaluateCoveragePaths(const ExplorationData& data, const std::vector<ExplorationConfig>& configs, const std::string data_storage_path)
	{
		// evaluate the individual configurations
		for(std::vector<ExplorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			evaluateCoveragePaths(data, *config, data_storage_path);
		}
	}

	// function that reads out the calculated paths and does the evaluation for one configuration
	void evaluateCoveragePaths(const ExplorationData& data, const ExplorationConfig& config, const std::string data_storage_path)
	{
		const std::string configuration_folder_name = config.generateConfigurationFolderString() + "/";
		std::cout << configuration_folder_name << data.map_name_ << std::endl;


		// 1. get the location of the results and open this file, read out the given paths and computation times for all rooms
		std::vector<std::vector<geometry_msgs::Pose2D> > paths;		// in [pixels]
		std::vector<double> calculation_times;
		readResultsFile(data, config, data_storage_path, paths, calculation_times);


		// 2. prepare the data
		cv::Mat map = data.floor_plan_.clone();
		cv::Point2d fov_circle_center_point_in_px;
		double grid_spacing_in_pixel = 0;
		computeFOVCenterAndGridSpacing(data, fov_circle_center_point_in_px, grid_spacing_in_pixel);

		// 3. path map, path length, turns, crossings statistics
		//    overall, average path length and variance of it for the calculated paths and get the numbers of turns
		std::vector<double> pathlengths_for_map;	// in [m], stores the individual path lengths for each trajectory in each room
		std::vector<std::vector<geometry_msgs::Pose2D> > interpolated_paths; // in [m], variable that stores the rough path points of paths and the cell-fine trajectories between them
		int nonzero_paths = 0;
		std::vector<double> rotation_values;
		std::vector<int> number_of_rotations, number_of_crossings;
		cv::Mat path_map = map.clone();
		statisticsPathLengthCrossingsTurns(data, map, path_map, fov_circle_center_point_in_px, paths, interpolated_paths,
				pathlengths_for_map, nonzero_paths, number_of_crossings, rotation_values, number_of_rotations);
		// save the map with the drawn in coverage paths
		const std::string image_path = data_storage_path + configuration_folder_name + data.map_name_ + "_paths_eval.png";
		cv::imwrite(image_path.c_str(), path_map);
//		cv::imshow("path_map", path_map);
//		cv::waitKey();
		// calculate the overall path length, the average and the stddev
		const double pathlength_total = std::accumulate(pathlengths_for_map.begin(), pathlengths_for_map.end(), 0.0);
		const double pathlength_mean = pathlength_total / std::max(1.0, (double)pathlengths_for_map.size());
		const double pathlength_stddev = stddev(pathlengths_for_map, pathlength_mean);
		std::cout << "Computing and drawing paths completed." << std::endl;


		// 4. calculate turn specific values
		const double rotation_values_total = std::accumulate(rotation_values.begin(), rotation_values.end(), 0.0);
		const double rotation_values_mean = rotation_values_total / std::max(1.0, (double)rotation_values.size());
		const double rotation_values_stddev = stddev(rotation_values, rotation_values_mean);
		const double number_of_rotations_total = std::accumulate(number_of_rotations.begin(), number_of_rotations.end(), 0.0);
		const double number_of_rotations_mean = number_of_rotations_total / std::max(1.0, (double)number_of_rotations.size());
		const double number_of_rotations_stddev = stddev(number_of_rotations, number_of_rotations_mean);


		// 5. calculate the execution time by using the robot speed and the rotation speed
		std::vector<double> travel_times_in_rooms(pathlengths_for_map.size());
		for(size_t i=0; i<pathlengths_for_map.size(); ++i)
			travel_times_in_rooms[i] = pathlengths_for_map[i]/data.robot_speed_ + rotation_values[i]/data.robot_rotation_speed_;
		const double execution_time_total = std::accumulate(travel_times_in_rooms.begin(), travel_times_in_rooms.end(), 0.0);
		const double execution_time_mean = execution_time_total / std::max(1.0, (double)travel_times_in_rooms.size());
		const double execution_time_stddev = stddev(travel_times_in_rooms, execution_time_mean);


		// 6. coverage percentage and number of covering each pixel when executing the coverage paths, room area
		std::vector<double> room_areas;		// in [m^2], the area of each room
		std::vector<double> area_covered_percentages;	// in [0,1], the ratio of coverage for each room
		std::vector<double> numbers_of_coverages;		// counts how often a map cell has been covered
		cv::Mat map_coverage;
		cv::Mat map_path_coverage;
		statisticsCoverageArea(data, map, path_map, map_coverage, map_path_coverage, paths, interpolated_paths, room_areas, area_covered_percentages, numbers_of_coverages);
		// save the map with the drawn in coverage areas
		const std::string coverage_image_path = data_storage_path + configuration_folder_name + data.map_name_ + "_coverage_eval.png";
		cv::imwrite(coverage_image_path.c_str(), map_coverage);
		// save the map with the drawn in path and coverage areas
		const std::string path_coverage_image_path = data_storage_path + configuration_folder_name + data.map_name_ + "_coverage_paths_eval.png";
		cv::imwrite(path_coverage_image_path.c_str(), map_path_coverage);
		// calculate average coverage and deviation
		const double room_area_mean = std::accumulate(room_areas.begin(), room_areas.end(), 0.0) / std::max(1.0, (double)room_areas.size());
		const double coverage_percentage_mean = std::accumulate(area_covered_percentages.begin(), area_covered_percentages.end(), 0.0) / std::max(1.0, (double)area_covered_percentages.size());
		const double coverage_percentage_stddev = stddev(area_covered_percentages, coverage_percentage_mean);
		const double coverage_number_mean = std::accumulate(numbers_of_coverages.begin(), numbers_of_coverages.end(), 0.0) / std::max(1.0, (double)numbers_of_coverages.size());
		const double coverage_number_stddev = stddev(numbers_of_coverages, coverage_number_mean);
		std::cout << "Checked coverage for all rooms." << std::endl;


		// 7. compute average computation time and deviation
		const double computation_time_mean = std::accumulate(calculation_times.begin(), calculation_times.end(), 0.0) / std::max(1.0, (double)calculation_times.size());
		const double computation_time_stddev = stddev(calculation_times, computation_time_mean);


		// 8. parallelism: for each part of the path calculate the parallelism with respect to the nearest wall and the nearest trajectory part
		std::vector<double> wall_angle_score_means, trajectory_angle_score_means;	// score for parallelism of trajectory to walls and previous parts of the trajectory itself, in range [0,1], high values are good
		std::vector<double> revisit_time_means; // vector that stores the index-differences of the current pose and the point of its nearest neighboring trajectory, values in [0,1], low values are good
		statisticsParallelism(data, map, path_map, paths, interpolated_paths, grid_spacing_in_pixel, wall_angle_score_means, trajectory_angle_score_means, revisit_time_means);
		// calculate mean and stddev of the wall angle scores
		const double wall_angle_score_mean = std::accumulate(wall_angle_score_means.begin(), wall_angle_score_means.end(), 0.0) / std::max(1.0, (double)wall_angle_score_means.size());
		const double wall_angle_score_stddev = stddev(wall_angle_score_means, wall_angle_score_mean);
		// calculate mean and stddev of the trajectory angle scores
		const double trajectory_angle_score_mean = std::accumulate(trajectory_angle_score_means.begin(), trajectory_angle_score_means.end(), 0.0) / std::max(1.0, (double)trajectory_angle_score_means.size());
		const double trajectory_angle_score_stddev = stddev(trajectory_angle_score_means, trajectory_angle_score_mean);
		// calculate mean and stddev of the trajectory revisit times
		const double revisit_time_mean = std::accumulate(revisit_time_means.begin(), revisit_time_means.end(), 0.0) / std::max(1.0, (double)revisit_time_means.size());;
		const double revisit_time_stddev = stddev(revisit_time_means, revisit_time_mean);
		std::cout << "Checked parallelism for all rooms." << std::endl;


		// 9. calculate the number of crossings related values
		const double crossings_mean = std::accumulate(number_of_crossings.begin(), number_of_crossings.end(), 0.0) / std::max(1.0, (double)number_of_crossings.size());
		const double crossings_stddev = stddev(number_of_crossings, crossings_mean);

		// 10. calculate the subjective measure for the paths
		const double subjective_measure_mean = wall_angle_score_mean + trajectory_angle_score_mean
				-0.25*(crossings_mean/room_area_mean + pathlength_mean/room_area_mean + rotation_values_mean/(PI*room_area_mean) + revisit_time_mean);


		// ---------- 11. store all results to a file ----------
		// print the found average evaluation values to a local file
		std::stringstream output;
		output << "Expl" << config.exploration_algorithm_ << ", number of rooms: " << paths.size() << ", number of valid paths: "
				<< nonzero_paths << std::endl;
		output << "calculation time average [s]\t" << "calculation time stddev [s]\t"
				<< "pathlength total [m]\t" << "pathlength average [m]\t" << "pathlength stddev [m]\t"
				<< "rotation values total [rad]\t" << "rotation values average [rad]\t" << "rotation values stddev [rad]\t"
				<< "number of rotations total\t" << "number of rotations average\t" << "number of rotations stddev\t"
				<< "execution time total [s]\t" << "execution time average [s]\t" << "execution time stddev\t"
				<< "covered area average [0,1]\t" << "covered area stddev\t" << "coverage per map cell average\t" << "coverage per map cell deviation\t"
				<< "number of crossings average\t" << "number of crossings stddev\t"
				<< "wall angle score average\t" << "wall angle score stddev\t"
				<< "trajectory angle score average\t" << "trajectory angle score stddev\t"
				<< "average time until traj. is near previous traj.\t" << "stddev of revisit time\t"
				<< "subjective measure\t"<< std::endl;
		output << computation_time_mean << "\t" << computation_time_stddev << "\t"
				<< pathlength_total << "\t" << pathlength_mean << "\t" << pathlength_stddev << "\t"
				<< rotation_values_total << "\t" << rotation_values_mean << "\t" << rotation_values_stddev << "\t"
				<< number_of_rotations_total << "\t" << number_of_rotations_mean << "\t" << number_of_rotations_stddev << "\t"
				<< execution_time_total << "\t" << execution_time_mean << "\t" << execution_time_stddev << "\t"
				<< coverage_percentage_mean << "\t" << coverage_percentage_stddev << "\t" << coverage_number_mean << "\t" << coverage_number_stddev << "\t"
				<< crossings_mean << "\t" << crossings_stddev << "\t"
				<< wall_angle_score_mean << "\t" << wall_angle_score_stddev << "\t"
				<< trajectory_angle_score_mean << "\t" << trajectory_angle_score_stddev << "\t"
				<< revisit_time_mean << "\t" << revisit_time_stddev << "\t"
				<< subjective_measure_mean;
		std::string filename = data_storage_path + configuration_folder_name + data.map_name_ + "_results_eval.txt";
		std::ofstream file(filename.c_str(), std::ofstream::out);
		if (file.is_open())
			file << output.str();
		else
			ROS_ERROR("Could not write to file '%s'.", filename.c_str());
		file.close();

		// print detailed information for each room to a separate file
		if (calculation_times.size()!=pathlengths_for_map.size() || calculation_times.size()!=rotation_values.size() ||
			calculation_times.size()!=area_covered_percentages.size() || calculation_times.size()!= room_areas.size() ||
			calculation_times.size()!=trajectory_angle_score_means.size() || calculation_times.size()!= wall_angle_score_means.size() || calculation_times.size()!= revisit_time_means.size() ||
			calculation_times.size()!=number_of_crossings.size() || calculation_times.size()!=number_of_rotations.size())
		{
			std::cout << "Error in evaluation: array sizes do not match:\n calculation_times.size()=" << calculation_times.size()
					<< "\n pathlengths_for_map.size()=" << pathlengths_for_map.size() << "\n rotation_values.size()=" << rotation_values.size()
					<< "\n area_covered_percentages.size()=" << area_covered_percentages.size() << "\n room_areas.size()=" << room_areas.size()
					<< "\n trajectory_angle_score_means.size()=" << trajectory_angle_score_means.size() << "\n room_wall_averages.size()=" << wall_angle_score_means.size()
					<< "\n room_revisit_averages.size()=" << revisit_time_means.size() << "\n numbers_of_crossings.size()=" << number_of_crossings.size()
					<< "\n number_of_rotations.size()=" << number_of_rotations.size() << std::endl;
		}
		std::stringstream output2;
		for (size_t i=0; i<pathlengths_for_map.size(); ++i)
		{
			output2 << calculation_times[i] << "\t" << pathlengths_for_map[i] << "\t" << rotation_values[i] << "\t" << area_covered_percentages[i]
					<< "\t" << room_areas[i] << "\t" << trajectory_angle_score_means[i] << "\t" << wall_angle_score_means[i] << "\t" << revisit_time_means[i]
					<< "\t" << number_of_crossings[i] << "\t" << number_of_rotations[i] << std::endl;
		}
		filename = data_storage_path + configuration_folder_name + data.map_name_ + "_results_eval_per_room.txt";
		file.open(filename.c_str(), std::ofstream::out);
		if (file.is_open())
			file << output2.str();
		else
			ROS_ERROR("Could not write to file '%s'.", filename.c_str());
		file.close();
	}

	bool readResultsFile(const ExplorationData& data, const ExplorationConfig& config, const std::string data_storage_path,
			std::vector<std::vector<geometry_msgs::Pose2D> >& paths, std::vector<double>& calculation_times)
	{
		// 1. get the location of the results and open this file
		const std::string configuration_folder_name = config.generateConfigurationFolderString() + "/";
		std::string log_filename = data_storage_path + configuration_folder_name + data.map_name_ + "_results.txt";
		std::cout << "Reading file " << log_filename << std::endl;
		std::ifstream reading_file(log_filename.c_str(), std::ios::in);

		// 2. if the file could be opened, read out the given paths for all rooms
		std::vector<geometry_msgs::Pose2D> current_path;
		if (reading_file.is_open()==true)
		{
			std::string line;
			bool initial = true;
			while(getline(reading_file, line))
			{
				// check if the current line is empty --> shows a new room
				if(line.empty()==true)
				{
					// save the previously found calculation times and paths
					paths.push_back(current_path);
					// reset temporary vectors
					current_path.clear();
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
						//std::cout << "bug or exceeded calculation time" << std::endl;
						// set max calculation time
//							if(config.exploration_algorithm_==5) // higher max time for flowNetwork explorator
//								calculation_time = 10800;
//							else
//								calculation_time = 1800;

						// save a invalid pose, to show that this room has no coverage path
						geometry_msgs::Pose2D false_pose;
						false_pose.x = -1;
						false_pose.y = -1;
						current_path.push_back(false_pose);
					}
					else
					{
						const char* str = line.c_str();
						double calculation_time = 0.;
						sscanf(str, "%*[^0-9]%lf", &calculation_time);
						calculation_times.push_back(calculation_time);
						//std::cout << "calculation time: " << calculation_time << "s" << std::endl;
					}
					initial = false;
				}
				// else read out x,y and theta and create a new Pose
				else
				{
					double x=0., y=0., theta=0.;
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
					//std::cout << "   x: " << x << ", y: " << y << ", theta: " << theta << std::endl;

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
		{
			ROS_WARN("Error on reading file '%s'", log_filename.c_str());
			return false;
		}
		reading_file.close();

		std::cout << "Finished reading file " << log_filename << std::endl;

		return true;
	}

	void computeFOVCenterAndGridSpacing(const ExplorationData& data, cv::Point2d& fov_circle_center_point_in_px, double& grid_spacing_in_pixel)
	{
		// find the middle-point distance of the given field of view
		std::vector<Eigen::Matrix<float, 2, 1> > fov_corners_meter(4);
		for(int i = 0; i < 4; ++i)
			fov_corners_meter[i] << data.fov_points_[i].x, data.fov_points_[i].y;
		float fitting_circle_radius_in_meter=0;
		Eigen::Matrix<float, 2, 1> fitting_circle_center_point_in_meter;
		computeFOVCenterAndRadius(fov_corners_meter, fitting_circle_radius_in_meter, fitting_circle_center_point_in_meter, 1000);
		// convert the middle-point to pixel measures
		fov_circle_center_point_in_px = cv::Point2d(fitting_circle_center_point_in_meter(0,0)/data.map_resolution_, fitting_circle_center_point_in_meter(1,0)/data.map_resolution_);
		// determine the grid spacing
		double grid_spacing_in_meter = 0.0;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius
		if(data.planning_mode_ == FIELD_OF_VIEW)	// derive grid spacing from FOV
		{
			// get the edge length of the grid square that fits into the fitting_circle_radius
			grid_spacing_in_meter = fitting_circle_radius_in_meter*std::sqrt(2);
		}
		else	// if planning should be done for the footprint, read out the given coverage radius
		{
			grid_spacing_in_meter = data.coverage_radius_*std::sqrt(2);
		}
		grid_spacing_in_pixel = grid_spacing_in_meter/data.map_resolution_;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius, multiply with sqrt(2) to receive the whole working width
		std::cout << "grid size: " << grid_spacing_in_meter << " m   (" << grid_spacing_in_pixel << " px)" << std::endl;
	}

	// compute the direction of the gradient for each pixel and save the occurring gradients
	cv::Mat computeGradientMap(const cv::Mat& map, bool return_angles=false)
	{
		// calculate the gradient x/y directions for each pixel in the map
		cv::Mat gradient_x, gradient_y;
		cv::Mat gradient_map;
		cv::Sobel(map, gradient_x, CV_64F, 1, 0, 3, 1.0, 0.0, cv::BORDER_DEFAULT);
		cv::Sobel(map, gradient_y, CV_64F, 0, 1, 3, 1.0, 0.0, cv::BORDER_DEFAULT);

		if (return_angles==true)
		{
			gradient_map = cv::Mat(map.rows, map.cols, CV_64F, cv::Scalar(0));
			// compute the direction of the gradient for each pixel and save the occurring gradients
			for(size_t y=0; y<map.rows; ++y)
			{
				for(size_t x=0; x<map.cols; ++x)
				{
					int dx = gradient_x.at<double>(y,x);
					int dy = gradient_y.at<double>(y,x);
					if(dy*dy+dx*dx!=0)
					{
						double current_gradient = std::atan2(dy, dx);
						gradient_map.at<double>(y,x) = current_gradient;
					}
				}
			}
		}
		else
		{
			std::vector<cv::Mat> channels;
			channels.push_back(gradient_x);
			channels.push_back(gradient_y);
			cv::merge(channels, gradient_map);
		}
		return gradient_map;
	}

	// path map, path length, turns, crossings statistics
	void statisticsPathLengthCrossingsTurns(const ExplorationData& data, const cv::Mat& map, cv::Mat& path_map, const cv::Point2d& fov_circle_center_point_in_px,
			const std::vector<std::vector<geometry_msgs::Pose2D> >& paths, std::vector<std::vector<geometry_msgs::Pose2D> >& interpolated_paths,
			std::vector<double>& pathlengths_for_map, int& nonzero_paths, std::vector<int>& number_of_crossings,
			std::vector<double>& rotation_values, std::vector<int>& number_of_rotations)
	{
		AStarPlanner path_planner;
		MapAccessibilityAnalysis map_accessibility_analysis;
		cv::Mat inflated_map;
		const int robot_radius_in_pixel = floor(data.robot_radius_ / data.map_resolution_);
		map_accessibility_analysis.inflateMap(map, inflated_map, robot_radius_in_pixel);
		interpolated_paths.resize(paths.size());

		// draw paths
		for(size_t room=0; room<paths.size(); ++room)
		{
			//std::cout << "room " << room << ", size of path: " << paths[room].size() << std::endl;

			// check for false pose
			if(paths[room].size()==0 || (paths[room][0].x==-1 && paths[room][0].y==-1))
			{
				std::cout << "room " << room << " has invalid trajectory." << std::endl;
				continue;
			}
			else
				++nonzero_paths;

			// initialize statistics
			double current_pathlength = 0.0;
			double current_rotation_abs = 0.0;
			int current_number_of_rotations = 0, current_number_of_crossings = 0;

			// initialize path
			geometry_msgs::Pose2D current_pose_px;	// in [pixels]
			size_t initial_pose_index = 0;
			bool found_initial_pose = false;
			for (; initial_pose_index<paths[room].size(); ++initial_pose_index)		// try different starting poses until one works
			{
				// shift starting pose into accessible area
				current_pose_px = paths[room][initial_pose_index];
				if(current_pose_px.x==-1 && current_pose_px.y==-1)
				{
					ROS_WARN("ExplorationEvaluation:evaluateCoveragePaths: current_pose_px.x==-1 && current_pose_px.y==-1 --> this should never happen.");
					continue;
				}
				found_initial_pose = findAccessiblePose(inflated_map, current_pose_px, current_pose_px, data, fov_circle_center_point_in_px, false);
				if (found_initial_pose == true)
					break;
			}
			if(found_initial_pose == false)
			{
				// if any starting pose works, use first pose for further computations to achieve proper error handling
				ROS_WARN("ExplorationEvaluation:evaluateCoveragePaths: No starting position is accessible.");
				current_pose_px = paths[room][0];
				size_t initial_pose_index = 0;
			}
			geometry_msgs::Pose2D initial_pose_m;	// in [m]
			initial_pose_m.x = (current_pose_px.x*data.map_resolution_)+data.map_origin_.position.x;
			initial_pose_m.y = (current_pose_px.y*data.map_resolution_)+data.map_origin_.position.y;
			initial_pose_m.theta = current_pose_px.theta;
			std::vector<geometry_msgs::Pose2D> current_pose_path_meter;	// in [m,m,rad]
			current_pose_path_meter.push_back(initial_pose_m);

			// loop through trajectory points
			for(std::vector<geometry_msgs::Pose2D>::const_iterator pose_px=paths[room].begin()+initial_pose_index+1; pose_px!=paths[room].end(); ++pose_px)
			{
				// if a false pose has been saved, skip it
				if(current_pose_px.x==-1 && current_pose_px.y==-1)
				{
					ROS_WARN("ExplorationEvaluation:evaluateCoveragePaths: current_pose_px.x==-1 && current_pose_px.y==-1 --> this should never happen.");
					continue;
				}

				// find an accessible next pose
				geometry_msgs::Pose2D next_pose_px = *pose_px;
				bool found_next = findAccessiblePose(inflated_map, current_pose_px, next_pose_px, data, fov_circle_center_point_in_px);
				if(found_next==false)
				{
					std::cout << "   skipping next_pose_px=(" << next_pose_px.x << "," << next_pose_px.y << ") inaccessible from current_pose_px=(" << current_pose_px.x << "," << current_pose_px.y << ")" << std::endl;
					continue;	// if no accessible position could be found, go to next possible path point
				}

				// find pathlength and path between two consecutive poses
				std::vector<cv::Point> current_interpolated_path;	// vector that stores the current path from one pose to another
				const cv::Point current_pose_px_pt(current_pose_px.x, current_pose_px.y);
				const cv::Point next_pose_px_pt(next_pose_px.x, next_pose_px.y);
				// first query for direct current_pose_px_pt to next_pose_px_pt connection
				double length_planner = generateDirectConnection(inflated_map, current_pose_px_pt, next_pose_px_pt, current_interpolated_path);
				if (length_planner < 0.)  // kind of a hack: if there is no accessible connection between two points, try to find a path on the original (not inflated) map, this path could possibly not be driven by the robot in reality
					length_planner = generateDirectConnection(map, current_pose_px_pt, next_pose_px_pt, current_interpolated_path);
				// use A* if there is no direct connection
				if (length_planner < 0.)
					length_planner = path_planner.planPath(inflated_map, current_pose_px_pt, next_pose_px_pt, 1.0, 0.0, data.map_resolution_, 0, &current_interpolated_path);
				// kind of a hack: if there is no accessible connection between two points, try to find a path on the original (not inflated) map, this path could possibly not be driven by the robot in reality
				if (current_interpolated_path.size()==0)
					length_planner = path_planner.planPath(map, current_pose_px_pt, next_pose_px_pt, 1.0, 0.0, data.map_resolution_, 0, &current_interpolated_path);
				current_pathlength += (length_planner>1e90 || length_planner<0 ? cv::norm(cv::Point(next_pose_px.x-current_pose_px.x, next_pose_px.y-current_pose_px.y)) : length_planner);

				// if there is any proper connection between the two points, just use the goal point as "path"
				if (current_interpolated_path.size()<2)
				{
					current_interpolated_path.push_back(cv::Point(current_pose_px.x, current_pose_px.y));
					current_interpolated_path.push_back(cv::Point(next_pose_px.x, next_pose_px.y));
				}

				// transform the cv::Point path to geometry_msgs::Pose2D --> last point has, first point was already gone a defined angle
				// also create output map to show path --> and check if one point has already been visited
				bool has_crossing = false;
				//cv::circle(path_map, cv::Point(next_pose_px.x, next_pose_px.y), 1, cv::Scalar(196), CV_FILLED);
				for(std::vector<cv::Point>::iterator point=current_interpolated_path.begin()+1; point!=current_interpolated_path.end(); ++point)
				{
					// check if point has been visited before and draw point into map
					if(path_map.at<uchar>(*point)==127)
						has_crossing = true;
					else
						path_map.at<uchar>(*point)=127;

					// transform to world coordinates
					geometry_msgs::Pose2D current_pose;
					current_pose.x = (point->x*data.map_resolution_)+data.map_origin_.position.x;
					current_pose.y = (point->y*data.map_resolution_)+data.map_origin_.position.y;
					current_pose.theta = 0;		// the angles are computed afterwards with some smoothing interpolation

					// add the pose to the path
					current_pose_path_meter.push_back(current_pose);
				}
				if (has_crossing == true)
					++current_number_of_crossings;

				// set robot_position to new one
				current_pose_px = next_pose_px;
			}

			// angles and turn: compute the angles along the pixel-wise path and add to the cumulative rotation
			const int offset = 2;
			for (size_t i=0; i<current_pose_path_meter.size(); ++i)
			{
				// compute angle as direction between point 2 steps previous and point 2 steps ahead in the current path
				const geometry_msgs::Pose2D& pose_1 = current_pose_path_meter[std::max(int(i)-offset,0)];
				const geometry_msgs::Pose2D& pose_2 = current_pose_path_meter[std::min(i+offset,current_pose_path_meter.size()-1)];
				current_pose_path_meter[i].theta = std::atan2(pose_2.y-pose_1.y, pose_2.x-pose_1.x);

				// determine angle differences for the statistics
				if (i>1)
				{
					double angle_difference = current_pose_path_meter[i].theta - current_pose_path_meter[i-1].theta;
					angle_difference = std::abs(angles::normalize_angle(angle_difference));
					current_rotation_abs += angle_difference;
					if (angle_difference > 0.52)		// only count substantial rotations > 30deg
						++current_number_of_rotations;
				}
			}

			// save number of crossings of the path
			number_of_crossings.push_back(current_number_of_crossings);

			// save rotation values
			rotation_values.push_back(current_rotation_abs);
			number_of_rotations.push_back(current_number_of_rotations);

			// save the interpolated path between
			interpolated_paths[room]=current_pose_path_meter;

			// transform the pixel length to meter
			current_pathlength *= data.map_resolution_;
			pathlengths_for_map.push_back(current_pathlength);
		}
	}

	void statisticsCoverageArea(const ExplorationData& data, const cv::Mat& map, const cv::Mat& path_map, cv::Mat& map_coverage, cv::Mat& map_path_coverage,
			const std::vector<std::vector<geometry_msgs::Pose2D> >& paths, const std::vector<std::vector<geometry_msgs::Pose2D> >& interpolated_paths,
			std::vector<double>& room_areas, std::vector<double>& area_covered_percentages, std::vector<double>& numbers_of_coverages)
	{
		map_coverage = map.clone();
		for(size_t room=0; room<paths.size(); ++room)
		{
			// ignore paths with size 0 or wrong data
			if(paths[room].size()==0 || (paths[room][0].x==-1 && paths[room][0].y==-1))
				continue;

			// map that has the covered areas drawn in
			cv::Mat coverage_map, number_of_coverage_image;

			// use the coverage check server to check which areas have been seen
			//   --> convert path to cv format
			std::vector<cv::Point3d> path;
			for (size_t i=0; i<interpolated_paths[room].size(); ++i)
				path.push_back(cv::Point3d(interpolated_paths[room][i].x, interpolated_paths[room][i].y, interpolated_paths[room][i].theta));
			//   --> convert field of view to Eigen format
			std::vector<Eigen::Matrix<float, 2, 1> > field_of_view;
			for(size_t i = 0; i < data.fov_points_.size(); ++i)
			{
				Eigen::Matrix<float, 2, 1> current_vector;
				current_vector << data.fov_points_[i].x, data.fov_points_[i].y;
				field_of_view.push_back(current_vector);
			}
			//   --> convert field of view origin to Eigen format
			Eigen::Matrix<float, 2, 1> fov_origin;
			fov_origin << data.fov_origin_.x, data.fov_origin_.y;
			//   --> call coverage checker
			CoverageCheckServer coverage_checker;
			if (coverage_checker.checkCoverage(data.room_maps_[room], data.map_resolution_, cv::Point2d(data.map_origin_.position.x, data.map_origin_.position.y),
					path, field_of_view, fov_origin, data.coverage_radius_, (data.planning_mode_==FOOTPRINT), true, coverage_map, number_of_coverage_image) == true)
			{
				for (int v=0; v<coverage_map.rows; ++v)
					for (int u=0; u<coverage_map.cols; ++u)
						if (coverage_map.at<uchar>(v,u)==127)
							map_coverage.at<uchar>(v,u)=208;
			}
			else
			{
				ROS_INFO("Error when calling the coverage check server.");
			}

			// service interface - can be deleted
//			// use the coverage check server to check which areas have been seen
//			ipa_building_msgs::CheckCoverageRequest coverage_request;
//			ipa_building_msgs::CheckCoverageResponse coverage_response;
//			// fill request
//			std::string coverage_service_name = "/room_exploration/coverage_check_server/coverage_check";
//		//	cv::Mat eroded_room_map;
//		//	cv::erode(data.room_maps_[room], eroded_room_map, cv::Mat(), cv::Point(-1, -1), robot_radius_in_pixel);
//			sensor_msgs::ImageConstPtr service_image;
//			cv_bridge::CvImage cv_image;
//			cv_image.encoding = "mono8";
//			cv_image.image = data.room_maps_[room];	//eroded_room_map;
//			service_image = cv_image.toImageMsg();
//			coverage_request.map_resolution = data.map_resolution_;
//			coverage_request.input_map = *service_image;
//			coverage_request.map_origin = data.map_origin_;
//			coverage_request.path = interpolated_paths[room];
//			coverage_request.field_of_view = data.fov_points_;
//			coverage_request.field_of_view_origin = data.fov_origin_;
//			coverage_request.coverage_radius = data.coverage_radius_;
//			if (data.planning_mode_ == FOOTPRINT)
//				coverage_request.check_for_footprint = true;
//			else if (data.planning_mode_ == FIELD_OF_VIEW)
//				coverage_request.check_for_footprint = false;
//			coverage_request.check_number_of_coverages = true;
//			// send request
//			if(ros::service::call(coverage_service_name, coverage_request, coverage_response)==true)
//			{
//				cv_bridge::CvImagePtr cv_ptr_obj;
//				cv_ptr_obj = cv_bridge::toCvCopy(coverage_response.coverage_map, sensor_msgs::image_encodings::MONO8);
//				coverage_map = cv_ptr_obj->image;
//
//				for (int v=0; v<coverage_map.rows; ++v)
//					for (int u=0; u<coverage_map.cols; ++u)
//						if (coverage_map.at<uchar>(v,u)==127)
//							map_coverage.at<uchar>(v,u)=208;
//
//				cv_ptr_obj = cv_bridge::toCvCopy(coverage_response.number_of_coverage_image, sensor_msgs::image_encodings::TYPE_32SC1);
//				number_of_coverages_image = cv_ptr_obj->image;
//			}
//			else
//			{
//				ROS_INFO("Error when calling the coverage check server.");
//			}
//				cv::imshow("seen", coverage_map);
//				cv::waitKey();

			// get the area of the whole room
			const int white_room_pixels = cv::countNonZero(data.room_maps_[room]);
			const double room_area = data.map_resolution_ * data.map_resolution_ * (double) white_room_pixels;
			room_areas.push_back(room_area);

			// get the covered area of the room
			cv::threshold(coverage_map, coverage_map, 150, 255, cv::THRESH_BINARY); // covered area drawn in as 127 --> find still white pixels
			const int not_covered_pixels = cv::countNonZero(coverage_map);
			const double not_covered_area = data.map_resolution_ * data.map_resolution_ * (double) not_covered_pixels;

			// get and save the percentage of coverage
			double coverage_percentage = (room_area-not_covered_area)/room_area;
			area_covered_percentages.push_back(coverage_percentage);

			// check how often pixels have been covered
			double average_coverage_number = 0.0, coverage_number_deviation = 0.0;
			for(size_t u=0; u<number_of_coverage_image.rows; ++u)
				for(size_t v=0; v<number_of_coverage_image.cols; ++v)
					if(number_of_coverage_image.at<int>(u,v)!=0)
						numbers_of_coverages.push_back(number_of_coverage_image.at<int>(u,v));
		}
		// create the map with the drawn in path and coverage areas
		map_path_coverage = map.clone();
		for (int v=0; v<path_map.rows; ++v)
		{
			for (int u=0; u<path_map.cols; ++u)
			{
				if (map_coverage.at<uchar>(v,u)==255)
					map_path_coverage.at<uchar>(v,u) = 176;		// leftover uncovered areas
				if (path_map.at<uchar>(v,u)==127 || path_map.at<uchar>(v,u)==196)
					map_path_coverage.at<uchar>(v,u) = path_map.at<uchar>(v,u);
			}
		}
	}

	void statisticsParallelism(const ExplorationData& data, const cv::Mat& map, const cv::Mat& path_map,
			const std::vector<std::vector<geometry_msgs::Pose2D> >& paths, const std::vector<std::vector<geometry_msgs::Pose2D> >& interpolated_paths,
			const double grid_spacing_in_pixel,
			std::vector<double>& wall_angle_score_means, std::vector<double>& trajectory_angle_score_means, std::vector<double>& revisit_time_means)
	{
		// compute the direction of the gradient for each pixel and save the occurring gradients
		cv::Mat gradient_map = computeGradientMap(map);

		const double trajectory_parallelism_check_range = 2.0*grid_spacing_in_pixel;	//1.0/data.map_resolution_; // valid check-radius when checking for the parallelism to another part of the trajectory, [pixels]
		for (size_t room=0; room<paths.size(); ++room)
		{
			if (paths[room].size()==0 || (paths[room][0].x==-1 && paths[room][0].y==-1))
				continue;

			std::vector<double> current_wall_angle_scores, current_trajectory_angle_scores;		// values in [0,1], high values are good
			std::vector<double> current_revisit_times;		// values in [0,1], low values are good
			for (std::vector<geometry_msgs::Pose2D>::const_iterator pose=paths[room].begin(); pose!=paths[room].end()-1; ++pose)
			{
				double dx = (pose+1)->x - pose->x;
				double dy = (pose+1)->y - pose->y;
				double norm = std::sqrt(dy*dy + dx*dx);
				if(norm==0)
					continue;	// skip if the point and its successor are the same
				dx = dx/norm;
				dy = dy/norm;

				// go in the directions of both normals and find the nearest wall
				bool hit_wall = false, hit_trajectory = false, exceeded_trajectory_parallelism_check_range = false;
				bool n1_ok = true, n2_ok = true;
				cv::Point2f n1(pose->x, pose->y), n2(pose->x, pose->y);
				cv::Point wall_pixel, trajectory_pixel;
				do
				{
					// update normals
					n1.x -= dy;
					n1.y += dx;
					n2.x += dy;
					n2.y -= dx;

					// test for coordinates inside image
					if (n1.x<0.f || n1.y<0.f || (int)n1.x >= map.cols || (int)n1.y >= map.rows)
						n1_ok = false;
					if (n2.x<0.f || n2.y<0.f || (int)n2.x >= map.cols || (int)n2.y >= map.rows)
						n2_ok = false;

					// test if a wall/obstacle has been hit
					if (hit_wall==false && n1_ok==true && map.at<uchar>(n1)==0)
					{
						hit_wall = true;
						n1_ok = false;		// do not further search with direction that has found a wall
						wall_pixel = n1;
					}
					else if (hit_wall==false && n2_ok==true && map.at<uchar>(n2)==0)
					{
						hit_wall = true;
						n2_ok = false;		// do not further search with direction that has found a wall
						wall_pixel = n2;
					}

					// only search for the parallelism to another trajectory if the range hasn't been exceeded yet
					if (exceeded_trajectory_parallelism_check_range==false && hit_trajectory==false)
					{
						// test if another trajectory part has been hit, if the check-radius is still satisfied
						const double dist1 = cv::norm(n1-cv::Point2f(pose->x, pose->y));
						const double dist2 = cv::norm(n2-cv::Point2f(pose->x, pose->y));

						if (n1_ok==true && dist1<=trajectory_parallelism_check_range && path_map.at<uchar>(n1)==127)
						{
							hit_trajectory = true;
							trajectory_pixel = n1;
						}
						else if (n2_ok==true && dist2<=trajectory_parallelism_check_range && path_map.at<uchar>(n2)==127)
						{
							hit_trajectory = true;
							trajectory_pixel = n2;
						}

						// if both distances exceed the valid check range, mark as finished
						if (dist1>trajectory_parallelism_check_range && dist2>trajectory_parallelism_check_range)
							exceeded_trajectory_parallelism_check_range = true;
					}

//					cv::Mat test_map = map.clone();
//					cv::circle(test_map, cv::Point(pose->x, pose->y), 2, cv::Scalar(127), CV_FILLED);
//					cv::circle(test_map, cv::Point((pose+1)->x, (pose+1)->y), 2, cv::Scalar(127), CV_FILLED);
//					cv::circle(test_map, n1, 2, cv::Scalar(127), CV_FILLED);
//					cv::circle(test_map, n2, 2, cv::Scalar(127), CV_FILLED);
//					cv::imshow("normals", test_map);
//					cv::waitKey();
				} while ((n1_ok || n2_ok) && ((hit_wall==false) || (hit_trajectory==false && exceeded_trajectory_parallelism_check_range==false)));

				// if a wall/obstacle was found, determine the gradient at this position and compare it to the direction of the path
				if (hit_wall==true)
				{
					cv::Vec2d gradient = gradient_map.at<cv::Vec2d>(wall_pixel);
					cv::Point2f normal_vector(-gradient.val[1], gradient.val[0]);
					const double normal_norm = cv::norm(normal_vector);
					normal_vector *= (float)(normal_norm!=0. ? 1./normal_norm : 1.);
					const double delta_theta = std::acos(normal_vector.x*dx + normal_vector.y*dy);
					const double delta_theta_score = std::abs(0.5*PI-delta_theta)*(1./(0.5*PI));// parallel if delta_theta close to 0 or PI
					current_wall_angle_scores.push_back(delta_theta_score);
				}

				// if another trajectory part could be found, determine the parallelism to it
				if (hit_trajectory==true)
				{
					// find the trajectory point in the interpolated path
					cv::Point2f trajectory_point_m((trajectory_pixel.x*data.map_resolution_)+data.map_origin_.position.x, (trajectory_pixel.y*data.map_resolution_)+data.map_origin_.position.y); // transform in world coordinates
					int pose_index = pose-paths[room].begin();
					int neighbor_index = -1;
					for (std::vector<geometry_msgs::Pose2D>::const_iterator neighbor=interpolated_paths[room].begin(); neighbor!=interpolated_paths[room].end(); ++neighbor)
						if (cv::norm(trajectory_point_m-cv::Point2f(neighbor->x,neighbor->y)) < 0.5*data.map_resolution_)
							neighbor_index = neighbor-interpolated_paths[room].begin();
					if (neighbor_index == -1)
						ROS_WARN("ExplorationEvaluation:evaluateCoveragePaths: parallelism check to trajectory, neighbor_index==-1 --> did not find the neighbor for trajectory point (%f,%f)m.", trajectory_point_m.x, trajectory_point_m.y);
//						std::cout << "index: " << pose_index << ", n: " << neighbor_index << std::endl;

					// save the found index difference, i.e. the difference in percentage of path completion between current node and neighboring path point
					current_revisit_times.push_back(std::abs((double)pose_index/(double)paths[room].size() - (double)neighbor_index/(double)interpolated_paths[room].size()));

					// calculate the trajectory direction at the neighbor to get the difference
					const double n_dx = cos(interpolated_paths[room][neighbor_index].theta);
					const double n_dy = sin(interpolated_paths[room][neighbor_index].theta);
					const double delta_theta = std::acos(n_dx*dx + n_dy*dy);	// acos delivers in range [0,Pi]
					const double delta_theta_score = std::abs(0.5*PI-delta_theta)*(1./(0.5*PI));// parallel if delta_theta close to 0 or PI
					current_trajectory_angle_scores.push_back(delta_theta_score);
				}
			}
			// save found values
			wall_angle_score_means.push_back(std::accumulate(current_wall_angle_scores.begin(), current_wall_angle_scores.end(), 0.0) / std::max(1.0, (double)current_wall_angle_scores.size()));
			trajectory_angle_score_means.push_back(std::accumulate(current_trajectory_angle_scores.begin(), current_trajectory_angle_scores.end(), 0.0) / std::max(1.0, (double)current_trajectory_angle_scores.size()));
			revisit_time_means.push_back(std::accumulate(current_revisit_times.begin(), current_revisit_times.end(), 0.0) / std::max(1.0, (double)current_revisit_times.size()));
		}
	}

	bool findAccessiblePose(const cv::Mat& inflated_map, const geometry_msgs::Pose2D& current_pose_px, geometry_msgs::Pose2D& target_pose_px, const ExplorationData& data,
			const cv::Point2d fov_circle_center_point_in_px, const bool approach_path_accessibility_check=true)
	{
		MapAccessibilityAnalysis map_accessibility_analysis;
		bool found_next = false;
		if(inflated_map.at<uchar>(target_pose_px.y, target_pose_px.x)!=0) // if calculated target pose is accessible, use it as next pose
		{
			found_next = true;
		}
		else // use the map accessibility server to find another accessible target pose
		{
			const MapAccessibilityAnalysis::Pose target_pose_px_copy(target_pose_px.x, target_pose_px.y, target_pose_px.theta);
			if (data.planning_mode_ == FOOTPRINT || (fov_circle_center_point_in_px.x==0 && fov_circle_center_point_in_px.y==0))	// if the fov center is at the robot center it behaves like footprint planning
			{
				const int max_radius = std::max(1, cvRound(1.55*data.coverage_radius_/data.map_resolution_));	// in [pixel]
				// check circles with growing radius around the desired point until a dislocation of data.coverage_radius_ would be exceeded
				for (double radius=1; radius<=max_radius && found_next==false; ++radius)
				{
					// check perimeter for accessible poses
					std::vector<MapAccessibilityAnalysis::Pose> accessible_poses_on_perimeter;
					map_accessibility_analysis.checkPerimeter(accessible_poses_on_perimeter, target_pose_px_copy, radius, PI/32., inflated_map,
							approach_path_accessibility_check, cv::Point(current_pose_px.x, current_pose_px.y));

					// find the closest accessible point on this perimeter
					double min_distance_sqr = std::numeric_limits<double>::max();
					for(std::vector<MapAccessibilityAnalysis::Pose>::iterator new_pose=accessible_poses_on_perimeter.begin(); new_pose!=accessible_poses_on_perimeter.end(); ++new_pose)
					{
						const double dist_sqr = (new_pose->x-current_pose_px.x)*(new_pose->x-current_pose_px.x) + (new_pose->y-current_pose_px.y)*(new_pose->y-current_pose_px.y);
						if (dist_sqr < min_distance_sqr)
						{
							target_pose_px.x = cvRound(new_pose->x);	// the approach_path_accessibility_check uses (u,v) coordinates obtained with cvRound, so this has to be used
							target_pose_px.y = cvRound(new_pose->y);	// here for rounding as well, otherwise the robot can slip into the inaccessible space through rounding
							//target_pose_px.theta = target_pose_px.theta;	// use the orientation of the original pose
							min_distance_sqr = dist_sqr;
							found_next = true;
						}
					}
				}
			}
			else if (data.planning_mode_ == FIELD_OF_VIEW)
			{
				// get the desired FoV-center position
				MapAccessibilityAnalysis::Pose fov_center_px;		// in [px,px,rad]
				fov_center_px.x = (target_pose_px_copy.x + std::cos(target_pose_px_copy.orientation)*fov_circle_center_point_in_px.x - std::sin(target_pose_px_copy.orientation)*fov_circle_center_point_in_px.y);
				//fov_center_px.x = (fov_center_px.x-data.map_origin_.position.x) / data.map_resolution_;
				fov_center_px.y = (target_pose_px_copy.y + std::sin(target_pose_px_copy.orientation)*fov_circle_center_point_in_px.x + std::cos(target_pose_px_copy.orientation)*fov_circle_center_point_in_px.y);
				//fov_center_px.y = (fov_center_px.y-data.map_origin_.position.y) / data.map_resolution_;
				fov_center_px.orientation = target_pose_px_copy.orientation;

				std::cout << "target_pose_px_copy: " << target_pose_px_copy.x << ", " << target_pose_px_copy.y << ", " << target_pose_px_copy.orientation << std::endl;
				std::cout << "fov_center_px: " << fov_center_px.x << ", " << fov_center_px.y << ", " << fov_center_px.orientation << std::endl;

				const double optimal_distance_to_fov_center = cv::norm(fov_circle_center_point_in_px);
				for (double factor_add=0.; factor_add<0.45 && found_next==false; factor_add*=-1.)
				{
					const double factor = 1.0 + factor_add;
					if (factor_add<=0.)
						factor_add -= 0.1;

					// check perimeter for accessible poses
					std::vector<MapAccessibilityAnalysis::Pose> accessible_poses_on_perimeter;
					map_accessibility_analysis.checkPerimeter(accessible_poses_on_perimeter, fov_center_px, factor*optimal_distance_to_fov_center,
							PI/32., inflated_map, approach_path_accessibility_check, cv::Point(current_pose_px.x, current_pose_px.y));

					// find the closest accessible point on this perimeter
					double min_distance_sqr = std::numeric_limits<double>::max();
					for(std::vector<MapAccessibilityAnalysis::Pose>::iterator new_pose=accessible_poses_on_perimeter.begin(); new_pose!=accessible_poses_on_perimeter.end(); ++new_pose)
					{
						const double dist_sqr = (new_pose->x-target_pose_px_copy.x)*(new_pose->x-target_pose_px_copy.x) + (new_pose->y-target_pose_px_copy.y)*(new_pose->y-target_pose_px_copy.y);
						if (dist_sqr < min_distance_sqr)
						{
							target_pose_px.x = cvRound(new_pose->x);	// the approach_path_accessibility_check uses (u,v) coordinates obtained with cvRound, so this has to be used
							target_pose_px.y = cvRound(new_pose->y);	// here for rounding as well, otherwise the robot can slip into the inaccessible space through rounding
							target_pose_px.theta = new_pose->orientation;
							min_distance_sqr = dist_sqr;
							found_next = true;
						}
					}
				}
			}
		}
		return found_next;
	}

	// return path length if a direct connection is possible, otherwise -1
	double generateDirectConnection(const cv::Mat& map, const cv::Point& start, const cv::Point& goal, std::vector<cv::Point>& current_interpolated_path)
	{
		if (start==goal)
			return 0.;

		// try with direct connecting line
		cv::LineIterator it(map, start, goal);
		bool direct_connection = true;
		for (int k=0; k<it.count && direct_connection==true; k++, ++it)
			if (**it < 250)
				direct_connection = false;		// if a pixel in between is not accessible, direct connection is not possible
		if (direct_connection == true)
		{
			// compute distance and path
			current_interpolated_path.clear();
			double length = 0.;
			cv::LineIterator it2(map, start, goal);
			current_interpolated_path.resize(it2.count);
			current_interpolated_path[0] = it2.pos();
			it2++;
			for (int k=1; k<it2.count; ++k, ++it2)
			{
				cv::Point diff = it2.pos()-current_interpolated_path[k-1];
				length += ((abs(diff.x)+abs(diff.y))>1 ? std::sqrt(2.) : 1);
				current_interpolated_path[k] = it2.pos();
			}
			return length;
		}

		return -1.;
	}

	// accumulate all statistics into one file
	void writeCumulativeStatistics(const std::vector<ExplorationData>& evaluation_data, const std::vector<ExplorationConfig>& configs,
			const std::string& data_storage_path)
	{
		for(std::vector<ExplorationConfig>::const_iterator config=configs.begin(); config!=configs.end(); ++config)
		{
			const std::string configuration_folder_name = config->generateConfigurationFolderString() + "/";
			std::stringstream cumulative_statistics;
			for (size_t i=0; i<evaluation_data.size(); ++i)
			{
				const std::string filename = data_storage_path + configuration_folder_name + evaluation_data[i].map_name_ + "_results_eval_per_room.txt";
				std::ifstream file(filename.c_str(), std::ifstream::in);
				if (file.is_open())
				{
					std::string line;
					while(getline(file, line))
						if (line.length()>0)
							cumulative_statistics << line << std::endl;
				}
				else
					ROS_ERROR("Could not open file '%s' for reading cumulative data.", filename.c_str());
				file.close();
			}
			const std::string filename_out = data_storage_path + configuration_folder_name + "all_evaluations_per_room.txt";
			std::ofstream file_out(filename_out.c_str(), std::ofstream::out);
			if (file_out.is_open())
				file_out << cumulative_statistics.str();
			else
				ROS_ERROR("Could not open file '%s' for writing cumulative data.", filename_out.c_str());
			file_out.close();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_evaluation");
	ros::NodeHandle nh;

	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
	const std::string data_storage_path = "room_exploration_evaluation/";

	// prepare relevant floor map data
	std::vector< std::string > map_names;
	map_names.push_back("lab_ipa");
	map_names.push_back("lab_c_scan");
	map_names.push_back("Freiburg52_scan");
//	map_names.push_back("Freiburg79_scan");
//	map_names.push_back("lab_b_scan");
//	map_names.push_back("lab_intel");
//	map_names.push_back("Freiburg101_scan");
//	map_names.push_back("lab_d_scan");
//	map_names.push_back("lab_f_scan");
//	map_names.push_back("lab_a_scan");
//	map_names.push_back("NLB");
//	map_names.push_back("office_a");
//	map_names.push_back("office_b");
//	map_names.push_back("office_c");
//	map_names.push_back("office_d");
//	map_names.push_back("office_e");
//	map_names.push_back("office_f");
//	map_names.push_back("office_g");
//	map_names.push_back("office_h");
//	map_names.push_back("office_i");
//	map_names.push_back("lab_ipa_furnitures");
//	map_names.push_back("lab_c_scan_furnitures");
//	map_names.push_back("Freiburg52_scan_furnitures");
//	map_names.push_back("Freiburg79_scan_furnitures");
//	map_names.push_back("lab_b_scan_furnitures");
//	map_names.push_back("lab_intel_furnitures");
//	map_names.push_back("Freiburg101_scan_furnitures");
//	map_names.push_back("lab_d_scan_furnitures");
//	map_names.push_back("lab_f_scan_furnitures");
//	map_names.push_back("lab_a_scan_furnitures");
//	map_names.push_back("NLB_furnitures");
//	map_names.push_back("office_a_furnitures");
//	map_names.push_back("office_b_furnitures");
//	map_names.push_back("office_c_furnitures");
//	map_names.push_back("office_d_furnitures");
//	map_names.push_back("office_e_furnitures");
//	map_names.push_back("office_f_furnitures");
//	map_names.push_back("office_g_furnitures");
//	map_names.push_back("office_h_furnitures");
//	map_names.push_back("office_i_furnitures");

	std::vector<int> exploration_algorithms;
//	exploration_algorithms.push_back(1);	// grid point exploration
	exploration_algorithms.push_back(2);	// boustrophedon exploration
//	exploration_algorithms.push_back(3);	// neural network exploration
//	exploration_algorithms.push_back(4);	// convex SPP exploration
//	exploration_algorithms.push_back(5);	// flow network exploration
//	exploration_algorithms.push_back(6);	// energy functional exploration
//	exploration_algorithms.push_back(7);	// voronoi exploration

	// coordinate system definition: x points in forward direction of robot and camera, y points to the left side  of the robot and z points upwards. x and y span the ground plane.
	// measures in [m]
	std::vector<geometry_msgs::Point32> fov_points(4);
//	fov_points[0].x = 0.04035;		// this field of view represents the off-center iMop floor wiping device
//	fov_points[0].y = -0.136;
//	fov_points[1].x = 0.04035;
//	fov_points[1].y = 0.364;
//	fov_points[2].x = 0.54035;		// todo: this definition is mirrored on x (y-coordinates are inverted) to work properly --> check why, make it work the intuitive way
//	fov_points[2].y = 0.364;
//	fov_points[3].x = 0.54035;
//	fov_points[3].y = -0.136;
//	int planning_mode = 2;	// viewpoint planning
//	fov_points[0].x = 0.15;		// this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
//	fov_points[0].y = 0.35;
//	fov_points[1].x = 0.15;
//	fov_points[1].y = -0.35;
//	fov_points[2].x = 1.15;
//	fov_points[2].y = -0.65;
//	fov_points[3].x = 1.15;
//	fov_points[3].y = 0.65;
//	int planning_mode = 2;	// viewpoint planning
	fov_points[0].x = -0.3;		// this is the working area of a vacuum cleaner with 60 cm width
	fov_points[0].y = 0.3;
	fov_points[1].x = -0.3;
	fov_points[1].y = -0.3;
	fov_points[2].x = 0.3;
	fov_points[2].y = -0.3;
	fov_points[3].x = 0.3;
	fov_points[3].y = 0.3;
	int planning_mode = 2;	// footprint planning
	geometry_msgs::Point32 fov_origin;
	fov_origin.x = 0.;
	fov_origin.y = 0.;

	const double robot_radius = 0.3;		// [m]
	const double coverage_radius = 0.3;		// [m]
	const double robot_speed = 0.3;			// [m/s]
	const double robot_rotation_speed = 0.52;	// [rad/s]
	const float map_resolution = 0.05;		// [m/cell]

	ExplorationEvaluation ev(nh, test_map_path, map_names, map_resolution, data_storage_path, robot_radius, coverage_radius, fov_points, fov_origin,
			planning_mode, exploration_algorithms, robot_speed, robot_rotation_speed, true, true);
	ros::shutdown();

	//exit
	return 0;
}

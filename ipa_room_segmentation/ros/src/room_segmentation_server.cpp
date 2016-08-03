/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
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
 * ROS package name: ipa_room_segmentation
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 08.2015
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

#include <ipa_room_segmentation/room_segmentation_server.h>

#include <ros/package.h>
#include <ipa_room_segmentation/meanshift2d.h>

RoomSegmentationServer::RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_segmentation_server_(node_handle_, name_of_the_action, boost::bind(&RoomSegmentationServer::execute_segmentation_server, this, _1), false)
{
	//Start action server
	room_segmentation_server_.start();

	//set this parameter to check if the algorithm needs to be trained
	train_the_algorithm_ = false;

	// Parameters
	std::cout << "\n--------------------------\nRoom Segmentation Parameters:\n--------------------------\n";
	node_handle_.param("room_segmentation_algorithm", room_segmentation_algorithm_, 1);
	std::cout << "room_segmentation/room_segmentation_algorithm = " << room_segmentation_algorithm_ << std::endl << std::endl;
	if (room_segmentation_algorithm_ == 1)
		ROS_INFO("You have chosen the morphological segmentation method.");
	else if (room_segmentation_algorithm_ == 2)
		ROS_INFO("You have chosen the distance segmentation method.");
	else if (room_segmentation_algorithm_ == 3)
		ROS_INFO("You have chosen the voronoi segmentation method.");
	else if (room_segmentation_algorithm_ == 4)
		ROS_INFO("You have chosen the semantic segmentation method.");
	else if (room_segmentation_algorithm_ == 5)
		ROS_INFO("You have chosen the voronoi random field segmentation method.");
	std::cout << std::endl;

	//Set mapsamplingfactor, which is the same for every algorithm because it depends on the map
	node_handle_.param("map_sampling_factor_check", map_sampling_factor_check_, 1.5);
	std::cout << "room_segmentation/map_sampling_factor_check = " << map_sampling_factor_check_ << std::endl;

	if (room_segmentation_algorithm_ == 1) //set morphological parameters
	{
		node_handle_.param("room_area_factor_upper_limit_morphological", room_upper_limit_morphological_, 47.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_morphological_ << std::endl;
		node_handle_.param("room_area_factor_lower_limit_morphological", room_lower_limit_morphological_, 0.8);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_morphological_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 2) //set distance parameters
	{
		node_handle_.param("room_area_factor_upper_limit_distance", room_upper_limit_distance_, 163.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_distance_ << std::endl;
		node_handle_.param("room_area_factor_lower_limit_distance", room_lower_limit_distance_, 0.35);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_distance_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 3) //set voronoi parameters
	{
		node_handle_.param("room_area_factor_upper_limit_voronoi", room_upper_limit_voronoi_, 120.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_voronoi_ << std::endl;
		node_handle_.param("room_area_factor_lower_limit_voronoi", room_lower_limit_voronoi_, 1.53);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_voronoi_ << std::endl;
		node_handle_.param("voronoi_neighborhood_index", voronoi_neighborhood_index_, 310);
		std::cout << "room_segmentation/voronoi_neighborhood_index = " << voronoi_neighborhood_index_ << std::endl;
		node_handle_.param("max_iterations", max_iterations_, 150);
		std::cout << "room_segmentation/max_iterations = " << max_iterations_ << std::endl;
		node_handle_.param("min_critical_point_distance_factor", min_critical_point_distance_factor_, 27.0);
		std::cout << "room_segmentation/min_critical_point_distance_factor = " << min_critical_point_distance_factor_ << std::endl;
		node_handle_.param("max_area_for_merging", max_area_for_merging_, 12.5);
		std::cout << "room_segmentation/max_area_for_merging = " << max_area_for_merging_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 4) //set semantic parameters
	{
		node_handle_.param("room_area_factor_upper_limit_semantic", room_upper_limit_semantic_, 23.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_semantic_ << std::endl;
		node_handle_.param("room_area_factor_lower_limit_semantic", room_lower_limit_semantic_, 1.0);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_semantic_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 5) //set voronoi random field parameters
	{
		node_handle_.param("room_upper_limit_voronoi_random", room_upper_limit_voronoi_random_, 10000.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_voronoi_random_ << std::endl;

		node_handle_.param("room_lower_limit_voronoi_random", room_lower_limit_voronoi_random_, 1.53);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_voronoi_random_ << std::endl;

		node_handle_.param("voronoi_random_field_epsilon_for_neighborhood", voronoi_random_field_epsilon_for_neighborhood_, 7);
		std::cout << "room_segmentation/voronoi_random_field_epsilon_for_neighborhood = " << voronoi_random_field_epsilon_for_neighborhood_ << std::endl;

		node_handle_.param("min_neighborhood_size", min_neighborhood_size_, 5);
		std::cout << "room_segmentation/min_neighborhood_size = " << min_neighborhood_size_ << std::endl;

		node_handle_.param("max_iterations", max_iterations_, 150);
		std::cout << "room_segmentation/max_iterations = " << max_iterations_ << std::endl;

		node_handle_.param("min_voronoi_random_field_node_distance", min_voronoi_random_field_node_distance_, 7.0);
		std::cout << "room_segmentation/min_voronoi_random_field_node_distance = " << min_voronoi_random_field_node_distance_ << std::endl;

		node_handle_.param("max_voronoi_random_field_inference_iterations", max_voronoi_random_field_inference_iterations_, 9000);
		std::cout << "room_segmentation/max_voronoi_random_field_inference_iterations = " << max_voronoi_random_field_inference_iterations_ << std::endl;

		node_handle_.param("max_area_for_merging", max_area_for_merging_, 12.5);
		std::cout << "room_segmentation/max_area_for_merging = " << max_area_for_merging_ << std::endl;
	}

	node_handle_.param("display_segmented_map", display_segmented_map_, false);
	std::cout << "room_segmentation/display_segmented_map_ = " << display_segmented_map_ << std::endl;
}

void RoomSegmentationServer::execute_segmentation_server(const ipa_building_msgs::MapSegmentationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Segmentation action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);
	ROS_INFO("map sampling factor is : %f", map_sampling_factor_check_);

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	const float map_resolution = goal->map_resolution;
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);

	//segment the given map
	const int room_segmentation_algorithm_value = room_segmentation_algorithm_;
	if (goal->room_segmentation_algorithm > 0 && goal->room_segmentation_algorithm < 6)
	{
		room_segmentation_algorithm_ = goal->room_segmentation_algorithm;
		if(room_segmentation_algorithm_ == 1) //morpho
		{
			room_lower_limit_morphological_ = 0.8;
			room_upper_limit_morphological_ = 47.0;
			ROS_INFO("You have chosen the morphologcial segmentation.");
		}
		if(room_segmentation_algorithm_ == 2) //distance
		{
			room_lower_limit_distance_ = 0.35;
			room_upper_limit_distance_ = 163.0;
			ROS_INFO("You have chosen the distance segmentation.");
		}
		if(room_segmentation_algorithm_ == 3) //voronoi
		{
			room_lower_limit_voronoi_ = 0.1;	//1.53;
			room_upper_limit_voronoi_ = 1000000.;	//120.0;
			voronoi_neighborhood_index_ = 280;
			max_iterations_ = 150;
			min_critical_point_distance_factor_ = 0.5; //1.6;
			max_area_for_merging_ = 12.5;
			ROS_INFO("You have chosen the Voronoi segmentation");
		}
		if(room_segmentation_algorithm_ == 4) //semantic
		{
			room_lower_limit_semantic_ = 1.0;
			room_upper_limit_semantic_ = 1000000.;//23.0;
			ROS_INFO("You have chosen the semantic segmentation.");
		}
		if(room_segmentation_algorithm_ == 5) //voronoi random field
		{
			room_lower_limit_voronoi_random_ = 1.53; //1.53
			room_upper_limit_voronoi_random_ = 1000000.; //1000000.0
			voronoi_random_field_epsilon_for_neighborhood_ = 7;
			min_neighborhood_size_ = 5;
			min_voronoi_random_field_node_distance_ = 7; // [pixel]
			max_voronoi_random_field_inference_iterations_ = 9000;
			max_area_for_merging_ = 12.5;
			ROS_INFO("You have chosen the voronoi random field segmentation.");
		}
	}
	cv::Mat segmented_map;
	if (room_segmentation_algorithm_ == 1)
	{
		MorphologicalSegmentation morphological_segmentation; //morphological segmentation method
		morphological_segmentation.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_morphological_, room_upper_limit_morphological_);
	}
	else if (room_segmentation_algorithm_ == 2)
	{
		DistanceSegmentation distance_segmentation; //distance segmentation method
		distance_segmentation.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_distance_, room_upper_limit_distance_);
	}
	else if (room_segmentation_algorithm_ == 3)
	{
		VoronoiSegmentation voronoi_segmentation; //voronoi segmentation method
		voronoi_segmentation.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_voronoi_, room_upper_limit_voronoi_,
			voronoi_neighborhood_index_, max_iterations_, min_critical_point_distance_factor_, max_area_for_merging_, display_segmented_map_);
	}
	else if (room_segmentation_algorithm_ == 4)
	{
		AdaboostClassifier semantic_segmentation; //semantic segmentation method
		const std::string package_path = ros::package::getPath("ipa_room_segmentation");
		const std::string classifier_path = package_path + "/common/files/training_results/";
		if (train_the_algorithm_)
		{
			//load the training maps, change to your maps when you want to train different ones
			std::vector<cv::Mat> room_training_maps(5);
			cv::Mat first_room_training_map = cv::imread(package_path + "/common/files/training_maps/lab_ipa_room_training_map.png", 0);
			room_training_maps[0] = first_room_training_map;
			cv::Mat second_room_training_map = cv::imread(package_path + "/common/files/training_maps/lab_d_room_training_map.png", 0);
			room_training_maps[1] = second_room_training_map;
			cv::Mat third_room_training_map = cv::imread(package_path + "/common/files/training_maps/Freiburg52_scan_room_training.png", 0);
			room_training_maps[2] = third_room_training_map;
			cv::Mat fourth_room_training_map = cv::imread(package_path + "/common/files/training_maps/Freiburg52_scan_furnitures_room_training.png", 0);
			room_training_maps[3] = fourth_room_training_map;
			cv::Mat fifth_room_training_map = cv::imread(package_path + "/common/files/training_maps/lab_intel_furnitures_room_training_map.png", 0);
			room_training_maps[4] = fifth_room_training_map;


			std::vector<cv::Mat> hallway_training_maps(5);
			cv::Mat first_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/lab_ipa_hallway_training_map.png", 0);
			hallway_training_maps[0] = first_hallway_training_map;
			cv::Mat second_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/lab_a_hallway_training_map.png", 0);
			hallway_training_maps[1] = second_hallway_training_map;
			cv::Mat third_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/Freiburg52_scan_hallway_training.png", 0);
			hallway_training_maps[2] = third_hallway_training_map;
			cv::Mat fourth_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/Freiburg52_scan_furnitures_hallway_training.png", 0);
			hallway_training_maps[3] = fourth_hallway_training_map;
			cv::Mat fifth_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/lab_intel_hallway_training_map.png", 0);
			hallway_training_maps[4] = fifth_hallway_training_map;
			//train the algorithm
			semantic_segmentation.trainClassifiers(room_training_maps, hallway_training_maps, classifier_path);
		}
		semantic_segmentation.semanticLabeling(original_img, segmented_map, map_resolution, room_lower_limit_semantic_, room_upper_limit_semantic_,
			classifier_path, display_segmented_map_);
	}
	else if (room_segmentation_algorithm_ == 5)
	{
		VoronoiRandomFieldSegmentation vrf_segmentation(false, false); //voronoi random field segmentation method
		const std::string package_path = ros::package::getPath("ipa_room_segmentation");
		std::string conditional_weights_path = package_path + "/common/files/training_results/conditional_field_weights.txt";
		std::string boost_file_path = package_path + "/common/files/training_results/";
		// vector that stores the possible labels that are drawn in the training maps. Order: room - hallway - doorway
		std::vector<uint> possible_labels(3);
		possible_labels[0] = 77;
		possible_labels[1] = 115;
		possible_labels[2] = 179;
		if (train_the_algorithm_)
		{
			// load the training maps
			cv::Mat training_map;
			std::vector<cv::Mat> training_maps;
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr52.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_Fr101.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_intel.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_d_furniture.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_ipa.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_NLB_furniture.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_office_e.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_office_h.png", 0);
			training_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/training_maps/training_lab_c_furnitures.png", 0);
			training_maps.push_back(training_map);
			// load the voronoi maps
			std::vector<cv::Mat> voronoi_maps;
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr52_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/Fr101_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_intel_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_d_furnitures_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_ipa_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/NLB_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_e_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/office_h_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_maps/lab_c_furnitures_voronoi.png", 0);
			voronoi_maps.push_back(training_map);
			// load the voronoi-nodes maps
			std::vector<cv::Mat> voronoi_node_maps;
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr52_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/Fr101_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_intel_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_d_furnitures_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_ipa_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/NLB_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_e_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/office_h_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/voronoi_node_maps/lab_c_furnitures_voronoi_nodes.png", 0);
			voronoi_node_maps.push_back(training_map);
			// load the original maps
			std::vector<cv::Mat> original_maps;
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr52_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/Fr101_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_intel_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_d_furnitures_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_ipa_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/NLB_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/office_e_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/office_h_original.png", 0);
			original_maps.push_back(training_map);
			training_map = cv::imread(package_path + "/common/files/training_maps/voronoi_random_field_training/original_maps/lab_c_furnitures_original.png", 0);
			original_maps.push_back(training_map);
			//train the algorithm
			vrf_segmentation.trainAlgorithms(training_maps, voronoi_maps, voronoi_node_maps, original_maps, possible_labels, conditional_weights_path, boost_file_path);
		}
		doorway_points_.clear();
		vrf_segmentation.segmentMap(original_img, segmented_map, voronoi_random_field_epsilon_for_neighborhood_, max_iterations_,
				min_neighborhood_size_, possible_labels, min_voronoi_random_field_node_distance_,
				display_segmented_map_, conditional_weights_path, boost_file_path, max_voronoi_random_field_inference_iterations_,
				map_resolution, room_lower_limit_voronoi_random_, room_upper_limit_voronoi_random_, max_area_for_merging_, &doorway_points_);
	}
	else
	{
		ROS_ERROR("Undefined algorithm selected.");
		room_segmentation_algorithm_ = room_segmentation_algorithm_value;
		return;
	}
	room_segmentation_algorithm_ = room_segmentation_algorithm_value;

	ROS_INFO("********Segmented the map************");
//	looping_rate.sleep();


// get the min/max-values and the room-centers
// compute room label codebook
	std::map<int, size_t> label_vector_index_codebook; // maps each room label to a position in the rooms vector
	size_t vector_index = 0;
	for (int v = 0; v < segmented_map.rows; ++v)
	{
		for (int u = 0; u < segmented_map.cols; ++u)
		{
			const int label = segmented_map.at<int>(v, u);
			if (label > 0 && label < 65280) // do not count walls/obstacles or free space as label
			{
				if (label_vector_index_codebook.find(label) == label_vector_index_codebook.end())
				{
					label_vector_index_codebook[label] = vector_index;
					vector_index++;
				}
			}
		}
	}
	//min/max y/x-values vector for each room. Initialized with extreme values
	std::vector<int> min_x_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_x_value_of_the_room(label_vector_index_codebook.size(), 0);
	std::vector<int> min_y_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_y_value_of_the_room(label_vector_index_codebook.size(), 0);
	//vector of the central Point for each room, initially filled with Points out of the map
	std::vector<int> room_centers_x_values(label_vector_index_codebook.size(), -1);
	std::vector<int> room_centers_y_values(label_vector_index_codebook.size(), -1);
	//***********************Find min/max x and y coordinate and center of each found room********************
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int y = 0; y < segmented_map.rows; ++y)
	{
		for (int x = 0; x < segmented_map.cols; ++x)
		{
			const int label = segmented_map.at<int>(y, x);
			if (label > 0 && label < 65280) //if Pixel is white or black it is no room --> doesn't need to be checked
			{
				const int index = label_vector_index_codebook[label];
				min_x_value_of_the_room[index] = std::min(x, min_x_value_of_the_room[index]);
				max_x_value_of_the_room[index] = std::max(x, max_x_value_of_the_room[index]);
				max_y_value_of_the_room[index] = std::max(y, max_y_value_of_the_room[index]);
				min_y_value_of_the_room[index] = std::min(y, min_y_value_of_the_room[index]);
			}
		}
	}
	//get centers for each room
//	for (size_t idx = 0; idx < room_centers_x_values.size(); ++idx)
//	{
//		if (max_x_value_of_the_room[idx] != 0 && max_y_value_of_the_room[idx] != 0 && min_x_value_of_the_room[idx] != 100000000 && min_y_value_of_the_room[idx] != 100000000)
//		{
//			room_centers_x_values[idx] = (min_x_value_of_the_room[idx] + max_x_value_of_the_room[idx]) / 2;
//			room_centers_y_values[idx] = (min_y_value_of_the_room[idx] + max_y_value_of_the_room[idx]) / 2;
//			cv::circle(segmented_map, cv::Point(room_centers_x_values[idx], room_centers_y_values[idx]), 2, cv::Scalar(200*256), CV_FILLED);
//		}
//	}
	// use distance transform and mean shift to find good room centers that are reachable by the robot
	// first check whether a robot radius shall be applied to obstacles in order to exclude room center points that are not reachable by the robot
	cv::Mat segmented_map_copy = segmented_map;
	cv::Mat connection_to_other_rooms = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_8UC1);	// stores for each pixel whether a path to another rooms exists for a robot of size robot_radius
	if (goal->robot_radius > 0.0)
	{
		// consider robot radius for exclusion of non-reachable points
		segmented_map_copy = segmented_map.clone();
		cv::Mat map_8u, eroded_map;
		segmented_map_copy.convertTo(map_8u, CV_8UC1, 1., 0.);
		int number_of_erosions = (goal->robot_radius / goal->map_resolution);
		cv::erode(map_8u, eroded_map, cv::Mat(), cv::Point(-1, -1), number_of_erosions);
		for (int v=0; v<segmented_map_copy.rows; ++v)
			for (int u=0; u<segmented_map_copy.cols; ++u)
				if (eroded_map.at<uchar>(v,u) == 0)
					segmented_map_copy.at<int>(v,u) = 0;

		// compute connectivity to other rooms
		bool stop = false;
		while (stop == false)
		{
			stop = true;
			for (int v=1; v<segmented_map_copy.rows-1; ++v)
			{
				for (int u=1; u<segmented_map_copy.cols-1; ++u)
				{
					// skip already identified cells
					if (connection_to_other_rooms.at<uchar>(v,u) != 0)
						continue;

					// only consider cells labeled as a room
					const int label = segmented_map_copy.at<int>(v,u);
					if (label <= 0 || label >= 65280)
						continue;

					for (int dv=-1; dv<=1; ++dv)
					{
						for (int du=-1; du<=1; ++du)
						{
							if (dv==0 && du==0)
								continue;
							const int neighbor_label = segmented_map_copy.at<int>(v+dv,u+du);
							if (neighbor_label>0 && neighbor_label<65280 && (neighbor_label!=label || (neighbor_label==label && connection_to_other_rooms.at<uchar>(v+dv,u+du)==255)))
							{
								// either the room cell has a direct border to a different room or the room cell has a neighbor from the same room label with a connecting path to another room
								connection_to_other_rooms.at<uchar>(v,u) = 255;
								stop = false;
							}
						}
					}
				}
			}
		}
	}
	MeanShift2D ms;
	for (std::map<int, size_t>::iterator it = label_vector_index_codebook.begin(); it != label_vector_index_codebook.end(); ++it)
	{
		int trial = 1; 	// use robot_radius to avoid room centers that are not accessible by a robot with a given radius
		if (goal->robot_radius <= 0.)
			trial = 2;

		for (; trial <= 2; ++trial)
		{
			// compute distance transform for each room
			const int label = it->first;
			int number_room_pixels = 0;
			cv::Mat room = cv::Mat::zeros(segmented_map_copy.rows, segmented_map_copy.cols, CV_8UC1);
			for (int v = 0; v < segmented_map_copy.rows; ++v)
				for (int u = 0; u < segmented_map_copy.cols; ++u)
					if (segmented_map_copy.at<int>(v, u) == label && (trial==2 || connection_to_other_rooms.at<uchar>(v,u)==255))
					{
						room.at<uchar>(v, u) = 255;
						++number_room_pixels;
					}
			if (number_room_pixels == 0)
				continue;
			cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
			cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
			// find point set with largest distance to obstacles
			double min_val = 0., max_val = 0.;
			cv::minMaxLoc(distance_map, &min_val, &max_val);
			std::vector<cv::Vec2d> room_cells;
			for (int v = 0; v < distance_map.rows; ++v)
				for (int u = 0; u < distance_map.cols; ++u)
					if (distance_map.at<float>(v, u) > max_val * 0.95f)
						room_cells.push_back(cv::Vec2d(u, v));
			if (room_cells.size()==0)
				continue;
			// use meanshift to find the modes in that set
			cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, map_resolution);
			const int index = it->second;
			room_centers_x_values[index] = room_center[0];
			room_centers_y_values[index] = room_center[1];

			if (room_cells.size() > 0)
				break;
		}
	}

	cv::Mat indexed_map = segmented_map.clone();
	for (int y = 0; y < segmented_map.rows; ++y)
	{
		for (int x = 0; x < segmented_map.cols; ++x)
		{
			const int label = segmented_map.at<int>(y,x);
			if (label > 0 && label < 65280)
				indexed_map.at<int>(y,x) = label_vector_index_codebook[label]+1;//start value from 1 --> 0 is reserved for obstacles
		}
	}

	if (display_segmented_map_ == true)
	{
		// colorize the segmented map with the indices of the room_center vector
		cv::Mat color_segmented_map = indexed_map.clone();
		color_segmented_map.convertTo(color_segmented_map, CV_8U);
		cv::cvtColor(color_segmented_map, color_segmented_map, CV_GRAY2BGR);
		for(size_t i = 1; i <= room_centers_x_values.size(); ++i)
		{
			//choose random color for each room
			const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
			for(size_t v = 0; v < indexed_map.rows; ++v)
				for(size_t u = 0; u < indexed_map.cols; ++u)
					if(indexed_map.at<int>(v,u) == i)
						color_segmented_map.at<cv::Vec3b>(v,u) = color;
		}
//		cv::Mat disp = segmented_map.clone();
		for (size_t index = 0; index < room_centers_x_values.size(); ++index)
			cv::circle(color_segmented_map, cv::Point(room_centers_x_values[index], room_centers_y_values[index]), 2, cv::Scalar(256), CV_FILLED);
		cv::imshow("segmentation", color_segmented_map);
		cv::waitKey();
	}

	//****************publish the results**********************
	ipa_building_msgs::MapSegmentationResult action_result;
	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "32SC1";
	cv_image.image = indexed_map;
	cv_image.toImageMsg(action_result.segmented_map);

	//setting value to the action msgs to publish
	action_result.map_resolution = goal->map_resolution;
	action_result.map_origin = goal->map_origin;

	//setting massages in pixel value
	action_result.room_information_in_pixel.clear();
	if (goal->return_format_in_pixel == true)
	{
		std::vector<ipa_building_msgs::RoomInformation> room_information(room_centers_x_values.size());
		for (size_t i=0; i<room_centers_x_values.size(); ++i)
		{
			room_information[i].room_center.x = room_centers_x_values[i];
			room_information[i].room_center.y = room_centers_y_values[i];
			room_information[i].room_min_max.points.resize(2);
			room_information[i].room_min_max.points[0].x = min_x_value_of_the_room[i];
			room_information[i].room_min_max.points[0].y = min_y_value_of_the_room[i];
			room_information[i].room_min_max.points[1].x = max_x_value_of_the_room[i];
			room_information[i].room_min_max.points[1].y = max_y_value_of_the_room[i];
		}
		action_result.room_information_in_pixel = room_information;

		// returning doorway points if the vector is not empty
		if(doorway_points_.empty() == false)
		{
			std::vector<geometry_msgs::Point32> found_doorway_points(doorway_points_.size());
			for(size_t i = 0; i < doorway_points_.size(); ++i)
			{
				found_doorway_points[i].x = doorway_points_[i].x;
				found_doorway_points[i].y = doorway_points_[i].y;
			}
			doorway_points_.clear();

			action_result.doorway_points = found_doorway_points;
		}
	}
	//setting massages in meter
	action_result.room_information_in_meter.clear();
	if (goal->return_format_in_meter == true)
	{
		std::vector<ipa_building_msgs::RoomInformation> room_information(room_centers_x_values.size());
		for (size_t i=0; i<room_centers_x_values.size(); ++i)
		{
			room_information[i].room_center.x = convert_pixel_to_meter_for_x_coordinate(room_centers_x_values[i], map_resolution, map_origin);
			room_information[i].room_center.y = convert_pixel_to_meter_for_y_coordinate(room_centers_y_values[i], map_resolution, map_origin);
			room_information[i].room_min_max.points.resize(2);
			room_information[i].room_min_max.points[0].x = convert_pixel_to_meter_for_x_coordinate(min_x_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[0].y = convert_pixel_to_meter_for_y_coordinate(min_y_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[1].x = convert_pixel_to_meter_for_x_coordinate(max_x_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[1].y = convert_pixel_to_meter_for_y_coordinate(max_y_value_of_the_room[i], map_resolution, map_origin);
		}
		action_result.room_information_in_meter = room_information;

		// returning doorway points if the vector is not empty
		if(doorway_points_.empty() == false)
		{
			std::vector<geometry_msgs::Point32> found_doorway_points(doorway_points_.size());
			for(size_t i = 0; i < doorway_points_.size(); ++i)
			{
				found_doorway_points[i].x = convert_pixel_to_meter_for_x_coordinate(doorway_points_[i].x, map_resolution, map_origin);;
				found_doorway_points[i].y = convert_pixel_to_meter_for_y_coordinate(doorway_points_[i].y, map_resolution, map_origin);
			}
			doorway_points_.clear();

			action_result.doorway_points = found_doorway_points;
		}
	}


	//publish result
	room_segmentation_server_.setSucceeded(action_result);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_segmentation_server");

	ros::NodeHandle nh;

	RoomSegmentationServer segmentationAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room segmentation has been initialized......");
	ros::spin();

	return 0;
}

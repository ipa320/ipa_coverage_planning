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

	//set the parameter to check if the algorithm needs to be trained
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
		node_handle_.param("max_area_for_merging", max_area_for_merging_, 20.0);
		std::cout << "room_segmentation/max_area_for_merging = " << max_area_for_merging_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 4) //set semantic parameters
	{
		node_handle_.param("room_area_factor_upper_limit_semantic", room_upper_limit_semantic_, 23.0);
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_semantic_ << std::endl;
		node_handle_.param("room_area_factor_lower_limit_semantic", room_lower_limit_semantic_, 1.0);
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_semantic_ << std::endl;
	}

	node_handle_.param("display_segmented_map", display_segmented_map_, false);
	std::cout << "room_segmentation/display_segmented_map_ = " << display_segmented_map_ << std::endl;
}

void RoomSegmentationServer::execute_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal)
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
	cv::Mat segmented_map;
	if (room_segmentation_algorithm_ == 1)
	{
		morphological_segmentation_.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_morphological_, room_upper_limit_morphological_);
	}
	else if (room_segmentation_algorithm_ == 2)
	{
		distance_segmentation_.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_distance_, room_upper_limit_distance_);
	}
	else if (room_segmentation_algorithm_ == 3)
	{
		voronoi_segmentation_.segmentationAlgorithm(original_img, segmented_map, map_resolution, room_lower_limit_voronoi_, room_upper_limit_voronoi_,
		        voronoi_neighborhood_index_, max_iterations_, min_critical_point_distance_factor_, max_area_for_merging_);
	}
	else if (room_segmentation_algorithm_ == 4)
	{
		const std::string package_path = ros::package::getPath("ipa_room_segmentation");
		const std::string classifier_path = package_path + "/common/files/training_results/";
		if (train_the_algorithm_)
		{
			//load the training maps, change to your maps when you want to train different ones
			cv::Mat first_room_training_map = cv::imread(package_path + "/common/files/training_maps/room_training_map.png", 0);
			cv::Mat second_room_training_map = cv::imread(package_path + "/common/files/training_maps/lab_d_room_training_map.png", 0);
			cv::Mat first_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/hallway_training_map.png", 0);
			cv::Mat second_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/lab_a_hallway_training_map.png", 0);
			//train the algorithm
			semantic_segmentation_.trainClassifiers(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map,
			        classifier_path);
		}
		semantic_segmentation_.semanticLabeling(original_img, segmented_map, map_resolution, room_lower_limit_semantic_, room_upper_limit_semantic_,
		        classifier_path);
	}
	else
	{
		ROS_ERROR("Undefined algorithm selected.");
		return;
	}

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
	MeanShift2D ms;
	for (std::map<int, size_t>::iterator it = label_vector_index_codebook.begin(); it != label_vector_index_codebook.end(); ++it)
	{
		// compute distance transform for each room
		const int label = it->first;
		cv::Mat room = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_8UC1);
		for (int v = 0; v < segmented_map.rows; ++v)
			for (int u = 0; u < segmented_map.cols; ++u)
				if (segmented_map.at<int>(v, u) == label)
					room.at < uchar > (v, u) = 255;
		cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
		// find point set with largest distance to obstacles
		double min_val = 0., max_val = 0.;
		cv::minMaxLoc(distance_map, &min_val, &max_val);
		std::vector < cv::Vec2d > room_cells;
		for (int v = 0; v < distance_map.rows; ++v)
			for (int u = 0; u < distance_map.cols; ++u)
				if (distance_map.at<float>(v, u) > max_val * 0.95f)
					room_cells.push_back(cv::Vec2d(u, v));
		// use meanshift to find the modes in that set
		cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, map_resolution);
		const int index = it->second;
		room_centers_x_values[index] = room_center[0];
		room_centers_y_values[index] = room_center[1];
	}

	if (display_segmented_map_ == true)
	{
		cv::Mat disp = segmented_map.clone();
		for (size_t index = 0; index < room_centers_x_values.size(); ++index)
			cv::circle(disp, cv::Point(room_centers_x_values[index], room_centers_y_values[index]), 2, cv::Scalar(200 * 256), CV_FILLED);

		cv::imshow("segmentation", disp);
		cv::waitKey();
	}

	//****************publish the results**********************
	ipa_room_segmentation::MapSegmentationResult action_result;
	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = segmented_map;
	cv_image.toImageMsg(action_result.segmented_map);

	//setting value to the action msgs to publish
	action_result.map_resolution = goal->map_resolution;
	action_result.map_origin = goal->map_origin;

	//setting massages in pixel value
	action_result.room_information_in_pixel.clear();
	if (goal->return_format_in_pixel == true)
	{
		std::vector<ipa_room_segmentation::RoomInformation> room_information(room_centers_x_values.size());
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
	}
	//setting massages in meter
	action_result.room_information_in_meter.clear();
	if (goal->return_format_in_meter == true)
	{
		std::vector<ipa_room_segmentation::RoomInformation> room_information(room_centers_x_values.size());
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

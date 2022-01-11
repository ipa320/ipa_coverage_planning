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

#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <ipa_room_segmentation/RoomSegmentationConfig.h>

#include <iostream>
#include <list>
#include <string>
#include <vector>


#include <ipa_building_msgs/MapSegmentationAction.h>
#include <ipa_building_msgs/RoomInformation.h>
#include <ipa_building_msgs/ExtractAreaMapFromLabeledMap.h>

#include <ipa_room_segmentation/distance_segmentation.h>
#include <ipa_room_segmentation/morphological_segmentation.h>
#include <ipa_room_segmentation/voronoi_segmentation.h>
#include <ipa_room_segmentation/adaboost_classifier.h>
#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

class RoomSegmentationServer
{
protected:

	// parameters
	//limits for the room-areas
	double room_upper_limit_morphological_, room_upper_limit_distance_, room_upper_limit_voronoi_, room_upper_limit_semantic_, room_upper_limit_voronoi_random_, room_upper_limit_passthrough_;
	double room_lower_limit_morphological_, room_lower_limit_distance_, room_lower_limit_voronoi_, room_lower_limit_semantic_, room_lower_limit_voronoi_random_, room_lower_limit_passthrough_;
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation,
										// 1 = morphological segmentation
										// 2 = distance segmentation
										// 3 = Voronoi segmentation
										// 4 = semantic segmentation
										// 5 = voronoi-random-field segmentation
										// 99 = pass through segmentation

	bool train_semantic_, train_vrf_; //Boolean to say if the algorithm needs to be trained
	bool load_semantic_features_; //Boolean to say if the training of the semantic algorithm should load precomputed features

	int voronoi_neighborhood_index_; //Variable for the Voronoi method that specifies the neighborhood that is looked at for critical Point extraction
	int voronoi_random_field_epsilon_for_neighborhood_; //Variable that specifies the neighborhood for the vrf-segmentation.
	int max_iterations_; //number of iterations for search of neighborhood in voronoi method and vrf method
	int min_neighborhood_size_; //Variable that stores the minimum size of a neighborhood, used for the vrf method.
	double min_voronoi_random_field_node_distance_; //Variable that shows how near two nodes of the conditional random field can be in the vrf method. [pixel]
	int max_voronoi_random_field_inference_iterations_; //Variable that shows how many iterations should max. be done when infering in the conditional random field.
	double min_critical_point_distance_factor_; //Variable that sets the minimal distance between two critical Points before one gets eliminated
	double max_area_for_merging_; //Variable that shows the maximal area of a room that should be merged with its surrounding rooms
	bool display_segmented_map_;	// displays the segmented map upon service call
	bool publish_segmented_map_;	// publishes the segmented map as grid map upon service call
	std::vector<cv::Point> doorway_points_; // vector that saves the found doorway points, when using the 5th algorithm (vrf)

	std::vector<std::string> semantic_training_maps_room_file_list_;	// list of files containing maps with room labels for training the semantic segmentation
	std::vector<std::string> semantic_training_maps_hallway_file_list_;	// list of files containing maps with hallway labels for training the semantic segmentation
	std::vector<std::string> vrf_original_maps_file_list_;	// list of files containing the original maps for training the VRF segmentation
	std::vector<std::string> vrf_training_maps_file_list_;	// list of files containing the labeled maps for training the VRF segmentation
	std::vector<std::string> vrf_voronoi_maps_file_list_;	// list of files containing the Voronoi maps for training the VRF segmentation - these files are optional for training and just yield a speedup
	std::vector<std::string> vrf_voronoi_node_maps_file_list_;	// list of files containing the Voronoi node maps for training the VRF segmentation - these files are optional for training and just yield a speedup

	//converter-> Pixel to meter for X coordinate
	double convert_pixel_to_meter_for_x_coordinate(const int pixel_valued_object_x, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution) + map_origin.x;
		return meter_value_obj_x;
	}
	//converter-> Pixel to meter for Y coordinate
	double convert_pixel_to_meter_for_y_coordinate(int pixel_valued_object_y, const float map_resolution, const cv::Point2d map_origin)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution) + map_origin.y;
		return meter_value_obj_y;
	}

	//This is the execution function used by action server
	void execute_segmentation_server(const ipa_building_msgs::MapSegmentationGoalConstPtr &goal);

	//Callback for dynamic reconfigure server
	void dynamic_reconfigure_callback(ipa_room_segmentation::RoomSegmentationConfig &config, uint32_t level);

	// service for generating a map of one single room from a labeled map
	// The request message provides a segmented map which consists of cells with label 0 for inaccessible areas and other number from 1 to N
	// for labeling membership with one of the N segmented areas.
	// The return message shall deliver the same map but with only one area (segment_of_interest) labeled as 255 and the remainder labeled
	// as inaccessible with 0.
	bool extractAreaMapFromLabeledMap(ipa_building_msgs::ExtractAreaMapFromLabeledMapRequest& request, ipa_building_msgs::ExtractAreaMapFromLabeledMapResponse& response);

	//!!Important!!
	// define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	ros::Publisher map_pub_;
	ros::ServiceServer extract_area_map_from_labeled_map_server_;
	actionlib::SimpleActionServer<ipa_building_msgs::MapSegmentationAction> room_segmentation_server_;
	dynamic_reconfigure::Server<ipa_room_segmentation::RoomSegmentationConfig> room_segmentation_dynamic_reconfigure_server_;

public:
	//initialize the action-server
	RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action);

	//Default destructor for the class
	~RoomSegmentationServer(void)
	{
	}
};

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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <actionlib/server/simple_action_server.h>


#include <iostream>
#include <list>
#include <string>
#include <vector>


#include <ipa_room_segmentation/MapSegmentationAction.h>

#include <ipa_room_segmentation/distance_segmentation.h>

#include <ipa_room_segmentation/morphological_segmentation.h>

#include <ipa_room_segmentation/voronoi_segmentation.h>

#include <ipa_room_segmentation/adaboost_classifier.h>

class RoomSegmentationServer
{
protected:

	// parameters
	double map_sampling_factor_check_;	//sampling-factor of the map
	//limits for the room-areas
	double room_upper_limit_morphological_, room_upper_limit_distance_, room_upper_limit_voronoi_, room_upper_limit_semantic_, room_upper_limit_voronoi_random_;
	double room_lower_limit_morphological_, room_lower_limit_distance_, room_lower_limit_voronoi_, room_lower_limit_semantic_, room_lower_limit_voronoi_random_;
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation,
										// 1 = morphological segmentation
										// 2 = distance segmentation
										// 3 = Voronoi segmentation
										// 4 = semantic segmentation
										// 5 = voronoi-random-field segmentation

	bool train_the_algorithm_; //Boolean to say if the algorithm needs to be trained

	int voronoi_neighborhood_index_; //Variable for the Voronoi method that specifies the neighborhood that is looked at for critical Point extraction
	int voronoi_random_field_epsilon_for_neighborhood_; //Variable that specifies the neighborhood for the vrf-segmentation.
	int max_iterations_; //number of iterations for search of neighborhood in voronoi method and vrf method
	unsigned int min_neighborhood_size_; //Variable that stores the minimum size of a neighborhood, used for the vrf method.
	double min_voronoi_random_field_node_distance_; //Variable that shows how near two nodes of the conditional random field can be in the vrf method. [pixel]
	size_t max_voronoi_random_field_inference_iterations_; //Variable that shows how many iterations should max. be done when infering in the conditional random field.
	double min_critical_point_distance_factor_; //Variable that sets the minimal distance between two critical Points before one gets eliminated
	double max_area_for_merging_; //Variable that shows the maximal area of a room that should be merged with its surrounding rooms
	bool display_segmented_map_;	// displays the segmented map upon service call

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
	void execute_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal);


	//!!Important!!
	// define the Nodehandle before the action server, or else the server won't start
	//
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_room_segmentation::MapSegmentationAction> room_segmentation_server_;

public:
	//initialize the action-server
	RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action);

	//Default destructor for the class
	~RoomSegmentationServer(void)
	{
	}
};

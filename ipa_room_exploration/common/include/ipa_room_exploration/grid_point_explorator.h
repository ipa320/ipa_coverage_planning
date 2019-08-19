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
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan
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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <ipa_building_navigation/tsp_solvers.h>
#include <ipa_building_navigation/distance_matrix.h>
#include <ipa_room_exploration/room_rotator.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/grid.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>



// Class that generates a room exploration path by laying a small grid over the given map and then planning the best path trough
// all points, by defining an traveling salesman problem (TSP). This class only produces a static path, regarding the given map
// in form of a point series. To react on dynamic obstacles, one has to do this in upper algorithms.
//
class GridPointExplorator
{
public:
	// constructor
	GridPointExplorator();

	// separate, interruptible thread for the external solvers
	void tsp_solver_thread_concorde(ConcordeTSPSolver& tsp_solver, std::vector<int>& optimal_order,
			const cv::Mat& distance_matrix, const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node);

	void tsp_solver_thread_genetic(GeneticTSPSolver& tsp_solver, std::vector<int>& optimal_order,
			const cv::Mat& distance_matrix, const std::map<int,int>& cleaned_index_to_original_index_mapping, const int start_node);

	void tsp_solver_thread(const int tsp_solver, std::vector<int>& optimal_order, const cv::Mat& original_map,
		const std::vector<cv::Point>& points, const double downsampling_factor, const double robot_radius, const double map_resolution,
		const int start_node);

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const double map_resolution,
			const cv::Point starting_position, const cv::Point2d map_origin, const int cell_size, const bool plan_for_footprint,
			const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, int tsp_solver, int64_t tsp_solver_timeout);
};

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <stdio.h>

#include <Eigen/Dense>
#include <libqsopt/qsopt.h>

#include <ipa_room_exploration/nearest_neighbor_TSP.h>
#include <ipa_room_exploration/A_star_pathplanner.h>
#include <ipa_room_exploration/fow_to_robot_mapper.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

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
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 11.2016
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

#pragma once

#define PI 3.14159265359

// This class provides a coverage path planning algorithm, based on the work of
//
// Arain, M. A., Cirillo, M., Bennetts, V. H., Schaffernicht, E., Trincavelli, M., & Lilienthal, A. J. (2015, May). Efficient measurement planning for remote gas sensing with mobile robots. In 2015 IEEE International Conference on Robotics and Automation (ICRA) (pp. 3428-3434). IEEE.
//
// In this work originally a gas sensing robot is assumed, but by specifically defining the visibility function one can
// use this work for coverage path planning, especially for planning a route s.t. the field of view covers the free space.
// In the proposed paper the free space gets discretized into cells, that need to be covered. To do so a integer linear
// program of the form
//		min 	C^T Ts
//		s.t. 	VC >= 1 (elementwise)
//				C[i] = {0, 1}
// is used, with C a column vector, representing if a sensing pose is used (1) or not (0), Ts a column vector storing the
// sensing cost at a specific sensing pose, V a matrix storing what cell is observed from a sensing pose. The first
// constraint ensures that each cell has been seen at least once and the objective function minimizes the number of sensing
// poses. To improve the efficiency of the algorithm a re-weighted convex relaxation method is used to enhance the
// sparsity of the above linear program. This is done by solving
//		min 	(W o C)^T Ts
//		s.t. 	VC >= 1 (elementwise)
//				0 <= C[i] <= 1
// a few times, with calculating W new after each step. When the sparsity of C doesn't change much anymore or a number of
// iterations is reached, the zero elements of C are discarded and the first linear program is solved for the final result.
// This gives a minimal set of sensing poses s.t. all free cells can be observed with these.
//
class convexSPPExplorator
{
protected:
	// function that is used to create and solve a Qsopt optimization problem out of the given matrices and vectors
	template<typename T>
	void solveOptimizationProblem(std::vector<T>& C, const cv::Mat& V, const std::vector<double>* W);

	// object to find a path trough the chosen sensing poses by doing a repetitive nearest neighbor algorithm
	NearestNeighborTSPSolver tsp_solver_;

	// object that plans a path from A to B using the Astar method
	AStarPlanner path_planner_;

public:
	// constructor
	convexSPPExplorator();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at. The footprint stores a polygon that is used to determine the visibility at a specific
	// sensing pose. delta_theta provides an angular step to determine candidates for sensing poses.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
				const cv::Point starting_position, const cv::Point2d map_origin,
				const int cell_size, const double delta_theta, const geometry_msgs::Polygon& room_min_max_coordinates,
				const std::vector<geometry_msgs::Point32>& footprint, const Eigen::Matrix<float, 2, 1>& robot_to_fow_middlepoint_vector,
				const double max_fow_angle, const double smallest_robot_to_fow_distance, const double largest_robot_to_fow_distance,
				const uint sparsity_check_range, const bool plan_for_footprint);
};

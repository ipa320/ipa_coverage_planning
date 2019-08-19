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
 * \date Date of creation: 01.2017
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

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <ipa_room_exploration/meanshift2d.h>
#include <ipa_room_exploration/room_rotator.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/grid.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265359
const double PI_2_INV = 1./(0.5*PI);



// Struct that is used to create a node and save the corresponding neighbors.
struct EnergyExploratorNode
{
	cv::Point center_;
	bool obstacle_;
	bool visited_;
	std::vector<EnergyExploratorNode*> neighbors_;

	int countNonObstacleNeighbors()
	{
		int non_obstacle_neighbors = 0;
		for(std::vector<EnergyExploratorNode*>::iterator neighbor=neighbors_.begin(); neighbor!=neighbors_.end(); ++neighbor)
			if((*neighbor)->obstacle_==false)
				++non_obstacle_neighbors;
		return non_obstacle_neighbors;
	}
};

// Struct used to create sets of energyExplorationNodes
struct cv_Point_cmp
{
  bool operator() (const cv::Point& lhs, const cv::Point& rhs) const
  {
	  return ((lhs.y < rhs.y) || (lhs.y == rhs.y && lhs.x < rhs.x));
  }
};

// This class provides the functionality of coverage path planning, based on the work in
//
//	Bormann Richard, Joshua Hampp, and Martin Hägele. "New brooms sweep clean-an autonomous robotic cleaning assistant for
//	professional office cleaning." Robotics and Automation (ICRA), 2015 IEEE International Conference on. IEEE, 2015.
//
// In this method the area that should be covered is discretized into a grid, using the given grid size parameter d_0. Each of these
// resulting nodes should be visited in order to cover the whole free space. The path starts at the corner of the area that is
// closest to the starting position. After the start node, the next node is chosen from the neighbors that minimizes the energy functional
//		E(l,n) = d_t(l,n) + d_r(l,n) + N(n),
// where l is the current robot pose, n the to be checked next pose. The summands represent different aspects that need to
// be taken into account. d_t is the translational distance computed as
//		d_t(l,n) = ||l_xy-n_xy||_2/d_0,
// where l_xy or n_xy is the vector storing the position of the corresponding location. d_r is the rotational distance, computed as
//		d_r(l,n) = 2*(l_theta-n_theta)/pi,
// where l_theta or n_theta are the angles of the corresponding poses. Function N(n) represents half the number of not yet visited
// locations among the 8 neighbors Nb8(n) around n and is computed as
//		N(n) = 4 - sum_(k in Nb8(n)) |k ∩ L|/2,
// where L is the number of already visited nodes. If no accessible node in the direct neighborhood could be found, the algorithm
// searches for the next node in the whole grid. This procedure is repeated until all nodes have been visited.
// This class only produces a static path, regarding the given map in form of a point series. To react on dynamic
// obstacles, one has to do this in upper algorithms.
//
class EnergyFunctionalExplorator
{
protected:
	// function to compute the energy function for each pair of nodes
	double E(const EnergyExploratorNode& location, const EnergyExploratorNode& neighbor, const double cell_size_in_pixel, const double previous_travel_angle);

public:
	// constructor
	EnergyFunctionalExplorator();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
			const cv::Point starting_position, const cv::Point2d map_origin, const double grid_spacing_in_pixel,
			const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fov_vector);
};

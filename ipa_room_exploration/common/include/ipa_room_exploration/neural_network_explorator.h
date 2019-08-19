#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <Eigen/Dense>

#include <ipa_room_exploration/neuron_class.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/room_rotator.h>
#include <ipa_room_exploration/grid.h>

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

// Definition if the operator == for geometry_msgs::Pose2D for checking how often a specific pose is in the so far calculated
// path.
inline bool operator==(const geometry_msgs::Pose2D& A, const geometry_msgs::Pose2D& B)
{
	if(A.x == B.x && A.y == B.y)
		return true;
	return false;
}

// This class provides a room explorator based on an artificial neural network. This network is used to compute a
// coverage path s.t. the whole environment is visited at least once. The used method is stated in:
//
// Yang, Simon X., and Chaomin Luo. "A neural network approach to complete coverage path planning." IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics) 34.1 (2004): 718-724.
//
// In this algorithm the free space gets sampled into several neurons that span a neural network over the space. Each
// neuron then needs to be visited once to clean it. Each neuron has a state that is used later to determine the next
// neuron that needs to be visited. Going then trough the space over time produces a path that covers all neurons, see
// the stated paper for reference.
// This class provides the functionality to provide a room map and discretize it into several neurons, based on the given
// sampling distance and the radius of the robot/field-of-view (assuming that the footprint/fov gets approximated by a
// inner circle). After this the coverage path gets computed based on the stated paper. This implementation only provides
// a static path, any reaction to unexpected behavior (e.g. sudden obstacles) need to be done in an upper program.
//
class NeuralNetworkExplorator
{
protected:

	// vector that stores the neurons of the given map
	std::vector<std::vector<Neuron> > neurons_;

	// step size used for integrating the states of the neurons
	double step_size_;

	// parameters for the neural network
	double A_, B_, D_, E_, mu_, delta_theta_weight_;

public:

	// constructor
	NeuralNetworkExplorator();

	// function to set the step size to a certain value
	void setStepSize(double step_size)
	{
		step_size_ = step_size;
	}

	// function to set the parameters needed for the neural network
	void setParameters(double A, double B, double D, double E, double mu, double step_size, double delta_theta_weight)
	{
		A_ = A;
		B_ = B;
		D_ = D;
		E_ = E;
		mu_ = mu;
		step_size_ = step_size;
		delta_theta_weight_ = delta_theta_weight;
	}

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
					 const cv::Point starting_position, const cv::Point2d map_origin, const double grid_spacing_in_pixel,
					 const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, bool show_path_evolution=false);
};

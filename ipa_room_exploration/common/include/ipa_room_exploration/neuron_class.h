#include <iostream>
#include <list>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>

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

// This class provides a neuron for an artificial neural network. This network is used to compute a coverage path s.t.
// the whole environment is visited at least once. The used method is stated in:
//
// Yang, Simon X., and Chaomin Luo. "A neural network approach to complete coverage path planning." IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics) 34.1 (2004): 718-724.
//
// In this algorithm the free space gets sampled into several neurons that span a neural network over the space. Each
// neuron then needs to be visited once to clean it. Each neuron has a state that is used later to determine the next
// neuron that needs to be visited. Going then trough the space over time produces a path that covers all neurons, see
// the stated paper for reference.
// This class additionally contains the position of the neuron, the parameters used to update the state of it and
// booleans to mark if this neuron has already been cleaned or not. The function I() provides an "external input" to the
// neuron, see the paper for reference.
//
class Neuron
{
protected:

	// vector that stores the direct neighbors of this neuron
	std::vector<Neuron*> neighbors_;

	// vector that stores the weights to the neighbors --> used for updating the state
	std::vector<double> weights_;

	// position of the neuron
	cv::Point position_;

	// booleans to check if this neuron is cleaned or an obstacle
	bool visited_, obstacle_;

	// state (activity) of this neuron at current time step and last
	double state_, previous_state_;

	// parameters used to update the state
	double A_, B_, D_, E_, mu_;

	// step size for updating the state
	double step_size_;

	// function to generate the external input
	double I()
	{
		if(obstacle_ == true)
			return -1.0*E_;
		else if(visited_ == false)
			return E_;
		else
			return 0.0;
	}

public:

	// constructor
	Neuron(cv::Point position, double A, double B, double D, double E, double mu, double step_size, bool obstacle, bool visited=false)
	{
		state_ = 0;
		previous_state_ = 0;
		position_ = position;
		A_ = A;
		B_ = B;
		D_ = D;
		E_ = E;
		mu_ = mu;
		step_size_ = step_size;
		obstacle_ = obstacle;
		visited_ = visited;
	}

	// function to insert a neighbor
	void addNeighbor(Neuron* new_neighbor)
	{
		// save pointer to neighbor
		neighbors_.push_back(new_neighbor);

		// calculate distance and corresponding weight to it
		cv::Point difference = position_ - new_neighbor->position_;
		double distance = cv::norm(difference);
		weights_.push_back(mu_/distance);
	}

	// function to get the position of the neuron
	cv::Point getPosition()
	{
		return position_;
	}

	// function to get the state of the neuron, return the previous state if wanted
	double getState(bool previous=false)
	{
		if(previous == true)
			return previous_state_;
		return state_;
	}

	// function to save the current state as previous state
	void saveState()
	{
		previous_state_ = state_;
	}

	// function to get the neighbors
	void getNeighbors(std::vector<Neuron*>& neighbors)
	{
		neighbors = neighbors_;
	}

	// function to mark the neuron as cleaned
	void markAsVisited()
	{
		visited_ = true;
	}

	// function to check if the current neuron is an obstacle or not
	bool isObstacle()
	{
		return obstacle_;
	}

	// function to check if the neuron has been visited or not
	bool visitedNeuron()
	{
		return visited_;
	}

	// function to update the state of the neuron using euler discretization
	void updateState()
	{
		// get external input
		const double input = I();

		// get the current sum of weights times the state of the neighbor
		double weight_sum = 0;
		for(size_t neighbor=0; neighbor<neighbors_.size(); ++neighbor)
			weight_sum += weights_[neighbor]*std::max(neighbors_[neighbor]->getState(true), 0.0);

		// calculate current gradient --> see stated paper from the beginning
		double gradient = -A_*state_ + (B_-state_)*(std::max(input, 0.0) + weight_sum) - (D_+state_)*std::max(-1.0*input, 0.0);

		// update state using euler method
		state_ += step_size_*gradient;
	}
};

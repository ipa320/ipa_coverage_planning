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
 * \date Date of creation: 11.2015
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
//
// This class is used to easily define cliques from a graph. A Clique is a subgraph of a given graph, in which all Points are
// connected to each other. So this class has a element member_Points that stores all Points that are part of this clique.
// Also it has different functions:
//		1. getMemberPoints(): This function returns a vector<cv::Point> with all members in it.
//		2. insertMember(cv::Point/vector<cv::Point)): This function takes one cv::Point or a vector of them and adds them to
//													  the member Points.
//		3. getNumberOfMembers(): This function returns the number of members in the clique.
//		4. getCliquePotential(vector<unsigned int> labels, vector<cv::CvBoost> boost_classifiers):
//				This function is used for the Voronoi Random Field Segmentation. It gets a list of AdaBoost classifiers
//				( from OpenCV ) and calculates the feature for this clique, depending on the given labels for each Point.
//				The labels show to which class each Point belongs and are describing the position of the AdaBoost classifier
//				in the given vector. It then calculates Phi(y_k, x) = exp(w_k^T * f_k(y_k, x)). Phi is the clique potential
//				needed for the conditional random field, w_k is the weight-vector, defined for each classifier and put together
//				for each point here and f_k is the feature vector, that calculates individual features for each point to label
//				them as a specific class. y_k are the member points of a clique and x is the hidden node of the conditional
//				random field, in this case the given gridmap.
//

#include <ipa_room_segmentation/contains.h>
#include <ipa_room_segmentation/voronoi_random_field_features.h>
#include <ipa_room_segmentation/raycasting.h>

#pragma once

class Clique
{
protected:
	std::vector<cv::Point> member_points_; // vector that stores the member points of the clique

	std::vector< std::vector<double> > beams_for_members_; // vector that stores the simulated beams for each member (simulated using raycasting)

public:

	Clique(); // default constructor

	Clique(cv::Point first_member); // constructor if one node is known

	Clique(std::vector<cv::Point> members); // constructor if more than one member point is known

	std::vector<cv::Point> getMemberPoints(); // function that returns a vector with all member points stored in

	void insertMember(cv::Point& new_member); // function that inserts a member if it isn't already a member

	void insertMember(std::vector<cv::Point>& new_members); // function that inserts multiple members, if they are not already members

	bool containsMember(const cv::Point& point); // function that checks if a given point is part of this clique

	unsigned int getNumberOfMembers(); // function that returns the number of members stored in this clique

	void setBeamsForMembers(const std::vector< std::vector<double> > beams); // function that stores the given beams in the class parameter

	std::vector< std::vector<double> > getBeams(); // function that returns the stored laser-beams for the member points

};

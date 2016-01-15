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
 * \date Date of creation: 10.2015
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

#include <iostream>
#include <list>
#include <vector>

#include <math.h>

#include <dlib/optimization.h>

#include <ctime>

#include <ipa_room_segmentation/contains.h> // some useful functions defined for all segmentations
#include <ipa_room_segmentation/voronoi_random_field_features.h>
#include <ipa_room_segmentation/raycasting.h>
#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/clique_class.h>

#pragma once

typedef dlib::matrix<double,0,1> column_vector; // typedef used for the dlib optimization

class VoronoiRandomFieldSegmentation
{
protected:

	std::vector<double> angles_for_simulation_; // Vector that saves the angles, used to simulate the laser measurements
												// for the AdaBoost classifier.

	CvBoostParams params_; // Parameters for the classifiers

	int number_of_classifiers_; // Number of weak classifiers used from the OpenCV AdaBoost function.

	bool trained_; // Variable that shows if the classifiers has already been trained.

	CvBoost room_boost_, hallway_boost_, doorway_boost_; // The AdaBoost-Classifier to induct the features needed in the conditional random field.

	// Function to check if the given point is more far away from each point in the given vector than the min_distance.
	bool pointMoreFarAway(const std::vector<cv::Point>& points, const cv::Point& point, const double min_distance);

	// Function to draw the approximated voronoi graph into a given map. It doesn't draw lines of the graph that start or end
	// in a black region. This is necessary because the voronoi graph gets approximated by diskretizing the maps contour and
	// using these points as centers for the graph. It gets wrong lines, that are eliminated in this function. See the .cpp
	// files for further information.
	void drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color,
			const std::vector<cv::Point>& contour, const std::vector<std::vector<cv::Point> >& hole_contours);

	// Function to calculate the feature vector for a given clique, using the trained AdaBoost classifiers.
	void getAdaBoostFeatureVector(std::vector<double>& feature_vector, Clique& clique,
			const unsigned int given_label, const std::vector<unsigned int>& possible_labels, const cv::Mat& original_map);


	void createPrunedVoronoiGraph(cv::Mat& map_for_voronoi_generation, std::vector<cv::Point>& node_points); // Function that takes a map and draws a pruned voronoi
																	    									// graph in it.

	void createConditionalField(const cv::Mat& voronoi_map, const std::vector<cv::Point>& node_points, 					// Function to create a conditional random field out of given points. It needs
			std::vector<Clique>& conditional_random_field_cliques, const std::vector<cv::Point> voronoi_node_points);	// the voronoi-map extracted from the original map to find the neighbors for each point
																														// and the voronoi-node-points to add the right points as nodes.
public:

	VoronoiRandomFieldSegmentation(bool trained); // constructor

	void trainBoostClassifiers(std::vector<cv::Mat>& room_training_maps, std::vector<cv::Mat>& hallway_training_maps,
				std::vector<cv::Mat>& doorway_training_maps, const std::string& classifier_storage_path); // Function to train the AdaBoost classifiers, used for feature induction of the conditional
									  	  	  	  	  	  	  	  	  	  	 	 	 	 	 	 	 	  // random field.
	void findConditionalWeights(const std::vector<cv::Mat>& training_maps, const std::vector<cv::Mat>& voronoi_maps,
			const std::vector<cv::Mat>& voronoi_node_maps, const unsigned char voronoi_node_color, const std::vector<unsigned int>& possible_labels); // Function to find the weights used to calculate the clique potentials.


	column_vector findMinValue(); // Function to find the minimal value of a function. Used to find the optimal weights for
								  // the conditional random field.

	// Function to segment a given map into different regions.
	void segmentMap(cv::Mat& original_map, const int epsilon_for_neighborhood,
			const int max_iterations, unsigned int min_neighborhood_size, const double min_node_distance, bool show_nodes);

};

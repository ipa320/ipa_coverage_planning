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
#pragma once
#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>
#include <algorithm>

#include <math.h>
#include <functional>

#include <libdlib/optimization.h>

// OpenGM-library headers
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/discretespace.hxx>
#include <opengm/functions/explicit_function.hxx>
#include <opengm/operations/multiplier.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/inference/messagepassing/messagepassing.hxx>
#include <opengm/operations/maximizer.hxx>

#include <ctime>

#include <ipa_room_segmentation/contains.h> // some useful functions defined for all segmentations
#include <ipa_room_segmentation/voronoi_random_field_features.h>
#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/clique_class.h>
#include <ipa_room_segmentation/room_class.h>
#include <ipa_room_segmentation/abstract_voronoi_segmentation.h>

#pragma once


// Typedef used for the dlib optimization. This Type stores n times 1 elements in a matirx --> represents a column-vector.
typedef dlib::matrix<double,0,1> column_vector;

// Typedefs for the OpenGM library. This library is a template-library for discrete factor-graphs (https://en.wikipedia.org/wiki/Factor_graph)
// and for operations on these, used in this algorithm to do loopy-belief-propagation for the conditional random field.
//
// Typedef for the Label-Space. This stores n variables that each can have m labels.
typedef opengm::SimpleDiscreteSpace<size_t, size_t> LabelSpace;

// Typedef for a factor-graph that stores doubles as results, adds the factors and has discrete labels for each variable.
typedef opengm::GraphicalModel <double, opengm::Adder, opengm::ExplicitFunction<double>, LabelSpace > FactorGraph;

// Typedef for the update rule of messages in a factor graph, when using message propagation.
// Second Typedef is the Belief-Propagation used in this algorithm. It can be used on the defined FactorGraph, maximizes the defined
// functions and uses the Update-Rule for the messages that maximize the overall Graph. MaxDistance is a metric used to measure
// the distance of two messages in the graph.
typedef opengm::BeliefPropagationUpdateRules <FactorGraph, opengm::Maximizer> UpdateRules;
typedef opengm::MessagePassing<FactorGraph, opengm::Maximizer, UpdateRules, opengm::MaxDistance> LoopyBeliefPropagation;

// Overload of the + operator for vectors:
//		Given vectors a and b it returns a+b. If the vectors are not the same size a -1 vector gets returned to show an error.
template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
//	assert(a.size() == b.size());

	if(a.empty()) // if a doesn't store any element it is the zero-vector, so return b
		return b;
	else if(b.empty()) // if b doesn't store any element it is the zero-vector, so return a
		return a;
	else if(a.size() != b.size()) // if the vectors are not the same size return a vector with -1 as entries as failure
		return std::vector<T>(100, -1);

	std::vector<T> result; // create temporary new vector
	result.reserve(a.size());

	// add each element of the vectors and store them at the corresponding temporary vector position
	std::transform(a.begin(), a.end(), b.begin(),
					std::back_inserter(result), std::plus<T>());

	return result;
}

// Overload of the += operator for vectors:
//		Given vector a and b from arbitrary sizes this operator takes vector b and expands a by these elements.
template <typename T>
std::vector<T>& operator+=(std::vector<T>& a, const std::vector<T>& b)
{
	a.insert(a.end(), b.begin(), b.end());
	return a;
}

class VoronoiRandomFieldSegmentation : public AbstractVoronoiSegmentation
{
protected:

	std::vector<double> angles_for_simulation_; // Vector that saves the angles, used to simulate the laser measurements
												// for the AdaBoost classifier.

	int number_of_classifiers_; // Number of weak classifiers used from the OpenCV AdaBoost function.

	int number_of_classes_; // Number of classes this algorithm can detect.

	bool trained_boost_, trained_conditional_field_; // Variable that shows if the classifiers has already been trained.

#if CV_MAJOR_VERSION == 2
	CvBoostParams params_; // Parameters for the classifiers
	CvBoost room_boost_, hallway_boost_, doorway_boost_; // The AdaBoost-Classifier to induct the features needed in the conditional random field.
#else
	cv::Ptr<cv::ml::Boost> room_boost_, hallway_boost_, doorway_boost_; // The AdaBoost-Classifier to induct the features needed in the conditional random field.
#endif

	std::vector<double> trained_conditional_weights_; // The weights that are needed for the feature-induction in the conditional random field.

	// Function to check if the given point is more far away from each point in the given set than the min_distance.
	bool pointMoreFarAway(const std::set<cv::Point, cv_Point_comp>& points, const cv::Point& point, const double min_distance);

	std::vector<double> raycasting(const cv::Mat& map, const cv::Point& location);

	// Function to get all possible configurations for n variables that each can have m labels. E.g. with 2 variables and 3 possible
	// labels for each variable there are 9 different configurations.
	void getPossibleConfigurations(std::vector<std::vector<uint> >& possible_configurations, const std::vector<uint>& possible_labels,
			const uint number_of_variables);

	// Function that swaps the label-configurations of CRF-nodes in a way s.t. the nodes are sorted in increasing order. Needed
	// to use OpenGM for inference later.
	void swapConfigsRegardingNodeIndices(std::vector<std::vector<uint> >& configurations, size_t point_indices[]);

	// Function to calculate the feature vector for a given clique, using the trained AdaBoost classifiers.
	void getAdaBoostFeatureVector(std::vector<double>& feature_vector, Clique& clique,
			 std::vector<uint>& given_labels, std::vector<unsigned int>& possible_labels);

	// Function that takes a map and draws a pruned voronoi graph in it.
	void createPrunedVoronoiGraph(cv::Mat& map_for_voronoi_generation, std::set<cv::Point, cv_Point_comp>& node_points);

	// Function to find the Nodes for the conditional random field, given a voronoi-graph.
	void findConditonalNodes(std::set<cv::Point, cv_Point_comp>&  conditional_nodes, const cv::Mat& voronoi_map,
			const cv::Mat& distance_map, const std::set<cv::Point, cv_Point_comp>& voronoi_nodes,
			const int epsilon_for_neighborhood, const int max_iterations, const int min_neighborhood_size,
			const double min_node_distance);

	// Function to create a conditional random field out of given points. It needs
	// the voronoi-map extracted from the original map to find the neighbors for each point
	// and the voronoi-node-points to add the right points as nodes.
	void createConditionalField(const cv::Mat& voronoi_map, const std::set<cv::Point, cv_Point_comp>& node_points,
			std::vector<Clique>& conditional_random_field_cliques, const std::set<cv::Point, cv_Point_comp>& voronoi_node_points,
			const cv::Mat& original_map);

	// Function that takes all given training maps and calculates the AdaBoost-Classifiers for them to best label a
	// room, hallway and doorway.
	void trainBoostClassifiers(const std::vector<cv::Mat>& training_maps,
			std::vector< std::vector<Clique> >& cliques_of_training_maps, const std::vector<uint> possible_labels,
			const std::string& classifier_storage_path); // Function to train the AdaBoost classifiers, used for feature induction of the conditional
									  	  	  	  	  	  	  	  	  	  	 	 	 	 	 	 	 	  // random field.

	// Function to find the weights used to calculate the clique potentials.
	void findConditionalWeights(std::vector< std::vector<Clique> >& conditional_random_field_cliques,
			std::vector<std::set<cv::Point, cv_Point_comp> >& random_field_node_points, const std::vector<cv::Mat>& training_maps,
			std::vector<uint>& possible_labels, const std::string weights_filepath);


public:
	// Constructor
	VoronoiRandomFieldSegmentation();

	// This function is used to train the algorithm. The above defined functions separately train the AdaBoost-classifiers and
	// the conditional random field. By calling this function the training is done in the right order, because the AdaBoost-classifiers
	// need to be trained to calculate features for the conditional random field.
	void trainAlgorithms(const std::vector<cv::Mat>& original_maps, const std::vector<cv::Mat>& training_maps,
			std::vector<cv::Mat>& voronoi_maps, const std::vector<cv::Mat>& voronoi_node_maps,
			std::vector<unsigned int>& possible_labels, const std::string storage_path,
			const int epsilon_for_neighborhood, const int max_iterations, const int min_neighborhood_size,
			const double min_node_distance);

	// This function is called to find minimal values of a defined log-likelihood-function using the library Dlib.
	// This log-likelihood-function is made over all training data to get a likelihood-estimation linear in the weights.
	// By minimizing this function the best weights are chosen, what is done here. See voronoi_random_field_segmentation.cpp at
	// the beginning for detailed information.
	// !!!!Important: The more training maps you have, the more factors appear in the log-likelihood over all maps. Be sure not to
	//				  use too much training-maps, because then the log-likelihood-function easily produces values that are out of the
	//				  double range, which Dlib can't handle.
	column_vector findMinValue(unsigned int number_of_weights, double sigma,
			const std::vector<std::vector<double> >& likelihood_parameters, const std::vector<double>& starting_weights); // Function to find the minimal value of a function. Used to find the optimal weights for
								  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  // the conditional random field.

	// Function to segment a given map into different regions. It uses the above trained AdaBoost-classifiers and conditional-random-field.
	// Also it uses OpenGM to do a inference in the created crf, so it uses the above defined typedefs.
	void segmentMap(const cv::Mat& original_map, cv::Mat& segmented_map, const int epsilon_for_neighborhood,
			const int max_iterations, const int min_neighborhood_size, std::vector<uint>& possible_labels,
			const double min_node_distance, bool show_results,
			const std::string classifier_storage_path, const std::string classifier_default_path, const int max_inference_iterations,
			double map_resolution_from_subscription, double room_area_factor_lower_limit, double room_area_factor_upper_limit,
			double max_area_for_merging, std::vector<cv::Point>* door_points = NULL);

	// Function used to test several features separately. Not relevant.
	void testFunc(const cv::Mat& original_map);

};

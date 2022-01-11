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

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h>

#include <boost/filesystem.hpp>

#include <ipa_room_segmentation/timer.h>
#include <ipa_room_segmentation/cv_boost_loader.h>

// This function is the optimization function L(w) = -1 * sum(i)(log(p(y_i|MB(y_i, w), x)) + ((w - w_r)^T (w - w_r)) / 2 * sigma^2)
// to find the optimal weights for the given prelabeled map. to find these the function has to be minimized.
// i indicates the labeled example
// w is the weights vector
// y_i is one node in the build graph
// MB(y_i, w) is the Markov blanket of y_i, in this case the current neighbors of y_i
// w_r is a starting point for w (can be 0 if not known better, so small weights get favorized)
// sigma is the standard deviation, can be chosen freely
// the last part of this function is the gaussian shrinking function to prevent the weights from getting too large
// the local likelihoods are given by the function
// p(y_i|x) = 1/Z(X) * exp(sum(k of K)(w_k^T * f_k(y_k, x)))
//			- K are all cliques in the graph
//			- w_k is the weightvector, regarding to the calculateable features for this clique
//			- f_k is the feature function, calculating a vector of features, that are individual for each clique
// An example for this function, regarding to the voronoi random fields, is:
// L(w) = -(log( (exp(5w_1 + 10w_2)) / (exp(5w_1 + 10w_2) + exp(4w_1 + 7w_2)) ) + log( (exp(7w_1 + 8w_2)) / (exp(7w_1 + 8w_2) + exp(4w_1 + 1w_2)) )) + (w_1-2 w_2-2)^T * (w_1-2 w_2-2) / 2 * 3^2
//
class pseudoLikelihoodOptimization
{
public:
	// number of weights that have to bee calculated
	unsigned int number_of_weights;

	// vector that saves the parameters for each part of the optimization function
	std::vector<std::vector<double> > log_parameters;

	// vector to save the starting_point for the weights
	std::vector<double> starting_weights;

	// the sigma used for the gaussian shrinking function
	double sigma;

	// initializing constructor
	pseudoLikelihoodOptimization()
	{
		number_of_weights = 0;
		sigma = 1.;
	}

	// overload of the () operator, which is needed from Dlib
	double operator()(const column_vector& weights) const
	{
		double result = 0;
		// go trough each part of the function and calculate the log(.) and add it to the result
		for(unsigned int function_part = 0; function_part < log_parameters.size(); ++function_part)
		{
			double splitting_of_exponent = 20; // because this function often produces exponents around 1000 the calculation of one exponential part needs to be splitted
			long double log_numerator = 1., log_denominator = 0.; // numerator and denominator for each log
			long double exp_exponent = 0; // helping variable to get each exponent for exp(.)
			// get the log_numerator for each function part
			for(unsigned int numerator_factor = 0; numerator_factor < number_of_weights; ++numerator_factor)
			{
				exp_exponent += log_parameters[function_part][numerator_factor] * weights(numerator_factor);
			}
			exp_exponent = exp_exponent / splitting_of_exponent;
			for(size_t split = 0; split < splitting_of_exponent; ++split)
				log_numerator = log_numerator * exp(exp_exponent);

			// used for debugging, when parts of the function become too large for double
			if(exp_exponent > 250.0)
			{
				std::cout << "exp exponent: " << exp_exponent << " numerator: " << log_numerator<<  std::endl;
//				for(int i = 0; i < number_of_weights; ++i)
//					std::cout << weights(i) << " ";
//				std::cout << std::endl;
			}

			// add the numerator to the denominator, because it has to appear here
			log_denominator += log_numerator;
			// add each clique-value to the denominator
			unsigned int vector_position = number_of_weights; // variable to get the current absolute starting position
															  // for the vector
			do
			{
				exp_exponent = 0;
				for(unsigned int relative_position = 0; relative_position < number_of_weights; ++relative_position)
				{
					exp_exponent += log_parameters[function_part][vector_position + relative_position] * weights(relative_position);
				}
				exp_exponent = exp_exponent / splitting_of_exponent;
				// update the absolute vector position
				vector_position += number_of_weights;
				// update the denominator
				long double denominator_part = 1.0;
				for(size_t split = 0; split < splitting_of_exponent; ++split)
					denominator_part *= exp(exp_exponent);

				log_denominator += denominator_part;

			}while(vector_position < log_parameters[function_part].size());

			// update the result to return
			result -= log10(log_numerator / log_denominator);
		}

		// add the gaussian shrinking function
		double gaussian_numerator = 0;
		for(unsigned int weight = 0; weight < number_of_weights; ++weight)
		{
			gaussian_numerator += std::pow(weights(weight) - starting_weights[weight], 2.0);
		}
		result += gaussian_numerator / (2.0 * sigma * sigma);
//		std::cout << "gaussian shrinking: " << gaussian_numerator << "/" << (2.0 * sigma * sigma) << std::endl;
//		std::cout << "res: " << result << std::endl;
		return result;
	}
};

// Structs that are used to sort the label configurations in a way s.t. OpenGM can use it properly when defining functions
// and factors.
struct labelWithIndex
{
public:
	uint label; // current label of the configuration configuration
	size_t absolute_index; // index of the node in the global CRF-node-saver
};

struct compLabelsByIndices
{
	bool operator()(labelWithIndex const &a, labelWithIndex const &b)
	{
		return (a.absolute_index < b.absolute_index);
	}
};

// Struct used to sort contours regarding their size
struct compContoursSize
{
	bool operator()(std::vector<cv::Point> const &a, std::vector<cv::Point> const &b)
	{
		return (cv::contourArea(a) >= cv::contourArea(b));
	}
} contourComparer;


// Constructor
VoronoiRandomFieldSegmentation::VoronoiRandomFieldSegmentation()
{
	//save the angles between the simulated beams, used in the following algorithm
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation_.push_back(angle);
	}

	// set number of classes this algorithm can detect
	number_of_classes_ = 3;

	// Set up boosting parameters
	number_of_classifiers_ = 35;
#if CV_MAJOR_VERSION == 2
	CvBoostParams params(CvBoost::DISCRETE, number_of_classifiers_, 0, 2, false, 0);
	params_ = params;
#endif
	trained_boost_ = false;
	trained_conditional_field_ = false;
}

// This Function checks if the given cv::Point is more far away from all the cv::Points in the given set. If one point gets found
// that this Point is nearer than the defined min_distance the function returns false to stop it immediately.
bool VoronoiRandomFieldSegmentation::pointMoreFarAway(const std::set<cv::Point, cv_Point_comp>& points, const cv::Point& point, const double min_distance)
{
	double square_distance = min_distance * min_distance;
	for(std::set<cv::Point, cv_Point_comp>::const_iterator current_point = points.begin(); current_point != points.end(); ++current_point)
	{
		double dx = current_point->x - point.x;
		double dy = current_point->y - point.y;
		if( ((dx*dx + dy*dy)) <= square_distance)
			return false;
	}
	return true;
}

std::vector<double> VoronoiRandomFieldSegmentation::raycasting(const cv::Mat& map, const cv::Point& location)
{
//	cv::Mat test_map (map.rows, map.cols, map.type(), cv::Scalar(255));
	//Raycasting Algorithm. It simulates the laser measurment at the given location and returns the lengths
	//of the simulated beams
	double simulated_x, simulated_y, simulated_cos, simulated_sin;
	double temporary_distance;
	std::vector<double> distances(360, 0);
	double delta_x, delta_y;
	double pi_to_rad = PI / 180;
	for (double angle = 0; angle < 360; angle++)
	{
		simulated_cos = std::cos(angle * pi_to_rad);
		simulated_sin = std::sin(angle * pi_to_rad);
		temporary_distance = 90000001;
		for (double distance = 0; distance < 1000000; ++distance)
		{
			simulated_x = simulated_cos * distance;
			simulated_y = simulated_sin * distance;
			//make sure the simulated Point isn't out of the boundaries of the map
			if (location.x + simulated_x > 0 && location.x + simulated_x < map.rows && location.y + simulated_y > 0 && location.y + simulated_y < map.cols)
			{
				if (map.at<unsigned char>(location.x + simulated_x, location.y + simulated_y) == 0 && distance < temporary_distance)
				{
					temporary_distance = distance;
					break;
				}
			}
		}
		if (temporary_distance > 90000000)
		{
			temporary_distance = 10;
		}
		distances[angle] = temporary_distance;
	}

	return distances;
}


// This function computes all possible configurations for n variables that each can have m labels, e.g. when there are 2 variabels
// with 3 possible label for each there are 9 possible configurations. Important is that this function does compute multiple
// configurations like (1,2) and (2,1).
// This is done using the std::next_permutation function that computes the permutations of a given vector/array. To get all possible
// configurations first a vector is created and filled that stores n*m elements, representing the label for each variable. Then
// after the vector gets sorted all possible permutations are listed and the first n elements in this permutation are one configuration
// and going to be saved, if it hasn't occurred yet.
void VoronoiRandomFieldSegmentation::getPossibleConfigurations(std::vector<std::vector<uint> >& possible_configurations,
		const std::vector<uint>& possible_labels, const uint number_of_variables)
{
	// check how many possible labels there are and create a vector with number_of_variables*number_of_labels length
	uint number_of_labels = possible_labels.size();
	std::vector<uint> label_vector(number_of_labels*number_of_variables);

	// fill the created vector with the labels for each variable
	for(size_t variable = 0; variable < number_of_variables; ++variable)
		for(size_t label = 0; label < number_of_labels; ++label)
			label_vector[label + number_of_labels * variable] = possible_labels[label];

	// sort the vector (expected from std::next_permutation)
	std::sort (label_vector.begin(),label_vector.end());

	// set to save the found configurations --> to easily check if one configuration has already been found
	std::set<std::vector<uint> > found_configurations;

	// get all permutations and create one configuration out of each
	do
	{
		// configuration-vector
		std::vector<uint> current_config(number_of_variables);

		// resave the first number_of_variables elements
		for(size_t v = 0; v < number_of_variables; ++v)
			current_config[v] = label_vector[v];

		// check if the current_configuration has occurred yet, if not add it to the possible configurations
		if(found_configurations.find(current_config) == found_configurations.end())
		{
			found_configurations.insert(current_config);
			possible_configurations.push_back(current_config);
		}

	}while (std::next_permutation(label_vector.begin(),label_vector.end()));
}

//
// This function is used to swap label-configurations in a order s.t. the nodes, which the labels are assigned to, are ordered
// with increasing indices. The indices show the position in the global set that stores all nodes. This is necessary, because
// OpenGM (The library used for inference in this graphical model) expects it this way. To do this the above defined structs
// labelWithIndex and compLabelsByIndices and the function std::sort are used. This function then creates a vector that stores
// the configurations as the struct labelWithIndex and applies a sort on all of it. Then it assignes the new found labels into
// the original vector.
void VoronoiRandomFieldSegmentation::swapConfigsRegardingNodeIndices(std::vector<std::vector<uint> >& configurations,
		size_t point_indices[])
{
	for(size_t configuration = 0; configuration < configurations.size(); ++configuration)
	{
		// go trough configuration and create labels with the corresponding index
		std::vector<labelWithIndex> current_vector;
		for(size_t current_node = 0; current_node < configurations[configuration].size(); ++current_node)
		{
			labelWithIndex current_label = {configurations[configuration][current_node], point_indices[current_node]};
			current_vector.push_back(current_label);
		}
		// sort the current vector
		std::sort(current_vector.begin(), current_vector.end(), compLabelsByIndices());

		// reassign the vector-elements
		for(size_t node = 0; node < configurations[configuration].size(); ++node)
			configurations[configuration][node] = current_vector[node].label;
	}
}

//
// ******* Function to create a conditional random field out of the given points *******************
//
// This function constructs the Conditional Random Field out of the given points, by creating the edges of
// this graph from the cliques as described in segment. This is done by:
//		1. Searching the two nearest neighbors for each Point that isn't a voronoi-node and the 4 nearest neighbors for
//		   points that are voronoi-nodes (if possible to find 4, else 3 is enough, for example when the end of a hallway is reached).
//		   The nearest nodes are found by going along the pruned Voronoi graph to ensure that the found clique is the wanted clique,
//		   containing nodes that are connected by the voronoi-graph.
//		   The searching in a direction on the graph stops, if a new found node is a conditional-random-field-node,
//		   because in this direction the nearest neighbor has been found, and if the algorithm can't find new
//  	   voronoi-graph-points. The second case occurs, when the current node is a dead end and has only one neighbor.
//		2. The found neighbors are defined as a new clique with the current looked at point.
//		3. For each found clique simulate the laserbeams at this point by using the defined raycasting-function. This step is done
//		   because the beams only need to be computed once for each clique and doing this at this point saves much time later.
void VoronoiRandomFieldSegmentation::createConditionalField(const cv::Mat& voronoi_map, const std::set<cv::Point, cv_Point_comp>& node_points,
		std::vector<Clique>& conditional_random_field_cliques, const std::set<cv::Point, cv_Point_comp>& voronoi_node_points,
		const cv::Mat& original_map)
{
	// 1. Search for the n neighbors of the each point by going along the voronoi graph until a conditional-field-node gets
	//	  found.
	std::map<cv::Point, std::vector<double>, cv_Point_comp > raycasts; // map that stores the simulated rays for given OpenCV Points --> some points would get raycasted several times, this saves computation-time

	for(std::set<cv::Point, cv_Point_comp>::const_iterator current_point = node_points.begin(); current_point != node_points.end(); ++current_point)
	{
		// check how many neighbors need to be found --> 4 if the current node is a voronoi graph node, 2 else
		// ( 4 because e.g. a node in the middle of a cross has four neighbors)
		int number_of_neighbors = 2;
		if(voronoi_node_points.find(*current_point) != voronoi_node_points.end())
			number_of_neighbors = 4;

		// vector to save the searched points
		std::vector<cv::Point> searched_points;
		searched_points.push_back(*current_point);

		// vector to save the found neighbors
		std::set<cv::Point, cv_Point_comp> found_neighbors;

		// integer to check if new voronoi nodes could be found
		unsigned int previous_size_of_searched_nodes;

		// go along the Voronoi Graph starting from the current node and find the nearest conditional random field nodes
		do
		{
			// save the size of the searched-points vector
			previous_size_of_searched_nodes = searched_points.size();

			// create temporary-vector to save the new found nodes
			std::vector<cv::Point> temporary_point_vector = searched_points;

			// check each visited point for neighbors that are voronoi nodes
			// remark: I check every point again, because it produces better results than when only new points get expanded.
			for(std::vector<cv::Point>::iterator searching_point = searched_points.begin(); searching_point != searched_points.end(); ++searching_point)
			{
				bool random_field_node = false; // if the current node is a node of the conditional random field, don't go further in this direction
				if(found_neighbors.find(*searching_point) != found_neighbors.end())
					random_field_node = true;

				if(random_field_node == false)
				{
					// check around a 3x3 region for nodes of the voronoi graph
					for(int du = -1; du <= 1; du++)
					{
						for(int dv = -1; dv <= 1; dv++) // && abs(du) + abs(dv) != 0
						{
							// don't check point itself
							if(du == 0 && dv == 0)
								continue;

							// get point that needs to be expanded
							cv::Point point_to_check = cv::Point(searching_point->x + dv, searching_point->y + du);

							// voronoi node is drawn with a value of 127 in the map, don't check already checked points
							if(voronoi_map.at<unsigned char>(point_to_check) == 127
									&& contains(temporary_point_vector, point_to_check) == false)
							{
								// add found voronoi node to searched-points vector
								temporary_point_vector.push_back(point_to_check);

								// Check if point is a conditional random field node. Check on size is to prevent addition of
								// points that appear in the same step and would make the clique too large.
								if(node_points.find(point_to_check) != node_points.end()
										&& found_neighbors.size() < number_of_neighbors)
									found_neighbors.insert(point_to_check);
							}
						}
					}
				}
			}

			// assign the temporary-vector as the new reached-points vector
			searched_points = temporary_point_vector;

		}while(found_neighbors.size() < number_of_neighbors && previous_size_of_searched_nodes != searched_points.size());

		// 2. create a clique out of the current node and its found neighbors
		conditional_random_field_cliques.push_back(Clique(*current_point));
		std::vector<cv::Point> neighbor_vector(found_neighbors.begin(), found_neighbors.end()); // convert set to a vector to easily insert the new members
		conditional_random_field_cliques.back().insertMember(neighbor_vector);

		// 3. Simulate the laser-beams at each found member and store them. This step saves a lot of computation time later.
		std::vector<cv::Point> clique_members = conditional_random_field_cliques.back().getMemberPoints();
		std::vector< std::vector<double> > laser_beams(clique_members.size());

		for(size_t member = 0; member < clique_members.size(); ++member)
		{
			laser_beams[member] = raycasting(original_map, cv::Point(clique_members[member].y, clique_members[member].x));
		}

		conditional_random_field_cliques.back().setBeamsForMembers(laser_beams);
	}
}

//**************************Training-Algorithm for the AdaBoost-classifiers*****************************
// This Function trains the AdaBoost-classifiers from OpenCV. It takes the given training maps and finds the Points
// that are labeled as the specified classes and calculates the features defined in
//
// ipa_room_segmentation/voronoi_random_field_features.h
//
// These vectors are put in a format that OpenCV expects for the classifiers and then they are trained.
void VoronoiRandomFieldSegmentation::trainBoostClassifiers(const std::vector<cv::Mat>& training_maps,
		std::vector< std::vector<Clique> >& cliques_of_training_maps, std::vector<uint> possible_labels,
		const std::string& classifier_storage_path)
{
	boost::filesystem::path storage_path(classifier_storage_path);
	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: VoronoiRandomFieldSegmentation::trainBoostClassifiers: Could not create directory " << storage_path << std::endl;
			return;
		}
	}

	std::cout << "starting to train the Boost Classifiers." << std::endl;

	// vectors that store the given labels and features for each point (order: room-hallway-doorway)
	std::vector< std::vector<int> > labels_for_classes(number_of_classes_);
	std::vector< std::vector<double> > features_for_points;

	// go trough each found clique and take the first point of the clique as current point
	//	--> each possible point is only once the first (central) point of a clique
	voronoiRandomFieldFeatures vrf_feature_computer;
	for(size_t map = 0; map < training_maps.size(); ++map)
	{
		cv::Mat current_map = training_maps[map];
		for(std::vector<Clique>::iterator current_clique = cliques_of_training_maps[map].begin(); current_clique != cliques_of_training_maps[map].end(); ++current_clique)
		{
			// get all members of the current clique (used later)
			std::vector<cv::Point> current_clique_members = current_clique->getMemberPoints();

			// get the central point of the clique
			cv::Point current_point = current_clique_members[0];

			// get the stored labels for these points
			std::vector<uint> current_labels_for_points(current_clique_members.size());

			for(size_t point = 0; point < current_clique_members.size(); ++point)
			{
				current_labels_for_points[point] = current_map.at<uchar>(current_clique_members[point]);
			}

			// get the stored laser-beams for the central point
			std::vector<double> current_beams = current_clique->getBeams()[0];

			// get the feature for the current point and store it in the global vector
			std::vector<double> current_features;

			vrf_feature_computer.getFeatures(current_beams, angles_for_simulation_, current_clique_members, current_labels_for_points, possible_labels, current_point, current_features);
			features_for_points.push_back(current_features);

			// get the labels-vector for each class
			//		--> OpenCV expects the labels: +1 if it belongs to the class, -1 if it doesn't
			for(size_t current_class = 0; current_class < number_of_classes_; ++current_class)
			{
				if(current_labels_for_points[0] == possible_labels[current_class])
					labels_for_classes[current_class].push_back(1);
				else
					labels_for_classes[current_class].push_back(-1);
			}
		}
	}

	std::cout << "found all features and labels." << std::endl;

	// Train each AdaBoost-classifier.
	//
	//*************room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_classes[0]);
	cv::Mat features_Mat(features_for_points.size(), vrf_feature_computer.getFeatureCount(), CV_32FC1);
	for (int i = 0; i < labels_for_classes[0].size(); i++)
	{
		for (int f = 0; f < vrf_feature_computer.getFeatureCount(); f++)
		{
			features_Mat.at<float>(i, f) = (float) features_for_points[i][f];
		}
	}
	std::string filename_room = classifier_storage_path + "vrf_room_boost.xml";
#if CV_MAJOR_VERSION == 2
	// Train a boost classifier
	room_boost_.train(features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	room_boost_.save(filename_room.c_str(), "boost");
#else
	// Train a boost classifier
	room_boost_ = cv::ml::Boost::create();
	room_boost_->setBoostType(cv::ml::Boost::REAL);
	room_boost_->setWeakCount(number_of_classifiers_);
	room_boost_->setWeightTrimRate(0);
	room_boost_->setMaxDepth(2);
	room_boost_->setUseSurrogates(false);
	room_boost_->setPriors(cv::Mat());
	room_boost_->train(features_Mat, cv::ml::ROW_SAMPLE, room_labels_Mat);
	//save the trained booster
	room_boost_->save(filename_room.c_str());
#endif
	std::cout << "Trained room classifier" << std::endl;

	//
	//*************hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_classes[1]);
	std::string filename_hallway = classifier_storage_path + "vrf_hallway_boost.xml";
#if CV_MAJOR_VERSION == 2
	// Train a boost classifier
	hallway_boost_.train(features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	hallway_boost_.save(filename_hallway.c_str(), "boost");
#else
	// Train a boost classifier
	hallway_boost_ = cv::ml::Boost::create();
	hallway_boost_->setBoostType(cv::ml::Boost::REAL);
	hallway_boost_->setWeakCount(number_of_classifiers_);
	hallway_boost_->setWeightTrimRate(0);
	hallway_boost_->setMaxDepth(2);
	hallway_boost_->setUseSurrogates(false);
	hallway_boost_->setPriors(cv::Mat());
	hallway_boost_->train(features_Mat, cv::ml::ROW_SAMPLE, hallway_labels_Mat);
	//save the trained booster
	hallway_boost_->save(filename_hallway.c_str());
#endif
	std::cout << "Trained hallway classifier" << std::endl;

	//
	//*************doorway***************
	//save the found labels and features in Matrices
	cv::Mat doorway_labels_Mat(labels_for_classes[2]);
	std::string filename_doorway = classifier_storage_path + "vrf_doorway_boost.xml";
#if CV_MAJOR_VERSION == 2
	// Train a boost classifier
	doorway_boost_.train(features_Mat, CV_ROW_SAMPLE, doorway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	doorway_boost_.save(filename_doorway.c_str(), "boost");
#else
	// Train a boost classifier
	doorway_boost_ = cv::ml::Boost::create();
	doorway_boost_->setBoostType(cv::ml::Boost::REAL);
	doorway_boost_->setWeakCount(number_of_classifiers_);
	doorway_boost_->setWeightTrimRate(0);
	doorway_boost_->setMaxDepth(2);
	doorway_boost_->setUseSurrogates(false);
	doorway_boost_->setPriors(cv::Mat());
	doorway_boost_->train(features_Mat, cv::ml::ROW_SAMPLE, doorway_labels_Mat);
	//save the trained booster
	doorway_boost_->save(filename_doorway.c_str());
#endif
	std::cout << "Trained doorway classifier" << std::endl;

	// set the trained Boolean for the AdaBoost-classifiers to true
	trained_boost_ = true;
	std::cout << "Finished training the Boost algorithm." << std::endl;
}

//
// ********************* Function to calculate the feature-vector for a given clique. ***********************
//
// This function calculates the feature vector for a given clique, using the trained AdaBoost classifiers. These calculate
// different boost-features specific for the given point and neighbors of it and then multiplies these with a weight-vector
// producing weak hypothesis to specify the label of the current point. These weak hypothesis are used as features for the
// conditional random field. The possible_labels vector stores the possible labels a point in the training map can have in
// the order:
//		room, hallway, doorway
void VoronoiRandomFieldSegmentation::getAdaBoostFeatureVector(std::vector<double>& feature_vector, Clique& clique,
		std::vector<uint>& given_labels, std::vector<unsigned int>& possible_labels)
{
	// Get the points that belong to the clique and the stored simulated beams for each one.
	std::vector<cv::Point> clique_members = clique.getMemberPoints();
	std::vector< std::vector<double> > beams_for_points = clique.getBeams();

	// vector that is used to sum up the calculated features
	std::vector<double> temporary_feature_vector(feature_vector.size(), 0.0);

	// For each member of this clique calculate the weak-hypothesis and add the resulting vectors in the end
	for(size_t point = 0; point < clique_members.size(); ++point)
	{
		// Check which classifier (room, hallway or doorway) needs to be used.
		unsigned int classifier;
		for(size_t label = 0; label < possible_labels.size(); ++label)
		{
			if(possible_labels[label] == given_labels[point])
			{
				classifier = label;
				break;
			}
		}

		// get the features for the central point of the clique
		voronoiRandomFieldFeatures vrf_feature_computer;
		cv::Mat featuresMat(1, vrf_feature_computer.getFeatureCount(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
		std::vector<double> current_features;
		vrf_feature_computer.getFeatures(beams_for_points[point], angles_for_simulation_, clique_members, given_labels, possible_labels, clique_members[point], current_features);

		for (int f = 1; f <= vrf_feature_computer.getFeatureCount(); ++f)
		{
			featuresMat.at<float>(0, f - 1) = (float) current_features[f-1];
		}

		// Calculate the weak hypothesis by using the wanted classifier.
#if CV_MAJOR_VERSION<=3
		CvMat features = featuresMat;
		cv::Mat weaker (1, number_of_classifiers_, CV_32F);
		CvMat weak_hypothesis = weaker;	// Wanted from OpenCV to get the weak hypothesis from the
										// separate weak classifiers.
#else
		cv::Mat features = featuresMat;
		cv::Mat weaker (1, number_of_classifiers_, CV_32F);
#endif

		// For each point the classifier depends on the given label. If the point is labeled as a room the room-boost should be
		// used and so on.
		switch(classifier)
		{
#if CV_MAJOR_VERSION == 2
		case 0:
			room_boost_.predict(&features, 0, &weak_hypothesis);
			break;
		case 1:
			hallway_boost_.predict(&features, 0, &weak_hypothesis);
			break;
		case 2:
			doorway_boost_.predict(&features, 0, &weak_hypothesis);
			break;
#else
		case 0:
			room_boost_->predict(featuresMat, weaker);
			break;
		case 1:
			hallway_boost_->predict(featuresMat, weaker);
			break;
		case 2:
			doorway_boost_->predict(featuresMat, weaker);
			break;
#endif
		}


		// Write the weak hypothesis in the feature vector.
		for(size_t f = 0; f < number_of_classifiers_; ++f)
		{
#if CV_MAJOR_VERSION<=3
			temporary_feature_vector[f] = temporary_feature_vector[f] + (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
#else
			temporary_feature_vector[f] = temporary_feature_vector[f] + (double) weaker.at<float>(0, f);
#endif
		}
	}

	// copy the summed vector to the given feature-vector
	feature_vector = temporary_feature_vector;
}

//
//********************* Function to find the conditional field weights. ****************
//
// This function is used to find the weights, that are used to compute the clique potentials. The AdaBoost classifier trained
// before gives the vector f(y_k, x) that stores the values for each feature, calculated for a given clique in the
// random field. to get the clique potential this vector gets multiplied by w^T, which is the transposed weight-vector, calculated
// here. These weights define the importance of one single feature in classifying the clique. To calculate the weights the
// following steps are done:
//		I. For each given field-node calculate the features of the clique, using the AdaBoost classifiers. The features for the
//		   conditonal-random-field are the weak hypothesis, produced by the AdaBoost classifers. This hypothesis is the
//		   predicted label for a given point, multiplied by the weight for the selected classifier. For more details see
//		   the trainBoostClassifiers() or any document on Boosting.
//		II. The above defined features are used to maximize the pseudo-likelihood over all training-data by minimizing a
//			feature-function, by using the Dlib-c++-library. The found weights are then saved at the location, which is given
//			to this function.
//
void VoronoiRandomFieldSegmentation::findConditionalWeights(std::vector< std::vector<Clique> >& conditional_random_field_cliques,
		std::vector<std::set<cv::Point, cv_Point_comp> >& random_field_node_points, const std::vector<cv::Mat>& training_maps,
		std::vector<uint>& possible_labels, const std::string weights_filepath)
{
	// check if the AdaBoost-classifiers has already been trained yet, if not the conditional field can't be trained
	if(trained_boost_ == false)
		ROS_ERROR("AdaBoost-classifiers haven't been trained yet. First train the AdaBoost algorithm before training the conditional-random-field");

	std::cout << "Starting to train the conditional-random-field." << std::endl;

	// ************ I. Go trough each found point and compute the pseudo-likelihood of it to get one big likelihood. ****************

	std::vector<std::vector<double> > all_point_feature_vectors; // Vector that stores every feature vector calculated for the
																 // found nodes of the CRF. One vector of it stores all features
																 // for different labels, in a specified order. The first
																 // values are for the real training label and the rest of it
																 // for labels different than the given one.

	for(size_t current_map_index = 0; current_map_index < training_maps.size(); ++current_map_index)
	{
		for(std::set<cv::Point, cv_Point_comp>::iterator current_point = random_field_node_points[current_map_index].begin(); current_point != random_field_node_points[current_map_index].end(); ++current_point)
		{
			// vector to save the cliques that were found for one point
			std::vector<Clique> cliques_for_point;

			// vector that stores the labels of each clique-member for the current point
			std::vector< std::vector<uint> > labels_of_cliques;

			// set the given training label for this point
			unsigned int real_label = training_maps[current_map_index].at<unsigned char>(*current_point);

			// for each point find the cliques that this point belongs to
			for(std::vector<Clique>::iterator current_clique = conditional_random_field_cliques[current_map_index].begin(); current_clique != conditional_random_field_cliques[current_map_index].end(); ++current_clique)
			{
				if(current_clique->containsMember(*current_point))
				{
					cliques_for_point.push_back(*current_clique);

					// search for the labels of the clique-members
					std::vector<uint> temporary_clique_labels(current_clique->getNumberOfMembers());
					std::vector<cv::Point> members = current_clique->getMemberPoints();

					for(size_t member = 0; member < current_clique->getNumberOfMembers(); ++member)
						temporary_clique_labels[member] = training_maps[current_map_index].at<unsigned char>(members[member]);

					// save the found labels
					labels_of_cliques.push_back(temporary_clique_labels);
				}
			}

			// For each found clique compute the feature vector for different labels. The first label is the label that was
			// given to the algorithm by the training data and the other are the remaining labels, different from the first.
			std::vector<std::vector<double> > feature_vectors(number_of_classes_); // vector to store the found feature-vectors for each class

			// vector to store the feature-vectors computed for the different cliques
			std::vector<std::vector<double> > temporary_feature_vectors(cliques_for_point.size(), std::vector<double>(number_of_classifiers_, 0.0));

			// get the clique-feature-vectors for the given training label and add them to the first feature-vector for this label
			for(size_t clique = 0; clique < cliques_for_point.size(); ++clique)
			{
				getAdaBoostFeatureVector(temporary_feature_vectors[clique], cliques_for_point[clique], labels_of_cliques[clique], possible_labels);
				feature_vectors[0] = feature_vectors[0] + temporary_feature_vectors[clique];
			}

			// assign the first feature-vector to the complete feature-vector
			all_point_feature_vectors.push_back(feature_vectors[0]);

			// get the other feature-vectors for the different labels
			unsigned int label_index = 1; // variable to access the right places in feature_vectors for each possible label
			for(size_t label_position = 0; label_position < possible_labels.size(); ++label_position)
			{
				// only compute different labels
				if(possible_labels[label_position] != real_label)
				{
					for(size_t clique = 0; clique < cliques_for_point.size(); ++clique)
					{
						// copy the real-label-vector and change the label for the current point --> see how clique potential changes
						std::vector<uint> temporary_labels = labels_of_cliques[clique];
						std::vector<cv::Point> current_clique_members = cliques_for_point[clique].getMemberPoints();

						int point_position = std::find(current_clique_members.begin(), current_clique_members.end(), *current_point) - current_clique_members.begin();

						temporary_labels[point_position] = possible_labels[label_position];

						// get the AdaBoost-feature-vector
						getAdaBoostFeatureVector(temporary_feature_vectors[clique], cliques_for_point[clique], temporary_labels, possible_labels);
						feature_vectors[label_index] = feature_vectors[label_index] + temporary_feature_vectors[clique];
					}
					// append the last vector in all_point_feature_vector by the calculated vector
					all_point_feature_vectors.back() += feature_vectors[label_index];
					// set index for labels one step higher
					++label_index;
				}
			}
		}
	}

	//
	// ********* II. Find the weights that minimize the total likelihood by using the Dlib-library. ***********
	//
	// define the mean-weights for the gaussian shrinking function
	std::vector<double> mean_weights(number_of_classifiers_, 0);

	voronoiRandomFieldFeatures vrf_features;
	cv::Mat featuresMat(1, vrf_features.getFeatureCount(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
	for (int f = 1; f <= vrf_features.getFeatureCount(); ++f)
		featuresMat.at<float>(0, f - 1) = (float) 1;

	// Calculate the weak hypothesis by using the wanted classifier. The weak hypothesis is given by h_i(x) = w_i * f_i(x)
#if CV_MAJOR_VERSION<=3
	CvMat features = featuresMat;
	cv::Mat weaker (1, number_of_classifiers_, CV_32F);
	CvMat weak_hypothesis = weaker;	// Wanted from OpenCV to get the weak hypothesis from the
									// separate weak classifiers.
#else
	cv::Mat features = featuresMat;
	cv::Mat weaker(1, number_of_classifiers_, CV_32F);
#endif

	// Get weights for room, hallway and doorway classifier.
#if CV_MAJOR_VERSION == 2
	room_boost_.predict(&features, 0, &weak_hypothesis);
#else
	room_boost_->predict(featuresMat, weaker);
#endif

	for(size_t f = 0; f < number_of_classifiers_; ++f)
#if CV_MAJOR_VERSION<=3
		mean_weights[f] += (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
#else
		mean_weights[f] += (double) weaker.at<float>(0, f);
#endif

#if CV_MAJOR_VERSION == 2
	hallway_boost_.predict(&features, 0, &weak_hypothesis);
#else
	hallway_boost_->predict(featuresMat, weaker);
#endif

	for(size_t f = 0; f < number_of_classifiers_; ++f)
#if CV_MAJOR_VERSION<=3
		mean_weights[f] *= (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
#else
		mean_weights[f] *= (double) weaker.at<float>(0, f);
#endif

#if CV_MAJOR_VERSION == 2
	doorway_boost_.predict(&features, 0, &weak_hypothesis);
#else
	doorway_boost_->predict(featuresMat, weaker);
#endif

	for(size_t f = 0; f < number_of_classifiers_; ++f)
#if CV_MAJOR_VERSION<=3
		mean_weights[f] *= (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
#else
		mean_weights[f] *= (double) weaker.at<float>(0, f);
#endif

	// find the best weights --> minimize the defined function for the pseudo-likelihood
	std::cout << "finding weights using Dlib" << std::endl;
	column_vector weight_results;
	weight_results = findMinValue(number_of_classifiers_, 9.0, all_point_feature_vectors, mean_weights);

	// clear the already found weights, if trained more than one time
	trained_conditional_weights_.clear();

	// save the found weights to a std::vector<double>
	for(size_t weight = 0; weight < number_of_classifiers_; ++weight)
		trained_conditional_weights_.push_back(weight_results(0, weight));

	// save the weights to a file
	boost::filesystem::path storage_path(weights_filepath);
	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: VoronoiRandomFieldSegmentation::findConditionalWeights: Could not create directory " << storage_path << std::endl;
			return;
		}
	}
	std::string filename = weights_filepath + "vrf_conditional_field_weights.txt";
	std::ofstream output_file(filename.c_str(), std::ios::out);
	if (output_file.is_open()==true)
		output_file << weight_results;
	output_file.close();

	//set the trained-variable true, so the labeling-algorithm knows the classifiers have been trained already
	trained_conditional_field_ = true;
	std::cout << "Finished training the Conditional Field." << std::endl;

}

//
//********************* Function to train the whole algorithm. ****************
//
//		I. Go trough each given training map and find the drawn points, that represent the nodes for the conditional random
//		   field. Also it finds the voronoi-nodes that are drawn in a different color than the other nodes. These points are
//		   used in the second step of this function to create the cliques in the conditional random field.
//		II. For each given training map the found nodes are used to create conditional random fields. The whole algorithm is
//			based on this and to train the algorithm a crf for each training map is needed. This step produces cliques, that
//			all have a clique-potential, depending on features and the crf-weights, that will get maximized later.
//		III. In the next step the AdaBoost-classifiers are trained. This is done by giving the above defined function
//			 trainBoostClassifiers() the given training maps. This function searches for points labeled as room/hallway/doorway
//			 and adds them as demonstration-data for AdaBoost-classifiers.
//		IV. In the last step the weights for the conditional-random-field are found. As said above the clique-potentials are
//			depending on these weights and so they are chosen to maximize the potentials over all training maps. Because this
//			would be really hard to do directly, a log-likelihood estimation is applied.
void VoronoiRandomFieldSegmentation::trainAlgorithms(const std::vector<cv::Mat>& original_maps, const std::vector<cv::Mat>& training_maps,
		std::vector<cv::Mat>& voronoi_maps, const std::vector<cv::Mat>& voronoi_node_maps,
		std::vector<unsigned int>& possible_labels, const std::string storage_path,
		const int epsilon_for_neighborhood, const int max_iterations, const int min_neighborhood_size,
		const double min_node_distance)
{
	// ********** I. Go trough each map and find the drawn node-points for it and check if it is a voronoi-node. *****************
	std::vector<std::set<cv::Point, cv_Point_comp> > random_field_node_points, voronoi_node_points;

	std::cout << "Starting to find the conditional-random-field-cliques." << std::endl;

	bool compute_voronoi_maps = false;
	if (voronoi_maps.size() != original_maps.size())
	{
		compute_voronoi_maps = true;
		voronoi_maps.resize(original_maps.size());
		std::cout << "Creating the voronoi graphs before training." << std::endl;
	}
	for(size_t current_map_index = 0; current_map_index < training_maps.size(); ++current_map_index)
	{
		// Find conditional field nodes by checking each pixel for its color.
		const cv::Mat& current_map = training_maps[current_map_index];
		std::set<cv::Point, cv_Point_comp> current_nodes, current_voronoi_nodes;
		if (compute_voronoi_maps == false && voronoi_node_maps.size() == original_maps.size())
		{
			const cv::Mat& current_voronoi_node_map = voronoi_node_maps[current_map_index];
			for(size_t v = 0; v < current_map.rows; ++v)
			{
				for(size_t u = 0; u < current_map.cols; ++u)
				{
					// check if the current point is a voronoi-node by checking the color in the voronoi-node map
					if(current_voronoi_node_map.at<unsigned char>(v, u) != 255 && current_voronoi_node_map.at<unsigned char>(v, u) != 0)
						current_voronoi_nodes.insert(cv::Point(u, v));
				}
			}
		}
		else
		{
			voronoi_maps[current_map_index] = original_maps[current_map_index].clone();
			createPrunedVoronoiGraph(voronoi_maps[current_map_index], current_voronoi_nodes);
		}

		// read in a fully labeled map (not only points) and generate current_nodes accordingly
		// find the conditional random field nodes for the current map
		cv::Mat distance_map; //distance-map of the original-map (used to check the distance of each point to nearest black pixel)
#if CV_MAJOR_VERSION<=3
		cv::distanceTransform(original_maps[current_map_index], distance_map, CV_DIST_L2, 5);
#else
		cv::distanceTransform(original_maps[current_map_index], distance_map, cv::DIST_L2, 5);
#endif
		cv::convertScaleAbs(distance_map, distance_map);

		// find all nodes for the conditional random field
		findConditonalNodes(current_nodes, voronoi_maps[current_map_index], distance_map, current_voronoi_nodes, epsilon_for_neighborhood, max_iterations, min_neighborhood_size, min_node_distance);

		// save the found nodes
		random_field_node_points.push_back(current_nodes);
		voronoi_node_points.push_back(current_voronoi_nodes);
	}

	// ********** II. Create the conditional random fields. *****************
	std::cout << "Creating the conditional-random-field-cliques." << std::endl;

	std::vector<std::vector<Clique> > conditional_random_field_cliques;
	for(size_t current_map = 0; current_map < training_maps.size(); ++current_map)
	{
		// create conditional random field
		std::vector<Clique> current_cliques;
		createConditionalField(voronoi_maps[current_map], random_field_node_points[current_map], current_cliques, voronoi_node_points[current_map], original_maps[current_map]);

		// save the found cliques
		conditional_random_field_cliques.push_back(current_cliques);
	}

	// ********** III. Train the AdaBoost-classifiers. *****************
	trainBoostClassifiers(training_maps, conditional_random_field_cliques, possible_labels, storage_path);

	// ********** IV. Find the conditional-random-field weights. *****************
	findConditionalWeights(conditional_random_field_cliques, random_field_node_points, training_maps, possible_labels, storage_path);
}

//
//****************Create the pruned generalized Voronoi-Graph**********************
//
//This function is here to create the pruned generalized voronoi-graph in the given map. It does following steps:
//	1. Creates a Voronoi Graph
//  2. It reduces the graph until the nodes in the graph. A node is a point on the voronoi graph, that has at least 3
//	   neighbors. This deletes errors from the approximate generation of the graph that hasn't been eliminated from
//	   the drawVoronoi function. the resulting graph is the pruned generalized voronoi graph.
//	3. It returns the map that has the pruned generalized voronoi-graph drawn in.
void VoronoiRandomFieldSegmentation::createPrunedVoronoiGraph(cv::Mat& map_for_voronoi_generation, std::set<cv::Point, cv_Point_comp>& node_points)
{
	//********************1. Create the Voronoi graph******************************
	createVoronoiGraph(map_for_voronoi_generation);

	//********************2. Reduce the graph until its nodes******************************
	pruneVoronoiGraph(map_for_voronoi_generation, node_points);
}

// This function finds nodes for the conditional random field.
//	It looks at a given pruned voronoi graph and concentrates a defined region on this graph into one point, that is
//	used as a node in a graph. In this graph nodes are connected, that
//		i) are right beside each other
//		ii) and if a Point in the graph has three or more neighbors all of these four nodes are connected to each other
//	so that different cliques occur. This is necessary for using a Conditional Random Filed to label the nodes as a
//	defined class. To do so the following steps are done:
//		1. Add the previously found node points on the voronoi graph to the Conditional Random Field graph. The algorithm
//		   checks if they are too close to each other and only adds one of these close points to the crf.
//		2. Look at an epsilon neighborhood on the graph and choose the point farthest away from the black pixels as
//		   point for the crf. The farthest point is chosen, because else it would be possible  that the chosen point
//		   is too near at the black pixels. Also the new node has to be more far away than a defined  min. distance to
//		   each already found node s.t. two nodes are not too close to each other, or else the  crf would be created wrong.
void VoronoiRandomFieldSegmentation::findConditonalNodes(std::set<cv::Point, cv_Point_comp>&  conditional_nodes,
		const cv::Mat& voronoi_map,	const cv::Mat& distance_map, const std::set<cv::Point, cv_Point_comp>& voronoi_nodes,
		const int epsilon_for_neighborhood,	const int max_iterations, const int min_neighborhood_size,
		const double min_node_distance)
{
	// add the given voronoi nodes as conditional nodes, if they are far away enough from each other
	for(std::set<cv::Point, cv_Point_comp>::iterator node = voronoi_nodes.begin(); node != voronoi_nodes.end(); ++node)
	{
		if(pointMoreFarAway(conditional_nodes, *node, min_node_distance) == true)
			conditional_nodes.insert(*node);
	}

	// create a copy of the given voronoi map to keep track of which points already have been looked at
	cv::Mat voronoi_map_for_node_extraction = voronoi_map.clone();

	// go trough the copied voronoi map and concentrate neighborhoods into one central point
	for (int v = 0; v < voronoi_map_for_node_extraction.rows; v++)
	{
		for (int u = 0; u < voronoi_map_for_node_extraction.cols; u++)
		{
			if (voronoi_map_for_node_extraction.at<unsigned char>(v, u) == 127)
			{
				int loopcounter = 0; // if a part of the graph is not connected to the rest this variable helps to stop the loop
				std::set<cv::Point, cv_Point_comp> neighbor_points;	// neighboring-variable, which is different for each point
				std::set<cv::Point, cv_Point_comp> temporary_points, searching_points;	// Sets that are used to save new found points and expand them in the next iteration.
																						// temporary points to expand the neighborhood in iterative steps and not in one

				int neighbor_count = 0;		// variable to save the number of neighbors for each point

				neighbor_points.insert(cv::Point(u, v)); //add the current Point to the neighborhood and to the expanding-queue
				searching_points.insert(cv::Point(u, v));

				do
				{
					loopcounter++;
					// check every point in the neighborhood for other neighbors connected to it
					for (std::set<cv::Point, cv_Point_comp>::iterator current_neighbor_point = searching_points.begin(); current_neighbor_point != searching_points.end(); ++current_neighbor_point)
					{
						for (int row_counter = -1; row_counter <= 1; row_counter++)
						{
							for (int column_counter = -1; column_counter <= 1; column_counter++)
							{
								// don't check the point itself
								if(row_counter == 0 && column_counter == 0)
									continue;

								// check the neighboring points
								// (if it already is in the neighborhood it doesn't need to be checked again)
								const int nu = current_neighbor_point->x + column_counter;
								const int nv = current_neighbor_point->y + row_counter;

								if (neighbor_points.find(cv::Point(nu, nv)) == neighbor_points.end() &&
										nv >= 0 && nu >= 0 &&
										nv < voronoi_map_for_node_extraction.rows && nu < voronoi_map_for_node_extraction.cols &&
										voronoi_map_for_node_extraction.at<unsigned char>(nv, nu) == 127)
								{
									neighbor_count++;
									temporary_points.insert(cv::Point(nu, nv));
								}
							}
						}
					}

					// go trough every found point after all neighborhood points have been checked and add them to it
					for (std::set<cv::Point, cv_Point_comp>::iterator temporary_point = temporary_points.begin(); temporary_point != temporary_points.end(); ++temporary_point)
					{
						neighbor_points.insert(*temporary_point);

						// make the found points white in the voronoi-map (already looked at)
						voronoi_map_for_node_extraction.at<unsigned char>(*temporary_point) = 255;
					}

					// make the current point white --> doesn't need to be looked at anymore
					voronoi_map_for_node_extraction.at<unsigned char>(v, u) = 255;

					// reassign the new found nodes as new to-be-searched nodes
					searching_points = temporary_points;

					// check if enough neighbors have been checked or checked enough times (e.g. at a small segment of the graph)
				}while (neighbor_count <= epsilon_for_neighborhood && loopcounter < max_iterations);

				// only check the neighborhood, if it is large enough --> to prevent nodes that are close to each other
				if(neighbor_count >= min_neighborhood_size)
				{
					// check every found point in the neighborhood if it is the local minimum in the distanceMap and if
					// this point is far enough away from other node points to prevent too close nodes
					cv::Point current_conditional_field_point = cv::Point(u, v);
					for (std::set<cv::Point, cv_Point_comp>::iterator point = neighbor_points.begin(); point != neighbor_points.end(); ++point)
					{
						if (distance_map.at<unsigned char>(*point) < distance_map.at<unsigned char>(current_conditional_field_point)
								&& pointMoreFarAway(conditional_nodes, *point, min_node_distance) == true)
						{
							current_conditional_field_point = *point;
						}
					}
					// add the local minimum point to the critical points and check a last time if the node is far enough away
					// from other nodes (because if no new node is found the initialized gets added every time, neglecting
					// this constraint)
					if(pointMoreFarAway(conditional_nodes, current_conditional_field_point, min_node_distance) == true)
						conditional_nodes.insert(current_conditional_field_point);
				}
			}
		}
	}
}

// This function is called to find minimal values of a defined log-likelihood-function using the library Dlib.
// This log-likelihood-function is made over all training data to get a likelihood-estimation linear in the weights.
// By minimizing this function the best weights are chosen, what is done here. See beginning of this file for detailed information.
// !!!!Important: Numerical problems might occur --> non finite outputs. This is because the derivative gets approximated.
//				  Change the starting point and the last entry of find_min_using_approximate_derivatives when this occurs.
column_vector VoronoiRandomFieldSegmentation::findMinValue(unsigned int number_of_weights, double sigma,
		const std::vector<std::vector<double> >& likelihood_parameters, const std::vector<double>& starting_weights)
{
	std::cout << "finding min values" << std::endl;
	// create a column vector as starting search point, that is needed from Dlib to find the min. value of a function
	column_vector starting_point(number_of_weights);

	// initialize the starting point as zero to favor small weights
	starting_point = 1e-1;

	// create a Likelihood-optimizer object to find the weights that maximize the pseudo-likelihood
	pseudoLikelihoodOptimization minimizer;

	// set the values for this optimization-object
	minimizer.sigma = sigma;
	minimizer.number_of_weights = number_of_weights;

	minimizer.log_parameters = likelihood_parameters;
	minimizer.starting_weights = starting_weights;

	// find the best weights for the given parameters
	dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), minimizer, starting_point, -1, 1e-10);

	return starting_point;
}



//
//****************** Segmentation Function *********************
//
// This function segments the given original_map into different regions by using the voronoi random field method from
// Stephen Friedman and Dieter Fox ( http://www.cs.washington.edu/robotics/projects/semantic-mapping/abstracts/vrf-place-labeling-ijcai-07.abstract.html ).
// This algorithm has two parts, the training step and the actual segmentation. The training should be finished, see above for details.
// In the segmentation step following actions are made:
//		I.) From the given map that should be labeled (original_map) a pruned generalized Voronoi diagram is extracted ( https://www.sthu.org/research/voronoidiagrams/ ).
//			This is done using the method from Karimipour and Ghandehari ( A Stable Voronoi-based Algorithm for Medial Axis Extraction through Labeling Sample Points )
//			that samples the building contour to get centerpoints to compute the voronoi graph. This approximated graph has
//			some errors in it, so two elimination steps are done:
//				i) eliminate lines of the graph that start or end in black regions
//				ii) reduce the graph from after the first step until the nodes of the graph
//			See the createPrunedVoronoiGraph() function above for better information. OpenCV is used to do this.
//		II.) It finds Nodes for the Conditional Random Field, using the above defined function findConditionalNodes().
//		III.) It constructs the Conditional Random Field graph from the previously found points, by using the above defined function.
//		IV.) It creates a factor graph out of the defined crf. This is necessary, because OpenGM, the library used for inference,
//			 needs it this way. To do so the algorithm goes trough each found clique and computes the clique-potential of it
//			 for all possible label-configuration. The results of this are saved in an opengm::function so OpenGM can use it.
//		V.) After the factor graph has been build, an inference algorithm is applied to find the labels that maximize the complete
//		 	value of the graph, meaning the potential of the crf. To do this not the Product of the factors are maximized, but the
//			sum of the exponents. The potentials are given by exp(w^T * f) and the graph-potential is the product of these, so
//			it is possible to just maximize the sum of all exponents. This is convenient, because the factors will scale very high
//			and would go out of the double range.
//		VI.) At the last step the algorithm takes the above found best labels and draws it into a copy of the original map. The
//			 rooms and hallways are drawn with the color of this class and doorways are drawn black. This is done because it
//			 produces intersections between different segments, most likely between rooms and hallways. These intersections are
//			 wanted because they create separate segments for each room/hallway and don't put several together as one big. The
//			 drawing into the map copy is done by finding base points for each crf-node (two black pixels that are closest to this
//			 node) and drawing lines in the wanted color to both. Then a wavefront-region-growing is applied on the map-copy to
//			 fill the segments with one color, generating several rooms and hallways. In the last step the contours of the rooms
//			 and hallways are searched and drawn in the given map with a unique color into the map, if they are not too small or big.
void VoronoiRandomFieldSegmentation::segmentMap(const cv::Mat& original_map, cv::Mat& segmented_map, const int epsilon_for_neighborhood,
		const int max_iterations, const int min_neighborhood_size, std::vector<uint>& possible_labels,
		const double min_node_distance,  bool show_results, const std::string classifier_storage_path, const std::string classifier_default_path,
		const int max_inference_iterations, double map_resolution_from_subscription, double room_area_factor_lower_limit,
		double room_area_factor_upper_limit, double max_area_for_merging, std::vector<cv::Point>* door_points)
{
	// check if path for storing classifier models exists
	boost::filesystem::path storage_path(classifier_storage_path);
	if (boost::filesystem::exists(storage_path) == false)
	{
		if (boost::filesystem::create_directories(storage_path) == false && boost::filesystem::exists(storage_path) == false)
		{
			std::cout << "Error: VoronoiRandomFieldSegmentation::segmentMap: Could not create directory " << storage_path << std::endl;
			return;
		}
	}

	// save a copy of the original image
	cv::Mat original_image = original_map.clone();

	// if the training results haven't been loaded or trained before load them
	if(trained_boost_ == false)
	{
		// load the AdaBoost-classifiers
		std::string filename_room = classifier_storage_path + "vrf_room_boost.xml";
		std::string filename_room_default = classifier_default_path + "vrf_room_boost.xml";
		if (boost::filesystem::exists(boost::filesystem::path(filename_room)) == false)
			boost::filesystem::copy_file(filename_room_default, filename_room);
		loadBoost(room_boost_,filename_room);

		std::string filename_hallway = classifier_storage_path + "vrf_hallway_boost.xml";
		std::string filename_hallway_default = classifier_default_path + "vrf_hallway_boost.xml";
		if (boost::filesystem::exists(boost::filesystem::path(filename_hallway)) == false)
			boost::filesystem::copy_file(filename_hallway_default, filename_hallway);
		loadBoost(hallway_boost_,filename_hallway);

		std::string filename_doorway = classifier_storage_path + "vrf_doorway_boost.xml";
		std::string filename_doorway_default = classifier_default_path + "vrf_doorway_boost.xml";
		if (boost::filesystem::exists(boost::filesystem::path(filename_doorway)) == false)
			boost::filesystem::copy_file(filename_doorway_default, filename_doorway);
		loadBoost(doorway_boost_,filename_doorway);

		// set the trained-Boolean true to only load parameters once
		trained_boost_ = true;
	}

	if(trained_conditional_field_ == false)
	{
		// clear weights that might be standing from before
		trained_conditional_weights_.clear();

		// load the weights out of the file
		std::string filename_crf = classifier_storage_path + "vrf_conditional_field_weights.txt";
		std::string filename_crf_default = classifier_default_path + "vrf_conditional_field_weights.txt";
		if (boost::filesystem::exists(boost::filesystem::path(filename_crf)) == false)
			boost::filesystem::copy_file(filename_crf_default, filename_crf);
		std::ifstream input_file(filename_crf.c_str());
		std::string line;
		double value;
		if (input_file.is_open())
		{
			while (getline(input_file, line))
			{
				std::istringstream iss(line);
				while (iss >> value)
				{
					trained_conditional_weights_.push_back(value);
				}
			}
			input_file.close();
		}

		// set the trained-Boolean to true so the weights only get read in once
		trained_conditional_field_ = true;
	}

	// ************* I. Create the pruned generalized Voronoi graph *************
	cv::Mat voronoi_map = original_map.clone();

	std::set<cv::Point, cv_Point_comp> node_points; //variable for node point extraction

	// use the above defined function to create a pruned Voronoi graph
	std::cout << "creating voronoi graph" << std::endl;
	Timer timer; // variable to measure computation-time
	createPrunedVoronoiGraph(voronoi_map, node_points);
	std::cout << "created graph. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// ************* II. Extract the nodes used for the conditional random field *************
	//
	std::set<cv::Point, cv_Point_comp> conditional_field_nodes;

	// get the distance transformed map, which shows the distance of every white pixel to the closest zero-pixel
	cv::Mat distance_map; //distance-map of the original-map (used to check the distance of each point to nearest black pixel)
#if CV_MAJOR_VERSION<=3
	cv::distanceTransform(original_map, distance_map, CV_DIST_L2, 5);
#else
	cv::distanceTransform(original_map, distance_map, cv::DIST_L2, 5);
#endif
	cv::convertScaleAbs(distance_map, distance_map);

	// find all nodes for the conditional random field
	timer.start();
	findConditonalNodes(conditional_field_nodes, voronoi_map, distance_map, node_points, epsilon_for_neighborhood, max_iterations, min_neighborhood_size, min_node_distance);
	std::cout << "found all conditional field nodes. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// show the node points if wanted
	cv::Mat node_map = original_map.clone();
	if(show_results == true)
	{
#if CV_MAJOR_VERSION<=3
		cv::cvtColor(node_map, node_map, CV_GRAY2BGR);
#else
		cv::cvtColor(node_map, node_map, cv::COLOR_GRAY2BGR);
#endif
		for(std::set<cv::Point, cv_Point_comp>::iterator node = conditional_field_nodes.begin(); node != conditional_field_nodes.end(); ++node)
		{
#if CV_MAJOR_VERSION<=3
			cv::circle(node_map, *node, 0, cv::Scalar(250,0,0), CV_FILLED);
#else
			cv::circle(node_map, *node, 0, cv::Scalar(250,0,0), cv::FILLED);
#endif
		}

//		cv::imshow("nodes of the conditional random field", node_map);
//		cv::waitKey();
	}

	// ************* III. Construct the Conditional Random Field from the found nodes *************
	//
	// Go along the voronoi graph each point and find the 2 or 3 nearest neighbors and construct a clique out of them.
	// If enough neighbors are found or no new voronoi-nodes were found in the last step, the algorithm stops. If no new
	// Voronoi-nodes got found, the current node is a dead end and has only one neighbor.
	// This is done using the above defined function createConditionalField, so see this function for further information.
	cv::Mat neighbor_map = node_map.clone(); // map to show the resulting cliques if wanted

	std::vector<Clique> conditional_random_field_cliques; // vector to save the found cliques of the conditional random field

	// construct the conditional random field
	std::cout << "starting to create conditional field" << std::endl;
	timer.start();
	createConditionalField(voronoi_map, conditional_field_nodes, conditional_random_field_cliques, node_points, original_image);

	std::cout << "Created field. Time: " << timer.getElapsedTimeInMilliSec() << "ms. Number of cliques: " << conditional_random_field_cliques.size() << std::endl;

	// show the found cliques if wanted
	if(show_results == true)
	{
		for(size_t i = 0; i < conditional_random_field_cliques.size(); ++i)
		{
			int blue = rand() % 250;
			int green = rand() % 250;
			int red = rand() % 250;

			std::vector<cv::Point> clique_points = conditional_random_field_cliques[i].getMemberPoints();

			if(clique_points.size() > 0)
			{
				for(size_t p = 0; p < clique_points.size(); ++p)
					for(size_t u = 0; u < clique_points.size(); ++u)
						if(u != p)
							cv::line(neighbor_map, clique_points[p], clique_points[u], cv::Scalar(blue, green, red), 1);
			}

//			cv::imshow("neighbors", neighbor_map);
//			cv::waitKey();
		}
	}

	// ************* IV. Construct the Factor graph from the calculated random-Field *************
	//
	// The calculated CRF has to be a Factor graph to use the OpenGM libraries Belief-Propagation algorithm. A Factor graph is a
	// graphical model that calculates a large function by calculating each part of the function for itself (https://en.wikipedia.org/wiki/Factor_graph).
	// Meaning it is a function in the form f(x_0:i) = f(x_j:k) * f(x_l:m) *... . In this case each clique-potential is one part
	// of the whole function and the overall function should be minimized later. To do this the Typedefs in the header gets used.

	// 1. Create the Label-Space and a factor graph. A Label-Space consists of all variables and how many labels each variable
	//	  can obtain.
	LabelSpace space(conditional_field_nodes.size(), number_of_classes_);

	FactorGraph factor_graph(space);

	// 2. Create each part of the factor graph. Each Clique-potential is one part in the factor graph, so for each clique
	// 	  a opengm-function-object gets calculated. The opengm::ExplicitFunction<double> template gets used, meaning for each
	//	  possible configuration of the variable-labels the double-value of the function needs to be put in the object. So each
	//	  object is like a lookup-table later on. The variables for a function are defined later, when the factors for the graph
	//	  are defined. The factor stores the indices of the CRF-nodes as they are stored in the set containing all nodes. Because
	//	  of this the indices need to be sorted by size, meaning e.g. 1 needs to be before 4. So the functions also need to be
	//	  sorted by this, to ensure that the values of it are assigned to the right variable-combination.

	// vector that stores the possible label configurations for a clique, meaning e.g. if one clique has three members with
	// two possible labels for each, it stores a vector that stores the label-configurations {(0,0,0), (0,0,1), (0,1,0), ...}.
	// It has a size of 4, because in this algorithm only cliques with 2-5 members are possible.
	std::vector<std::vector<std::vector<uint> > > label_configurations(5);

	// vector that stores the index for each possible label as element
	// 		--> to get the order as index list, so it can be used to assign the value of functions
	std::vector<uint> label_indices(possible_labels.size());
	for(uint index = 0; index < possible_labels.size(); ++index)
		label_indices[index] = index;

	timer.start();

	for(uint size = 1; size <= 5; ++size)
	{
		// vector that stores all possible configurations for one member-size
		std::vector<std::vector<uint> > possible_configurations;

		// use the above defined function to find all possible configurations for the possible labels and save them in the map
		getPossibleConfigurations(possible_configurations, label_indices, size);
		label_configurations[size-1] = possible_configurations;
	}

	std::cout << "Created all possible label-configurations. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	timer.start();
	// Go trough each clique and define the function and factor for it.
	for(std::vector<Clique>::iterator current_clique = conditional_random_field_cliques.begin(); current_clique != conditional_random_field_cliques.end(); ++current_clique)
	{
		// get the number of members in this clique and depending on this the possible label configurations defined above
		size_t number_of_members = current_clique->getNumberOfMembers();

		std::vector<std::vector<uint> > current_possible_configurations = label_configurations[number_of_members-1]; // -1 because this vector stores configurations for cliques with 1-5 members (others are not possible in this case).

		// find the real labels and assign them into the current configuration so the feature-vector gets calculated correctly
		for(size_t configuration = 0; configuration < current_possible_configurations.size(); ++configuration)
			for(size_t variable = 0; variable < current_possible_configurations[configuration].size(); ++variable)
				current_possible_configurations[configuration][variable] = possible_labels[current_possible_configurations[configuration][variable]];

		// define an array that has as many elements as the clique has members and assign the number of possible labels for each
		size_t variable_space[number_of_members];
		std::fill_n(variable_space, number_of_members, number_of_classes_);

		// define a explicit function-object from OpenGM containing the initial value -1.0 for each combination
		opengm::ExplicitFunction<double> f(variable_space, variable_space + number_of_members, -1.0);

		// go trough all points of the clique and find the index of it in the vector the nodes of the CRF are stored in
		//  --> necessary to sort the nodes correctly
		size_t indices[current_clique->getNumberOfMembers()]; // array the indices are stored in
		std::vector<cv::Point> clique_points = current_clique->getMemberPoints();
		for(size_t point = 0; point < clique_points.size(); ++point)
		{
			// get the iterator to the element and use the std::distance function to calculate the index of the point
			std::set<cv::Point>::iterator iterator = conditional_field_nodes.find(clique_points[point]);

			if(iterator != conditional_field_nodes.end()) // check if element was found --> should be
				indices[point] = std::distance(conditional_field_nodes.begin(), iterator);
			else
				std::cout << "element not in set" << std::endl;
		}

		// get the possible configurations and swap them, respecting the indices, then sort the indices themself
		std::vector<std::vector<uint> > swap_configurations = label_configurations[number_of_members-1]; // -1 because this vector stores configurations for cliques with 1-5 members (others are not possible in this case).
		swapConfigsRegardingNodeIndices(swap_configurations, indices);
		std::sort(indices, indices + current_clique->getNumberOfMembers());

		// Go trough each possible configuration and compute the function value for it. Use the original configuration, because
		// the nodes are stored in this way, but later the value is assigned in the position using the swaped configurations.
		for(size_t configuration = 0; configuration < current_possible_configurations.size(); ++configuration)
		{
			std::vector<uint> current_configuration = current_possible_configurations[configuration];

			// get current feature-vector and multiply it with the trained weights
			std::vector<double> current_features(number_of_classifiers_);
			getAdaBoostFeatureVector(current_features, *current_clique, current_configuration, possible_labels);

			double clique_potential = 0;
			for(size_t weight = 0; weight < number_of_classifiers_; ++weight)
			{
				clique_potential += trained_conditional_weights_[weight] * current_features[weight];
			}

			// assign the calculated clique potential at the right position in the function --> !!Important: factors need the variables to be sorted
			//																								 as increasing index
			f(swap_configurations[configuration].begin()) = clique_potential;//std::exp(clique_potential);
		}

		// add the defined function to the model and catch the returned function-identifier to specify which variables
		// this function needs
		FactorGraph::FunctionIdentifier identifier = factor_graph.addFunction(f);

		// add the Factor to the graph, that represents which variables (and labels of each) are used for the above defined function
		factor_graph.addFactor(identifier, indices, indices+current_clique->getNumberOfMembers());

	}
	std::cout << "calculated all features for the cliques. Time: " << timer.getElapsedTimeInSec() << "s" << std::endl;

	// ************* V. Do inference in the defined factor-graph to find best labels. *************
	//
	// Do Inference in the above created graphical model using OpengM. This function has three control parameters:
	//		i. The maximum number of iterations done
	//		ii. The convergence Bound, which is used to check if the messages haven't changed a lot after the last step.
	//		iii. The damping-factor, which implies how many messages should be dumped, in this case 0.
	const double convergence_bound = 1e-7;
	const double damping_factor = 0.0;
	LoopyBeliefPropagation::Parameter parameters(max_inference_iterations, convergence_bound, damping_factor);

	// create LoopyBeliefPropagation object that does inference on the graphical model defined above
	LoopyBeliefPropagation belief_propagation(factor_graph, parameters);

	// do inference
	timer.start();
	belief_propagation.infer();
	std::cout << "Done Inference. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// obtain the labels that get the max value of the defined function
	std::vector<FactorGraph::LabelType> best_labels(conditional_field_nodes.size());
	belief_propagation.arg(best_labels);

	// print the solution if wanted
	if(show_results == true)
	{
		cv::Mat resulting_map = original_image.clone();

		for(std::set<cv::Point, cv_Point_comp>::iterator i = conditional_field_nodes.begin(); i != conditional_field_nodes.end(); ++i)
		{
			size_t distance = std::distance(conditional_field_nodes.begin(), i);
#if CV_MAJOR_VERSION<=3
			cv::circle(resulting_map, *i, 3, cv::Scalar(possible_labels[best_labels[distance]]), CV_FILLED);
#else
			cv::circle(resulting_map, *i, 3, cv::Scalar(possible_labels[best_labels[distance]]), cv::FILLED);
#endif
		}

		cv::imshow("node-map", resulting_map);
//		cv::waitKey();
	}
	std::cout << "complete Potential: " << belief_propagation.value() << std::endl;

	// for optimization purpose
//	return belief_propagation.value();

	// ************* VI. Search for different regions of same color and make them a individual segment *************
	//
	// 1. Connect the found nodes to the two nearest black pixels (base points) of them. Connect the nodes that are labeled
	//    as doorway with black lines to create intersections. This is done because it gives good results, when finding
	//	  segments labeled only as one class, because later on this map with the base-lines a wavefront-region-growing is
	//	  applied.

	// erode the map to close small gaps and remove errors --> also done when producing the voronoi-graph.
	cv::Mat map_copy, eroded_map;
	cv::Point anchor(-1, -1);
	cv::erode(original_image, eroded_map, cv::Mat(), anchor, 2);
	map_copy = eroded_map.clone();

	// find the layout of the map and discretize it to get possible base points
	std::vector < std::vector<cv::Point> > map_contours;
	std::vector < cv::Vec4i > hierarchy;
#if CV_MAJOR_VERSION<=3
	cv::findContours(map_copy, map_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
#else
	cv::findContours(map_copy, map_contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
#endif

	timer.start();

	// reassign the map because findContours destroys it and erode it to close small errors
	map_copy = eroded_map.clone();

	// go trough all crf-nodes
	for(std::set<cv::Point, cv_Point_comp>::iterator node = conditional_field_nodes.begin(); node != conditional_field_nodes.end(); ++node)
	{
		// find index of point
		size_t distance = std::distance(conditional_field_nodes.begin(), node);

		// set initial points and values for the basis points so the distance comparison can be done
		cv::Point basis_point_1 = map_contours[0][0];
		cv::Point basis_point_2 = map_contours[0][1];

		// initial values of the first vector from the current critical point to the contour points and for the squared distance of it
		double vector_x_1 = node->x - map_contours[0][0].x;
		double vector_y_1 = node->y - map_contours[0][0].y;
		double distance_basis_square_1 = vector_x_1*vector_x_1 + vector_y_1*vector_y_1;

		// initial values of the second vector from the current critical point to the contour points and for the squared distance of it
		double vector_x_2 = node->x - map_contours[0][1].x;
		double vector_y_2 = node->y - map_contours[0][1].y;
		double distance_basis_square_2 = vector_x_2*vector_x_2 + vector_y_2*vector_y_2;

		// find first basis point
		int basis_vector_1_x, basis_vector_2_x, basis_vector_1_y, basis_vector_2_y;
		for (int c = 0; c < map_contours.size(); c++)
		{
			for (int p = 0; p < map_contours[c].size(); p++)
			{
				// calculate the squared Euclidian distance from the critical Point to the Point on the contour
				const double vector_x = map_contours[c][p].x - node->x;
				const double vector_y = map_contours[c][p].y - node->y;
				const double current_distance = vector_x*vector_x + vector_y*vector_y;

				// compare the distance to the saved distances if it is smaller
				if (current_distance < distance_basis_square_1)
				{
					distance_basis_square_1 = current_distance;
					basis_point_1 = map_contours[c][p];
					basis_vector_1_x = vector_x;
					basis_vector_1_y = vector_y;
				}
			}
		}
		// find second basisPpoint
		for (int c = 0; c < map_contours.size(); c++)
		{
			for (int p = 0; p < map_contours[c].size(); p++)
			{
				// calculate the squared Euclidian distance from the critical point to the point on the contour
				const double vector_x = map_contours[c][p].x - node->x;
				const double vector_y = map_contours[c][p].y - node->y;
				const double current_distance = vector_x*vector_x + vector_y*vector_y;

				// calculate the distance between the current contour point and the first basis point to make sure they
				// are not too close to each other
				const double vector_x_basis = basis_point_1.x - map_contours[c][p].x;
				const double vector_y_basis = basis_point_1.y - map_contours[c][p].y;
				const double basis_distance = vector_x_basis*vector_x_basis + vector_y_basis*vector_y_basis;
				if (current_distance > distance_basis_square_1 && current_distance < distance_basis_square_2 &&
					basis_distance > (double) distance_map.at<unsigned char>(*node)*distance_map.at<unsigned char>(*node))
				{
					distance_basis_square_2 = current_distance;
					basis_point_2 = map_contours[c][p];
					basis_vector_2_x = vector_x;
					basis_vector_2_y = vector_y;
				}
			}
		}

		// if the node is labeled as doorway draw the base-lines black --> as intersection
		if(best_labels[distance] == 2)
		{
			// draw a line from the node to the two basis points
			cv::line(map_copy, *node, basis_point_1, 0, 2);
			cv::line(map_copy, *node, basis_point_2, 0, 2);

			// return the door-points if wanted
			if(door_points != NULL)
				door_points->push_back(*node);
		}
		else
		{
			// draw a line from the node to the two basis points
			cv::line(map_copy, *node, basis_point_1, possible_labels[best_labels[distance]], 1);
			cv::line(map_copy, *node, basis_point_2, possible_labels[best_labels[distance]], 1);
		}
	}

	std::cout << "drawn all segments. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// 2. Apply a wavefront algorithm to the map copy to fill all segments with the labels that are given to the crf-nodes.
	map_copy.convertTo(map_copy, CV_32SC1, 256, 0);
	wavefrontRegionGrowing(map_copy);

	if(show_results == true)
	{
		cv::imshow("intersected map", map_copy);
//		cv::waitKey();
	}

	// 3. search for points where the map copy is still white, because these points could get colored wrong
	// 	  by too large contours
	std::vector<cv::Point> white_points;
	for(unsigned int u = 0; u < map_copy.rows; ++u)
		for(unsigned int v = 0; v < map_copy.cols; ++v)
			if(map_copy.at<int>(u, v) == 255*256)
				white_points.push_back(cv::Point(v, u));

	// 3. Make everything black except the found segments of rooms/hallways, that become totally white. This is done because
	//	  it makes possible to find the contours of the segments and draw them into the original map with a random color.
	//	  Only rooms or hallways stay at one step to ensure that borders from hallways to rooms are recognized.
	std::set<std::vector<cv::Point>, compContoursSize> segments; // variable to save found contours

	timer.start();
	for(int color = 0; color <= 1; ++color)
	{
		// clear previously used variables for contour extraction
		map_contours.clear();
		hierarchy.clear();

		// get the color that should get white --> label given as 8bit color, but it has changed to a 32bit color
		int current_color = possible_labels[color] * 256;

		// create a map_copy
		cv::Mat temporary_map = cv::Mat(map_copy.rows, map_copy.cols, original_image.type());

		// make regions black and white
		for(unsigned int u = 0; u < map_copy.rows; ++u)
		{
			for(unsigned int v = 0; v < map_copy.cols; ++v)
			{
				// check if color is found and pixel hasn't been drawn accidentally in a color
				if(map_copy.at<int>(u, v) == current_color && original_image.at<unsigned char>(u, v) != 0)
					temporary_map.at<unsigned char>(u, v) = 255;
				else
					temporary_map.at<unsigned char>(u, v) = 0;
			}
		}
		// find the contours of the rooms/hallways
#if CV_MAJOR_VERSION<=3
		cv::findContours(temporary_map, map_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
#else
		cv::findContours(temporary_map, map_contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
#endif

		// save the contours that are not holes (check with hierarchy --> [{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour})
		//  --> save everything with hierarchy[3] == -1
		for(size_t contour = 0; contour < map_contours.size(); ++contour)
			if(hierarchy[contour][3] == -1 && map_contours.size() > 1)
				segments.insert(map_contours[contour]);
	}

	std::cout << "found segments: " << segments.size() << ". Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// 4. Draw the found contours with a unique color and apply a wavefront-region-growing algorithm to get rid of remaining
	//	  white spaces. Also save the found segments as rooms to merge rooms together in the next step.
	timer.start();
	original_map.convertTo(segmented_map, CV_32SC1, 256, 0); // convert input image to CV_32SC1 (needed for wavefront and to have enoguh possible rooms)

	std::vector < cv::Scalar > already_used_colors; //saving-vector to save the already used colors

	std::vector<Room> rooms; // vector to save the rooms in this map

	for(std::set<std::vector<cv::Point> >::iterator current_contour = segments.begin(); current_contour != segments.end(); ++current_contour)
	{
		// calculate area for the contour and check if it is large enough to be a separate segment
		double room_area = map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(*current_contour);
		if (room_area >= room_area_factor_lower_limit && room_area <= room_area_factor_upper_limit)
		{
			// Draw the region with a random color into the map if it is large/small enough
			bool drawn = false;
			int loop_counter = 0; //counter if the loop gets into a endless loop
			do
			{
				loop_counter++;
				int random_number = rand() % 52224 + 13056;
				cv::Scalar fill_color(random_number);

				//check if color has already been used
				if (!contains(already_used_colors, fill_color) || loop_counter > 1000)
				{
#if CV_MAJOR_VERSION<=3
					cv::drawContours(segmented_map, std::vector<std::vector<cv::Point> >(1,*current_contour), -1, fill_color, CV_FILLED);
#else
					cv::drawContours(segmented_map, std::vector<std::vector<cv::Point> >(1,*current_contour), -1, fill_color, cv::FILLED);
#endif
					already_used_colors.push_back(fill_color);
					Room current_room(random_number); //add the current Contour as a room
					for (int point = 0; point < current_contour->size(); point++) //add contour points to room
					{
						current_room.insertMemberPoint(cv::Point(current_contour->at(point)), map_resolution_from_subscription);
					}
					rooms.push_back(current_room);
					drawn = true;
				}
			} while (!drawn);
		}
		// draw too small segments white, to prevent that they are covered by large closed contours
		if(room_area < room_area_factor_lower_limit)
#if CV_MAJOR_VERSION<=3
			cv::drawContours(segmented_map, std::vector<std::vector<cv::Point> >(1,*current_contour), -1, 255*256, CV_FILLED);
#else
			cv::drawContours(segmented_map, std::vector<std::vector<cv::Point> >(1,*current_contour), -1, 255*256, cv::FILLED);
#endif
	}

	// Make black what has been black before (drawContours draws filled areas and might overwrite black holes).
	// Also make regions that are black on the eroded map but white on the original map also white to prevent large closed contours
	// to go around small contours in them. Also make the intersections in got by connecting the voronoi nodes with their
	// base points for doorways white for the same reason.
	for(unsigned int u = 0; u < segmented_map.rows; ++u)
	{
		for(unsigned int v = 0; v < segmented_map.cols; ++v)
		{
			if(original_image.at<unsigned char>(u, v) == 0 && eroded_map.at<unsigned char>(u, v) == 0)
			{
				segmented_map.at<int>(u, v) = 0;
			}

			if(original_image.at<unsigned char>(u, v) == 255 && eroded_map.at<unsigned char>(u, v) == 0)
			{
				segmented_map.at<int>(u, v) = 255*256;
			}
			if(map_copy.at<int>(u, v) == 0 && original_image.at<unsigned char>(u, v) == 255)
			{
				segmented_map.at<int>(u, v) = 255*256;
			}
		}
	}

	// go trough the saved white points and make them white in the segmented map again
	for(std::vector<cv::Point>::iterator point = white_points.begin(); point != white_points.end(); ++point)
		segmented_map.at<int>(*point) = 255*256;

	// color remaining white space
	wavefrontRegionGrowing(segmented_map);

	std::cout << "filled map with unique colors. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// 5. Merge rooms that are too small together with the surrounding big rooms. This is done because it is possible to
	//	  create very small segments in the last step, tha don't make very much sense.
	timer.start();

	if(show_results == true)
		cv::imshow("before merge", segmented_map);

	mergeRooms(segmented_map, rooms, map_resolution_from_subscription, max_area_for_merging, false);

	std::cout << "merged rooms together. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	if(show_results == true)
	{
		cv::imshow("segmented map", segmented_map);
		cv::waitKey();
	}
}

// Function to test several functions of this algorithm independent of other functions
// this current implementation is used for optimizing the trained algorithm. The idea behind this is: I train the algorithm
// with the same training-maps several times and calculate the crf-graph-potential for several maps and add them together. If
// then the current sum is better than a saved best sum, the trained parameters are saved, what is done with this function.
// It is good to do this, because OpenCV uses a Decision-Tree for the AdaBoost classifiers, which is depending on probabilites
// and so every training done creates different results.
void VoronoiRandomFieldSegmentation::testFunc(const cv::Mat& original_map)
{
	std::cout << "testfunc" << std::endl;

//	// if the training results haven't been loaded or trained before load them
//	std::string filename_room = boost_storage_path + "vrf_room_boost.xml";
//	std::string filename_hallway = boost_storage_path + "vrf_hallway_boost.xml";
//	std::string filename_doorway = boost_storage_path + "vrf_doorway_boost.xml";
//
//	room_boost_.save(filename_room.c_str(), "boost");
//	hallway_boost_.save(filename_hallway.c_str(), "boost");
//	doorway_boost_.save(filename_doorway.c_str(), "boost");
//
//	std::ofstream output_file(crf_storage_path.c_str(), std::ios::out);
//	if (output_file.is_open()==true)
//	{
//		for(size_t weight = 0; weight < trained_conditional_weights_.size(); ++ weight)
//		{
//			output_file << trained_conditional_weights_[weight] << std::endl;
//		}
//	}
//	output_file.close();
//
//
//	if(trained_boost_ == false)
//	{
//		// load the AdaBoost-classifiers
//		room_boost_.load(filename_room.c_str());
//		hallway_boost_.load(filename_hallway.c_str());
//		doorway_boost_.load(filename_doorway.c_str());
//
//		// set the trained-Boolean true to only load parameters once
//		trained_boost_ = true;
//	}
//
//	if(trained_conditional_field_ == false)
//	{
//		// load the weights out of the file
//		std::ifstream input_file(crf_storage_path.c_str());
//		std::string line;
//		double value;
//		if (input_file.is_open())
//		{
//			while (getline(input_file, line))
//			{
//				std::istringstream iss(line);
//				while (iss >> value)
//				{
//					trained_conditional_weights_.push_back(value);
//				}
//			}
//			input_file.close();
//		}
//
//		// set the trained-Boolean to true so the weights only get read in once
//		trained_conditional_field_ = true;
//	}
//
//	std::cout << "reading weights: " << std::endl;
//
//	cv::Mat featuresMat(1, getFeatureCount(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
//	for (int f = 1; f <= getFeatureCount(); ++f)
//	{
//		//get the features for each room and put it in the featuresMat
//		featuresMat.at<float>(0, f - 1) = (float) 1;
//	}
//
//	// Calculate the weak hypothesis by using the wanted classifier.
//	CvMat features = featuresMat;
//	cv::Mat weaker (1, number_of_classifiers_, CV_32F);
//	CvMat weak_hypothesis = weaker;	// Wanted from OpenCV to get the weak hypothesis from the
//									// separate weak classifiers.
//
//	// For each point the classifier depends on the given label. If the point is labeled as a room the room-boost should be
//	// used and so on.
//	room_boost_.predict(&features, 0, &weak_hypothesis);
//
//	for(size_t f = 0; f < number_of_classifiers_; ++f)
//	{
//		std::cout << (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f) << std::endl;
//	}
//
//	std::cout << "cols: " << weights.cols << " rows: " << weights.rows << std::endl;
//
//	for(size_t i = 0; i < weights->cols; ++i)
//		for(size_t j = 0; j < weights->rows; ++j)
//			std::cout << (double) CV_MAT_ELEM(*weights, float, i, j) << std::endl;
//
//	std::cout << "loaded files" << std::endl;
}


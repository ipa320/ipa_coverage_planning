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

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h> // header

#include <ipa_room_segmentation/timer.h>

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

	pseudoLikelihoodOptimization()
	{
		number_of_weights = 0;
		sigma = 1.;
	}

	double operator()(const column_vector& weights) const
	{
		double result = 0;
		// go trough each part of the function and calculate the log(.) and add it to the result
		for(unsigned int function_part = 0; function_part < log_parameters.size(); ++function_part)
		{
			double splitting_of_exponent = 10; // because this function often produces exponents around 1000 the calculation of one exponential part needs to be splitted
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
//			if(((log_numerator / log_denominator) == (log_numerator / log_denominator)) == false)
//				std::cout << "quotient: " << (log_numerator / log_denominator) << " numerator: " << log_numerator << " denominator: " << log_denominator <<  std::endl;
			result -= log10(log_numerator / log_denominator);
//			if((result == result) == false)
//				std::cout << "quotient: " << (log_numerator / log_denominator) << std::endl;
		}
//		if((result == result) == false)
//			std::cout << "first result: " << result << std::endl;

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

// Struct that is used as function for the factor graph in OpenGM. It does the same as getAdaBoostClassifiers, but it is better
// to give OpenGM an object that computes all function-values than calculating them before and then saving them.
//struct adaBoostFunction
//{
//public:
//	// the number of classifiers that produce a single feature-value
//	size_t number_of_classifiers;
//
//	// OpenCV Boost objects that are used to calculate the AdaBoost weak-hypothesis
//	CvBoost room_boost, hallway_boost, doorway_boost;
//
//	// vector that stores the angles used for raycasting --> to compute features
//	std::vector<double> angles_for_simulation;
//
//	// vector that stores the weights of the conditional random field, which are used together with the features to get a
//	// clique potential
//	std::vector<double> trained_conditional_weights;
//
//	// vector that stores each label that is possible to assign to a node
//	std::vector<unsigned int> possible_labels;
//
//	// the clique this function calculates a value for
//	Clique clique;
////	std::vector<cv::Point> clique_members;
////	std::vector< std::vector<double> > beams_for_points;
//
//	// functions needed from OpenGM
//	// how many variables there are in this function (possible labels in this case)
////	size_t dimension()const
////	{
////		return possible_labels.size();
////	};
////
////	size_t shape(const size_t i)const
////	{
////		return 2;
////	};
////
////	size_t size()const
////	{
////		return 16;
////	};
//
//	template<class Iterator>
//	inline const double operator()(Iterator input)
//	{
//		double value = 0;
//
//		// Get the points that belong to the clique and the stored simulated beams for each one.
//		std::vector<cv::Point> clique_members = clique.getMemberPoints();
//		std::vector< std::vector<double> > beams_for_points = clique.getBeams();
//
//		// vector that is used to sum the calculated features
//		std::vector<double> temporary_feature_vector(number_of_classifiers, 0.0);
//
//		// For each member of this clique calculate the weak-hypothesis and add the resulting vectors in the end
//		for(size_t point = 0; point < clique_members.size(); ++point)
//		{
//			// Check which classifier (room, hallway or doorway) needs to be used.
//			unsigned int classifier;
//			for(size_t label = 0; label < possible_labels.size(); ++label)
//			{
//				if(possible_labels[label] == input[point])
//				{
//					classifier = label;
//					break;
//				}
//			}
//
//	//		std::cout << "got label to use" << std::endl;
//
//			// get the features for the central point of the clique
//			cv::Mat featuresMat(1, getFeatureCount(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
//			for (int f = 1; f <= getFeatureCount(); ++f)
//			{
//				//get the features for each room and put it in the featuresMat
//				featuresMat.at<float>(0, f - 1) = (float) getFeature(beams_for_points[point], angles_for_simulation, clique_members, input, possible_labels, clique_members[point], f);
////				if(std::count(input.begin(), input.end(), 77) == clique_members.size())
////					std::cout << "filled entry " << f << ": " << featuresMat.at<float>(0, f - 1) << std::endl;
//			}
//			// Calculate the weak hypothesis by using the wanted classifier.
//			CvMat features = featuresMat;
//			cv::Mat weaker (1, number_of_classifiers, CV_32F);
//			CvMat weak_hypothesis = weaker;	// Wanted from OpenCV to get the weak hypothesis from the
//												// separate weak classifiers.
//
//			switch(classifier)
//			{
//			case 0:
////				std::cout << "using rooms" << std::endl;
//				room_boost.predict(&features, 0, &weak_hypothesis);
//				break;
//			case 1:
////				std::cout << "using hallways" << std::endl;
//				hallway_boost.predict(&features, 0, &weak_hypothesis);
//				break;
//			case 2:
////				std::cout << "using doorways" << std::endl;
//				doorway_boost.predict(&features, 0, &weak_hypothesis);
//				break;
//			}
//
////			std::cout << "predicted" << std::endl;
//
//			// Write the weak hypothesis in the feature vector.
////			std::cout << "resaving weak responses " << std::endl;
//			for(size_t f = 0; f < number_of_classifiers; ++f)
//			{
////				std::cout << "f: " << f;
//				temporary_feature_vector[f] = temporary_feature_vector[f] + (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
////				std::cout << ". assigned hypothesis" << std::endl;
//			}
//
////			std::cout << "assigned weak hypothesis" << std::endl;
//		}
//
//		// calculate the weighted sum of features and weights
//		for(size_t weight = 0; weight < number_of_classifiers; ++weight)
//			value += temporary_feature_vector[weight] * trained_conditional_weights[weight];
//
//		return value;
//	};
//};

// Struct that is used to sort the label configurations in a way s.t. OpenGM can use it properly when defining functions and factors.
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
		return a.absolute_index < b.absolute_index;
	}
};


// Constructor
VoronoiRandomFieldSegmentation::VoronoiRandomFieldSegmentation(bool trained_boost, bool trained_conditional_field)
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
	CvBoostParams params(CvBoost::DISCRETE, number_of_classifiers_, 0, 2, false, 0);
	params_ = params;
	trained_boost_ = trained_boost;
	trained_conditional_field_ = trained_conditional_field;
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
//
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
// *********** Function to draw Voronoi graph ****************
//
// This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
// map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
// inside a hole-contour and doesn't draw the lines that are not.
//
void VoronoiRandomFieldSegmentation::drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color,
		//const std::vector<cv::Point>& contour, const std::vector<std::vector<cv::Point> >& hole_contours)
		const cv::Mat& eroded_map)
{
	//This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
	//map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
	//inside a hole-contour and doesn't draw the lines that are not.
	for (std::vector<std::vector<cv::Point2f> >::const_iterator current_contour = facets_of_voronoi.begin(); current_contour != facets_of_voronoi.end(); ++current_contour)
	{
		// saving-variable for the last Point that has been looked at
		cv::Point2f last_point = current_contour->back();
		// draw each line of the voronoi-cell
		for (size_t c = 0; c < current_contour->size(); ++c)
		{
			// variable to check, whether a Point is inside a white area or not
			bool inside = true;
			cv::Point2f current_point = current_contour->at(c);
			// only draw lines that are inside the map-contour
			if (((int)current_point.x<0) || ((int)current_point.x >= eroded_map.cols) ||
				((int)current_point.y<0) || ((int)current_point.y >= eroded_map.rows) ||
				eroded_map.at<uchar>((int)current_point.y, (int)current_point.x) == 0 ||
				((int)last_point.x<0) || ((int)last_point.x >= eroded_map.cols) ||
				((int)last_point.y<0) || ((int)last_point.y >= eroded_map.rows) ||
				eroded_map.at<uchar>((int)last_point.y, (int)last_point.x) == 0)
				inside = false;
//			if (cv::pointPolygonTest(contour, current_point, false) < 0 || cv::pointPolygonTest(contour, last_point, false) < 0)
//			{
//				inside = false;
//			}
//			// only draw Points inside the contour that are not inside a hole-contour
//			for (std::vector<std::vector<cv::Point> >::const_iterator hole = hole_contours.begin(); hole != hole_contours.end(); ++hole)
//			{
//				if (cv::pointPolygonTest(*hole, current_point, false) >= 0 || cv::pointPolygonTest(*hole, last_point, false) >= 0)
//				{
//					inside = false;
//				}
//			}
			if (inside)
			{
				cv::line(img, last_point, current_point, voronoi_color, 1);
			}
			last_point = current_point;
		}
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
//		   because the beams only need to be computed once for each clique and doing this at this point saves time later.
void VoronoiRandomFieldSegmentation::createConditionalField(const cv::Mat& voronoi_map, const std::set<cv::Point, cv_Point_comp>& node_points,
		std::vector<Clique>& conditional_random_field_cliques, const std::set<cv::Point, cv_Point_comp>& voronoi_node_points,
		const cv::Mat& original_map)
{
	// 1. Search for the n neighbors of the each point by going along the voronoi graph until a conditional-field-node gets
	//	  found.
	//
	std::map<cv::Point, std::vector<double>, cv_Point_comp > raycasts; // map that stores a simulated raycast for given OpenCV Points --> some points would get raycasted several times, this saves computation-time
//	double time = 0; // double to compute the total time needed to raycast all points
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

		// vector that stores the points that should get expanded next
		std::vector<cv::Point> to_be_expanded_points;
		to_be_expanded_points.push_back(*current_point);

		// go along the Voronoi Graph starting from the current node and find the nearest conditional random field nodes
		do
		{
			// save the size of the searched-points vector
			previous_size_of_searched_nodes = searched_points.size();

//			std::cout << "starting new iteration. Points to find: " << number_of_neighbors - found_neighbors.size() << std::endl;

			// create temporary-vector to save the new found nodes
			std::vector<cv::Point> temporary_point_vector = searched_points;

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
							if(du == 0 && dv == 0)
								continue;

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

//			cv::Mat path_map = node_map.clone();
//			for(size_t i = 0; i < searched_point_vector.size(); ++i)
//				cv::circle(path_map, searched_point_vector[i], 0, cv::Scalar(0,0,250), CV_FILLED);
////				path_map.at<unsigned char>(searched_point_vector[i]) = 0;
//			cv::resize(path_map, path_map, cv::Size(), 1.5, 1.5, cv::INTER_LINEAR);
//			cv::imshow("path map", path_map);
//			cv::waitKey();

		}while(found_neighbors.size() < number_of_neighbors && previous_size_of_searched_nodes != searched_points.size());

//		std::cout << "Found all neighbors. Size of found points: " << found_neighbors.size() << " Size of searched points: " << searched_points.size() << std::endl;

		// 2. create a clique out of the current node and its found neighbors
		//
		conditional_random_field_cliques.push_back(Clique(*current_point));
		std::vector<cv::Point> neighbor_vector(found_neighbors.begin(), found_neighbors.end()); // convert set to a vector to easily insert the new members
		conditional_random_field_cliques.back().insertMember(neighbor_vector);

		// 3. Simulate the laser-beams at each found member and store them.
		std::vector<cv::Point> clique_members = conditional_random_field_cliques.back().getMemberPoints();
		std::vector< std::vector<double> > laser_beams(clique_members.size());

//		std::cout << "starting to simulate beams" << std::endl;
//		Timer tim;

		for(size_t member = 0; member < clique_members.size(); ++member)
		{
//			if(raycasts.find(clique_members[member]) == raycasts.end()) // point hasn't been simulated yet
//			{
				laser_beams[member] = raycasting(original_map, cv::Point(clique_members[member].y, clique_members[member].x));
//				raycasts[clique_members[member]] = laser_beams[member];
//			}
//			else
//				laser_beams[member] = raycasts[clique_members[member]];
		}
//		time += tim.getElapsedTimeInMilliSec();

//		std::cout << "finished to simulate. Time: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl << std::endl;

		conditional_random_field_cliques.back().setBeamsForMembers(laser_beams);
	}
//	std::cout << "Raycasting-time: " << time << "ms" << std::endl;
}

//void trainBoostClassifiers_Obsolete(std::vector<cv::Mat>& room_training_maps, std::vector<cv::Mat>& hallway_training_maps,
//		std::vector<cv::Mat>& doorway_training_maps, const std::string& classifier_storage_path)
//{
//	std::vector<float> labels_for_rooms, labels_for_hallways, labels_for_doorways;
//	std::vector< std::vector<float> > room_features, hallway_features, doorway_features;
//	std::vector<double> temporary_beams;
//	std::vector<float> temporary_features;
//	std::vector<cv::Point> clique_points;
//	std::cout << "Starting to train the boost algorithm." << std::endl;
//	std::cout << "number of room training maps: " << room_training_maps.size() << std::endl;
//	//
//	// Train the room classifiers
//	//
//	//Get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it doesn't
//	for(size_t map = 0; map < room_training_maps.size(); ++map)
//	{
//		for (int y = 0; y < room_training_maps[map].cols; y++)
//		{
//			for (int x = 0; x < room_training_maps[map].rows; x++)
//			{
//				if (room_training_maps[map].at<unsigned char>(x, y) != 0)
//				{
//					// check for label of each Pixel (if it belongs to rooms the label is 1, otherwise it is -1)
//					if (room_training_maps[map].at<unsigned char>(x, y) < 127)
//					{
//						labels_for_rooms.push_back(1.0);
//					}
//					else if(room_training_maps[map].at<unsigned char>(x, y) > 127 && room_training_maps[map].at<unsigned char>(x, y) != 255)
//					{
//						labels_for_rooms.push_back(-1.0);
//					}
//					//simulate the beams and features for every position and save it
//					temporary_beams = raycasting(room_training_maps[map], cv::Point(x, y));
//					for (int f = 1; f <= getFeatureCount(); f++)
//					{
//						temporary_features.push_back((float) getFeature(temporary_beams, angles_for_simulation_, clique_points, cv::Point(x, y), f));
//					}
//					room_features.push_back(temporary_features);
//					temporary_features.clear();
//				}
//			}
//		}
//		std::cout << "extracted features from one room map" << std::endl;
//	}
//	//
//	// Train the hallway classifiers
//	//
//	//Get the labels for every training point. 1.0 means it belongs to a hallway and -1.0 means it doesn't
//	for(size_t map = 0; map < hallway_training_maps.size(); ++map)
//	{
//		for (int y = 0; y < hallway_training_maps[map].cols; y++)
//		{
//			for (int x = 0; x < hallway_training_maps[map].rows; x++)
//			{
//				if (hallway_training_maps[map].at<unsigned char>(x, y) != 0)
//				{
//					// check for label of each Pixel (if it belongs to hallways the label is 1, otherwise it is -1)
//					if (hallway_training_maps[map].at<unsigned char>(x, y) < 127)
//					{
//						labels_for_hallways.push_back(1.0);
//					}
//					else if(hallway_training_maps[map].at<unsigned char>(x, y) > 127 && hallway_training_maps[map].at<unsigned char>(x, y) != 255)
//					{
//						labels_for_hallways.push_back(-1.0);
//					}
//					//simulate the beams and features for every position and save it
//					temporary_beams = raycasting(hallway_training_maps[map], cv::Point(x, y));
//					for (int f = 1; f <= getFeatureCount(); f++)
//					{
//						temporary_features.push_back((float) getFeature(temporary_beams, angles_for_simulation_, clique_points, cv::Point(x, y), f));
//					}
//					hallway_features.push_back(temporary_features);
//					temporary_features.clear();
//				}
//			}
//		}
//		std::cout << "extracted features from one hallway map" << std::endl;
//	}
//	//
//	// Train the doorway classifiers
//	//
//	//Get the labels for every training point. 1.0 means it belongs to a doorway and -1.0 means it doesn't
//	for(size_t map = 0; map < doorway_training_maps.size(); ++map)
//	{
//		for (int y = 0; y < doorway_training_maps[map].cols; y++)
//		{
//			for (int x = 0; x < doorway_training_maps[map].rows; x++)
//			{
//				if (doorway_training_maps[map].at<unsigned char>(x, y) != 0)
//				{
//					// check for label of each Pixel (if it belongs to doorway the label is 1, otherwise it is -1)
//					if (doorway_training_maps[map].at<unsigned char>(x, y) < 127)
//					{
//						labels_for_doorways.push_back(1.0);
//					}
//					else if(doorway_training_maps[map].at<unsigned char>(x, y) > 127 && doorway_training_maps[map].at<unsigned char>(x, y) != 255)
//					{
//						labels_for_doorways.push_back(-1.0);
//					}
//					//simulate the beams and features for every position and save it
//					temporary_beams = raycasting(doorway_training_maps[map], cv::Point(x, y));
//					for (int f = 1; f <= getFeatureCount(); f++)
//					{
//						temporary_features.push_back((float) getFeature(temporary_beams, angles_for_simulation_, clique_points, cv::Point(x, y), f));
//					}
//					doorway_features.push_back(temporary_features);
//					temporary_features.clear();
//				}
//			}
//		}
//		std::cout << "extracted features from one doorway map" << std::endl;
//	}
//
//	//
//	//*************room***************
//	//save the found labels and features in Matrices
//	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
//	cv::Mat room_features_Mat(room_features.size(), getFeatureCount(), CV_32FC1);
//	for (int i = 0; i < labels_for_rooms.size(); i++)
//	{
//		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
//		for (int f = 0; f < getFeatureCount(); f++)
//		{
//			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
//		}
//	}
//	// Train a boost classifier
//	room_boost_.train(room_features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
//	//save the trained booster
//	std::string filename_room = classifier_storage_path + "voronoi_room_boost.xml";
//	room_boost_.save(filename_room.c_str(), "boost");
//	std::cout << "Trained room classifier" << std::endl;
//
//	//
//	//*************hallway***************
//	//save the found labels and features in Matrices
//	cv::Mat hallway_labels_Mat(labels_for_hallways.size(), 1, CV_32FC1);
//	cv::Mat hallway_features_Mat(hallway_features.size(), getFeatureCount(), CV_32FC1);
//	for (int i = 0; i < labels_for_hallways.size(); i++)
//	{
//		hallway_labels_Mat.at<float>(i, 0) = labels_for_hallways[i];
//		for (int f = 0; f < getFeatureCount(); f++)
//		{
//			hallway_features_Mat.at<float>(i, f) = (float) hallway_features[i][f];
//		}
//	}
//	// Train a boost classifier
//	hallway_boost_.train(hallway_features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
//	//save the trained booster
//	std::string filename_hallway = classifier_storage_path + "voronoi_hallway_boost.xml";
//	hallway_boost_.save(filename_hallway.c_str(), "boost");
//	std::cout << "Trained hallway classifier" << std::endl;
//
//	//
//	//*************doorway***************
//	//save the found labels and features in Matrices
//	cv::Mat doorway_labels_Mat(labels_for_doorways.size(), 1, CV_32FC1);
//	cv::Mat doorway_features_Mat(doorway_features.size(), getFeatureCount(), CV_32FC1);
//	for (int i = 0; i < labels_for_doorways.size(); i++)
//	{
//		doorway_labels_Mat.at<float>(i, 0) = labels_for_doorways[i];
//		for (int f = 0; f < getFeatureCount(); f++)
//		{
//			doorway_features_Mat.at<float>(i, f) = (float) doorway_features[i][f];
//		}
//	}
//	// Train a boost classifier
//	doorway_boost_.train(doorway_features_Mat, CV_ROW_SAMPLE, doorway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
//	//save the trained booster
//	std::string filename_doorway = classifier_storage_path + "voronoi_doorway_boost.xml";
//	doorway_boost_.save(filename_doorway.c_str(), "boost");
//	std::cout << "Trained doorway classifier" << std::endl;
//
//	//set the trained-variabel true, so the labeling-algorithm knows the classifiers have been trained already
//	trained_boost_ = true;
//	ROS_INFO("Finished training the Boost algorithm.");
//}

//**************************Training-Algorithm for the AdaBoost-classifiers*****************************
// This Function trains the AdaBoost-classifiers from OpenCV. It takes the given training maps and finds the Points
// that are labeled as the specified classes and calculates the features defined in
// ipa_room_segmentation/voronoi_random_field_features.h.
// These vectors are put in a format that OpenCV expects for the classifiers and then they are trained.
//
void VoronoiRandomFieldSegmentation::trainBoostClassifiers(const std::vector<cv::Mat>& training_maps,
		std::vector< std::vector<Clique> >& cliques_of_training_maps, std::vector<uint> possible_labels,
		const std::string& classifier_storage_path)
{
	std::cout << "starting to train the Boost Classifiers." << std::endl;

	// vectors that store the given labels and features for each point (order: room-hallway-doorway)
	std::vector< std::vector<float> > labels_for_classes(number_of_classes_);
	std::vector< std::vector<float> > features_for_points;

	// go trough each found clique and take the first point of the clique as current point
	//	--> each possible point is only once the first (central) point of a clique
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
//				if(current_labels_for_points[point] != possible_labels[0] && current_labels_for_points[point] != possible_labels[1] && current_labels_for_points[point] != possible_labels[2])
//					std::cout << current_labels_for_points[point] << std::endl;
			}

			// get the stored laser-beams for the central point
			std::vector<double> current_beams = current_clique->getBeams()[0];

//			cv::Mat testmap = current_map.clone();
//			double pi_to_rad = PI / 180;
//			for(size_t i = 0; i < current_beams.size(); ++i)
//			{
//				cv::Point current_beampoint (current_clique_members[0].x + sin(angles_for_simulation_[i]*pi_to_rad)*current_beams[i], current_clique_members[0].y + cos(angles_for_simulation_[i]*pi_to_rad)*current_beams[i]);
//				cv::line(testmap, current_clique_members[0], current_beampoint, cv::Scalar(50), 1);
//			}
//			cv::circle(testmap, current_clique_members[0], 3, cv::Scalar(50), CV_FILLED);

			// get the feature for the current point and store it in the global vector
			std::vector<float> current_features(getFeatureCount());

			for(int f = 1; f <= getFeatureCount(); ++f)
			{
				current_features[f-1] = getFeature(current_beams, angles_for_simulation_, current_clique_members, current_labels_for_points, possible_labels, current_point, f);
				if(current_features[f-1] > 10e6)
					std::cout << current_features[f-1] << " feature: " << f;
			}
			features_for_points.push_back(current_features);
//			std::cout << "f7: (" << current_features[6] << ") ";
//			std::cout << std::endl;

//			cv::imshow("tester", testmap);
//			cv::waitKey();

			// get the labels-vector for each class
			//		--> OpenCV expects the labels: +1 if it belongs to the class, -1 if it doesn't
			for(size_t current_class = 0; current_class < number_of_classes_; ++current_class)
			{
				if(current_labels_for_points[0] == possible_labels[current_class])
					labels_for_classes[current_class].push_back(1.0);
				else
					labels_for_classes[current_class].push_back(-1.0);
			}
		}
	}

	std::cout << "found all features and labels." << std::endl;

	// Train each AdaBoost-classifier.
	//
	//*************room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_classes[0].size(), 1, CV_32FC1);
	cv::Mat features_Mat(features_for_points.size(), getFeatureCount(), CV_32FC1);
	for (int i = 0; i < labels_for_classes[0].size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_classes[0][i];
		for (int f = 0; f < getFeatureCount(); f++)
		{
			features_Mat.at<float>(i, f) = (float) features_for_points[i][f];
//			std::cout << (float) features_for_points[i][f] << " ";
		}
//		std::cout << std::endl;
	}
	// Train a boost classifier
	room_boost_.train(features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_room = classifier_storage_path + "voronoi_room_boost.xml";
	room_boost_.save(filename_room.c_str(), "boost");
	std::cout << "Trained room classifier" << std::endl;
	//
	//*************hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_classes[1].size(), 1, CV_32FC1);
	for (int i = 0; i < labels_for_classes[1].size(); i++)
		hallway_labels_Mat.at<float>(i, 0) = labels_for_classes[1][i];
	// Train a boost classifier
	hallway_boost_.train(features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_hallway = classifier_storage_path + "voronoi_hallway_boost.xml";
	hallway_boost_.save(filename_hallway.c_str(), "boost");
	std::cout << "Trained hallway classifier" << std::endl;
	//
	//*************doorway***************
	//save the found labels and features in Matrices
	cv::Mat doorway_labels_Mat(labels_for_classes[2].size(), 1, CV_32FC1);
	for (int i = 0; i < labels_for_classes[2].size(); i++)
		doorway_labels_Mat.at<float>(i, 0) = labels_for_classes[2][i];
	// Train a boost classifier
	doorway_boost_.train(features_Mat, CV_ROW_SAMPLE, doorway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_doorway = classifier_storage_path + "voronoi_doorway_boost.xml";
	doorway_boost_.save(filename_doorway.c_str(), "boost");
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
//
void VoronoiRandomFieldSegmentation::getAdaBoostFeatureVector(std::vector<double>& feature_vector, Clique& clique,
		std::vector<uint>& given_labels, std::vector<unsigned int>& possible_labels)
{
	// Get the points that belong to the clique and the stored simulated beams for each one.
	std::vector<cv::Point> clique_members = clique.getMemberPoints();
	std::vector< std::vector<double> > beams_for_points = clique.getBeams();

	// vector that is used to sum the calculated features
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

//		std::cout << "got label to use" << std::endl;

		// get the features for the central point of the clique
		cv::Mat featuresMat(1, getFeatureCount(), CV_32FC1); //OpenCV expects a 32-floating-point Matrix as feature input
		for (int f = 1; f <= getFeatureCount(); ++f)
		{
			//get the features for each room and put it in the featuresMat
			featuresMat.at<float>(0, f - 1) = (float) getFeature(beams_for_points[point], angles_for_simulation_, clique_members, given_labels, possible_labels, clique_members[point], f);
//			if(std::count(given_labels.begin(), given_labels.end(), 77) == clique_members.size())
//				std::cout << "filled entry " << f << ": " << featuresMat.at<float>(0, f - 1) << std::endl;
		}
		// Calculate the weak hypothesis by using the wanted classifier.
		CvMat features = featuresMat;
		cv::Mat weaker (1, number_of_classifiers_, CV_32F);
		CvMat weak_hypothesis = weaker;	// Wanted from OpenCV to get the weak hypothesis from the
										// separate weak classifiers.

		switch(classifier)
		{
		case 0:
//			std::cout << "using rooms" << std::endl;
			room_boost_.predict(&features, 0, &weak_hypothesis);
			break;
		case 1:
//			std::cout << "using hallways" << std::endl;
			hallway_boost_.predict(&features, 0, &weak_hypothesis);
			break;
		case 2:
//			std::cout << "using doorways" << std::endl;
			doorway_boost_.predict(&features, 0, &weak_hypothesis);
			break;
		}

//		std::cout << "predicted" << std::endl;

		// Write the weak hypothesis in the feature vector.
//		std::cout << "resaving weak responses " << std::endl;
		for(size_t f = 0; f < number_of_classifiers_; ++f)
		{
//			std::cout << "f: " << f;
			temporary_feature_vector[f] = temporary_feature_vector[f] + (double) CV_MAT_ELEM(weak_hypothesis, float, 0, f);
//			std::cout << ". assigned hypothesis" << std::endl;
		}

//		std::cout << "assigned weak hypothesis" << std::endl;
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
		const size_t number_of_training_maps, std::vector<uint>& possible_labels, const std::string weights_filepath)
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

	for(size_t current_map_index = 0; current_map_index < number_of_training_maps; ++current_map_index)
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

	// find the best weights --> minimize the defined function for the pseudo-likelihood
	std::cout << "finding weights using Dlib" << std::endl;
	column_vector weight_results;
	weight_results = findMinValue(number_of_classifiers_, 9.0, all_point_feature_vectors, mean_weights);

	// save the found weights to a std::vector<double>
	for(size_t weight = 0; weight < number_of_classifiers_; ++weight)
		trained_conditional_weights_.push_back(weight_results(0, weight));

	// save the weights to a file
	// /home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/common/files/training_results/conditional_field_weights.txt
	std::ofstream output_file(weights_filepath.c_str(), std::ios::out);
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

void VoronoiRandomFieldSegmentation::trainAlgorithms(const std::vector<cv::Mat>& training_maps,
		const std::vector<cv::Mat>& voronoi_maps, const std::vector<cv::Mat>& voronoi_node_maps, std::vector<cv::Mat>& original_maps,
		std::vector<unsigned int>& possible_labels, const std::string weights_filepath, const std::string boost_filepath)
{
	// ********** I. Go trough each map and find the drawn node-points for it and check if it is a voronoi-node. *****************
	std::vector<std::set<cv::Point, cv_Point_comp> > random_field_node_points, voronoi_node_points;

	std::cout << "Starting to find the conditional-random-field-cliques." << std::endl;

	for(size_t current_map_index = 0; current_map_index < training_maps.size(); ++current_map_index)
	{
		// Find conditional field nodes by checking each pixel for its color.
		cv::Mat current_map = training_maps[current_map_index].clone();
		cv::Mat current_voronoi_node_map = voronoi_node_maps[current_map_index].clone();

		std::set<cv::Point, cv_Point_comp> current_nodes, current_voronoi_nodes;

		for(size_t v = 0; v < current_map.rows; ++v)
		{
			for(size_t u = 0; u < current_map.cols; ++u)
			{
				// if a pixel is drawn in a color different than white or black it is a node of the conditional field
				if(current_map.at<unsigned char>(v, u) != 255 && current_map.at<unsigned char>(v, u) != 0)
					current_nodes.insert(cv::Point(u,v));

				// check if the current point is a voronoi-node by checking the color in the voronoi-node map
				if(current_voronoi_node_map.at<unsigned char>(v, u) != 255 && current_voronoi_node_map.at<unsigned char>(v, u) != 0)
					current_voronoi_nodes.insert(cv::Point(u,v));
			}
		}

		// save the found nodes
		random_field_node_points.push_back(current_nodes);
		voronoi_node_points.push_back(current_voronoi_nodes);
	}

//	std::cout << "found random field nodes: " << random_field_node_points[5].size() << std::endl;


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
	trainBoostClassifiers(training_maps, conditional_random_field_cliques, possible_labels, boost_filepath);

	// ********** IV. Find the conditional-random-field weights. *****************
	findConditionalWeights(conditional_random_field_cliques, random_field_node_points, training_maps, training_maps.size(), possible_labels, weights_filepath);
}

//
//****************Create the pruned generalized Voronoi-Graph**********************
//
//This function is here to create the pruned generalized voronoi-graph in the given map. It does following steps:
//	1. It finds every discretized contour in the given map (they are saved as vector<Point>). Then it takes these
//	   contour-Points and adds them to the OpenCV Delaunay generator from which the voronoi-cells can be generated.
//	2. Then it finds the largest eroded contour in the given map, which is the contour of the map itself. It searches the
//	   largest contour, because smaller contours correspond to mapping errors
//	3. Then it gets the boundary-Points of the voronoi-cells with getVoronoiFacetList. It takes these facets
//	   and draws them using the drawVoronoi function. This function draws the facets that only have Points inside
//	   the map-contour (other lines go to not-reachable places and are not necessary to be looked at).
//  4. It reduces the graph until the nodes in the graph. A node is a point on the voronoi graph, that has at least 3
//	   neighbors. This deletes errors from the approximate generation of the graph that hasn't been eliminated from
//	   the drawVoronoi function. the resulting graph is the pruned generalized voronoi graph.
//	5. It returns the map that has the pruned generalized voronoi-graph drawn in.
//
void VoronoiRandomFieldSegmentation::createPrunedVoronoiGraph(cv::Mat& map_for_voronoi_generation,
		std::set<cv::Point, cv_Point_comp>& node_points)
{
	cv::Mat map_to_draw_voronoi_in = map_for_voronoi_generation.clone(); //variable to save the given map for drawing in the voronoi-diagram

	cv::Mat temporary_map_to_calculate_voronoi = map_for_voronoi_generation.clone(); //variable to save the given map in the createVoronoiGraph-function

	//apply a closing-operator on the map so bad parts are neglected
//	cv::erode(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());
//	cv::dilate(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());

	//********************1. Get OpenCV delaunay-traingulation******************************

	cv::Rect rect(0, 0, map_to_draw_voronoi_in.cols, map_to_draw_voronoi_in.rows); //variables to generate the voronoi-diagram, using OpenCVs delaunay-traingulation
	cv::Subdiv2D subdiv(rect);

	std::vector < std::vector<cv::Point> > hole_contours; //variable to save the hole-contours

	std::vector < std::vector<cv::Point> > contours; //variables for contour extraction and discretisation
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;

	//get contours of the map
	cv::Mat temp = map_to_draw_voronoi_in.clone();
	cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(map_to_draw_voronoi_in, contours, -1, cv::Scalar(255), CV_FILLED);

	//put every point of the map-contours into the Delaunay-generator of OpenCV
	for (int current_contour = 0; current_contour < contours.size(); current_contour++)
	{
		for (int current_point = 0; current_point < contours[current_contour].size(); current_point++)
		{
			cv::Point fp = contours[current_contour][current_point];
			subdiv.insert(fp);
		}
		//get the contours of the black holes --> it is necessary to check if points are inside these in drawVoronoi
		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
		{
			hole_contours.push_back(contours[current_contour]);
		}
	}

	//********************2. Get largest contour******************************

	std::vector < std::vector<cv::Point> > eroded_contours; //variable to save the eroded contours
	cv::Mat eroded_map;
	cv::Point anchor(-1, -1);

	//erode the map and get the largest contour of it so that points near the boundary are not drawn later (see drawVoronoi)
	cv::erode(temporary_map_to_calculate_voronoi, eroded_map, cv::Mat(), anchor, 2);
//	cv::findContours(eroded_map, eroded_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//	//set initial largest contour
//	largest_contour = eroded_contours[0];
//	for (size_t current_contour = 0; current_contour < eroded_contours.size(); ++current_contour)
//	{
//		//check if the current contour is larger than the saved largest-contour
//		if (cv::contourArea(largest_contour) < cv::contourArea(eroded_contours[current_contour]))
//		{
//			largest_contour = eroded_contours[current_contour];
//		}
//		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
//		{
//			hole_contours.push_back(eroded_contours[current_contour]);
//		}
//	}
	//********************3. Get facets and draw voronoi-Graph******************************
	//get the Voronoi regions from the delaunay-subdivision graph

	const cv::Scalar voronoi_color(127); //define the voronoi-drawing colour

	std::vector < std::vector<cv::Point2f> > voronoi_facets; //variables to find the facets and centers of the voronoi-cells
	std::vector < cv::Point2f > voronoi_centers;

	subdiv.getVoronoiFacetList(std::vector<int>(), voronoi_facets, voronoi_centers);
	//draw the voronoi-regions into the map
	drawVoronoi(map_to_draw_voronoi_in, voronoi_facets, voronoi_color, eroded_map);
	//make pixels black, which were black before and were colored by the voronoi-regions
	for (int v = 0; v < map_to_draw_voronoi_in.rows; v++)
	{
		for (int u = 0; u < map_to_draw_voronoi_in.cols; u++)
		{
			if (map_for_voronoi_generation.at<unsigned char>(v, u) == 0)
			{
				map_to_draw_voronoi_in.at<unsigned char>(v, u) = 0;
			}
		}
	}
	//********************4. Reduce the graph until its nodes******************************

	//1.extract the node-points that have at least three neighbors on the voronoi diagram
	for (int v = 1; v < map_to_draw_voronoi_in.rows-1; v++)
	{
		for (int u = 1; u < map_to_draw_voronoi_in.cols-1; u++)
		{
			if (map_to_draw_voronoi_in.at<unsigned char>(v, u) == 127)
			{
				int neighbor_count = 0;	// variable to save the number of neighbors for each point
				// check 3x3 region around current pixel
				for (int row_counter = -1; row_counter <= 1; row_counter++)
				{
					for (int column_counter = -1; column_counter <= 1; column_counter++)
					{
						// don't check the point itself
						if (row_counter == 0 && column_counter == 0)
							continue;

						//check if neighbors are colored with the voronoi-color
						if (map_to_draw_voronoi_in.at<unsigned char>(v + row_counter, u + column_counter) == 127)
						{
							neighbor_count++;
						}
					}
				}
				if (neighbor_count > 2)
				{
					node_points.insert(cv::Point(u, v));
				}
			}
		}
	}

	//2.reduce the side-lines along the voronoi-graph by checking if it has only one neighbor until a node-point is reached
	//	--> make it white
	//	repeat a large enough number of times so the graph converges
	bool real_voronoi_point; //variable for reducing the side-lines
	for (int step = 0; step < 100; step++)
	{
		for (int v = 0; v < map_to_draw_voronoi_in.rows; v++)
		{
			for (int u = 0; u < map_to_draw_voronoi_in.cols; u++)
			{
				// set that the point is a point along the graph and not a side-line
				real_voronoi_point = true;
				if (map_to_draw_voronoi_in.at<unsigned char>(v, u) == 127)
				{
					int neighbor_count = 0;		//variable to save the number of neighbors for each point
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							// don't check the point itself
							if (row_counter == 0 && column_counter == 0)
								continue;

							// check the surrounding points
							const int nv = v + row_counter;
							const int nu = u + column_counter;
							if (nv >= 0 && nu >= 0 && nv < map_to_draw_voronoi_in.rows && nu < map_to_draw_voronoi_in.cols &&
									map_to_draw_voronoi_in.at<unsigned char>(nv, nu) == 127)
							{
								neighbor_count++;
							}
						}
					}
					//if the current point is a node point found in the previous step, it belongs to the voronoi-graph
					if (neighbor_count <= 1 && node_points.find(cv::Point(u,v)) == node_points.end())
					{
						map_to_draw_voronoi_in.at<unsigned char>(v, u) = 255;
					}
				}
			}
		}
	}

	// Return the calculated map with the pruned voronoi graph drawn in.
	map_for_voronoi_generation = map_to_draw_voronoi_in;

	cv::imwrite("/home/rmb-fj/Pictures/voronoi_random_fields/pruned_voronoi.png", map_to_draw_voronoi_in);

}

column_vector VoronoiRandomFieldSegmentation::findMinValue(unsigned int number_of_weights, double sigma,
		const std::vector<std::vector<double> >& likelihood_parameters, const std::vector<double>& starting_weights)
{
	std::cout << "finding min values" << std::endl;
	// create a column vector as starting search point, that is needed from Dlib to find the min. value of a function
	column_vector starting_point(number_of_weights);

	// initialize the starting point as zero to favour small weights
	starting_point = 0;

	// create a Likelihood-optimizer object to find the weights that maximize the pseudo-likelihood
	pseudoLikelihoodOptimization minimizer;

	// set the values for this optimization-object
	minimizer.sigma = sigma;
	minimizer.number_of_weights = number_of_weights;

	minimizer.log_parameters = likelihood_parameters;
	minimizer.starting_weights = starting_weights;


	// find the best weights for the given parameters
	dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), minimizer, starting_point, -1);

//	column_vector test_vector(number_of_weights);
//
//	test_vector = 2., 2.;
//
//	std::cout << std::endl << "value: " << minimizer(test_vector) << " at vector: " << test_vector << std::endl;

	return starting_point;
}



//
//****************** Segmentation Function *********************
//
// This function segments the given original_map into different regions by using the voronoi random field method from
// Stephen Friedman and Dieter Fox ( http://www.cs.washington.edu/robotics/projects/semantic-mapping/abstracts/vrf-place-labeling-ijcai-07.abstract.html ).
// This algorithm has two parts, the training step and the actual segmentation. In the segmentation step following actions
// are made:
//		I.) From the given map that should be labeled (original_map) a pruned generalized Voronoi diagram is extracted ( https://www.sthu.org/research/voronoidiagrams/ ).
//			This is done using the method from Karimipour and Ghandehari ( A Stable Voronoi-based Algorithm for Medial Axis Extraction through Labeling Sample Points )
//			that samples the building contour to get centerpoints to compute the voronoi graph. This approximated graph has
//			some errors in it, so two elimination steps are done:
//				i) eliminate lines of the graph that start or end in black regions
//				ii) reduce the graph from after the first step until the nodes of the graph
//			See the createPrunedVoronoiGraph() function above for better information. OpenCV is used to do this.
//		II.) It looks at this pruned voronoi graph and concentrates a defined region on this graph into one point, that is
//			used as a node in a graph. In this graph nodes are connected, that
//				i) are right beside each other
//				ii) and if a Point in the graph has three or more neighbors all of these four nodes are connected to each other
//			so that different cliques occur. This is necessary to use a Conditional Random Filed to label the nodes as a
//			defined class. To do so the following steps are done:
//				1. Add the previously found node points on the voronoi graph to the Conditional Random Field graph. The algorithm
//				   checks if they are too close to each other and only adds one of these close points to the crf.
//				2. Look at an epsilon neighborhood on the graph and choose the point farthest away from the black pixels as
//				   point for the crf. The farthest point is chosen, because else it would be possible  that the chosen point
//				   is too near at the black pixels. Also the new node has to be more far away than a defined  min. distance to
//				   each already found node s.t. two nodes are not too close to each other, or else the  crf would be created wrong.
//		III.) It constructs the Conditional Random Field graph from the previously found points, by using the above defined function.
//
void VoronoiRandomFieldSegmentation::segmentMap(cv::Mat& original_map, const int epsilon_for_neighborhood,
		const int max_iterations, unsigned int min_neighborhood_size, std::vector<uint>& possible_labels,
		const double min_node_distance,  bool show_nodes, std::string crf_storage_path, std::string boost_storage_path,
		const size_t max_inference_iterations)
{
	// save a copy of the original image
	cv::Mat original_image = original_map.clone();

	// if the training results haven't been loaded or trained before load them
	std::string filename_room = boost_storage_path + "voronoi_room_boost.xml";
	std::string filename_hallway = boost_storage_path + "voronoi_hallway_boost.xml";
	std::string filename_doorway = boost_storage_path + "voronoi_doorway_boost.xml";
	if(trained_boost_ == false)
	{
		// load the AdaBoost-classifiers
		room_boost_.load(filename_room.c_str());
		hallway_boost_.load(filename_hallway.c_str());
		doorway_boost_.load(filename_doorway.c_str());

		// set the trained-Boolean true to only load parameters once
		trained_boost_ = true;
	}

	if(trained_conditional_field_ == false)
	{
		// load the weights out of the file
		std::ifstream input_file(crf_storage_path.c_str());
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

//		std::cout << "read in weights-vector: " << std::endl;
//		for(size_t i = 0; i < trained_conditional_weights_.size(); ++i)
//			std::cout << trained_conditional_weights_[i] << std::endl;
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
	// 1. Get the points for the conditional random field graph by looking at an epsilon neighborhood.

	std::set<cv::Point, cv_Point_comp> conditional_field_nodes;

	// get the distance transformed map, which shows the distance of every white pixel to the closest zero-pixel
	cv::Mat distance_map; //distance-map of the original-map (used to check the distance of each point to nearest black pixel)
	cv::distanceTransform(original_map, distance_map, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map, distance_map);

	cv::Mat voronoi_map_for_node_extraction = voronoi_map.clone();

	timer.start();
	// 2. add the voronoi graph node points to the conditional random field nodes if they are not too close to each other
	for(std::set<cv::Point, cv_Point_comp>::iterator node = node_points.begin(); node != node_points.end(); ++node)
	{
		if(pointMoreFarAway(conditional_field_nodes, *node, min_node_distance) == true)
			conditional_field_nodes.insert(*node);
	}
//	std::cout << "round time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// 3. Find the other node points. They are the points most far away from the black points in the defined epsilon neighborhood.
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
//								const cv::Point& current_neighbor_point = neighbor_points[current_neighbor_point_index];
								const int nu = current_neighbor_point->x + column_counter;
								const int nv = current_neighbor_point->y + row_counter;
//								if(u > 250 && v > 250)
//								{
//									cv::Mat tester = original_map.clone();
//									cv::circle(tester, cv::Point(nu, nv), 2, cv::Scalar(100), CV_FILLED);
//									cv::imshow("tester", tester);
//									cv::waitKey();
//								}
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
								&& pointMoreFarAway(conditional_field_nodes, *point, min_node_distance) == true)
						{
							current_conditional_field_point = *point;
						}
					}
					// add the local minimum point to the critical points and check a last time if the node is far enough away
					// from other nodes (because if no new node is found the initialized gets added every time, neglecting
					// this constraint)
					if(pointMoreFarAway(conditional_field_nodes, current_conditional_field_point, min_node_distance) == true)
						conditional_field_nodes.insert(current_conditional_field_point);
				}
			}
		}
	}

	std::cout << "found all conditional field nodes. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	// show the node points if wanted
	cv::Mat node_map = original_map.clone();
	if(show_nodes == true)
	{
		cv::cvtColor(node_map, node_map, CV_GRAY2BGR);
		for(std::set<cv::Point, cv_Point_comp>::iterator node = conditional_field_nodes.begin(); node != conditional_field_nodes.end(); ++node)
		{
			cv::circle(node_map, *node, 0, cv::Scalar(250,0,0), CV_FILLED);
		}

//		cv::imshow("nodes of the conditional random field", node_map);
//		cv::waitKey();
		cv::imwrite("/home/rmb-fj/Pictures/voronoi_random_fields/node_map.png", node_map);
	}

	// ************* III. Construct the Conditional Random Field from the found nodes *************
	//
	// Go along the voronoi graph each point and find the 2 or 3 nearest neighbors and construct a clique out of them.
	// If enough neighbors are found or no new voronoi-nodes were found in the last step, the algorithm stops. If no new
	// Voronoi-nodes got found, the current node is a dead end and has only one neighbor.
	// This is done using the above defined function createConditionalField, so see this function for further information.
	//
	cv::Mat neighbor_map = node_map.clone(); // map to show the resulting cliques if wanted

	std::vector<Clique> conditional_random_field_cliques; // vector to save the found cliques of the conditional random field

	// construct the conditional random field
	std::cout << "starting to create conditional field" << std::endl;
	timer.start();
	createConditionalField(voronoi_map, conditional_field_nodes, conditional_random_field_cliques, node_points, original_image);

	std::cout << "Created field. Time: " << timer.getElapsedTimeInMilliSec() << "ms. Number of cliques: " << conditional_random_field_cliques.size() << std::endl;

	// show the found cliques if wanted
	if(show_nodes == true)
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
		cv::imwrite("/home/rmb-fj/Pictures/voronoi_random_fields/neighbor_map.png", neighbor_map);
	}

	// ************* IV. Construct the Factor graph from the calculated random-Field *************
	//
	// The calculated CRF has to be a Factor graph to use the OpenGM libraries Belief-Propagation algorithm. A Factor graph is a
	// graphical model that calculates a large function by calculating each part of the function for itself (https://en.wikipedia.org/wiki/Factor_graph).
	// Meaning it is a function in the form f(x_0:i) = f(x_j:k) * f(x_l:m) *... . In this case each clique-potential is one part
	// of the whole function and the overall function should be minimized later. To do this the Typedefs in the header gets used.
	//
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
	std::vector<std::vector<std::vector<uint> > > label_configurations(4);

	// vector that stores the index for each possible label as element
	// 		--> to get the order as index list, so it can be used to assign the value of functions
	std::vector<uint> label_indices(possible_labels.size());
	for(uint index = 0; index < possible_labels.size(); ++index)
		label_indices[index] = index;

	timer.start();

	for(uint size = 2; size <= 5; ++size)
	{
		// vector that stores all possible configurations for one member-size
		std::vector<std::vector<uint> > possible_configurations;

		// use the above defined function to find all possible configurations for the possible labels and save them in the map
		getPossibleConfigurations(possible_configurations, label_indices, size);
		label_configurations[size-2] = possible_configurations;
	}

	std::cout << "Created all possible label-configurations. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	timer.start();

	// Go trough each clique and define the function and factor for it.
	for(std::vector<Clique>::iterator current_clique = conditional_random_field_cliques.begin(); current_clique != conditional_random_field_cliques.end(); ++current_clique)
	{
//		// create function object for this clique, load CvBoost separately and don't use = or else the Boosters would get deleted
//		// after this struct gets destroyed.
//		adaBoostFunction function;
//
//		function.angles_for_simulation = angles_for_simulation_;
////		function.clique_members = current_clique->getMemberPoints();
////		function.beams_for_points = current_clique->getBeams();
//		function.clique = *current_clique;
//		function.room_boost.load(filename_room.c_str());
//		function.hallway_boost .load(filename_hallway.c_str());
//		function.doorway_boost.load(filename_doorway.c_str());
//		function.number_of_classifiers = number_of_classifiers_;
//		function.possible_labels = possible_labels;
//		function.trained_conditional_weights = trained_conditional_weights_;

		// get the number of members in this clique and depending on this the possible label configurations defined above
		size_t number_of_members = current_clique->getNumberOfMembers();
		std::vector<std::vector<uint> > current_possible_configurations = label_configurations[number_of_members-2]; // -2 because this vector stores configurations for cliques with 2-5 members (others are not possible in this case).

		// find the real labels and assign them into the current configuration so the feature-vector gets calculated correctly
		for(size_t configuration = 0; configuration < current_possible_configurations.size(); ++configuration)
			for(size_t variable = 0; variable < current_possible_configurations[configuration].size(); ++variable)
				current_possible_configurations[configuration][variable] = possible_labels[current_possible_configurations[configuration][variable]];

//		for(size_t i = 0; i < current_possible_configurations.size(); ++i)
//		{
//			for(size_t j = 0; j < current_possible_configurations[i].size(); ++j)
//				std::cout << current_possible_configurations[i][j] << " ";
//			std::cout << std::endl;
//		}
//		std::cout << "Members in this clique: " << number_of_members << ". Number of possible configurations for this clique: " << current_possible_configurations.size() << std::endl;

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
		std::vector<std::vector<uint> > swap_configurations = label_configurations[number_of_members-2]; // -2 because this vector stores configurations for cliques with 2-5 members (others are not possible in this case).
		swapConfigsRegardingNodeIndices(swap_configurations, indices);
		std::sort(indices, indices + current_clique->getNumberOfMembers());

		// Go trough each possible configuration and compute the function value for it. Use the original configuration, because
		// the nodes are stored in this way, but later the value is assigned in the position using the swaped configurations.
		for(size_t configuration = 0; configuration < current_possible_configurations.size(); ++configuration)
		{
//			std::cout << "current configuration: " << std::endl;

			std::vector<uint> current_configuration = current_possible_configurations[configuration];

//			for(size_t i = 0; i < current_configuration.size(); ++i)
//				std::cout << current_configuration[i] << " ";
//			std::cout << std::endl;

			// get current feature-vector and multiply it with the trained weights
			std::vector<double> current_features(number_of_classifiers_);
			getAdaBoostFeatureVector(current_features, *current_clique, current_configuration, possible_labels);

//			double test_result = function(current_configuration);

			double clique_potential = 0;
			for(size_t weight = 0; weight < number_of_classifiers_; ++weight)
				clique_potential += trained_conditional_weights_[weight] * current_features[weight];

//			if(clique_potential != test_result)
//			{
//				std::cout << "config: ";
//				for(size_t i = 0; i < current_configuration.size(); ++i)
//					std::cout << current_configuration[i] << " ";
//				std::cout << std::endl << "Real: "  << clique_potential << " Func: " << test_result<< std::endl;
//			}

			// assign the calculated clique potential at the right position in the function --> !!Important: factors need the variables to be sorted
			//																								 as increasing index
			f(swap_configurations[configuration].begin()) = clique_potential;

//			std::cout << "got one feature-vector" << std::endl;

		}
//		std::cout << "one clique done" << std::endl;

		// add the defined function to the model and catch the returned function-identifier to specify which variables
		// this function needs
		FactorGraph::FunctionIdentifier identifier = factor_graph.addFunction(f);

		// add the Factor to the graph, that represents which variables (and labels of each) are used for the above defined function
		factor_graph.addFactor(identifier, indices, indices+current_clique->getNumberOfMembers());

	}
	std::cout << "calculated all features for the cliques. Time: " << timer.getElapsedTimeInSec() << "s" << std::endl;

	// 3. Do Inference in the above created graphical model using OpengM. This function has three control parameters:
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
	std::cout << "Done Inference. Time: " << timer.getElapsedTimeInSec() << "s" << std::endl;

	// obtain the labels that get the max value of the defined function
	std::vector<size_t> best_labels(conditional_field_nodes.size());
	belief_propagation.arg(best_labels);

	// print the solution
	cv::Mat resulting_map = original_image.clone();
//	for(size_t i = 0; i < best_labels.size(); ++i)
//	{
//		std::cout << best_labels[i] << " ";
//		cv::circle(resulting_map, conditional_field_nodes, 5, cv::Scalar(possible_labels[best_labels[i]]), CV_FILLED);
//	}
	for(std::set<cv::Point, cv_Point_comp>::iterator i = conditional_field_nodes.begin(); i != conditional_field_nodes.end(); ++i)
	{
		size_t distance = std::distance(conditional_field_nodes.begin(), i);
		std::cout << best_labels[distance] << " ";
		cv::circle(resulting_map, *i, 5, cv::Scalar(possible_labels[distance]), CV_FILLED);
	}
	std::cout << std::endl;
	cv::imshow("res", resulting_map);
	cv::imwrite("/home/rmb-fj/Pictures/voronoi_random_fields/result_map.png", resulting_map);
	cv::waitKey();

}

// Function to test several functions of this algorithm independent of other functions
void VoronoiRandomFieldSegmentation::testFunc(cv::Mat& original_map)
{
	std::cout << "testfunc" << std::endl;
//	std::set<cv::Point, cv_Point_comp> node_points; //variable for node point extraction
//	cv::Mat voronoi_map = original_map.clone();
//	createPrunedVoronoiGraph(voronoi_map, node_points);
//
//	cv::imshow("voronoi", voronoi_map);
//	cv::waitKey();

	std::vector<std::vector<uint> > configs;
	uint number_of_variables = 3;
	uint number_of_labels = 3;
	double myints[] = {1,1,1,1,2,2,2,2,3,3,3,3};
	std::vector<uint> labels(number_of_labels);
//	for(size_t i = 1; i <= number_of_labels; ++i)
//		labels[i-1] = i;
	labels[0] = 0;
	labels[1] = 1;
	labels[2] = 2;

	Timer timer;

	timer.start();
	getPossibleConfigurations(configs, labels, number_of_variables);
//	std::cout << "Time needed: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;
////	std::vector<int> myvector(number_of_labels*number_of_variables);// (myints, myints+(number_of_labels * number_of_variables));
////	std::cout << myvector.size() << std::endl;
////	for(size_t variable = 0; variable < number_of_variables; ++variable)
////		for(size_t label = 0; label < number_of_labels; ++label)
////		{
////			myvector[label + number_of_labels * variable] = (double) (label+1);
////			std::cout << label+1 << std::endl;
////		}
////	std::sort (myvector.begin(),myvector.end());
////
////	std::cout << "The 3! possible permutations with 3 elements:\n";
////	do
////	{
////		std::vector<double> current_config(number_of_variables);
////		for(size_t v = 0; v < number_of_variables; ++v)
////			current_config[v] = myvector[v];
//////		current_config[1] = myvector[1];
//////		current_config[2] = myvector[2];
//////		current_config[3] = myvector[3];
////
////		if(contains(configs, current_config) == false)
////			configs.push_back(current_config);
//////		std::cout << myints[0] << ' ' << myints[1] << ' ' << myints[2] << '\n';
////	}while ( std::next_permutation(myvector.begin(),myvector.end()) );
//

	size_t node_indices[3] = {3,5,4};

//	std::vector<std::vector<labelWithIndex> > swaped_configs;

	std::cout << "possible configurations(" << configs.size() << "): " << std::endl;
	for(size_t i = 0; i < configs.size(); ++i)
	{
//		std::vector<labelWithIndex> current_vector;
		for(size_t j = 0; j < configs[i].size(); ++j)
		{
//			labelWithIndex current_label = {configs[i][j], node_indices[j]};
//			current_vector.push_back(current_label);
			std::cout << configs[i][j] << " ";
		}
		// sort the current vector and add it to the configurations
//		std::sort(current_vector.begin(), current_vector.end(), compLabelsByIndices());
//		swaped_configs.push_back(current_vector);
		std::cout << std::endl;
	}

	timer.start();
	swapConfigsRegardingNodeIndices(configs, node_indices);
	std::cout << "swaped configs. Time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

	for(size_t i = 0; i < configs.size(); ++i)
	{
		for(size_t j = 0; j < configs[i].size(); ++j)
		{
			std::cout << configs[i][j] << " ";
		}
		std::cout << std::endl;
	}


	// swap the configurations to get the change with rising indices for the nodes

//	// define an array that has as many elements as the clique has members and assign the number of possible labels for each
//	size_t variable_space[2];
//	std::fill_n(variable_space, 2, number_of_classes_);
//
//	std::cout << "filled" << std::endl;
//	// define a explicit function-object from OpenGM containing the initial value -1.0 for each combination
//	opengm::ExplicitFunction<double> f(variable_space, variable_space + 2, -1.0);
//
//	std::vector<uint> r(2);
//	r[0] = 0;
//	r[1] = 0;
//
//	std::cout << "vector created" << std::endl;
//
//	uint t[2];
//	uint* a = &r[0];
////	std::fill_n(t, 2, 0);
//
//	f(r.begin()) = 3.0;
//
//	r[0] = 1;
//	a = &r[0];
//
//	f(a) = 1.0;
//
//	std::cout << f(0,0) << " " << f(1,0) << std::endl;

}


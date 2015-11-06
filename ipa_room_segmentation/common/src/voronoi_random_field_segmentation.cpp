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

#include <ipa_room_segmentation/wavefront_region_growing.h> // some useful functions defined for all segmentations
#include <ipa_room_segmentation/contains.h>

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
	}

	double operator()(const column_vector& weights) const
	{
		double result = 0;
		// go trough each part of the function and calculate the log(.) and add it to the result
		for(unsigned int function_part = 0; function_part < log_parameters.size(); ++function_part)
		{
			double log_numerator, log_denominator = 0; // nominator and denominator for each log
			double exp_exponent = 0; // helping variable to get each exponent for exp(.)
			// get the log_numerator for each function part
			for(unsigned int numerator_factor = 0; numerator_factor < number_of_weights; ++numerator_factor)
			{
				exp_exponent += log_parameters[function_part][numerator_factor] * weights(numerator_factor);
			}
			log_numerator = exp(exp_exponent);

			// add the numerator to the denominator, because it has to appear here
			log_denominator += log_numerator;
			// add each clique-value to the denominator
			unsigned int vector_position = number_of_weights; // variable to get the current absoulte starting position
															  // for the vector
			do
			{
				exp_exponent = 0;
				for(unsigned int relative_position = 0; relative_position < number_of_weights; ++relative_position)
				{
					exp_exponent += log_parameters[function_part][vector_position + relative_position] * weights(relative_position);
				}
				// update the absolute vector position
				vector_position += number_of_weights;
				// update the denominator
				log_denominator += exp(exp_exponent);
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
		result += gaussian_numerator / (2 * sigma * sigma);
		return result;
	}
};

VoronoiRandomFieldSegmentation::VoronoiRandomFieldSegmentation()
{
	//save the angles between the simulated beams, used in the following algorithm
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation_.push_back(angle);
	}
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 350, 0, 2, false, 0);
	params_ = params;
	trained_ = false;
}

void VoronoiRandomFieldSegmentation::drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color,
		const std::vector<cv::Point>& contour, const std::vector<std::vector<cv::Point> >& hole_contours)
{
	// This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
	// map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
	// inside a hole-contour and doesn't draw the lines that are not.
	for (int idx = 0; idx < facets_of_voronoi.size(); idx++)
	{
		// saving-variable for the last Point that has been looked at
		cv::Point2f last_point = facets_of_voronoi[idx].back();
		// draw each line of the voronoi-cell
		for (int c = 0; c < facets_of_voronoi[idx].size(); c++)
		{
			// variable to check, whether a Point is inside a white area or not
			bool inside = true;
			cv::Point2f current_point = facets_of_voronoi[idx][c];
			// only draw lines that are inside the map-contour
			if (cv::pointPolygonTest(contour, current_point, false) < 0 || cv::pointPolygonTest(contour, last_point, false) < 0)
			{
				inside = false;
			}
			// only draw Points inside the contour that are not inside a hole-contour
			for (int i = 0; i < hole_contours.size(); i++)
			{
				if (cv::pointPolygonTest(hole_contours[i], current_point, false) >= 0 || cv::pointPolygonTest(hole_contours[i], last_point, false) >= 0)
				{
					inside = false;
				}
			}
			if (inside)
			{
				cv::line(img, last_point, current_point, voronoi_color, 1);
			}
			last_point = current_point;
		}
	}
}

// Function to train the AdaBoost classifiers that are used for feature induction in the conditional random field.
void VoronoiRandomFieldSegmentation::trainBoostClassifiers(std::vector<cv::Mat> room_training_maps, const std::string& classifier_storage_path)
{
	//**************************Training-Algorithm for the AdaBoost-classifiers*****************************
	// This Alogrithm trains two AdaBoost-classifiers from OpenCV. It takes the given training maps and finds the Points
	// that are labeled as the specified classes and calculates the features defined in
	// ipa_room_segmentation/voronoi_random_field_segmentation.h.
	// Then these vectors are put in a format that OpenCV expects for the classifiers and then they are trained.
	std::vector<float> labels_for_hallways, labels_for_rooms;
	std::vector<std::vector<float> > hallway_features, room_features;
	std::vector<double> temporary_beams;
	std::vector<float> temporary_features;
	std::cout << "Starting to train the algorithm." << std::endl;
	std::cout << "number of room training maps: " << room_training_maps.size() << std::endl;
	//Get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it belongs to a hallway
	for(size_t map = 0; map < room_training_maps.size(); ++map)
	{
		for (int y = 0; y < room_training_maps[map].cols; y++)
		{
			for (int x = 0; x < room_training_maps[map].rows; x++)
			{
				if (room_training_maps[map].at<unsigned char>(x, y) != 0)
				{
					//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
					if (room_training_maps[map].at<unsigned char>(x, y) > 250)
					{
						labels_for_rooms.push_back(1.0);
					}
					else
					{
						labels_for_rooms.push_back(-1.0);
					}
					//simulate the beams and features for every position and save it
					temporary_beams = raycasting(room_training_maps[map], cv::Point(x, y));
					for (int f = 1; f <= get_feature_count(); f++)
					{
						temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation_, cv::Point(x, y), f));
					}
					room_features.push_back(temporary_features);
					temporary_features.clear();
				}
			}
		}
		std::cout << "extracted features from one room map" << std::endl;
	}

	//*************room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
	cv::Mat room_features_Mat(room_features.size(), get_feature_count(), CV_32FC1);
	for (int i = 0; i < labels_for_rooms.size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
		for (int f = 0; f < get_feature_count(); f++)
		{
			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
		}
	}
	// Train a boost classifier
	room_boost_.train(room_features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params_);
	//save the trained booster
	std::string filename_room = classifier_storage_path + "voronoi_room_boost.xml";
	room_boost_.save(filename_room.c_str(), "boost");
	//set the trained-variabel true, so the labeling-algorithm knows the classifiers have been trained already
	trained_ = true;
	ROS_INFO("Done room classifiers.");
	ROS_INFO("Finished training the algorithm.");
}

void VoronoiRandomFieldSegmentation::createPrunedVoronoiGraph(cv::Mat& map_for_voronoi_generation)
{
	//****************Create the pruned generalized Voronoi-Graph**********************
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

	cv::Mat map_to_draw_voronoi_in = map_for_voronoi_generation.clone(); //variable to save the given map for drawing in the voronoi-diagram

	cv::Mat temporary_map_to_calculate_voronoi = map_for_voronoi_generation.clone(); //variable to save the given map in the createVoronoiGraph-function

	//apply a closing-operator on the map so bad parts are neglected
	cv::erode(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());
	cv::dilate(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());

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

	std::vector < cv::Point > largest_contour; //variable to save the largest contour of the map --> the contour of the map itself

	//erode the map and get the largest contour of it so that points near the boundary are not drawn later (see drawVoronoi)
	cv::erode(temporary_map_to_calculate_voronoi, eroded_map, cv::Mat(), anchor, 2);
	cv::findContours(eroded_map, eroded_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//set initial largest contour
	largest_contour = eroded_contours[0];
	for (int current_contour = 0; current_contour < eroded_contours.size(); current_contour++)
	{
		//check if the current contour is larger than the saved largest-contour
		if (cv::contourArea(largest_contour) < cv::contourArea(eroded_contours[current_contour]))
		{
			largest_contour = eroded_contours[current_contour];
		}
		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
		{
			hole_contours.push_back(eroded_contours[current_contour]);
		}
	}
	//********************3. Get facets and draw voronoi-Graph******************************
	//get the Voronoi regions from the delaunay-subdivision graph

	cv::Scalar voronoi_color(127); //define the voronoi-drawing colour

	std::vector < std::vector<cv::Point2f> > voronoi_facets; //variables to find the facets and centers of the voronoi-cells
	std::vector < cv::Point2f > voronoi_centers;

	subdiv.getVoronoiFacetList(std::vector<int>(), voronoi_facets, voronoi_centers);
	//draw the voronoi-regions into the map
	drawVoronoi(map_to_draw_voronoi_in, voronoi_facets, voronoi_color, largest_contour, hole_contours);
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

	std::vector < cv::Point > node_points; //variable for node point extraction

	//1.extract the node-points that have at least three neighbors on the voronoi diagram
	for (int v = 1; v < map_to_draw_voronoi_in.rows-1; v++)
	{
		for (int u = 1; u < map_to_draw_voronoi_in.cols-1; u++)
		{
			if (map_to_draw_voronoi_in.at<unsigned char>(v, u) == 127)
			{
				int neighbor_count = 0;	//variable to save the number of neighbors for each point
				//check 3x3 region around current pixel
				for (int row_counter = -1; row_counter <= 1; row_counter++)
				{
					for (int column_counter = -1; column_counter <= 1; column_counter++)
					{
						//check if neighbors are colored with the voronoi-color
						if (map_to_draw_voronoi_in.at<unsigned char>(v + row_counter, u + column_counter) == 127 && (row_counter !=0 || column_counter != 0))
						{
							neighbor_count++;
						}
					}
				}
				if (neighbor_count > 2)
				{
					node_points.push_back(cv::Point(v, u));
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
				//set that the point is a point along the graph and not a side-line
				real_voronoi_point = true;
				if (map_to_draw_voronoi_in.at<unsigned char>(v, u) == 127)
				{
					int neighbor_count = 0;		//variable to save the number of neighbors for each point
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							const int nv = v + row_counter;
							const int nu = u + column_counter;
							if (nv >= 0 && nu >= 0 && nv < map_to_draw_voronoi_in.rows && nu < map_to_draw_voronoi_in.cols &&
									map_to_draw_voronoi_in.at<unsigned char>(nv, nu) == 127 && (row_counter != 0 || column_counter != 0))
							{
								neighbor_count++;
							}
						}
					}
					if (neighbor_count == 1)
					{
						//The point is a leaf-node
						real_voronoi_point = false;
					}
					//if the current point is a node point found in the previous step, it belongs to the voronoi-graph
					if (contains(node_points, cv::Point(v, u)))
					{
						real_voronoi_point = true;
					}
					if (!real_voronoi_point)
					{
						//if the Point isn't on the voronoi-graph make it white
						map_to_draw_voronoi_in.at<unsigned char>(v, u) = 255;
					}
				}
			}
		}
	}

	// Return the calculated map with the pruned voronoi graph drawn in.
	map_for_voronoi_generation = map_to_draw_voronoi_in;

}

column_vector VoronoiRandomFieldSegmentation::findMinValue()
{
	column_vector starting_point(1);

	starting_point = 1;

	pseudoLikelihoodOptimization tester;
	tester.sigma = 3.0;
	tester.number_of_weights = 1;

	double params_1[] = {10, 7};
	std::vector<double> params_1_vec(params_1, params_1 + sizeof(params_1) / sizeof(double) );
	double params_2[] = {8, 5};
	std::vector<double> params_2_vec(params_2, params_2 + sizeof(params_2) / sizeof(double) );
	std::vector<std::vector<double> > parameters;
	parameters.push_back(params_1_vec);
	parameters.push_back(params_2_vec);

	double weight_starts[] = {0};
	std::vector<double> weight_starts_vec(weight_starts, weight_starts + sizeof(weight_starts) / sizeof(double) );

	tester.log_parameters = parameters;
	tester.starting_weights = weight_starts_vec;


	dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), tester, starting_point, -1);

	return starting_point;
}

//****************** Segmentation Function *********************
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
//			so that different cliques occur. This is neccessary to use a Conditional Random Filed to label the nodes as a
//			defined class.
//
void VoronoiRandomFieldSegmentation::segmentMap(cv::Mat& original_map, const int size_of_region_on_voronoi)
{
	// ************* I. Create the pruned generalized Voronoi graph *************
	cv::Mat voronoi_map = original_map.clone();

	// use the above defined functio to create a pruned Voronoi graph
	createPrunedVoronoiGraph(voronoi_map);

	// ************* 2. Extract the graph used for the conditional random field *************
	//
	// 1. Find the node Points of the graph. A node point of the graph is a point in the voronoi graph that has at least
	//	  three neighbors.

	std::vector < cv::Point > node_points; //variable for node point extraction

	for (int v = 1; v < voronoi_map.rows-1; v++)
	{
		for (int u = 1; u < voronoi_map.cols-1; u++)
		{
			if (voronoi_map.at<unsigned char>(v, u) == 127)
			{
				int neighbor_count = 0;	//variable to save the number of neighbors for each point
				//check 3x3 region around current pixel
				for (int row_counter = -1; row_counter <= 1; row_counter++)
				{
					for (int column_counter = -1; column_counter <= 1; column_counter++)
					{
						//check if neighbors are colored with the voronoi-color
						if (voronoi_map.at<unsigned char>(v + row_counter, u + column_counter) == 127 && (row_counter !=0 || column_counter != 0))
						{
							neighbor_count++;
						}
					}
				}
				if (neighbor_count > 2)
				{
					node_points.push_back(cv::Point(v, u));
				}
			}
		}
	}

}




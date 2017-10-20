#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <ipa_building_navigation/contains.h>
#include <ipa_building_navigation/A_star_pathplanner.h>
#include <ipa_building_navigation/maximal_clique_finder.h>
#include <ipa_building_navigation/distance_matrix.h>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
//regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This algorithm provides a class to solve the set-cover problem for given cliques. This is done by using the greedy-search
//algorithm, which takes the clique with most unvisited nodes before the other nodes and removes the nodes in it from the
//unvisited. It repeats this step until no more node hasn't been visited. It then merges cliques together that have at least
//one node in common.
//
//!!!!!!!!!!!!!!!!Important!!!!!!!!!!!!!!!!!
//Make sure that the cliques cover all nodes in the graph or else this algorithm runs into an endless loop. For best results
//take the cliques from a maximal-clique finder like the Bron-Kerbosch algorithm.

class SetCoverSolver
{
protected:

	//Astar pathplanner to find the pathlengths from cv::Point to cv::Point
	AStarPlanner pathplanner_;

	//This object finds all maximal cliques in the given Graph. It needs a symmetrical distance matrix shwoing the pathlength
	//from one node to another and a miximal pathlength to cut edges that are larger than this value. See maximal_clique_finder.h
	//for further information.
	cliqueFinder maximal_clique_finder;

	//function to merge groups together, which have at least one node in common
	std::vector<std::vector<int> > mergeGroups(const std::vector<std::vector<int> >& found_groups);

//	//Function to construct the distance matrix, showing the pathlength from node to node
//	void constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const std::vector<cv::Point>& points,
//	        double downsampling_factor, double robot_radius, double map_resolution);

public:
	//Constructor
	SetCoverSolver();

	//algorithms to solve the set cover problem. There are three functions for different cases:
	//		1. The cliques already have been found
	//		2. The distance matrix already exists
	//		3. The distance matrix has to be computed and may be returned

	//cliques are given
	std::vector<std::vector<int> > solveSetCover(std::vector<std::vector<int> >& given_cliques, const int number_of_nodes,
			const int max_number_of_clique_members, const cv::Mat& distance_matrix);

	//the distance matrix is given
	std::vector<std::vector<int> > solveSetCover(const cv::Mat& distance_matrix, const std::vector<cv::Point>& points,
			const int number_of_nodes, double maximal_pathlength, const int max_number_of_clique_members);

	//the distance matrix has to be computed and may be returned
	std::vector<std::vector<int> > solveSetCover(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, double maximal_pathlength, const int max_number_of_clique_members, cv::Mat* distance_matrix=0);
};

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

#include <ipa_building_navigation/contains.h>

//This algorithm provides a class to solve the set-cover problem for given cliques. This is done by using the greedy-search
//algorithm, which takes the clique with most unvisited nodes before the other nodes and removes the nodes in it from the
//unvisited. It repeats this step until no more node hasn't been visited. It then merges cliques together that have at least
//one node in common.
//
//!!!!!!!!!!!!!!!!Important!!!!!!!!!!!!!!!!!
//Make sure that the cliques cover all nodes in the graph or else this algorithm runs into an endless loop. For best results
//take the cliques from a maximal-clique finder like the Bron-Kerbosch algorithm.

class setCoverSolver
{
protected:
	//function to merge groups together, which have at least one node in common
	std::vector<std::vector<int> > mergeGroups(const std::vector<std::vector<int> >& found_groups);

public:
	setCoverSolver();

	std::vector<std::vector<int> > solveSetCover(const std::vector<std::vector<int> >& given_cliques, const int number_of_nodes);
};

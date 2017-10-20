#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

#include <ipa_building_navigation/contains.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

#pragma once //make sure this header gets included only one time when multiple classes need it in the same project
			 //regarding to https://en.wikipedia.org/wiki/Pragma_once this is more efficient than #define

//This algorithm provides a class that finds all maximal cliques in a given graph. It uses the Bron-Kerbosch Implementation
//in Boost to do this. As input a symmetrical distance-Matrix is needed that shows the pathlenghts from one node to another.
//If the path from one node to another doesn't exist, the entry in the matrix must be 0 or smaller. so the format for this
//Matrix is:
// row: node to start from, column: node to go to
//		---					   ---
//		| 0.0 1.0  3.5  5.8  1.2 |
//		| 1.0 0.0  2.4  3.3  9.0 |
//		| 3.5 2.4  0.0 	7.7  88.0|
//		| 5.8 3.3  7.7  0.0  0.0 |
//		| 1.2 9.0  88.0 0.0  0.0 |
//		---					   ---
//It also needs a maximal pathlength that shows the algorithm when the paths are too long and should not be included in the
//Graph. This is neccessary because we assume that you can reach every node from the others and with a Graph in which all
//nodes are connected the only maximal clique are the nodes itself. For example the previous matrix gets
//	maxval = 7.0
//		---					   ---
//		| 0.0 1.0  3.5  5.8  1.2 |
//		| 1.0 0.0  2.4  3.3  0.0 |
//		| 3.5 2.4  0.0 	0.0  0.0 |
//		| 5.8 3.3  0.0  0.0  0.0 |
//		| 1.2 0.0  0.0  0.0  0.0 |
//		---					   ---
//
//The nodes in the graph are named after their position in the distance-Matrix and the cliques are
// std::vector<int> variables so you can easily acces the right nodes in the matrix outside this class.

class cliqueFinder
{
protected:
	//function that allows to add a Vertex with a name to a boost::Graph. See boosts helper.hpp ( http://www.boost.org/doc/libs/1_47_0/libs/graph/example/helper.hpp ) for explanation
	template<typename Graph, typename NameMap, typename VertexMap>
	typename boost::graph_traits<Graph>::vertex_descriptor addNamedVertex(Graph& g, NameMap nm, const std::string& name, VertexMap& vm);

	//function to create a graph out of the given distanceMatrix
	template<typename Graph>
	void createGraph(Graph& graph, cv::Mat& distance_Matrix);

	//function to set too long paths in the distance_matrix to zero
	void cutTooLongEdges(cv::Mat& complete_distance_matrix, double maxval);

public:
	cliqueFinder();

	std::vector<std::vector<int> > getCliques(const cv::Mat& distance_matrix, double maxval);
};

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

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

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

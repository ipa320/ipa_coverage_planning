#include <ipa_building_navigation/maximal_clique_finder.h>

using namespace std;
using namespace boost;

//
//***********************Maximal Clique Finder*****************************
//
//This class provides a maximal clique-finder for a given Graph that finds all maximal cliques in this. A maximal clique
//is a subgraph in the given Graph, in which all Nodes are connected to each other and cannot be enlarged by adding other
//Nodes ( https://en.wikipedia.org/wiki/Maximum_clique ). It uses the c++ Boost library by using the implementen Bron-Kerbosch
//function and defining the Graph as a boost::undirected_Graph. Due of a missing documentation for this function see
//
//		http://www.boost.org/doc/libs/1_56_0/libs/graph/example/bron_kerbosch_print_cliques.cpp
//		http://www.boost.org/doc/libs/1_58_0/libs/graph/example/bron_kerbosch_clique_number.cpp
//		http://www.boost.org/doc/libs/1_47_0/libs/graph/example/helper.hpp
//		http://stackoverflow.com/questions/23299406/maximum-weighted-clique-in-a-large-graph-with-high-density
//
//for further information.
//As input this function takes a symmetrical Matrix that stores the pathlengths from one node of the graph to another.
//If one Node has no connection to another the element in the matrix is zero, it also is at the main-diagonal.
//!!!!!!!!!!!!!See maximal_clique_finder.h for further information on formatting.!!!!!!!!!!!!!

static std::vector<std::vector<int> > names_; //vector to save the cliques achieved by Boost

// The Actor type stores the name of each vertex in the graph.
struct Actor
{
	string name;
};

//define some types that are used for the boost function
typedef undirected_graph<Actor> Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef graph_traits<Graph>::edge_descriptor Edge;
typedef property_map<Graph, string Actor::*>::type NameMap;

template<typename Graph, typename NameMap, typename VertexMap>
typename graph_traits<Graph>::vertex_descriptor cliqueFinder::addNamedVertex(Graph& g, NameMap nm, const string& name, VertexMap& vm)
{
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	typedef typename VertexMap::iterator Iterator;

	Vertex v;
	Iterator iter;
	bool inserted;
	boost::tie(iter, inserted) = vm.insert(make_pair(name, Vertex()));
	if (inserted)
	{
		// The name was unique so we need to add a vertex to the graph
		v = add_vertex(g);
		iter->second = v;
		put(nm, v, name); // store the name in the name map
	}
	else
	{
		// We had already inserted this name so we can return the
		// associated vertex.
		v = iter->second;
	}
	return v;
}

//This class is the Visitor for all nodes in the graph, which is used by boost::bron_kerbosch. the function clique() gets
//called everytime a maximal clique was found. This comes from the boost implementation.
template<typename OutputStream>
struct visitor
{
	vector<int> current_names;
	visitor()
	{

	}

	template<typename Clique, typename Graph>
	void clique(const Clique& c, const Graph& g)
	{
		// Iterate over the clique and print each vertex within it.
		typename Clique::const_iterator i, end = c.end();
		for (i = c.begin(); i != end; ++i)
		{
			current_names.push_back(atoi(g[*i].name.c_str()));
		}
		names_.push_back(current_names); //save the clique to return all later
		current_names.clear(); //clear the current clique
	}
};

cliqueFinder::cliqueFinder()
{

}

//This function creates a boost::undirected_graph out of the cutted distance-Matrix. The Graph is used for the
//boost::bron-kerbosch algorihtm.
template<typename Graph>
void cliqueFinder::createGraph(Graph& graph, cv::Mat& distance_Matrix)
{
	vector<Vertex> vertexes;
	NameMap nmap(get(&Actor::name, graph));
	std::map<std::string, Vertex> vert_map;
	//add all Nodes to the graph
	for (int counter = 0; counter < distance_Matrix.cols; counter++)
	{
		stringstream current_name;
		current_name << counter; //<< counter;
		Vertex current_vertex = addNamedVertex(graph, nmap, current_name.str(), vert_map);
		vertexes.push_back(current_vertex);
	}
	//add each Edge if there is a connection between the Nodes
	for (int current_vertex = 0; current_vertex < vertexes.size(); current_vertex++)
	{
		for (int neighbor_node = current_vertex; neighbor_node < vertexes.size(); neighbor_node++)
		{
			if (distance_Matrix.at<double>(current_vertex, neighbor_node) > 0)
			{
				add_edge(vertexes[current_vertex], vertexes[neighbor_node], graph);
			}
		}
	}
}

//This function cuts an edge if the distance between the two Nodes is too large. This is neccessary to find possible
//areas in the graph for cliques. If the complete graph is connected only one clique will be found, containing all
//Nodes in the graph, which isn't very useful for planning.
void cliqueFinder::cutTooLongEdges(cv::Mat& complete_distance_matrix, double maxval)
{
	for (int row = 0; row < complete_distance_matrix.rows; row++)
	{
		for (int col = 0; col < complete_distance_matrix.cols; col++)
		{
			if (complete_distance_matrix.at<double>(row, col) > maxval)
			{
				complete_distance_matrix.at<double>(row, col) = 0;
			}
		}
	}
}

//This function uses the previously described functions and finds all maximal cliques in a given graph. The maxval parameter
//is used to cut edges that are too long. See maximal_clique_finder.h for further information on formatting.
std::vector<std::vector<int> > cliqueFinder::getCliques(const cv::Mat& distance_matrix, double maxval)
{
	// Create a graph object
	Graph g;

	//cut the edges if they are too long
	cv::Mat cutted_distance_matrix = distance_matrix.clone();
	cutTooLongEdges(cutted_distance_matrix, maxval);

	//Create a graph out of the cutted distance matrix
	createGraph(g, cutted_distance_matrix);

	//Object needed from boost to return the results
	visitor<ostream> vis;

	// Use the Bron-Kerbosch algorithm to find all cliques
	bron_kerbosch_all_cliques(g, vis);

	//Make sure that nodes, which are too far away from other nodes are in the clique-vector
	//(a clique has at least two nodes, but in this case it is neccessary to have all nodes in the cliques and
	//nodes that are too far away from others count also as a possible group)
	for (int node = 0; node < distance_matrix.rows; node++)
	{
		bool add = true;
		for (int group = 0; group < names_.size(); group++)
		{
			if (contains(names_[group], node))
			{
				add = false;
			}
		}
		if(add)
		{
			std::vector<int> adding_vector;
			adding_vector.push_back(node);
			names_.push_back(adding_vector);
		}
	}

	//save the names_ vector and clear it for next usage
	std::vector<std::vector<int> > returning_vector(names_);
	names_.clear();

	return returning_vector;
}

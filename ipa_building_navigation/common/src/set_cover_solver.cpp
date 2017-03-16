#include <ipa_building_navigation/set_cover_solver.h>

//Default constructor
SetCoverSolver::SetCoverSolver()
{

}

////Function to construct the symmetrical distance matrix from the given points. The rows show from which node to start and
////the columns to which node to go. If the path between nodes doesn't exist or the node to go to is the same as the one to
////start from, the entry of the matrix is 0.
//void SetCoverSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map,
//		const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution)
//{
//	//create the distance matrix with the right size
//	cv::Mat pathlengths(cv::Size((int)points.size(), (int)points.size()), CV_64F);
//
//	// reduce image size already here to avoid resizing in the planner each time
//	const double one_by_downsampling_factor = 1./downsampling_factor;
//	cv::Mat downsampled_map;
//	pathplanner_.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);
//
//	for (int i = 0; i < points.size(); i++)
//	{
//		cv::Point current_center = downsampling_factor * points[i];
//		for (int j = 0; j < points.size(); j++)
//		{
//			if (j != i)
//			{
//				if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
//				{
//					cv::Point neighbor = downsampling_factor * points[j];
//					double length = one_by_downsampling_factor * pathplanner_.planPath(downsampled_map, current_center, neighbor, 1., 0., map_resolution);
//					pathlengths.at<double>(i, j) = length;
//					pathlengths.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computationtime
//				}
//			}
//			else
//			{
//				pathlengths.at<double>(i, j) = 0;
//			}
//		}
//	}
//
//	distance_matrix = pathlengths.clone();
//}

//This function takes a vector of found nodes and merges them together, if they have at least one node in common.
std::vector<std::vector<int> > SetCoverSolver::mergeGroups(const std::vector<std::vector<int> >& found_groups)
{
	std::vector < std::vector<int> > merged_groups; //The merged groups.

	std::vector<int> done_groups; //Vector to remember which groups has already been looked at

	for (int current_group = 0; current_group < found_groups.size(); current_group++)
	{
		if (!contains(done_groups, current_group)) //If the group is in the done-vector don't look at it, because it has already been looked at.
		{
			done_groups.push_back(current_group); //add the current group to the done groups

			std::vector<int> current_group_saver(found_groups[current_group]); //vector to save the current group

			std::vector<int> merging_candidates; //vector to save the groups which should be merged with the current group

			//check if the current group has at least one node in common with the other groups
			for (int next_group = 0; next_group < found_groups.size(); next_group++)
			{
				bool merge = false; //variable to check if the room should be added to the current group

				//if it is the same group or has already been looked at it doesn't need to be checked
				if (next_group != current_group && !contains(done_groups, next_group))
				{
					for (int node = 0; node < found_groups[next_group].size(); node++)
					{
						if (contains(found_groups[current_group], found_groups[next_group][node]))
						{
							merge = true;
						}
					}
				}
				if (merge) //If the group has at least one neighbor save it for merging
				{
					merging_candidates.push_back(next_group);
				}
			}

			//Add the merging-candidates nodes to the current group and add the candidates to the done groups
			for (int merge_candidate = 0; merge_candidate < merging_candidates.size(); merge_candidate++)
			{
				done_groups.push_back(merging_candidates[merge_candidate]);
				for (int node = 0; node < found_groups[merging_candidates[merge_candidate]].size(); node++)
				{
					if (!contains(current_group_saver, found_groups[merging_candidates[merge_candidate]][node]))
					{
						current_group_saver.push_back(found_groups[merging_candidates[merge_candidate]][node]);
					}
				}
			}
			//add the merged group to the vector
			merged_groups.push_back(current_group_saver);
		}
	}
	std::cout << "Finished merging." << std::endl;
	return merged_groups;
}

//This functions solves the set-cover Problem ( https://en.wikipedia.org/wiki/Set_cover_problem#Greedy_algorithm ) using
//the greedy-search algorithm. It chooses the clique that has the most uncovered nodes in it first. Then it uses the merge-function
//above to merge groups that have at least one node in common together. The vector stores the indexes of the nodes, which
//are the same as the ones from the clique-solver and also the distance-matrix. The variable max_number_of_clique_members
//implies how many members a clique is allowed to have.

//the cliques are given
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(std::vector<std::vector<int> >& given_cliques,
		const int number_of_nodes, const int max_number_of_clique_members, const cv::Mat& distance_matrix)
{
	std::vector < std::vector<int> > minimal_set;

	std::vector<std::vector<int> > cliques_for_covering;

	for(size_t clique = 0; clique < given_cliques.size(); clique++)
	{
		std::vector<int> temporary_clique;
		for(size_t node = 0; node < given_cliques[clique].size(); node++)
		{
			temporary_clique.push_back(given_cliques[clique][node]);
		}
		cliques_for_covering.push_back(temporary_clique);
	}

	//Put the nodes in a open-nodes vector. The nodes are named after their position in the room-centers-vector and so every
	//node from 0 to number_of_nodes-1 is in the Graph.
	std::vector<int> open_nodes;
	for (int new_node = 0; new_node < number_of_nodes; new_node++)
	{
		open_nodes.push_back(new_node);
	}

	std::cout << "Starting greedy search for set-cover-problem." << std::endl;

	//Search for the clique with the most unvisited nodes and choose this one before the others. Then remove the nodes of
	//this clique from the unvisited-vector. This is done until no more nodes can be visited.

	do
	{
		int covered_open_nodes;
		int best_covered_counter = 0;
		int best_clique = -1;
		for (int clique = 0; clique < cliques_for_covering.size(); clique++)
		{
			// skip too big cliques
			if(cliques_for_covering[clique].size() > max_number_of_clique_members)
				continue;

			covered_open_nodes = 0;
			for (int node = 0; node < cliques_for_covering[clique].size(); node++)
			{
				if (contains(open_nodes, cliques_for_covering[clique][node]))
				{
					covered_open_nodes++;
				}
			}
			if (covered_open_nodes > best_covered_counter)
			{
				best_covered_counter = covered_open_nodes;
				best_clique = clique;
			}
		}

		// check if a allowed clique could be found, if not split the biggest clique until it consists of cliques that are of the
		// allowed size
		if(best_clique == -1)
		{
			for (int clique = 0; clique < cliques_for_covering.size(); clique++)
			{
				covered_open_nodes = 0;
				for (int node = 0; node < cliques_for_covering[clique].size(); node++)
				{
					if (contains(open_nodes, cliques_for_covering[clique][node]))
					{
						covered_open_nodes++;
					}
				}
				if (covered_open_nodes > best_covered_counter)
				{
					best_covered_counter = covered_open_nodes;
					best_clique = clique;
				}
			}

			// save big clique
			std::vector<int> big_clique = cliques_for_covering[best_clique];

			// iteratively remove nodes far away from the remaining nodes to create small cliques
			bool removed_node = false;
			std::vector<std::vector<int> > found_subgraphs;
			do
			{
				removed_node = false; // reset checking boolean
				std::vector<int> current_subgraph = big_clique;
				while(current_subgraph.size() > max_number_of_clique_members)
				{
					removed_node = true;

					// find the node farthest away from the other nodes
					double max_distance = 0.0;
					int worst_node = -1;
					for(size_t node = 0; node < current_subgraph.size(); ++node)
					{
						// compute sum of distances from current node to neighboring nodes
						double current_distance = 0;
						for(size_t neighbor = 0; neighbor < current_subgraph.size(); ++neighbor)
						{
							// don't look at node itself
							if(node == neighbor)
								continue;

							current_distance += distance_matrix.at<double>(current_subgraph[node], current_subgraph[neighbor]);
						}

						// check if sum of distances is worse than the previously found ones
						if(current_distance > max_distance)
						{
							worst_node = node;
							max_distance = current_distance;
						}
					}

					// remove the node farthest away from all other nodes out of the subgraph
					current_subgraph.erase(current_subgraph.begin() + worst_node);
				}

				// save the found subgraph
				found_subgraphs.push_back(current_subgraph);

				// erase the covered nodes from the big clique
				for(size_t node = 0; node < current_subgraph.size(); ++node)
					big_clique.erase(std::remove(big_clique.begin(), big_clique.end(), current_subgraph[node]), big_clique.end());

			}while(removed_node == true && big_clique.size() > 0);

			// add found subgraphs to the minimal set
			for(size_t subgraph = 0; subgraph < found_subgraphs.size(); ++subgraph)
				minimal_set.push_back(found_subgraphs[subgraph]);
		}
		else
		{
			minimal_set.push_back(cliques_for_covering[best_clique]);
		}
		//remove nodes of best clique from all cliques (this is okay because if you remove a node from a clique it stays a clique, it only isn't a maximal clique anymore)
		for(size_t clique = 0; clique < cliques_for_covering.size(); clique++)
		{
			for(int node = 0; node < cliques_for_covering[best_clique].size(); node++)
			{
				if(clique != best_clique)
					cliques_for_covering[clique].erase(std::remove(cliques_for_covering[clique].begin(), cliques_for_covering[clique].end(), cliques_for_covering[best_clique][node]), cliques_for_covering[clique].end());
			}
		}
		for (int node = 0; node < cliques_for_covering[best_clique].size(); node++)
		{
			open_nodes.erase(std::remove(open_nodes.begin(), open_nodes.end(), cliques_for_covering[best_clique][node]), open_nodes.end());
		}
//		cliques_for_covering = new_set;
//		std::cout << open_nodes.size() << std::endl;
	} while (open_nodes.size() > 0);

	std::cout << "Finished greedy search." << std::endl;

//	std::cout << "Starting merging the found groups." << std::endl;
//
//	return mergeGroups(minimal_set);
	return minimal_set;
}

//the distance matrix is given, but not the cliques
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(const cv::Mat& distance_matrix, const std::vector<cv::Point>& points,
		const int number_of_nodes, double maximal_pathlength, const int max_number_of_clique_members)
{
	//get all maximal cliques for this graph
	std::vector < std::vector<int> > maximal_cliques = maximal_clique_finder.getCliques(distance_matrix, maximal_pathlength);

	//put the single nodes in the maximal cliques vector to make sure every node gets covered from the setCover solver
	for(int single_node = 0; single_node < number_of_nodes; single_node++)
	{
		std::vector<int> temp;
		temp.push_back(single_node);
		maximal_cliques.push_back(temp);
	}

	return (solveSetCover(maximal_cliques, number_of_nodes, max_number_of_clique_members, distance_matrix));
}

//the distance matrix and cliques aren't given and the matrix should not be returned
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, double maximal_pathlength, const int max_number_of_clique_members, cv::Mat* distance_matrix)
{
	//calculate the distance matrix
	cv::Mat distance_matrix_ref;
	if (distance_matrix != 0)
		distance_matrix_ref = *distance_matrix;
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.constructDistanceMatrix(distance_matrix_ref, original_map, points, downsampling_factor, robot_radius, map_resolution, pathplanner_);

	//get all maximal cliques for this graph and solve the set cover problem
	return (solveSetCover(distance_matrix_ref, points, (int)points.size(), maximal_pathlength, max_number_of_clique_members));
}

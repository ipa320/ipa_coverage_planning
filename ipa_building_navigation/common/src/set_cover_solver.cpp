#include <ipa_building_navigation/set_cover_solver.h>

//Default constructor
SetCoverSolver::SetCoverSolver()
{

}

//Function to construct the symmetrical distance matrix from the given points. The rows show from which node to start and
//the columns to which node to go. If the path between nodes doesn't exist or the node to go to is the same as the one to
//start from, the entry of the matrix is 0.

void SetCoverSolver::constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map,
		const std::vector<cv::Point>& points, double downsampling_factor, double robot_radius, double map_resolution)
{
	//create the distance matrix with the right size
	cv::Mat pathlengths(cv::Size((int)points.size(), (int)points.size()), CV_64F);

	// reduce image size already here to avoid resizing in the planner each time
	const double one_by_downsampling_factor = 1./downsampling_factor;
	cv::Mat downsampled_map;
	pathplanner_.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

	for (int i = 0; i < points.size(); i++)
	{
		cv::Point current_center = downsampling_factor * points[i];
		for (int j = 0; j < points.size(); j++)
		{
			if (j != i)
			{
				if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
				{
					cv::Point neighbor = downsampling_factor * points[j];
					double length = one_by_downsampling_factor * pathplanner_.planPath(downsampled_map, current_center, neighbor, 1., 0., map_resolution);
					pathlengths.at<double>(i, j) = length;
					pathlengths.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computationtime
				}
			}
			else
			{
				pathlengths.at<double>(i, j) = 0;
			}
		}
	}

	distance_matrix = pathlengths.clone();
}

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
//are the same as the ones from the clique-solver and also the distance-matrix.

//the cliques are given
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(const std::vector<std::vector<int> >& given_cliques, const int number_of_nodes)
{
	std::vector < std::vector<int> > minimal_set;

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
		int best_clique;
		for (int clique = 0; clique < given_cliques.size(); clique++)
		{
			covered_open_nodes = 0;
			for (int node = 0; node < given_cliques[clique].size(); node++)
			{
				if (contains(open_nodes, given_cliques[clique][node]))
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
		minimal_set.push_back(given_cliques[best_clique]);
		for (int node = 0; node < given_cliques[best_clique].size(); node++)
		{
			open_nodes.erase(std::remove(open_nodes.begin(), open_nodes.end(), given_cliques[best_clique][node]), open_nodes.end());
		}
	} while (open_nodes.size() > 0);

	std::cout << "Finished greedy search." << std::endl;

	std::cout << "Starting merging the found groups." << std::endl;

	return mergeGroups(minimal_set);
}

//the distance matrix is given, but not the cliques
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(const cv::Mat& distance_matrix, const int number_of_nodes, double maximal_pathlength)
{
	//get all maximal cliques for this graph
	std::vector < std::vector<int> > maximal_cliques = maximal_clique_finder.getCliques(distance_matrix, maximal_pathlength);

	return (solveSetCover(maximal_cliques, number_of_nodes));
}

//the distance matrix and cliques aren't given and the matrix should not be returned
std::vector<std::vector<int> > SetCoverSolver::solveSetCover(const cv::Mat& original_map, const std::vector<cv::Point>& points,
		double downsampling_factor, double robot_radius, double map_resolution, double maximal_pathlength, cv::Mat* distance_matrix)
{
	//calculate the distance matrix
	cv::Mat distance_matrix_ref;
	if (distance_matrix != 0)
		distance_matrix_ref = *distance_matrix;
	constructDistanceMatrix(distance_matrix_ref, original_map, points, downsampling_factor, robot_radius, map_resolution);

	//get all maximal cliques for this graph and solve the set cover problem
	return (solveSetCover(distance_matrix_ref, (int)points.size(), maximal_pathlength));
}

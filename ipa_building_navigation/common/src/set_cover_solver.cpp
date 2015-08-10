#include <ipa_building_navigation/set_cover_solver.h>

setCoverSolver::setCoverSolver()
{

}

//This function takes a vector of found nodes and merges them together, if they have at least one node in common.
std::vector<std::vector<int> > setCoverSolver::mergeGroups(const std::vector<std::vector<int> >& found_groups)
{
	std::vector < std::vector<int> > merged_groups; //The merged groups.

	std::vector<int> done_groups; //Vector to remember which groups has already been looked at

	for (int current_group = 0; current_group < found_groups.size(); current_group++)
	{
		if (!contains(done_groups, current_group)) //If the group is in the done-vector don't look at it, because it has already been looked at.
		{
			done_groups.push_back(current_group); //add the current group to the done groups

			std::vector<int> current_group_saver (found_groups[current_group]); //vector to save the current group

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
				if(merge) //If the group has at least one neighbor save it for merging
				{
					merging_candidates.push_back(next_group);
				}
			}

			//Add the merging-candidates nodes to the current group and add the candidates to the done groups
			for(int merge_candidate = 0; merge_candidate < merging_candidates.size(); merge_candidate++)
			{
				done_groups.push_back(merging_candidates[merge_candidate]);
				for(int node = 0; node < found_groups[merging_candidates[merge_candidate]].size(); node++)
				{
					if(!contains(current_group_saver, found_groups[merging_candidates[merge_candidate]][node]))
					{
						current_group_saver.push_back(found_groups[merging_candidates[merge_candidate]][node]);
					}
				}
			}
			//add the merged group to the vector
			merged_groups.push_back(current_group_saver);
		}
	}
	return merged_groups;
}

//This function solves the set-cover Problem ( https://en.wikipedia.org/wiki/Set_cover_problem#Greedy_algorithm ) using
//the greedy-search algorithm. It chooses the clique that has the most uncovered nodes in it first. Then it uses the merge-function
//above to merge groups that have at least one node in common together. The vector stores the indexes of the nodes, which
//are the same as the ones from the clique-solver and also the distance-matrix.
std::vector<std::vector<int> > setCoverSolver::solveSetCover(const std::vector<std::vector<int> >& given_cliques, const std::vector<int>& nodes)
{
	std::vector < std::vector<int> > minimal_set;

	std::vector<int> open_nodes = nodes;

	ROS_INFO("Starting greedy search for set-cover-problem.");

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

	ROS_INFO("Finished greedy search.");

	ROS_INFO("Starting merging the found groups.");

	return mergeGroups(minimal_set);
}

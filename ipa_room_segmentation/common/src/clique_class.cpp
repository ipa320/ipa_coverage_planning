#include <ipa_room_segmentation/clique_class.h>

//
// This class is used to easily create cliques that are subgraphs of a large graph.A Clique is a subgraph of this graph, in which
// all nodes are connected to each other. See the Header for further information.
//

// default constructor
Clique::Clique()
{

}

// constructor if one member is known
Clique::Clique(cv::Point first_member)
{
	member_points.push_back(first_member);
}

// constructor if a couple of members are known
Clique::Clique(std::vector<cv::Point> members)
{
	for(size_t member = 0; member < members.size(); ++member)
	{
		member_points.push_back(members[member]);
	}
}

// function that returns a vector containing all member points
std::vector<cv::Point> Clique::getMemberPoints()
{
	return member_points;
}

// function that inserts a single point as a new member
void Clique::insertMember(cv::Point& new_member)
{
	if(contains(member_points, new_member) == false)
		member_points.push_back(new_member);
}

// function that inserts multiple points as new members
void Clique::insertMember(std::vector<cv::Point>& new_members)
{
	for(std::vector<cv::Point>::iterator new_member = new_members.begin(); new_member != new_members.end(); ++new_member)
	{
		if(contains(member_points, *new_member) == false)
			member_points.push_back(*new_member);
	}
}

// function that returns the number of members of this clique
unsigned int Clique::getNumberOfMembers()
{
	return member_points.size();
}

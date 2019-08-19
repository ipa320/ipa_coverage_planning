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
	member_points_.push_back(first_member);
}

// constructor if a couple of members are known
Clique::Clique(std::vector<cv::Point> members)
{
	for(size_t member = 0; member < members.size(); ++member)
	{
		member_points_.push_back(members[member]);
	}
}

// function that returns a vector containing all member points
std::vector<cv::Point> Clique::getMemberPoints()
{
	return member_points_;
}

// function that inserts a single point as a new member
void Clique::insertMember(cv::Point& new_member)
{
	if(contains(member_points_, new_member) == false)
		member_points_.push_back(new_member);
}

// function that inserts multiple points as new members
void Clique::insertMember(std::vector<cv::Point>& new_members)
{
	for(std::vector<cv::Point>::iterator new_member = new_members.begin(); new_member != new_members.end(); ++new_member)
	{
		if(contains(member_points_, *new_member) == false)
			member_points_.push_back(*new_member);
	}
}

// function to check if a given point is part of the clique
bool Clique::containsMember(const cv::Point& point)
{
	return contains(member_points_, point);
}

// function that returns the number of members of this clique
unsigned int Clique::getNumberOfMembers()
{
	return member_points_.size();
}

// function to save the given beams in the class parameter
void Clique::setBeamsForMembers(const std::vector< std::vector<double> > beams)
{
	beams_for_members_ = beams ;
}

// function that returns the stored laser-beams
std::vector< std::vector<double> > Clique::getBeams()
{
	return beams_for_members_;
}

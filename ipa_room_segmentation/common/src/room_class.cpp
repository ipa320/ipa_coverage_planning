#include <ipa_room_segmentation/room_class.h>

Room::Room(int id_of_room)
{
	id_number_ = id_of_room;
	//initial values for the area and perimeter
	room_area_ = 0;
	room_perimeter_ = 0;
}

void Room::mergeRoom(Room& room_to_merge, double map_resolution)
{
	// member_points_, room_area_
	insertMemberPoints(room_to_merge.getMembers(), map_resolution);

	// neighbor_room_ids_
	const std::vector<int>& neighbor_ids = room_to_merge.getNeighborIDs();

//	std::cout << "neighbor_room_ids_:\n";
//	for (size_t i=0; i<neighbor_room_ids_.size(); ++i)
//		std::cout << neighbor_room_ids_[i] << "\n";
//	std::cout << "neighbor_ids:\n";
//		for (size_t i=0; i<neighbor_ids.size(); ++i)
//			std::cout << neighbor_ids[i] << "\n";

	for (size_t i=0; i<neighbor_ids.size(); ++i)
	{
		if (!contains(neighbor_room_ids_, neighbor_ids[i]) && neighbor_ids[i]!=id_number_)
			neighbor_room_ids_.push_back(neighbor_ids[i]);
	}
	for (std::vector<int>::iterator it = neighbor_room_ids_.begin(); it != neighbor_room_ids_.end();)
	{
		if (*it == id_number_ || *it == room_to_merge.getID())
			neighbor_room_ids_.erase(it);
		else
			++it;
	}

//	std::cout << "neighbor_room_ids_after_merge:\n";
//	for (size_t i=0; i<neighbor_room_ids_.size(); ++i)
//		std::cout << neighbor_room_ids_[i] << "\n";

	// neighbor_room_statistics_
	for (std::map<int,int>::const_iterator it = room_to_merge.getNeighborStatistics().begin(); it != room_to_merge.getNeighborStatistics().end(); ++it)
	{
		if (it->first != id_number_)
		{
			if (neighbor_room_statistics_.find(it->first) != neighbor_room_statistics_.end())
				neighbor_room_statistics_[it->first] += it->second;
			else
				neighbor_room_statistics_[it->first] = it->second;
		}
	}
	neighbor_room_statistics_.erase(room_to_merge.getID());
}

//function to add a Point to the Room
int Room::insertMemberPoint(cv::Point new_member, double map_resolution)
{
	if (!contains(member_points_, new_member))
	{
		member_points_.push_back(new_member);
		room_area_ += map_resolution * map_resolution;
		return 0;
	}
	return 1;
}

//function to add a few Points
int Room::insertMemberPoints(const std::vector<cv::Point>& new_members, double map_resolution)
{
	for (size_t point = 0; point < new_members.size(); point++)
	{
		if (!contains(member_points_, new_members[point]))
		{
			member_points_.push_back(new_members[point]);
		}
	}
	room_area_ += map_resolution * map_resolution * new_members.size();		// todo: theoretically might count too much area if member_points_ and new_members are not disjunct
	return 0;
}

//function to add a neighbor to the room statistics
void Room::addNeighbor(int new_neighbor_id)
{
	if (neighbor_room_statistics_.find(new_neighbor_id) == neighbor_room_statistics_.end())
		neighbor_room_statistics_[new_neighbor_id] = 1;
	else
		neighbor_room_statistics_[new_neighbor_id]++;
}

//function to add a neighbor to the Room
int Room::addNeighborID(int new_neighbor_id)
{
	if (!contains(neighbor_room_ids_, new_neighbor_id))
	{
		neighbor_room_ids_.push_back(new_neighbor_id);
		return 0;
	}
	return 1;
}

//function to get how many neighbors this room has
int Room::getNeighborCount()
{
	return neighbor_room_ids_.size();
}

std::map<int,int>& Room::getNeighborStatistics()
{
	return neighbor_room_statistics_;
}

void Room::getNeighborStatisticsInverse(std::map< int,int,std::greater<int> >& neighbor_room_statistics_inverse)
{
	//std::map< int,int,std::greater<int> > neighbor_room_statistics_inverse;	// common border length, room_id
	for (std::map<int,int>::iterator it=neighbor_room_statistics_.begin(); it!=neighbor_room_statistics_.end(); ++it)
		neighbor_room_statistics_inverse[it->second] = it->first;
}

int Room::getNeighborWithLargestCommonBorder(bool exclude_wall)
{
	if (neighbor_room_statistics_.size() == 0)
		return 0;

	std::map< int,int,std::greater<int> > neighbor_room_statistics_inverse;	// common border length, room_id
	for (std::map<int,int>::iterator it=neighbor_room_statistics_.begin(); it!=neighbor_room_statistics_.end(); ++it)
		neighbor_room_statistics_inverse[it->second] = it->first;

	if (exclude_wall == true && neighbor_room_statistics_inverse.begin()->second==0 && neighbor_room_statistics_inverse.size() > 1)
	{
		std::map<int,int>::iterator it = neighbor_room_statistics_inverse.begin();
		it++;
		return it->second;
	}

	return neighbor_room_statistics_inverse.begin()->second;
}

double Room::getPerimeterRatioOfXLargestRooms(const int number_rooms)
{
	if (neighbor_room_statistics_.size() == 0)
		return 0;

	std::map< int,int,std::greater<int> > neighbor_room_statistics_inverse;	// common border length, room_id
	for (std::map<int,int>::iterator it=neighbor_room_statistics_.begin(); it!=neighbor_room_statistics_.end(); ++it)
		neighbor_room_statistics_inverse[it->second] = it->first;

	int counter = 0;
	double value = 0.;
	for (std::map<int,int>::iterator it=neighbor_room_statistics_inverse.begin(); it!=neighbor_room_statistics_inverse.end() && counter<number_rooms; ++it)
	{
		value += it->first;
		if (it->second != 0)
			counter++;
	}

	return value/getPerimeter();
}

double Room::getWallToPerimeterRatio()
{
	double value = 0.;
	if (neighbor_room_statistics_.find(0) != neighbor_room_statistics_.end())
		value = neighbor_room_statistics_[0]/getPerimeter();

	return value;
}

std::vector<int>& Room::getNeighborIDs()
{
	return neighbor_room_ids_;
}

//function to get the area of this room, which has been set previously
double Room::getArea()
{
	if (room_area_ != 0)
	{
		return room_area_;
	}
	std::cout << "Warning: Room Area hasn't been set for this room." << std::endl;
	return -1;
}

//function to get the perimeter of this room, which has been set previously
double Room::getPerimeter()
{
//	if (room_perimeter_ != 0)
//	{
//		return room_perimeter_;
//	}
//	std::cout << "Warning: Room Perimeter hasn't been set for this room." << std::endl;
	room_perimeter_ = 0.;
	for (std::map<int,int>::iterator it=neighbor_room_statistics_.begin(); it!=neighbor_room_statistics_.end(); ++it)
		room_perimeter_ += it->second;

	return room_perimeter_;
}

//function to get the ID number of this room
int Room::getID() const
{
	return id_number_;
}

cv::Point Room::getCenter()
{
	cv::Scalar center = cv::mean(member_points_);
	return cv::Point(center[0], center[1]);
}

//function to get the Members of this room
const std::vector<cv::Point>& Room::getMembers()
{
	if (member_points_.size() == 0)
	{
		std::cout << "Warning: This room has no members." << std::endl;
	}
	return member_points_;
}

//This function sets the room ID to a different value. This is useful for merging different rooms together.
int Room::setRoomId(int new_value, cv::Mat& map)
{
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			if (map.at<int>(y, x) == id_number_)
			{
				map.at<int>(y, x) = new_value;
			}
		}
	}
	id_number_ = new_value;
	return 0;
}

//function to set the area of this room
int Room::setArea(double room_area)
{
	room_area_ = room_area;
	return 0;
}

//function to set the perimeter of this room
int Room::setPerimeter(double room_perimeter)
{
	room_perimeter_ = room_perimeter;
	return 0;
}

bool sortRoomsAscending(Room a, Room b)
{
	return (a.getArea() < b.getArea());
}

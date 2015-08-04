#include <ipa_room_segmentation/room_class.h>

Room::Room(int id_of_room)
{
	id_number_ = id_of_room;
	//initial values for the area and perimeter
	room_area_ = -1;
	room_perimeter_ = -1;
}

//function to add a Point to the Room
int Room::insertMemberPoint(cv::Point new_member)
{
	if (!contains(member_Points_, new_member))
	{
		member_Points_.push_back(new_member);
		return 0;
	}
	return 1;
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

std::vector<int> Room::getNeighborIDs()
{
	return neighbor_room_ids_;
}

//function to get the area of this room, which has been set previously
double Room::getArea()
{
	if (room_area_ != -1)
	{
		return room_area_;
	}
	std::cout << "Warning: Room Area hasn't been set for this room." << std::endl;
	return -1;
}

//function to get the perimeter of this room, which has been set previously
double Room::getPerimeter()
{
	if (room_perimeter_ != -1)
	{
		return room_perimeter_;
	}
	std::cout << "Warning: Room Perimeter hasn't been set for this room." << std::endl;
	return -1;
}

//function to get the ID number of this room
int Room::getID()
{
	return id_number_;
}

//function to get the Members of this room
std::vector<cv::Point> Room::getMembers()
{
	if (member_Points_.size() == 0)
	{
		std::cout << "Warning: This room has no members." << std::endl;
	}
	return member_Points_;
}

//This function sets the room ID to a different value. This is useful for merging different rooms together.
int Room::setRoomId(int new_value, cv::Mat& map)
{
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			if(map.at<int>(y, x) == id_number_)
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

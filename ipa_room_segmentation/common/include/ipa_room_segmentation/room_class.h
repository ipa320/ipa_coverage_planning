#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <list>
#include <string>
#include <vector>

#include <ipa_room_segmentation/contains.h>

//This is the class that represents a room. It has a ID-number, Points that belong to it and a list of neighbors.

class Room
{
protected:
	int id_number_;

	std::vector<cv::Point> member_Points_;

	std::vector<int> neighbor_room_ids_;

	double room_area_;

	double room_perimeter_;
public:
	Room(int id_of_room);

	int insertMemberPoint(cv::Point new_member);

	int addNeighborID(int new_neighbor_id);

	int getNeighborCount();

	std::vector<int> getNeighborIDs();

	double getArea();

	double getPerimeter();

	int getID();

	std::vector<cv::Point> getMembers();

	int setRoomId(int new_value, cv::Mat& map);

	int setArea(double room_area);

	int setPerimeter(double room_perimeter);
};

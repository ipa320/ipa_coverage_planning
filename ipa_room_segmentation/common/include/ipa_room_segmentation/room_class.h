#ifndef __ROOM_CLASS_H__
#define __ROOM_CLASS_H__

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

	std::vector<cv::Point> member_points_;

	std::vector<int> neighbor_room_ids_;

	std::map<int, int> neighbor_room_statistics_;		// maps from room labels of neighboring rooms to number of touching pixels of the respective neighboring room

	double room_area_;

	double room_perimeter_;
public:
	Room(int id_of_room);

	// merges the provided room into this room
	void mergeRoom(Room& room_to_merge, double map_resolution);

	int insertMemberPoint(cv::Point new_member, double map_resolution);

	int insertMemberPoints(const std::vector<cv::Point>& new_members, double map_resolution);

	void addNeighbor(int new_neighbor_id);

	int addNeighborID(int new_neighbor_id);

	int getNeighborCount();

	const std::map<int,int>& getNeighborStatistics();

	int getNeighborWithLargestCommonBorder(bool exclude_wall=true);

	double getWallToPerimeterRatio();

	const std::vector<int>& getNeighborIDs();

	double getArea();

	double getPerimeter();

	int getID();

	cv::Point getCenter();

	const std::vector<cv::Point>& getMembers();

	int setRoomId(int new_value, cv::Mat& map);

	int setArea(double room_area);

	int setPerimeter(double room_perimeter);
};


bool sortRoomsAscending(Room a, Room b);

#endif

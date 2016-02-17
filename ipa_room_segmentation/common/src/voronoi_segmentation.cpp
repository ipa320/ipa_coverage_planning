#include <ipa_room_segmentation/voronoi_segmentation.h>

#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>

#include <ipa_room_segmentation/timer.h>
#include <set>



struct cv_Point_comp
{
	bool operator()(const cv::Point& lhs, const cv::Point& rhs) const
	{
		return ((lhs.y < rhs.y) || (lhs.y == rhs.y && lhs.x < rhs.x));
	}
};

VoronoiSegmentation::VoronoiSegmentation()
{

}

bool VoronoiSegmentation::determineRoomIndexFromRoomID(const std::vector<Room>& rooms, const int room_id, size_t& room_index)
{
	bool found_id = false;
	for (size_t r = 0; r < rooms.size(); r++)
	{
		if (rooms[r].getID() == room_id)
		{
			room_index = r;
			found_id = true;
			break;
		}
	}

	return found_id;
}

void VoronoiSegmentation::mergeRoomPair(std::vector<Room>& rooms, const int target_index, const int room_to_merge_index, cv::Mat& segmented_map, const double map_resolution)
{
	// integrate room to merge into target and delete merged room
	const int target_id = rooms[target_index].getID();
	const int room_to_merge_id = rooms[room_to_merge_index].getID();
	rooms[target_index].mergeRoom(rooms[room_to_merge_index], map_resolution);
	rooms[room_to_merge_index].setRoomId(rooms[target_index].getID(), segmented_map);
	rooms.erase(rooms.begin()+room_to_merge_index);
	std::sort(rooms.begin(), rooms.end(), sortRoomsAscending);

	// update neighborhood statistics for remaining rooms
	for (size_t i=0; i<rooms.size(); ++i)
	{
		std::vector<int>& neighbor_ids = rooms[i].getNeighborIDs();
		std::vector<int>::iterator it = std::find(neighbor_ids.begin(), neighbor_ids.end(), room_to_merge_id);
		if (it != neighbor_ids.end())
		{
			std::vector<int>::iterator it2 = std::find(neighbor_ids.begin(), neighbor_ids.end(), target_id);
			if (it2 != neighbor_ids.end())
				neighbor_ids.erase(it);
			else
				*it = target_id;
		}

		std::map<int,int>& neighbor_statistics = rooms[i].getNeighborStatistics();
		std::map<int,int>::iterator it3 = neighbor_statistics.find(room_to_merge_id);
		if (it3 != neighbor_statistics.end())
		{
			std::map<int,int>::iterator it4 = neighbor_statistics.find(target_id);
			if (it4 != neighbor_statistics.end())
				it4->second += it3->second;
			else
				neighbor_statistics[target_id] = it3->second;
			neighbor_statistics.erase(it3);
		}
	}
}

void VoronoiSegmentation::drawVoronoi(cv::Mat &img, const std::vector<std::vector<cv::Point2f> >& facets_of_voronoi, const cv::Scalar voronoi_color, const cv::Mat& eroded_map)
{
	//This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
	//map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
	//inside a hole-contour and doesn't draw the lines that are not.
	for (std::vector<std::vector<cv::Point2f> >::const_iterator current_contour = facets_of_voronoi.begin(); current_contour != facets_of_voronoi.end(); ++current_contour)
	{
		// saving-variable for the last Point that has been looked at
		cv::Point2f last_point = current_contour->back();
		// draw each line of the voronoi-cell
		for (size_t c = 0; c < current_contour->size(); ++c)
		{
			// variable to check, whether a Point is inside a white area or not
			bool inside = true;
			cv::Point2f current_point = current_contour->at(c);
			// only draw lines that are inside the map-contour
			if (((int)current_point.x<0) || ((int)current_point.x >= eroded_map.cols) ||
				((int)current_point.y<0) || ((int)current_point.y >= eroded_map.rows) ||
				eroded_map.at<uchar>((int)current_point.y, (int)current_point.x) == 0 ||
				((int)last_point.x<0) || ((int)last_point.x >= eroded_map.cols) ||
				((int)last_point.y<0) || ((int)last_point.y >= eroded_map.rows) ||
				eroded_map.at<uchar>((int)last_point.y, (int)last_point.x) == 0)
				inside = false;
			if (inside)
			{
				cv::line(img, last_point, current_point, voronoi_color, 1);
			}
			last_point = current_point;
		}
	}
}

void VoronoiSegmentation::createVoronoiGraph(cv::Mat& map_for_voronoi_generation)
{
	//****************Create the Generalized Voronoi-Diagram**********************
	//This function is here to create the generalized voronoi-diagram in the given map. It does following steps:
	//	1. It finds every discretized contour in the given map (they are saved as vector<Point>). Then it takes these
	//	   contour-Points and adds them to the OpenCV Delaunay generator from which the voronoi-cells can be generated.
	//	2. Then it finds the largest eroded contour in the given map, which is the contour of the map itself. It searches the
	//	   largest contour, because smaller contours correspond to mapping errors (obsolete, drawVoronoi does not need the contour anymore)
	//	3. Finally it gets the boundary-Points of the voronoi-cells with getVoronoiFacetList. It takes these facets
	//	   and draws them using the drawVoronoi function. This function draws the facets that only have Points inside
	//	   the map-contour (other lines go to not-reachable places and are not necessary to be looked at).
	//	4. It returns the map that has the generalized voronoi-graph drawn in.

	Timer tim;

	cv::Mat map_to_draw_voronoi_in = map_for_voronoi_generation.clone(); //variable to save the given map for drawing in the voronoi-diagram

	cv::Mat temporary_map_to_calculate_voronoi = map_for_voronoi_generation.clone(); //variable to save the given map in the createVoronoiGraph-function

	//apply a closing-operator on the map so bad parts are neglected
	cv::erode(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());
	cv::dilate(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());

	//********************1. Get OpenCV delaunay-traingulation******************************

	cv::Rect rect(0, 0, map_to_draw_voronoi_in.cols, map_to_draw_voronoi_in.rows); //variables to generate the voronoi-diagram, using OpenCVs delaunay-triangulation
	cv::Subdiv2D subdiv(rect);

	std::vector < std::vector<cv::Point> > hole_contours; //variable to save the hole-contours (= black holes inside the white map)

	std::vector < std::vector<cv::Point> > contours; //variables for contour extraction and discretisation
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;

	//get contours of the map
	cv::Mat temp = map_to_draw_voronoi_in.clone();
	cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(map_to_draw_voronoi_in, contours, -1, cv::Scalar(255), CV_FILLED);

	//put every point of the map-contours into the Delaunay-generator of OpenCV
	for (int current_contour = 0; current_contour < contours.size(); current_contour++)
	{
		for (int current_point = 0; current_point < contours[current_contour].size(); current_point++)
		{
			cv::Point fp = contours[current_contour][current_point];
			subdiv.insert(fp);
		}
		//get the contours of the black holes --> it is necessary to check if points are inside these in drawVoronoi
		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
		{
			hole_contours.push_back(contours[current_contour]);
		}
	}

	//********************2. Erode map ******************************
	//erode the map and get the largest contour of it so that points near the boundary are not drawn later (see drawVoronoi)
	cv::Mat eroded_map;
	cv::Point anchor(-1, -1);
	cv::erode(temporary_map_to_calculate_voronoi, eroded_map, cv::Mat(), anchor, 2);

	//********************3. Get facets and draw voronoi-Graph******************************
	//get the Voronoi regions from the delaunay-subdivision graph
	cv::Scalar voronoi_color(127); //define the voronoi-drawing colour
	std::vector < std::vector<cv::Point2f> > voronoi_facets; //variables to find the facets and centers of the voronoi-cells
	std::vector < cv::Point2f > voronoi_centers;
	subdiv.getVoronoiFacetList(std::vector<int>(), voronoi_facets, voronoi_centers);

	//draw the voronoi-regions into the map
	drawVoronoi(map_to_draw_voronoi_in, voronoi_facets, voronoi_color, eroded_map); // largest_contour, hole_contours);

	//make pixels black, which were black before and were colored by the voronoi-regions
	for (int v = 0; v < map_to_draw_voronoi_in.rows; v++)
	{
		for (int u = 0; u < map_to_draw_voronoi_in.cols; u++)
		{
			if (map_for_voronoi_generation.at<unsigned char>(v, u) == 0)
			{
				map_to_draw_voronoi_in.at<unsigned char>(v, u) = 0;
			}
		}
	}
	map_for_voronoi_generation = map_to_draw_voronoi_in;
}

void VoronoiSegmentation::mergeRooms(cv::Mat& map_to_merge_rooms, std::vector<Room>& rooms, double map_resolution_from_subscription, double max_area_for_merging, bool display_map)
{
	//This function takes the segmented Map from the original Voronoi-segmentation-algorithm and merges rooms together,
	//that are small enough and have only two or one neighbor.

	Timer tim;

	// 1. go trough every pixel and add points to the rooms with the same ID
	for (int y = 0; y < map_to_merge_rooms.rows; y++)
	{
		for (int x = 0; x < map_to_merge_rooms.cols; x++)
		{
			int current_id = map_to_merge_rooms.at<int>(y, x);
			if (current_id != 0)
			{
				for (size_t current_room = 0; current_room < rooms.size(); current_room++) //add the Points with the same Id as a room to it
				{
					if (rooms[current_room].getID() == current_id) //insert the current point into the corresponding room
					{
						rooms[current_room].insertMemberPoint(cv::Point(x, y), map_resolution_from_subscription);
						break;
					}
				}
			}
		}
	}
	std::cout << "merge1: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();

	// 2. add the neighbor IDs for every point
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		const int current_id = rooms[current_room].getID();
		std::vector<cv::Point> considered_neighbors;		// storage for already counted neighborhood points
		const std::vector<cv::Point>& current_points = rooms[current_room].getMembers();
		for (int current_point = 0; current_point < current_points.size(); current_point++)
		{
			for (int row_counter = -1; row_counter <= 1; row_counter++)
			{
				for (int col_counter = -1; col_counter <= 1; col_counter++)
				{
					const int label = map_to_merge_rooms.at<int>(current_points[current_point].y + row_counter, current_points[current_point].x + col_counter);

					// collect neighbor IDs
					if (label != 0 && label != current_id)
						rooms[current_room].addNeighborID(label);

					// neighborhood statistics
					cv::Point neighbor_point(current_points[current_point].x + col_counter, current_points[current_point].y + row_counter);
					if (!contains(considered_neighbors, neighbor_point) && label != current_id)
					{
						rooms[current_room].addNeighbor(label);
						considered_neighbors.push_back(neighbor_point);
					}
				}
			}
		}
	}
	std::cout << "merge2: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();

	// 3. merge criteria
	// sort rooms ascending by area
	std::sort(rooms.begin(), rooms.end(), sortRoomsAscending);
	// a) rooms with one neighbor and max. 75% walls around
	for (int current_room_index = 0; current_room_index < rooms.size(); )
	{
		Room& current_room = rooms[current_room_index];
		bool merge_rooms = false;
		size_t merge_index = 0;

		if (current_room.getNeighborCount() == 1 && current_room.getArea() < max_area_for_merging && current_room.getWallToPerimeterRatio() <= 0.75)
		{
			// check every room if it should be merged with its neighbor that it shares the most boundary with
			merge_rooms = determineRoomIndexFromRoomID(rooms, current_room.getNeighborWithLargestCommonBorder(), merge_index);
		}

		if (merge_rooms == true)
		{
			//std::cout << "merge " << current_room.getCenter() << ", id=" << current_room.getID() << " into " << rooms[merge_index].getCenter() << ", id=" << rooms[merge_index].getID() << std::endl;
			mergeRoomPair(rooms, merge_index, current_room_index, map_to_merge_rooms, map_resolution_from_subscription);
			current_room_index = 0;
		}
		else
			current_room_index++;
	}
	if (display_map == true)
		cv::imshow("a", map_to_merge_rooms);
	std::cout << "merge3a: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();
	// b) small rooms
	for (int current_room_index = 0; current_room_index < rooms.size(); )
	{
		Room& current_room = rooms[current_room_index];
		bool merge_rooms = false;
		size_t merge_index = 0;

		const int max_border_neighbor_id = current_room.getNeighborWithLargestCommonBorder();
		if (current_room.getArea() < 2.0 && (double)current_room.getNeighborStatistics()[max_border_neighbor_id]/current_room.getPerimeter() > 0.2)		// todo: param
		{
			// merge with that neighbor that shares the most neighboring pixels
			merge_rooms = determineRoomIndexFromRoomID(rooms, max_border_neighbor_id, merge_index);
			if ((double)rooms[merge_index].getWallToPerimeterRatio() > 0.8) //0.8
				merge_rooms = false;
		}

		if (merge_rooms == true)
		{
			//std::cout << "merge " << current_room.getCenter() << ", id=" << current_room.getID() << " into " << rooms[merge_index].getCenter() << ", id=" << rooms[merge_index].getID() << std::endl;
			mergeRoomPair(rooms, merge_index, current_room_index, map_to_merge_rooms, map_resolution_from_subscription);
			current_room_index = 0;
		}
		else
			current_room_index++;
	}
	if (display_map == true)
		cv::imshow("b", map_to_merge_rooms);
	std::cout << "merge3b: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();
	// c) merge a room with one neighbor that has max. 2 neighbors and sufficient wall ratio (connect parts inside a room)
	for (int current_room_index = 0; current_room_index < rooms.size(); )
	{
		Room& current_room = rooms[current_room_index];
		bool merge_rooms = false;
		size_t merge_index = 0;

		// merge a room with one neighbor that has max. 2 neighbors and sufficient wall ratio (connect parts inside a room)
		const int max_border_neighbor_id = current_room.getNeighborWithLargestCommonBorder();
		if ((current_room.getNeighborCount()==1 || current_room.getPerimeterRatioOfXLargestRooms(1)>0.98) && current_room.getWallToPerimeterRatio() > 0.5 &&
			(double)current_room.getNeighborStatistics()[max_border_neighbor_id]/current_room.getPerimeter() > 0.15)
		{
			// merge with that neighbor that shares the most neighboring pixels
			merge_rooms = determineRoomIndexFromRoomID(rooms, max_border_neighbor_id, merge_index);
			if (rooms[merge_index].getNeighborCount() > 2 && rooms[merge_index].getPerimeterRatioOfXLargestRooms(2)<0.95) // || rooms[merge_index].getWallToPerimeterRatio() < 0.4)
				merge_rooms = false;
		}

		if (merge_rooms == true)
		{
			//std::cout << "merge " << current_room.getCenter() << ", id=" << current_room.getID() << " into " << rooms[merge_index].getCenter() << ", id=" << rooms[merge_index].getID() << std::endl;
			mergeRoomPair(rooms, merge_index, current_room_index, map_to_merge_rooms, map_resolution_from_subscription);
			current_room_index = 0;
		}
		else
			current_room_index++;
	}
	if (display_map == true)
		cv::imshow("c", map_to_merge_rooms);
	std::cout << "merge3c: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();
	// d) merge rooms that share a significant part of their perimeter
	for (int current_room_index = 0; current_room_index < rooms.size(); )
	{
		Room& current_room = rooms[current_room_index];
		bool merge_rooms = false;
		size_t merge_index = 0;

		std::map< int,int,std::greater<int> > neighbor_room_statistics_inverse;	// common border length, room_id
		current_room.getNeighborStatisticsInverse(neighbor_room_statistics_inverse);
//		std::vector<int>& neighbor_ids = current_room.getNeighborIDs();
//		for (size_t n=0; n<neighbor_ids.size(); ++n)
		for (std::map< int,int,std::greater<int> >::iterator it=neighbor_room_statistics_inverse.begin(); it!=neighbor_room_statistics_inverse.end(); ++it)
		{
			if (it->second==0)
				continue;		// skip wall

			const double neighbor_border_ratio = (double)current_room.getNeighborStatistics()[it->second]/current_room.getPerimeter();
			if (neighbor_border_ratio > 0.2 || (neighbor_border_ratio > 0.1 && current_room.getWallToPerimeterRatio() > (1-2*neighbor_border_ratio-0.05) && current_room.getWallToPerimeterRatio() < (1-neighbor_border_ratio)))		// todo: param
			{
				// merge with that neighbor that shares the most neighboring pixels
				merge_rooms = determineRoomIndexFromRoomID(rooms, it->second, merge_index);
				if ((double)rooms[merge_index].getNeighborStatistics()[current_room.getID()]/rooms[merge_index].getPerimeter() <= 0.1)
					merge_rooms = false;
				if (merge_rooms == true)
					break;
			}
		}

		if (merge_rooms == true)
		{
			//std::cout << "merge " << current_room.getCenter() << ", id=" << current_room.getID() << " into " << rooms[merge_index].getCenter() << ", id=" << rooms[merge_index].getID() << std::endl;
			mergeRoomPair(rooms, merge_index, current_room_index, map_to_merge_rooms, map_resolution_from_subscription);
			current_room_index = 0;
		}
		else
			current_room_index++;
	}
	if (display_map == true)
		cv::imshow("d", map_to_merge_rooms);
	std::cout << "merge3d: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();
	// e) largest room neighbor touches > 0.5 perimeter (happens often with furniture)
	for (int current_room_index = 0; current_room_index < rooms.size(); )
	{
		Room& current_room = rooms[current_room_index];
		bool merge_rooms = false;
		size_t merge_index = 0;

		const int max_border_neighbor_id = current_room.getNeighborWithLargestCommonBorder();
		if ((double)current_room.getNeighborStatistics()[max_border_neighbor_id]/current_room.getPerimeter() > 0.4)
		{
			// merge with that neighbor that shares the most neighboring pixels
			merge_rooms = determineRoomIndexFromRoomID(rooms, max_border_neighbor_id, merge_index);
		}

		if (merge_rooms == true)
		{
			//std::cout << "merge " << current_room.getCenter() << ", id=" << current_room.getID() << " into " << rooms[merge_index].getCenter() << ", id=" << rooms[merge_index].getID() << std::endl;
			mergeRoomPair(rooms, merge_index, current_room_index, map_to_merge_rooms, map_resolution_from_subscription);
			current_room_index = 0;
		}
		else
			current_room_index++;
	}
	std::cout << "merge3e: " << tim.getElapsedTimeInMilliSec() << "ms" << std::endl;
	tim.start();


//	//check every room if it should be merged with its neighbor that it shares the most boundary with
//	for (int current_room = 0; current_room < rooms.size(); current_room++)
//	{
//		//only merge rooms that have 2 or less neighbors and are small enough
//		if ((rooms[current_room].getNeighborCount() <= 2 && rooms[current_room].getArea() < max_area_for_merging) //12.5
//				|| (rooms[current_room].getNeighborCount() == 3 && rooms[current_room].getArea() < 3.5))//3.5 --> if room is too small merge it with neighbors		// todo: parameter
//		{
//			const std::vector<cv::Point>& current_room_members = rooms[current_room].getMembers();
//			double max_shared_perimeter = 0;
//			int room_indice = -1;
//			int largest_ID = 0;
//			std::vector<int> neighbor_ids = rooms[current_room].getNeighborIDs(); //get IDs for every neighbor of this room
//			for (int current_neighbor = 0; current_neighbor < rooms.size(); current_neighbor++)
//			{
//				if (contains(neighbor_ids, rooms[current_neighbor].getID()))
//				{
//					std::vector<cv::Point> neighboring_points;
//					std::vector<cv::Point> neighbor_members = rooms[current_neighbor].getMembers();
//					for(int room_point = 0; room_point < current_room_members.size(); room_point++)
//					{
//						//check 3x3 region around current point if a neighboring point is a member of the neighbor --> add it to neighboring vector
//						for(int row_counter = -1; row_counter <= 1; row_counter++)
//						{
//							for(int col_counter = -1; col_counter <= 1; col_counter++)
//							{
//								cv::Point temporary_point(current_room_members[room_point].x + col_counter, current_room_members[room_point].y + row_counter);
//								if(contains(neighbor_members, temporary_point) && !contains(neighboring_points, temporary_point))
//								{
//									neighboring_points.push_back(temporary_point);
//								}
//							}
//						}
//					}
//
//					//check if current shared boundary is larger than saved one and also check if boundary is larger than
//					//the typical shared boundary in a door
//					if(neighboring_points.size() >= max_shared_perimeter
//							&& neighboring_points.size() >= 24) //24 --> most doors fulfill this criterion			// todo: parameter
//					{
//						max_shared_perimeter = neighboring_points.size();
//						largest_ID = rooms[current_neighbor].getID();
//						room_indice = current_neighbor;
//					}
//				}
//			}
//			if(largest_ID != 0)//check if the largest ID has been set and isn't zero
//			{
//				rooms[current_room].setRoomId(largest_ID, map_to_merge_rooms);
//				rooms[room_indice].insertMemberPoints(current_room_members, map_resolution_from_subscription);
////				cv::imshow("test", map_to_merge_rooms);
////				cv::waitKey();
//			}
//		}
//	}
}

void VoronoiSegmentation::segmentationAlgorithm(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
		double room_area_factor_lower_limit, double room_area_factor_upper_limit, int neighborhood_index, int max_iterations,
		double min_critical_point_distance_factor, double max_area_for_merging, bool display_map)
{
	//****************Create the Generalized Voronoi-Diagram**********************
	//This function takes a given map and segments it with the generalized Voronoi-Diagram. It takes following steps:
	//	I. It calculates the generalized Voronoi-Diagram using the function createVoronoiGraph.
	//	II. It extracts the critical points, which show the border between two segments. This part takes these steps:
	//		1. Extract node-points of the Voronoi-Diagram, which have at least 3 neighbors.
	//		2. Reduce the leave-nodes (Point on graph with only one neighbor) of the graph until the reduction
	//		   hits a node-Point. This is done to reduce the lines along the real voronoi-graph, coming from the discretisation
	//		   of the contour.
	//		3. Find the critical points in the reduced graph by searching in a specified neighborhood for a local minimum
	//		   in distance to the nearest black pixel. The size of the epsilon-neighborhood is dynamic and goes larger
	//		   in small areas, so they are split into lesser regions.
	//	III. It gets the critical lines, which go from the critical point to its two nearest black pixels and separate the
	//		 regions from each other. This part does following steps:
	//			1. Get the discretized contours of the map and the holes, because these are the possible candidates for
	//			   basis-points.
	//			2. Find the basis-points for each critical-point by finding the two nearest neighbors of the vector from 1.
	//			   Also it saves the angle between the two vectors pointing from the critical-point to its two basis-points.
	//			3. Some critical-lines are too close to each other, so the next part eliminates some of them. For this the
	//			   algorithm checks, which critical points are too close to each other. Then it compares the angles of these
	//			   points, which were calculated in 3., and takes the one with the larger angle, because smaller angles
	//			   (like 90 degree) are more likely to be at edges of the map or are too close to the borders. If they have
	//			   the same angle, the point which comes first in the critical-point-vector is chosen (took good results for
	//			   me, but is only subjective).
	//			4. Draw the critical lines, selected by 3. in the map with color 0.
	//	IV. It finds the segments, which are seperated by the critical lines of III. and fills them with a random colour that
	//		hasn't been already used yet. For this it:
	//			1. It erodes the map with critical lines, so small gaps are closed, and finds the contours of the segments.
	//			   Only contours that are large/small enough are chosen to be drawn.
	//			2. It draws the contours from 1. in a map with a random colour. Contours that belong to holes are not drawn
	//			   into the map.
	//			3. Spread the colour-regions to the last white Pixels, using the watershed-region-spreading function.

	//*********************I. Calculate and draw the Voronoi-Diagram in the given map*****************

	Timer tim;

	cv::Mat voronoi_map = map_to_be_labeled.clone();
	createVoronoiGraph(voronoi_map); //voronoi-map for the segmentation-algorithm

	std::cout << "createVoronoiGraph: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim.start();

	//
	//***************************II. extract the possible candidates for critical Points****************************

	//1.extract the node-points that have at least three neighbors on the voronoi diagram
	//	node-points are points on the voronoi-graph that have at least 3 neighbors
	std::set<cv::Point, cv_Point_comp> node_points; //variable for node point extraction
	for (int v = 1; v < voronoi_map.rows-1; v++)
	{
		for (int u = 1; u < voronoi_map.cols-1; u++)
		{
			if (voronoi_map.at<unsigned char>(v, u) == 127)
			{
				int neighbor_count = 0;	//variable to save the number of neighbors for each point
				//check 3x3 region around current pixel
				for (int row_counter = -1; row_counter <= 1; row_counter++)
				{
					for (int column_counter = -1; column_counter <= 1; column_counter++)
					{
						if (row_counter == 0 && column_counter == 0)
							continue;

						//check if neighbors are colored with the voronoi-color
						if (voronoi_map.at<unsigned char>(v + row_counter, u + column_counter) == 127)
						{
							neighbor_count++;
						}
					}
				}
				if (neighbor_count > 2)
				{
					node_points.insert(cv::Point(u,v));
				}
			}
		}
	}

	//2.reduce the side-lines along the voronoi-graph by checking if it has only one neighbor until a node-point is reached
	//	--> make it white
	//	repeat a large enough number of times so the graph converges

	bool real_voronoi_point; //variable for reducing the side-lines

	for (int step = 0; step < 100; step++)
	{
		for (int v = 0; v < voronoi_map.rows; v++)
		{
			for (int u = 0; u < voronoi_map.cols; u++)
			{
				//set that the point is a point along the graph and not a side-line
				if (voronoi_map.at<unsigned char>(v, u) == 127)
				{
					int neighbor_count = 0;		//variable to save the number of neighbors for each point
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (row_counter == 0 && column_counter == 0)
								continue;

							const int nv = v + row_counter;
							const int nu = u + column_counter;
							if (nv >= 0 && nu >= 0 && nv < voronoi_map.rows && nu < voronoi_map.cols &&
									voronoi_map.at<unsigned char>(nv, nu) == 127)
							{
								neighbor_count++;
							}
						}
					}
					if (neighbor_count <= 1 && node_points.find(cv::Point(u,v)) == node_points.end())
					{
						//if the Point isn't on the voronoi-graph make it white
						voronoi_map.at<unsigned char>(v, u) = 255;
					}
				}
			}
		}
	}

	//3.find the critical points in the previously calculated generalized Voronoi-graph by searching in a specified
	//	neighborhood for the local minimum of distance to the nearest black pixel
	//	critical points need to have at least two neighbors (else they are end points, which would give a very small segment)

	//get the distance transformed map, which shows the distance of every white pixel to the closest zero-pixel
	cv::Mat distance_map; //distance-map of the original-map (used to check the distance of each point to nearest black pixel)
	cv::distanceTransform(map_to_be_labeled, distance_map, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map, distance_map);

	std::vector<cv::Point> critical_points; //saving-variable for the critical points found on the Voronoi-graph
	for (int v = 0; v < voronoi_map.rows; v++)
	{
		for (int u = 0; u < voronoi_map.cols; u++)
		{
			if (voronoi_map.at<unsigned char>(v, u) == 127)
			{
				//make the size of the region to be checked dependent on the distance of the current pixel to the closest
				//zero-pixel, so larger areas are split into more regions and small areas into fewer
				int eps = neighborhood_index / (int) distance_map.at<unsigned char>(v, u); //310
				int loopcounter = 0; //if a part of the graph is not connected to the rest this variable helps to stop the loop
				std::vector<cv::Point> temporary_points;	//neighboring-variables, which are different for each point
				std::set<cv::Point, cv_Point_comp> neighbor_points;	//neighboring-variables, which are different for each point
				int neighbor_count = 0;		//variable to save the number of neighbors for each point
				neighbor_points.insert(cv::Point(u,v)); //add the current Point to the neighborhood
				//find every Point along the voronoi graph in a specified neighborhood
				do
				{
					loopcounter++;
					//check every point in the neighborhood for other neighbors connected to it
					for(std::set<cv::Point, cv_Point_comp>::iterator it_neighbor_points = neighbor_points.begin(); it_neighbor_points != neighbor_points.end(); it_neighbor_points++)
					{
						for (int row_counter = -1; row_counter <= 1; row_counter++)
						{
							for (int column_counter = -1; column_counter <= 1; column_counter++)
							{
								if (row_counter == 0 && column_counter == 0)
									continue;

								//check the neighboring points
								//(if it already is in the neighborhood it doesn't need to be checked again)
								const cv::Point& current_neighbor_point = *it_neighbor_points;
								const int nu = current_neighbor_point.x + column_counter;
								const int nv = current_neighbor_point.y + row_counter;
								if (nv >= 0 && nu >= 0 && nv < voronoi_map.rows && nu < voronoi_map.cols &&
									voronoi_map.at<unsigned char>(nv, nu) == 127 && neighbor_points.find(cv::Point(nu, nv))==neighbor_points.end())
								{
									neighbor_count++;
									temporary_points.push_back(cv::Point(nu, nv));
								}
							}
						}
					}
					//go trough every found point after all neighborhood points have been checked and add them to it
					for (int temporary_point_index = 0; temporary_point_index < temporary_points.size(); temporary_point_index++)
					{
						neighbor_points.insert(temporary_points[temporary_point_index]);
						//make the found points white in the voronoi-map (already looked at)
						voronoi_map.at<unsigned char>(temporary_points[temporary_point_index].y, temporary_points[temporary_point_index].x) = 255;
						voronoi_map.at<unsigned char>(v, u) = 255;
					}
					//check if enough neighbors have been checked or checked enough times (e.g. at a small segment of the graph)
				} while (neighbor_count <= eps && loopcounter < max_iterations);
				//check every found point in the neighborhood if it is the local minimum in the distanceMap
				cv::Point current_critical_point = cv::Point(u, v);
				for(std::set<cv::Point, cv_Point_comp>::iterator it_neighbor_points = neighbor_points.begin(); it_neighbor_points != neighbor_points.end(); it_neighbor_points++)
				{
					if (distance_map.at<unsigned char>(it_neighbor_points->y, it_neighbor_points->x) < distance_map.at<unsigned char>(current_critical_point.y, current_critical_point.x))
					{
						current_critical_point = cv::Point(*it_neighbor_points);
					}
				}
				//add the local minimum point to the critical points
				critical_points.push_back(current_critical_point);
			}
		}
	}

	std::cout << "critical points: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim.start();

//	if(display_map == true)
//	{
//		cv::Mat display = map_to_be_labeled.clone();
//		for (size_t i=0; i<critical_points.size(); ++i)
//			cv::circle(display, critical_points[i], 2, cv::Scalar(128), -1);
//		cv::imshow("critical points", display);
//	}

	//
	//*************III. draw the critical lines from every found critical Point to its two closest zero-pixel****************
	//

	//map to draw the critical lines and fill the map with random colors
	map_to_be_labeled.convertTo(segmented_map, CV_32SC1, 256, 0); // rescale to 32 int, 255 --> 255*256 = 65280

	// 1. Get the points of the contour, which are the possible closest points for a critical point
	//clone the map to extract the contours, because after using OpenCV find-/drawContours
	//the map will be different from the original one
	cv::Mat temporary_map_to_extract_the_contours = segmented_map.clone();
	std::vector < std::vector<cv::Point> > contours;
	cv::findContours(temporary_map_to_extract_the_contours, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// 2. Get the basis-points for each critical-point
	std::vector<cv::Point> basis_points_1, basis_points_2;
	std::vector<double> length_of_critical_line;
	std::vector<double> angles; //the angles between the basis-lines of each critical Point
	for (int critical_point_index = 0; critical_point_index < critical_points.size(); critical_point_index++)
	{
		//set inital points and values for the basis points so the distance comparison can be done
		cv::Point basis_point_1 = contours[0][0];
		cv::Point basis_point_2 = contours[0][1];
		//inital values of the first vector from the current critical point to the contour points and for the distance of it
		const cv::Point& critical_point = critical_points[critical_point_index];
		double vector_x_1 = critical_point.x - contours[0][0].x;
		double vector_y_1 = critical_point.y - contours[0][0].y;
		double distance_basis_1 = std::sqrt(vector_x_1*vector_x_1 + vector_y_1*vector_y_1);
		//inital values of the second vector from the current critical point to the contour points and for the distance of it
		double vector_x_2 = critical_point.x - contours[0][1].x;
		double vector_y_2 = critical_point.y - contours[0][1].y;
		double distance_basis_2 = std::sqrt(vector_x_2*vector_x_2 + vector_y_2*vector_y_2);

		//find first basis point
		int basis_vector_1_x, basis_vector_2_x, basis_vector_1_y, basis_vector_2_y;
		for (int c = 0; c < contours.size(); c++)
		{
			for (int p = 0; p < contours[c].size(); p++)
			{
				//calculate the Euclidian distance from the critical Point to the Point on the contour
				const double vector_x = contours[c][p].x - critical_point.x;
				const double vector_y = contours[c][p].y - critical_point.y;
				const double current_distance = std::sqrt(vector_x*vector_x + vector_y*vector_y);
				//compare the distance to the saved distances if it is smaller
				if (current_distance < distance_basis_1)
				{
					distance_basis_1 = current_distance;
					basis_point_1 = contours[c][p];
					basis_vector_1_x = vector_x;
					basis_vector_1_y = vector_y;
				}
			}
		}
		//find second basisPpoint
		for (int c = 0; c < contours.size(); c++)
		{
			for (int p = 0; p < contours[c].size(); p++)
			{
				//calculate the Euclidian distance from the critical point to the point on the contour
				const double vector_x = contours[c][p].x - critical_point.x;
				const double vector_y = contours[c][p].y - critical_point.y;
				const double current_distance = std::sqrt(vector_x*vector_x + vector_y*vector_y);
				//calculate the distance between the current contour point and the first basis point to make sure they
				//are not too close to each other
				const double vector_x_basis = basis_point_1.x - contours[c][p].x;
				const double vector_y_basis = basis_point_1.y - contours[c][p].y;
				const double basis_distance = std::sqrt(vector_x_basis*vector_x_basis + vector_y_basis*vector_y_basis);
				if (current_distance > distance_basis_1 && current_distance < distance_basis_2 &&
					basis_distance > (double) distance_map.at<unsigned char>(critical_point.y, critical_point.x))
				{
					distance_basis_2 = current_distance;
					basis_point_2 = contours[c][p];
					basis_vector_2_x = vector_x;
					basis_vector_2_y = vector_y;
				}
			}
		}
		//calculate angle between the vectors from the critical Point to the found basis-points
		double current_angle = std::acos((basis_vector_1_x * basis_vector_2_x + basis_vector_1_y * basis_vector_2_y) / (distance_basis_1 * distance_basis_2)) * 180.0 / PI;

		//save the critical line with its calculated values
		basis_points_1.push_back(basis_point_1);
		basis_points_2.push_back(basis_point_2);
		length_of_critical_line.push_back(distance_basis_1 + distance_basis_2);
		angles.push_back(current_angle);
	}

	//3. Check which critical points should be used for the segmentation. This is done by checking the points that are
	//   in a specified distance to each other and take the point with the largest calculated angle, because larger angles
	//   correspond to a separation across the room, which is more useful
	for (int first_critical_point = 0; first_critical_point < critical_points.size(); first_critical_point++)
	{
		//reset variable for checking if the line should be drawn
		bool draw = true;
		for (int second_critical_point = 0; second_critical_point < critical_points.size(); second_critical_point++)
		{
			if (second_critical_point != first_critical_point)
			{
				//get distance of the two current Points
				const double vector_x = critical_points[second_critical_point].x - critical_points[first_critical_point].x;
				const double vector_y = critical_points[second_critical_point].y - critical_points[first_critical_point].y;
				const double critical_point_distance = std::sqrt(vector_x*vector_x + vector_y*vector_y);
				//check if the points are too close to each other corresponding to the distance to the nearest black pixel
				//of the current critical point. This is done because critical points at doors are closer to the black region
				//and shorter and may be eliminated in the following step. By reducing the checking distance at this point
				//it gets better.
				if (critical_point_distance < ((int) distance_map.at<unsigned char>(critical_points[first_critical_point].y, critical_points[first_critical_point].x) * min_critical_point_distance_factor)) //1.7
				{
					//if one point in neighborhood is found that has a larger angle the actual to-be-checked point shouldn't be drawn
					if (angles[first_critical_point] < angles[second_critical_point])
					{
						draw = false;
					}
					//if the angles of the two neighborhood points are the same the shorter one should be drawn, because it is more likely something like e.g. a door
					if (angles[first_critical_point] == angles[second_critical_point] &&
						length_of_critical_line[first_critical_point] > length_of_critical_line[second_critical_point] &&
						(length_of_critical_line[second_critical_point] > 3 || first_critical_point > second_critical_point))
					{
						draw = false;
					}
				}
			}
		}
		//4. draw critical-lines if angle of point is larger than the other
		if (draw)
		{
			cv::line(voronoi_map, critical_points[first_critical_point], basis_points_1[first_critical_point], cv::Scalar(0), 2);
			cv::line(voronoi_map, critical_points[first_critical_point], basis_points_2[first_critical_point], cv::Scalar(0), 2);
		}
	}
//	if(display_map == true)
//		cv::imshow("voronoi_map", voronoi_map);

	std::cout << "critical lines: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim.start();

	//***********************Find the Contours seperated from the critcal lines and fill them with colour******************

	Timer tim2;

	std::vector < cv::Scalar > already_used_colors; //saving-vector to save the already used coloures

	std::vector < cv::Vec4i > hierarchy; //variables for coloring the map

	std::vector<Room> rooms; //Vector to save the rooms in this map

	//1. Erode map one time, so small gaps are closed
//	cv::erode(voronoi_map_, voronoi_map_, cv::Mat(), cv::Point(-1, -1), 1);
	cv::findContours(voronoi_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for (int current_contour = 0; current_contour < contours.size(); current_contour++)
	{ //only draw contours that aren't holes
		if (hierarchy[current_contour][3] == -1)
		{
			//calculate area for the contour and check if it is large enough to be a room
			double room_area = map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[current_contour]);
			if (room_area >= room_area_factor_lower_limit && room_area <= room_area_factor_upper_limit)
			{
				//2. Draw the region with a random colour into the map if it is large/small enough
				bool drawn = false;
				int loop_counter = 0; //counter if the loop gets into a endless loop
				do
				{
					loop_counter++;
					int random_number = rand() % 52224 + 13056;
					cv::Scalar fill_colour(random_number);
					//check if colour has already been used
					if (!contains(already_used_colors, fill_colour) || loop_counter > 1000)
					{
						cv::drawContours(segmented_map, contours, current_contour, fill_colour, 1);
						already_used_colors.push_back(fill_colour);
						Room current_room(random_number); //add the current Contour as a room
						for (int point = 0; point < contours[current_contour].size(); point++) //add contour points to room
						{
							current_room.insertMemberPoint(cv::Point(contours[current_contour][point]), map_resolution_from_subscription);
						}
						rooms.push_back(current_room);
						drawn = true;
					}
				} while (!drawn);
			}
		}
	}
	std::cout << "Found " << rooms.size() << " rooms.\n";

	std::cout << "finish 1: " << tim2.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim2.start();

	//3.fill the last white areas with the surrounding color
	wavefrontRegionGrowing(segmented_map);

	std::cout << "finish 3: " << tim2.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim2.start();

	if(display_map == true)
	{
		cv::imshow("before", segmented_map);
		cv::waitKey(1);
	}

	//4.merge the rooms together if neccessary
	mergeRooms(segmented_map, rooms, map_resolution_from_subscription, max_area_for_merging, display_map);

	std::cout << "finish 4: " << tim2.getElapsedTimeInMilliSec() << " ms" << std::endl;
	tim2.start();

	std::cout << "finish: " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;
}

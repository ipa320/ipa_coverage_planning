#include <ipa_room_segmentation/voronoi_segmentation.h>

#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>

VoronoiSegmentation::VoronoiSegmentation()
{

}

void VoronoiSegmentation::drawVoronoi(cv::Mat &img, std::vector<std::vector<cv::Point2f> > facets_of_voronoi, cv::Scalar voronoi_color,
        std::vector<cv::Point> contour, std::vector<std::vector<cv::Point> > hole_contours)
{
	//This function draws the Voronoi-diagram into a given map. It needs the facets as vector of Points, the contour of the
	//map and the contours of the holes. It checks if the endpoints of the facets are both inside the map-contour and not
	//inside a hole-contour and doesn't draw the lines that are not.
	for (int idx = 0; idx < facets_of_voronoi.size(); idx++)
	{
		//saving-variable for the last Point that has been looked at
		cv::Point2f last_point = facets_of_voronoi[idx].back();
		//draw each line of the voronoi-cell
		for (int c = 0; c < facets_of_voronoi[idx].size(); c++)
		{
			//variable to check, weather a Point is inside a white area or not
			bool inside = true;
			cv::Point2f current_point = facets_of_voronoi[idx][c];
			//only draw lines that are inside the map-contour
			if (cv::pointPolygonTest(contour, current_point, false) < 0 || cv::pointPolygonTest(contour, last_point, false) < 0)
			{
				inside = false;
			}
			//only draw Points inside the contour that are not inside a hole-contour
			for (int i = 0; i < hole_contours.size(); i++)
			{
				if (cv::pointPolygonTest(hole_contours[i], current_point, false) >= 0 || cv::pointPolygonTest(hole_contours[i], last_point, false) >= 0)
				{
					inside = false;
				}
			}
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
	//	   largest contour, because smaller contours correspond to mapping errors
	//	3. Finally it gets the boundary-Points of the voronoi-cells with getVoronoiFacetList. It takes these facets
	//	   and draws them using the drawVoronoi function. This function draws the facets that only have Points inside
	//	   the map-contour (other lines go to not-reachable places and are not neccessary to be looked at).
	//	4. It returns the map that has the generalized voronoi-graph drawn in.

	cv::Mat map_to_draw_voronoi_in = map_for_voronoi_generation.clone(); //variable to save the given map for drawing in the voronoi-diagram

	cv::Mat temporary_map_to_calculate_voronoi = map_for_voronoi_generation.clone(); //variable to save the given map in the createVoronoiGraph-function

	//apply a closing-operator on the map so bad parts are neglegted
	cv::erode(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());
	cv::dilate(temporary_map_to_calculate_voronoi, temporary_map_to_calculate_voronoi, cv::Mat());

	//********************1. Get OpenCV delaunay-traingulation******************************

	cv::Rect rect(0, 0, map_to_draw_voronoi_in.cols, map_to_draw_voronoi_in.rows); //variables to generate the voronoi-diagram, using OpenCVs delaunay-traingulation
	cv::Subdiv2D subdiv(rect);

	std::vector < std::vector<cv::Point> > hole_contours; //variable to save the hole-contours

	std::vector < std::vector<cv::Point> > contours; //variables for contour extraction and discretisation
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;

	//get contours of the map
	cv::findContours(map_to_draw_voronoi_in, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(map_to_draw_voronoi_in, contours, -1, cv::Scalar(255), CV_FILLED);

	//put every point of the map-contours into the Delaunay-generator of OpenCV
	for (int current_contour = 0; current_contour < contours.size(); current_contour++)
	{
		for (int current_Point = 0; current_Point < contours[current_contour].size(); current_Point++)
		{
			cv::Point fp = contours[current_contour][current_Point];
			subdiv.insert(fp);
		}
		//get the contours of the black holes --> it is neccessary to check if Points are inside these in drawVoronoi
		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
		{
			hole_contours.push_back(contours[current_contour]);
		}
	}

	//********************2. Get largest contour******************************

	std::vector < std::vector<cv::Point> > eroded_contours; //variable to save the eroded contours
	cv::Mat eroded_map;
	cv::Point anchor(-1, -1);

	std::vector < cv::Point > largest_contour; //variable to save the largest contour of the map --> the contour of the map itself

	//erode the map and get the largest contour of it so that Points near the boundary are not drawn later
	//(see drawVoronoi)
	cv::erode(temporary_map_to_calculate_voronoi, eroded_map, cv::Mat(), anchor, 2);
	cv::findContours(eroded_map, eroded_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//set initial largest contour
	largest_contour = contours[0];
	for (int current_contour = 0; current_contour < eroded_contours.size(); current_contour++)
	{
		//check if the current contour is larger than the saved largest-contour
		if (contourArea(largest_contour) < contourArea(eroded_contours[current_contour]))
		{
			largest_contour = eroded_contours[current_contour];
		}
		if (hierarchy[current_contour][2] == -1 && hierarchy[current_contour][3] != -1)
		{
			hole_contours.push_back(eroded_contours[current_contour]);
		}
	}
	//********************3. Get facets and draw voronoi-Graph******************************
	//get the Voronoi regions from the delaunay-subdivision graph

	cv::Scalar voronoi_color(127); //define the voronoi-drawing colour

	std::vector < std::vector<cv::Point2f> > voronoi_facets; //variables to find the facets and centers of the voronoi-cells
	std::vector < cv::Point2f > voronoi_centers;

	subdiv.getVoronoiFacetList(std::vector<int>(), voronoi_facets, voronoi_centers);
	//draw the voronoi-regions into the map
	drawVoronoi(map_to_draw_voronoi_in, voronoi_facets, voronoi_color, largest_contour, hole_contours);
	//make Pixels black, which were black before and were colored by the voronoi-regions
	for (int x = 0; x < map_to_draw_voronoi_in.rows; x++)
	{
		for (int y = 0; y < map_to_draw_voronoi_in.cols; y++)
		{
			if (map_for_voronoi_generation.at<unsigned char>(x, y) == 0)
			{
				map_to_draw_voronoi_in.at<unsigned char>(x, y) = 0;
			}
		}
	}
	map_for_voronoi_generation = map_to_draw_voronoi_in;
}

void VoronoiSegmentation::mergeRooms(cv::Mat& map_to_merge_rooms, std::vector<Room> rooms, double map_resolution_from_subscription, double max_area_for_merging)
{
	//This function takes the segmented Map from the original Voronoi-segmentation-algorithm and merges rooms together,
	//that are small enough and have only two or one neighbor.

	//go trough every Pixel and add Points to the rooms with the same ID
	for (int y = 0; y < map_to_merge_rooms.rows; y++)
	{
		for (int x = 0; x < map_to_merge_rooms.cols; x++)
		{
			int current_id = map_to_merge_rooms.at<int>(y, x);
			if (current_id != 0)
			{
				for (int current_room = 0; current_room < rooms.size(); current_room++) //add the Points with the same Id as a room to it
				{
					if (rooms[current_room].getID() == current_id) //insert the current Point into the corresponding room
					{
						rooms[current_room].insertMemberPoint(cv::Point(x, y), map_resolution_from_subscription);
						break;
					}
				}
			}
		}
	}
	//set the area for each room
//	for (int room = 0; room < rooms.size(); room++)
//	{
//		std::vector<cv::Point> members = rooms[room].getMembers();
//		double room_area = map_resolution_from_subscription * map_resolution_from_subscription * members.size();
//		rooms[room].setArea(room_area);
//	}
	//add the neighbor IDs for every Point
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		std::vector < cv::Point > current_Points = rooms[current_room].getMembers();
		for (int current_point = 0; current_point < current_Points.size(); current_point++)
		{
			for (int row_counter = -1; row_counter <= 1; row_counter++)
			{
				for (int col_counter = -1; col_counter <= 1; col_counter++)
				{
					if (map_to_merge_rooms.at<int>(current_Points[current_point].y + row_counter, current_Points[current_point].x + col_counter) != 0
					        && map_to_merge_rooms.at<int>(current_Points[current_point].y + row_counter, current_Points[current_point].x + col_counter)
					                != rooms[current_room].getID())
					{
						rooms[current_room].addNeighborID(
						        map_to_merge_rooms.at<int>(current_Points[current_point].y + row_counter, current_Points[current_point].x + col_counter));
					}
				}
			}
		}
	}
	//check every room if it should be merged with its neighbor that it shares the most boundary with
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		//only merge rooms that have 2 or less neighbors and are small enough
		if ((rooms[current_room].getNeighborCount() <= 2 && rooms[current_room].getArea() < max_area_for_merging) //15.0
				|| (rooms[current_room].getNeighborCount() == 3 && rooms[current_room].getArea() < 3.5))//3.5 --> if room is too small merge it with neighbors
		{
			std::vector<cv::Point> current_room_members = rooms[current_room].getMembers();
			double max_shared_perimeter = 0;
			int room_indice = -1;
			int largest_ID = 0;
			std::vector<int> neighbor_ids = rooms[current_room].getNeighborIDs(); //get IDs for every neighbor of this room
			for (int current_neighbor = 0; current_neighbor < rooms.size(); current_neighbor++)
			{
				if (contains(neighbor_ids, rooms[current_neighbor].getID()))
				{
					std::vector<cv::Point> neighboring_points;
					std::vector<cv::Point> neighbor_members = rooms[current_neighbor].getMembers();
					for(int room_point = 0; room_point < current_room_members.size(); room_point++)
					{
						//check 3x3 region around current point if a neighboring point is a member of the neighbor --> add it to neighboring vector
						for(int row_counter = -1; row_counter <= 1; row_counter++)
						{
							for(int col_counter = -1; col_counter <= 1; col_counter++)
							{
								cv::Point temporary_point(current_room_members[room_point].x + col_counter, current_room_members[room_point].y + row_counter);
								if(contains(neighbor_members, temporary_point) && !contains(neighboring_points, temporary_point))
								{
									neighboring_points.push_back(temporary_point);
								}
							}
						}
					}

					//check if current shared boundary is larger than saved one and also check if boundary is larger than
					//the typical shared boundary in a door
					if(neighboring_points.size() >= max_shared_perimeter
							&& neighboring_points.size() >= 24) //24 --> most doors fulfill this criterion
					{
						max_shared_perimeter = neighboring_points.size();
						largest_ID = rooms[current_neighbor].getID();
						room_indice = current_neighbor;
					}
				}
			}
			if(largest_ID != 0)//check if the largest ID has been set and isn't zero
			{
				rooms[current_room].setRoomId(largest_ID, map_to_merge_rooms);
				rooms[room_indice].insertMemberPoints(current_room_members, map_resolution_from_subscription);
			}
		}
	}
}

void VoronoiSegmentation::segmentationAlgorithm(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
        double room_area_factor_lower_limit, double room_area_factor_upper_limit, int neihborhood_index, int max_iterations,
        double min_critical_Point_distance_factor, double max_area_for_merging)
{
	//****************Create the Generalized Voronoi-Diagram**********************
	//This function takes a given map and segments it with the generalized Voronoi-Diagram. It takes following steps:
	//	I. It calculates the generalized Voronoi-Diagram using the function createVoronoiGraph.
	//	II. It extracts the critical Points, which show the border between two segments. This Part takes these steps:
	//		1. Extract Node-Points of the Voronoi-Diagram, which have at least 3 neighbors.
	//		2. Reduce the leave-nodes (Point on graph with only one neighbor) of the graph until the reduction
	//		   hits a node-Point. This is done to reduce the lines along the real voronoi-graph, coming from the discretisation
	//		   of the contour.
	//		3. Find the critical Points in the reduced graph by searching in a specified neighbourhood for a local Minimum
	//		   in distance to the nearest black Pixel. The szie of the epsilon-neighbourhood is dynamical and goes larger
	//		   in small areas, so they are split into lesser regions.
	//	III. It gets the critical lines, which go from the critical Point to its two nearest black Pixels and seperate the
	//		 regions from each other. This part does following steps:
	//			1. Get the discretized Contours of the map and the holes, because these are the possible candidates for
	//			   basis-points.
	//			2. Find the basis-Points for each critical-Point by finding the two nearest neighbors of the vector from 1.
	//			   Also it saves the angle between the two vectors pointing from the critical-Point to its two basis-Points.
	//			3. Some critical-lines are too close to each other, so the next part eliminates some of them. For this the
	//			   algorithm checks, which critical-Points are too close to each other. Then it compares the angles of these
	//			   Points, which were calculated in 3., and takes the one with the larger angle, because smaller angles
	//			   (like 90degree) are more likely to be at edges of the map or are too close to the borders. If they have
	//			   the same angle, the Point which comes first in the critical-Point-vector is chosen (took good results for
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

	cv::Mat voronoi_map_ = map_to_be_labeled.clone();
	createVoronoiGraph(voronoi_map_); //voronoi-map for the segmentation-algotihm

	std::vector < cv::Point > node_Points; //variable for Nodepoint-extraction

	//
	//***************************II. extract the possible candidates for critical Points****************************

	int neighbor_count; //variable to save the number of neighbors for each Point

	//1.extract the node-Points that have at least three neighbors on the voronoi diagram
	//	node-Points are Points on the voronoi-graph that have at least 3 neighbors
	for (int x = 0; x < voronoi_map_.rows; x++)
	{
		for (int y = 0; y < voronoi_map_.cols; y++)
		{
			if (voronoi_map_.at<unsigned char>(x, y) == 127)
			{
				neighbor_count = 0;
				//check 3x3 region around current Pixel
				for (int row_counter = -1; row_counter <= 1; row_counter++)
				{
					for (int column_counter = -1; column_counter <= 1; column_counter++)
					{
						//check if neighbors are colored with the voronoi-color
						if (voronoi_map_.at<unsigned char>(x + row_counter, y + column_counter) == 127 && (abs(row_counter) + abs(column_counter)) != 0)
						{
							neighbor_count++;
						}
					}
				}
				if (neighbor_count > 2)
				{
					node_Points.push_back(cv::Point(x, y));
				}
			}
		}
	}
	//2.reduce the side-lines along the voronoi-graph by checking if it has only one neighbor until a node-Point is reached
	//	--> make it white
	//	repeat a large enough number of times so the graph converges

	bool real_voronoi_point; //variable for reducing the side-lines

	for (int step = 0; step < 100; step++)
	{
		for (int x = 0; x < voronoi_map_.rows; x++)
		{
			for (int y = 0; y < voronoi_map_.cols; y++)
			{
				//set that the Point is a Point along the graph and not a side-line
				real_voronoi_point = true;
				if (voronoi_map_.at<unsigned char>(x, y) == 127)
				{
					neighbor_count = 0;
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (voronoi_map_.at<unsigned char>(x + row_counter, y + column_counter) == 127 && (abs(row_counter) + abs(column_counter)) != 0)
							{
								neighbor_count++;
							}
						}
					}
					if (neighbor_count == 1)
					{
						//The Point is a leaf-node
						real_voronoi_point = false;
					}
					//if the current Point is a in the previous step found node Point it belongs to the voronoi-graph
					if (contains(node_Points, cv::Point(x, y)))
					{
						real_voronoi_point = true;
					}

					if (!real_voronoi_point)
					{
						//if the Point isn't on the voronoi-graph make it white
						voronoi_map_.at<unsigned char>(x, y) = 255;
					}
				}
			}
		}
	}

	//3.find the critical Points in the previously calculated generalized Voronoi-Graph by searching in a specified
	//	neighborhood for the local Minimum of distance to the nearest black Pixel
	//	critical Points need to have at least two neighbors (else they are endpoints, which would give a very small segment)

	cv::Mat distance_map; //distance-map of the original-map (used to check the distance of each Point to nearest black Pixel)

	std::vector < cv::Point > critical_Points; //saving-variable for the critical Points found on the voronoi-graph

	std::vector<double> angles; //the angles between the basis-lines of each critical Point

	cv::Point current_critical_Point; //variables for finding critical points
	std::vector<cv::Point> neighbor_Points, temporary_Points;
	int loopcounter, eps;

	//get the distance transformed map, which shows the distance of every white Pixel to the closest zero-Pixel
	cv::distanceTransform(map_to_be_labeled, distance_map, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map, distance_map);

	for (int x = 0; x < voronoi_map_.rows; x++)
	{
		for (int y = 0; y < voronoi_map_.cols; y++)
		{
			if (voronoi_map_.at<unsigned char>(x, y) == 127)
			{
				//make the size of the to be checked region dependend on the distance of the current Pixel to the closest
				//zero-Pixel, so larger areas are splittet into more regions and small areas into fewer
				eps = neihborhood_index / (int) distance_map.at<unsigned char>(x, y); //310
				loopcounter = 0; //if a Part of the graph is not connected to the rest this variable helps to stop the loop
				//reset the neighboring-variables, which are different for each Point
				neighbor_Points.clear();
				neighbor_count = 0;
				neighbor_Points.push_back(cv::Point(x, y)); //add the current Point to the neighborhood
				//find every Point along the voronoi graph in a specified neighborhood
				do
				{
					loopcounter++;
					//check every Point in the neighborhood for other neighbors connected to it
					for (int current_neighbor_Point = 0; current_neighbor_Point < neighbor_Points.size(); current_neighbor_Point++)
					{
						for (int row_counter = -1; row_counter <= 1; row_counter++)
						{
							for (int column_counter = -1; column_counter <= 1; column_counter++)
							{
								//check the neighboring Points
								//(if it already is in the neighborhood it doesn't need to be checked)
								if (voronoi_map_.at<unsigned char>(neighbor_Points[current_neighbor_Point].x + row_counter,
								        neighbor_Points[current_neighbor_Point].y + column_counter) == 127 && (abs(row_counter) + abs(column_counter)) != 0
								        && !contains(neighbor_Points,
								                cv::Point(neighbor_Points[current_neighbor_Point].x + row_counter,
								                        neighbor_Points[current_neighbor_Point].y + column_counter)))
								{
									neighbor_count++;
									temporary_Points.push_back(
									        cv::Point(neighbor_Points[current_neighbor_Point].x + row_counter,
									                neighbor_Points[current_neighbor_Point].y + column_counter));
								}
							}
						}
					}
					//go trough every found Points after all neighborhood-Points have been checked and add them to it
					for (int found_temporary_point = 0; found_temporary_point < temporary_Points.size(); found_temporary_point++)
					{
						neighbor_Points.push_back(temporary_Points[found_temporary_point]);
						//make the found Points white in the voronoi-map (already looked at)
						voronoi_map_.at<unsigned char>(cv::Point(temporary_Points[found_temporary_point].y, temporary_Points[found_temporary_point].x)) = 255;
						voronoi_map_.at<unsigned char>(cv::Point(y, x)) = 255;
					}
					temporary_Points.clear();
					//check if enough neighbors has been checked or checked enough times (e.g. at a small segment of the graph)
				} while (neighbor_count <= eps && loopcounter < max_iterations);
				//check every found Point in the neighborhood if it is the local Minimum in the distanceMap
				current_critical_Point = cv::Point(x, y);
				for (int p = 0; p < neighbor_Points.size(); p++)
				{
					if (distance_map.at<unsigned char>(neighbor_Points[p].x, neighbor_Points[p].y)
					        < distance_map.at<unsigned char>(current_critical_Point.x, current_critical_Point.y))
					{
						current_critical_Point = cv::Point(neighbor_Points[p]);
					}
				}
				//add the local Minimum Point to the critical Points
				critical_Points.push_back(current_critical_Point);
			}
		}
	}
	//
	//*************III. draw the critical lines from every found critical Point to its two closest zero-Pixel****************
	//

	//map to draw the critical lines and fill the map with random colors
	//segmented_map = map_to_be_labeled.clone();
	map_to_be_labeled.convertTo(segmented_map, CV_32SC1, 256, 0); // rescale to 32 int, 255 --> 255*256 = 65280

	//clone the Map to extract the contours, because after using OpenCV find-/drawContours
	//the map will be different from the original one
	cv::Mat temporary_map_to_extract_the_contours = segmented_map.clone();

	std::vector < std::vector<cv::Point> > contours;
	cv::Point basis_Point_1, basis_Point_2;
	std::vector<cv::Point> basis_Points_1, basis_Points_2;
	std::vector<double> length_of_critical_line;
	double current_distance, distance_basis_1, distance_basis_2;
	double current_angle;
	int basis_vector_1_x, basis_vector_2_x, basis_vector_1_y, basis_vector_2_y;
	std::vector < cv::Point > already_drawn_Points;
	bool draw;

	// 1. Get the Points of the contour, which are the possible closest Points for a critical Point
	cv::findContours(temporary_map_to_extract_the_contours, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	// 2. Get the basis-points for each critical-point
	for (int current_critical_point = 0; current_critical_point < critical_Points.size(); current_critical_point++)
	{
		//set inital Points and values for the basis-Points so the distance comparisation can be done
		draw = true;
		current_angle = 0;
		basis_Point_1 = contours[0][0];
		basis_Point_2 = contours[0][1];
		//inital values of the first vector from the current critical Point to the contour Points and for the distance of it
		double vector_x_1 = critical_Points[current_critical_point].x - contours[0][0].y;
		double vector_y_1 = critical_Points[current_critical_point].y - contours[0][0].x;
		distance_basis_1 = std::sqrt((std::pow(vector_x_1, 2)) + std::pow(vector_y_1, 2));
		//inital values of the second vector from the current critical Point to the contour Points and for the distance of it
		double vector_x_2 = critical_Points[current_critical_point].x - contours[0][1].y;
		double vector_y_2 = critical_Points[current_critical_point].y - contours[0][1].x;
		distance_basis_2 = std::sqrt((std::pow(vector_x_2, 2)) + std::pow(vector_y_2, 2));

		//find first basis-Point
		for (int c = 0; c < contours.size(); c++)
		{
			for (int p = 0; p < contours[c].size(); p++)
			{
				//calculate the euclidian distance from the critical Point to the Point on the contour
				double vector_x = critical_Points[current_critical_point].x - contours[c][p].y;
				double vector_y = critical_Points[current_critical_point].y - contours[c][p].x;
				current_distance = std::sqrt((std::pow(vector_x, 2)) + std::pow(vector_y, 2));
				//compare the distance to the saved distances if it is smaller
				if (current_distance < distance_basis_1)
				{
					distance_basis_1 = current_distance;
					basis_Point_1 = contours[c][p];
					basis_vector_1_x = -1 * vector_x;
					basis_vector_1_y = -1 * vector_y;
				}
			}
		}
		//find second basisPpoint
		for (int c = 0; c < contours.size(); c++)
		{
			for (int p = 0; p < contours[c].size(); p++)
			{
				//calculate the euclidian distance from the critical Point to the Point on the contour
				double vector_x = critical_Points[current_critical_point].x - contours[c][p].y;
				double vector_y = critical_Points[current_critical_point].y - contours[c][p].x;
				current_distance = std::sqrt((std::pow(vector_x, 2)) + std::pow(vector_y, 2));
				//calculate the distance between the current contour Point and the first Basis Point to make sure they
				//are not too close to each other
				double vector_x_basis = basis_Point_1.y - contours[c][p].y;
				double vector_y_basis = basis_Point_1.x - contours[c][p].x;
				double basis_distance = std::sqrt((std::pow(vector_x_basis, 2)) + std::pow(vector_y_basis, 2));
				if (current_distance > distance_basis_1 && current_distance < distance_basis_2
				        && basis_distance
				                > (double) distance_map.at<unsigned char>(
				                        cv::Point(critical_Points[current_critical_point].y, critical_Points[current_critical_point].x)))
				{
					distance_basis_2 = current_distance;
					basis_Point_2 = contours[c][p];
					basis_vector_2_x = -1 * vector_x;
					basis_vector_2_y = -1 * vector_y;
				}
			}
		}
		//calculate angle between the vectors from the critical Point to the found basis-points
		double g = basis_vector_1_x * basis_vector_2_x;
		double h = basis_vector_1_y * basis_vector_2_y;
		current_angle = std::acos((g + h) / (distance_basis_1 * distance_basis_2)) * 180.0 / PI;

		//save the crititical line with its calculated values
		basis_Points_1.push_back(basis_Point_1);
		basis_Points_2.push_back(basis_Point_2);
		length_of_critical_line.push_back(distance_basis_1 + distance_basis_2);
		angles.push_back(current_angle);

	}

	//3. check which critical Points should be used for the segmentation. This is done by cheking the Points that are
	//   in a specified distance to each other and take the Point with the largest calculated angle, because larger angles
	//   corresponend to a seperation across the room, which is more useful
	for (int first_critical_Point = 0; first_critical_Point < critical_Points.size(); first_critical_Point++)
	{
		//reset variable for checking if the line should be drawn
		draw = true;
		for (int second_critical_Point = 0; second_critical_Point < critical_Points.size(); second_critical_Point++)
		{
			if (second_critical_Point != first_critical_Point)
			{
				//get distance of the two current Points
				double vector_x = critical_Points[second_critical_Point].x - critical_Points[first_critical_Point].x;
				double vector_y = critical_Points[second_critical_Point].y - critical_Points[first_critical_Point].y;
				double critical_Point_distance = std::sqrt((std::pow(vector_x, 2.0)) + std::pow(vector_y, 2.0));
				//check if the Points are too close to each other corresponding to the distance to the nearest black Pixel
				//of the current critical Point. This is done because critical Points at doors are closer to the black region
				//and shorter and may be eliminated in the following step. By reducing the checking distance at this Point
				//it gets better.
				if (critical_Point_distance
				        < ((int) distance_map.at<unsigned char>(critical_Points[first_critical_Point].x, critical_Points[first_critical_Point].y)
				                * min_critical_Point_distance_factor)) //1.7
				{
					//if one Point in neighborhood is found that has a larger angle the actual to-be-checked Point shouldn't be drawn
					if (angles[first_critical_Point] < angles[second_critical_Point])
					{
						draw = false;
					}
					//if the angles of the two neighborhood Points are the same the shorter one should be drawn, because it more likely is e.g. a door
					if (angles[first_critical_Point] == angles[second_critical_Point]
					        && length_of_critical_line[first_critical_Point] > length_of_critical_line[second_critical_Point]
					        && length_of_critical_line[second_critical_Point] > 3)
					{
						draw = false;
					}
					else if (angles[first_critical_Point] == angles[second_critical_Point]
					        && length_of_critical_line[first_critical_Point] > length_of_critical_line[second_critical_Point]
					        && first_critical_Point < second_critical_Point)
					{
						draw = false;
					}
				}
			}
		}
		//4. draw critical-lines if angle of Point is larger than the other
		if (draw)
		{
			cv::line(voronoi_map_, cv::Point(critical_Points[first_critical_Point].y, critical_Points[first_critical_Point].x),
			        basis_Points_1[first_critical_Point], cv::Scalar(0), 2);
			cv::line(voronoi_map_, cv::Point(critical_Points[first_critical_Point].y, critical_Points[first_critical_Point].x),
			        basis_Points_2[first_critical_Point], cv::Scalar(0), 2);
		}
	}

	//***********************Find the Contours seperated from the critcal lines and fill them with colour******************

	std::vector < cv::Scalar > already_used_coloures; //saving-vector to save the already used coloures

	std::vector < cv::Vec4i > hierarchy; //variables for coloring the map

	std::vector<Room> rooms; //Vector to save the rooms in this map

	//1. Erode map one time, so small gaps are closed
//	cv::erode(voronoi_map_, voronoi_map_, cv::Mat(), cv::Point(-1, -1), 1);
	cv::findContours(voronoi_map_, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

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
					if (!contains(already_used_coloures, fill_colour) || loop_counter > 250)
					{
						cv::drawContours(segmented_map, contours, current_contour, fill_colour, 1);
						already_used_coloures.push_back(fill_colour);
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

	//3.fill the last white areas with the surrounding color
	wavefrontRegionGrowing(segmented_map);

	cv::imshow("before", segmented_map);
	cv::waitKey(1);
	//4.merge the rooms together if neccessary
	mergeRooms(segmented_map, rooms, map_resolution_from_subscription, max_area_for_merging);
}

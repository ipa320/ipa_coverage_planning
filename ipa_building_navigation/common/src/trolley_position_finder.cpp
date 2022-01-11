#include <ipa_building_navigation/trolley_position_finder.h>

//Defaul Constructor
TrolleyPositionFinder::TrolleyPositionFinder()
{

}

//This function takes one group and calculates the trolley position for it. It does following steps:
//		I.   Get the bounding box for all Points in the group. Then expand it by a little factor to make sure the best
//			 position is found, even when it is slightly outside the bounding Box.
//		II.  Put a grid over the bounding box to get cells in which the trolley-position possibly is.
//		     Find the Point in these cells that have the largest distance to the closest zero Pixel as possible candidates
//			 for trolley positions.
//		III. From these candidates the one is chosen, which gets the smallest pathlength to all group Points. If the group
//			 has only two members the algorithm chooses the candidate as trolley position that is the middlest between these.
cv::Point TrolleyPositionFinder::findOneTrolleyPosition(const std::vector<cv::Point> group_points, const cv::Mat& original_map,
		const double downsampling_factor, const double robot_radius, const double map_resolution)
{
	double largening_of_bounding_box = 5; //Variable to expand the bounding box of the roomcenters a little bit. This is done to make sure the best trolley position is found if it is a little bit outside this bounding box.
	double max_x_value = group_points[0].x; //max/min values of the Points that get the bounding box. Initialized with the coordinates of the first Point of the group.
	double min_x_value = group_points[0].x;
	double max_y_value = group_points[0].y;
	double min_y_value = group_points[0].y;

	//create eroded map, which is used to check if the trolley-position candidates are too close to the boundaries
	cv::Mat eroded_map;
	cv::erode(original_map, eroded_map, cv::Mat(), cv::Point(-1, -1), 4);

	//create the distance-map to find the candidates for trolley-Positions
	cv::Mat temporary_map = original_map.clone();
	cv::erode(temporary_map, temporary_map, cv::Mat());
	cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
#if CV_MAJOR_VERSION<=3
	cv::distanceTransform(temporary_map, distance_map, CV_DIST_L2, 5);
#else
	cv::distanceTransform(temporary_map, distance_map, cv::DIST_L2, 5);
#endif
	cv::convertScaleAbs(distance_map, distance_map); // conversion to 8 bit image

	//
	//******************************** I. Get bounding box of the group ********************************
	//
	//go trough each Point and find the min/max x/y values --> bounding box
	for (int point = 0; point < group_points.size(); point++)
	{
		if (group_points[point].x > max_x_value)
		{
			max_x_value = group_points[point].x;
		}
		if (group_points[point].x < min_x_value)
		{
			min_x_value = group_points[point].x;
		}
		if (group_points[point].y > max_y_value)
		{
			max_y_value = group_points[point].y;
		}
		if (group_points[point].y < min_y_value)
		{
			min_y_value = group_points[point].y;
		}
	}

	//expand the bounding box sligthly by the defined factor (check if the values aren't out of the map boundaries before doing this)
	if (max_x_value + largening_of_bounding_box < original_map.cols)
	{
		max_x_value += largening_of_bounding_box;
	}
	if (min_x_value - largening_of_bounding_box > 0)
	{
		min_x_value -= largening_of_bounding_box;
	}
	if (max_y_value + largening_of_bounding_box < original_map.rows)
	{
		max_y_value += largening_of_bounding_box;
	}
	if (min_y_value - largening_of_bounding_box > 0)
	{
		min_y_value -= largening_of_bounding_box;
	}

	//
	//******************************** II. Get the candidates for trolley positions ********************************
	//
	double cell_side_length = 10;
	double max_x_cell = min_x_value + cell_side_length;
	double min_x_cell = min_x_value;
	double max_y_cell = min_y_value + cell_side_length;
	double min_y_cell = min_y_value;

	bool out_of_y = false;
	bool out_of_x = false;

	std::vector < cv::Point > trolley_position_candidates;

	//go trough each cell and find the candidate for each of it
	do //go from y_min to y_max
	{
		max_x_cell = min_x_value + cell_side_length; //reset the x-values for the cell to start from beginning when a new y coordinate is reached
		min_x_cell = min_x_value;
		do //go from x_min to x_max
		{
			out_of_x = false;
			double best_x = min_x_cell;
			double best_y = min_y_cell;
			//go trough each Pixel of the cell and take the one that is most far away of the zero Pixels.
			for (int y = min_y_cell; y < max_y_cell; y++)
			{
				for (int x = min_x_cell; x < max_x_cell; x++)
				{
					if (distance_map.at<unsigned char>(best_y, best_x) < distance_map.at<unsigned char>(y, x) && eroded_map.at<unsigned char>(y, x) != 0)
					{
						best_x = x;
						best_y = y;
					}
				}
			}
			//check if candidate is far enough away from boundary
			if (eroded_map.at<unsigned char>(best_y, best_x) != 0)
			{
				trolley_position_candidates.push_back(cv::Point(best_x, best_y));
			}
			min_x_cell = max_x_cell; //set new x values for next step
			max_x_cell += cell_side_length;

			//check if x is out of box --> if so the next y values should be checked
			if (min_x_cell > max_x_value)
			{
				out_of_x = true;
			}
		} while (!out_of_x);

		min_y_cell = max_y_cell; //set new y values for next step
		max_y_cell += cell_side_length;

		//check if y is out of bounding box --> if true step is done
		if (min_y_cell > max_y_value)
		{
			out_of_y = true;
		}
	} while (!out_of_y);

	//
	//***************** III. Find the candidate that minimizes the pathlengths to all group points *****************
	//
	//variables to save the best candidate
	double best_pathlength = 1e10;
	double best_pathlength_point_distance = 1e10;
	int best_trolley_candidate = 0;

	// reduce image size already here to avoid resizing in the planner each time
	const double one_by_downsampling_factor = 1./downsampling_factor;
	cv::Mat downsampled_map;
	path_planner_.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

	//go trough each candidate and calculate the sum of pathlengths
	for (size_t candidate = 0; candidate < trolley_position_candidates.size(); candidate++)
	{
		cv::Point start_point = downsampling_factor * trolley_position_candidates[candidate];
		double current_pathlength = 0;
		std::vector<double> pathlengths;
		for (int room_center = 0; room_center < group_points.size(); room_center++)
		{
			cv::Point end_point = downsampling_factor * group_points[room_center];
			//get the pathlength to the current center and save it
			double center_pathlength = one_by_downsampling_factor * path_planner_.planPath(downsampled_map, start_point, end_point, 1., 0., map_resolution);
			pathlengths.push_back(center_pathlength);
			//add the pathlenght to the total pathlength
			current_pathlength += center_pathlength;
		}
		//check for the best position that has the shortest pathlength to all centers. Adding a little bit to the best_distances
		//because the downsampling generates an error and with this better positions can be found.
		if (group_points.size() == 2)
		{
			//If the group only has two members check for the position that is in the middlest of the connectionpath between
			//these points or else a random point will be chosen.
			double current_point_distance = std::abs(pathlengths[1] - pathlengths[0]);
			if (current_pathlength <= (best_pathlength + 0.05) && current_point_distance <= (best_pathlength_point_distance + 0.05)
					&& downsampled_map.at<unsigned char>(downsampling_factor * trolley_position_candidates[best_trolley_candidate]) != 0)
			{
				best_pathlength_point_distance = current_point_distance;
				best_pathlength = current_pathlength;
				best_trolley_candidate = candidate;
			}
		}
		else
		{
			if (current_pathlength <= (best_pathlength + 0.05))
			{
				best_pathlength = current_pathlength;
				best_trolley_candidate = candidate;
			}
		}
	}

	return trolley_position_candidates[best_trolley_candidate];
}

//This function takes all found groups and calculates for each of it the best trolley-position using the previously
//described functions.
std::vector<cv::Point> TrolleyPositionFinder::findTrolleyPositions(const cv::Mat& original_map, const std::vector<std::vector<int> >& found_groups,
		const std::vector<cv::Point>& room_centers, const double downsampling_factor, const double robot_radius, const double map_resolution)
{
	std::vector < cv::Point > trolley_positions;

	//go trough each group and find the best trolley position.
	for (int current_group = 0; current_group < found_groups.size(); current_group++)
	{
		std::vector < cv::Point > group_points_vector; //vector to save the Points for each group

		//add the Points from the given groups vector
		for (int index = 0; index < found_groups[current_group].size(); index++)
		{
			group_points_vector.push_back(room_centers[found_groups[current_group][index]]);
		}
		//calculate the trolley-position for each group that has at least 2 members
		if (found_groups[current_group].size() > 1)
		{
			trolley_positions.push_back(
						findOneTrolleyPosition(group_points_vector, original_map, downsampling_factor, robot_radius, map_resolution));
		}
		else //if the group has only one member this one is the trolley-position
		{
			cv::Point trolley_position_for_one_sized_groups = room_centers[found_groups[current_group][0]];
			trolley_positions.push_back(trolley_position_for_one_sized_groups);
		}
	}

	return trolley_positions;
}

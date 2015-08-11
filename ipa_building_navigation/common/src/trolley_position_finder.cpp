#include <ipa_building_navigation/trolley_position_finder.h>

trolleyPositionFinder::trolleyPositionFinder()
{

}

//This function takes one group and calculates the trolley position for it. This is done by getting the bounding Box for
//all Points in it and then putting a grid over it to get cells in which the trolley-position possibly is. Then it finds
//the Point in these cells that have the largest distance to the closest zero Pixel as candidates. From these candidates
//the one is chosen, which gets the smallest pathlength to all group Points.
cv::Point trolleyPositionFinder::findOneTrolleyPosition(const std::vector<cv::Point> group_points, const cv::Mat& original_map, double downsampling_factor)
{
	double n = 20;
	double max_x_value = group_points[0].x + n; //max/min values of the Points that get the bounding box. Initialized with the coordinates of the first Point of the group.
	double min_x_value = group_points[0].x - n;
	double max_y_value = group_points[0].y + n;
	double min_y_value = group_points[0].y - n;

	//create eroded map, which is used to check if the trolley-position candidates are too close to the boundaries
	cv::Mat eroded_map;
	cv::erode(original_map, eroded_map, cv::Mat(), cv::Point(-1, -1), 4);

	//create the distance-map to find the candidates for trolley-Positions
	cv::Mat temporary_map = original_map.clone();
	cv::erode(temporary_map, temporary_map, cv::Mat());
	cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
	cv::distanceTransform(temporary_map, distance_map, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map, distance_map); // conversion to 8 bit image

	//go trough each Point and find the min/max x/y values
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

	//get the candidates
	double max_x_cell = min_x_value + 5;
	double min_x_cell = min_x_value;
	double max_y_cell = min_y_value + 5;
	double min_y_cell = min_y_value;

	bool out_of_y = false;
	bool out_of_x = false;

	std::vector < cv::Point > trolley_position_candidates;

	do
	{
		max_x_cell = min_x_value + 5; //reset the x-values for the cell
		min_x_cell = min_x_value;
		do
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
			if (eroded_map.at<unsigned char>(best_y, best_x) != 0)
			{
				trolley_position_candidates.push_back(cv::Point(best_x, best_y));
			}
			min_x_cell = max_x_cell;
			max_x_cell += 5;
			if (min_x_cell > max_x_value)
			{
				out_of_x = true;
			}
		} while (!out_of_x);
		min_y_cell = max_y_cell;
		max_y_cell += 5;
		if (min_y_cell > max_y_value)
		{
			out_of_y = true;
		}
	} while (!out_of_y);

//	temporary_map = original_map.clone();
//
//	cv::line(temporary_map, cv::Point(min_x_value, min_y_value), cv::Point(max_x_value, min_y_value), cv::Scalar(127), 1);
//	cv::line(temporary_map, cv::Point(min_x_value, min_y_value), cv::Point(min_x_value, max_y_value), cv::Scalar(127), 1);
//	cv::line(temporary_map, cv::Point(max_x_value, max_y_value), cv::Point(max_x_value, min_y_value), cv::Scalar(127), 1);
//	cv::line(temporary_map, cv::Point(max_x_value, max_y_value), cv::Point(min_x_value, max_y_value), cv::Scalar(127), 1);
//	for (int i = 0; i < trolley_position_candidates.size(); i++)
//	{
//		cv::circle(temporary_map, trolley_position_candidates[i], 2, cv::Scalar(127), CV_FILLED);
//	}
//
//	cv::imwrite("/home/rmb-fj/Pictures/TSP/group.png", temporary_map);
//
//	temporary_map = original_map.clone();

	//find the candidate that minimizes the pathlengths to all roomcenters and choose this as trolley-position
	double best_pathlength = 9001;
	double best_pathlength_point_distance = 9001;
	int best_trolley_candidate = 0;
	for (int candidate = 0; candidate < trolley_position_candidates.size(); candidate++)
	{
		double current_pathlength = 0;
		std::vector<double> pathlengths;
		for (int room_center = 0; room_center < group_points.size(); room_center++)
		{
			cv::Point current_center = group_points[room_center];
			//get the pathlength to the current center and save it
			double center_pathlength = path_planner_.PlanPath(original_map, trolley_position_candidates[candidate], current_center, downsampling_factor);
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
			if (current_pathlength <= (best_pathlength + 0.05) && current_point_distance <= (best_pathlength_point_distance + 0.05))
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

//	cv::circle(temporary_map, trolley_position_candidates[best_trolley_candidate], 2, cv::Scalar(127), CV_FILLED);
//
//	cv::imshow("test", temporary_map);
//	cv::waitKey();
//	cv::imwrite("/home/rmb-fj/Pictures/TSP/position.png", temporary_map);

	return trolley_position_candidates[best_trolley_candidate];
}

//This function takes all found groups and calculates for each of it the best trolley-position.
std::vector<cv::Point> trolleyPositionFinder::findTrolleyPositions(const cv::Mat& original_map, const std::vector<std::vector<int> >& found_groups,
        const std::vector<cv::Point>& room_centers, double downsampling_factor_from_subscription)
{
	std::vector < cv::Point > trolley_positions;

	for (int current_group = 0; current_group < found_groups.size(); current_group++)
	{
		std::vector < cv::Point > group_points_vector;
		//create the Points out of the indexes
		for (int index = 0; index < found_groups[current_group].size(); index++)
		{
			group_points_vector.push_back(room_centers[found_groups[current_group][index]]);
		}
		if (found_groups[current_group].size() > 1) //calculate the trolley-position for each group that has at least 2 members
		{
			trolley_positions.push_back(findOneTrolleyPosition(group_points_vector, original_map, downsampling_factor_from_subscription));
		}
		else //if the group has only one member this one is the trolley-position
		{
			cv::Point trolley_position_for_one_sized_groups = room_centers[found_groups[current_group][0]];
			trolley_positions.push_back(trolley_position_for_one_sized_groups);
		}
	}

	return trolley_positions;
}



#pragma once

#include <vector>
#include <opencv/cv.h>
#include <ipa_building_navigation/A_star_pathplanner.h>

class DistanceMatrix
{
public:
	static void constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, AStarPlanner& path_planner)
	{
		//create the distance matrix with the right size
		distance_matrix.create((int)points.size(), (int)points.size(), CV_64F);

		// reduce image size already here to avoid resizing in the planner each time
		const double one_by_downsampling_factor = 1./downsampling_factor;
		cv::Mat downsampled_map;
		path_planner.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

		for (int i = 0; i < points.size(); i++)
		{
			cv::Point current_center = downsampling_factor * points[i];
			for (int j = 0; j < points.size(); j++)
			{
				if (j != i)
				{
					if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
					{
						cv::Point neighbor = downsampling_factor * points[j];
						double length = one_by_downsampling_factor * path_planner.planPath(downsampled_map, current_center, neighbor, 1., 0., map_resolution);
						if(length > 9000) //an empty route has been generated, check if the not downsampled map gives a rout
							length = path_planner.planPath(original_map, one_by_downsampling_factor * current_center, one_by_downsampling_factor * neighbor, 1., 0., map_resolution);
						distance_matrix.at<double>(i, j) = length;
						distance_matrix.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computation time
					}
				}
				else
				{
					distance_matrix.at<double>(i, j) = 0;
				}
			}
		}
	}
};

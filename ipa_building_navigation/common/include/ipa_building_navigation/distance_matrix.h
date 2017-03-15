

#pragma once

#include <vector>
#include <opencv/cv.h>
#include <ipa_building_navigation/A_star_pathplanner.h>

#include <ipa_building_navigation/timer.h>

class DistanceMatrix
{
public:
	// REMARK:	paths is a pointer that points to a 3D vector that has dimensionality NxN in the outer vectors to store
	//			the paths in a matrix manner
	static void constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, AStarPlanner& path_planner,
			std::vector<std::vector<std::vector<cv::Point> > >* paths=NULL)
	{
		Timer tim;

		//create the distance matrix with the right size
		distance_matrix.create((int)points.size(), (int)points.size(), CV_64F);

		// reduce image size already here to avoid resizing in the planner each time
		const double one_by_downsampling_factor = 1./downsampling_factor;
		cv::Mat downsampled_map;
		path_planner.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

		for (int i = 0; i < points.size(); i++)
		{
			//cv::Point current_center = downsampling_factor * points[i];
			for (int j = 0; j < points.size(); j++)
			{
				if (j != i)
				{
					if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
					{
						if(paths!=NULL)
						{
							std::vector<cv::Point> current_path;
							double length = path_planner.planPath(original_map, downsampled_map, points[i], points[j], downsampling_factor, 0., map_resolution, 0, NULL, &current_path);
							distance_matrix.at<double>(i, j) = length;
							distance_matrix.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computation time

							// remap path points to original map size
							for(std::vector<cv::Point>::iterator point=current_path.begin(); point!=current_path.end(); ++point)
							{
								point->x = point->x/downsampling_factor;
								point->y = point->y/downsampling_factor;
							}

							paths->at(i).at(j) = current_path;
							paths->at(j).at(i) = current_path;
						}
						else
						{
							double length = path_planner.planPath(original_map, downsampled_map, points[i], points[j], downsampling_factor, 0., map_resolution);
							distance_matrix.at<double>(i, j) = length;
							distance_matrix.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computation time

						}
					}
				}
				else
				{
					distance_matrix.at<double>(i, j) = 0;
				}
			}
		}

		std::cout << "Distance matrix created in " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;// "\nDistance matrix:\n" << distance_matrix << std::endl;
	}
};

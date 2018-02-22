

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <ipa_building_navigation/A_star_pathplanner.h>

#include <ipa_building_navigation/timer.h>

class DistanceMatrix
{
protected:

	bool abort_computation_;

public:

	DistanceMatrix()
	: abort_computation_(false)
	{
	}

	void abortComputation()
	{
		abort_computation_ = true;
	}

	// REMARK:	paths is a pointer that points to a 3D vector that has dimensionality NxN in the outer vectors to store
	//			the paths in a matrix manner
	void constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const std::vector<cv::Point>& points,
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
						if (abort_computation_==true)
							return;

						// try first with direct connecting line (often sufficient)
						cv::LineIterator it(original_map, points[i], points[j]);
						bool direct_connection = true;
						for (int k=0; k<it.count && direct_connection==true; k++, ++it)
							if (**it < 250)
								direct_connection = false;		// if a pixel in between is not accessible, direct connection is not possible
						if (direct_connection == true)
						{
							// compute distance
							const double length = cv::norm(points[i]-points[j]);
							distance_matrix.at<double>(i, j) = length;
							distance_matrix.at<double>(j, i) = length; //symmetrical-Matrix --> saves half the computation time
							if (paths!=NULL)
							{
								// store path
								cv::LineIterator it2(original_map, points[i], points[j]);
								std::vector<cv::Point> current_path(it2.count);
								for (int k=0; k<it2.count; k++, ++it2)
									current_path[k] = it2.pos();
								paths->at(i).at(j) = current_path;
								paths->at(j).at(i) = current_path;
							}
						}
						else
						{
							// A* path planner
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

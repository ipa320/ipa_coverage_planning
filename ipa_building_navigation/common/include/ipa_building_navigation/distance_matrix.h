

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

	//Function to construct the symmetrical distance matrix from the given points. The rows show from which node to start and
	//the columns to which node to go. If the path between nodes doesn't exist or the node to go to is the same as the one to
	//start from, the entry of the matrix is 0.
	// REMARK:	paths is a pointer that points to a 3D vector that has dimensionality NxN in the outer vectors to store
	//			the paths in a matrix manner
	void constructDistanceMatrix(cv::Mat& distance_matrix, const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, AStarPlanner& path_planner,
			std::vector<std::vector<std::vector<cv::Point> > >* paths=NULL)
	{
		std::cout << "DistanceMatrix::constructDistanceMatrix: Constructing distance matrix..." << std::endl;
		Timer tim;

		//create the distance matrix with the right size
		distance_matrix.create((int)points.size(), (int)points.size(), CV_64F);

		// hack: speed up trick
		if (points.size()>500)
			downsampling_factor *= 0.5;

		// reduce image size already here to avoid resizing in the planner each time
		const double one_by_downsampling_factor = 1./downsampling_factor;
		cv::Mat downsampled_map;
		path_planner.downsampleMap(original_map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

		if (points.size()>500)
			std::cout << "0         10        20        30        40        50        60        70        80        90        100" << std::endl;
//		int a=0, b=0;
		for (int i = 0; i < points.size(); i++)
		{
			//std::cout << "  a=" << a << "   b=" << b << std::endl;
			if (points.size()>500 && i%(std::max(1,(int)points.size()/100))==0)
				std::cout << "." << std::flush;
			//cv::Point current_center = downsampling_factor * points[i];
			for (int j = 0; j < points.size(); j++)
			{
				if (j != i)
				{
					if (j > i) //only compute upper right triangle of matrix, rest is symmetrically added
					{
						if (abort_computation_==true)
							return;

						// try first with direct connecting line (often sufficient and a significant speedup over A*)
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
//							++a;
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
//							++b;
						}
					}
				}
				else
				{
					distance_matrix.at<double>(i, j) = 0;
				}
			}
		}

		std::cout << "\nDistance matrix created in " << tim.getElapsedTimeInMilliSec() << " ms" << std::endl;// "\nDistance matrix:\n" << distance_matrix << std::endl;
	}

	// check whether distance matrix contains infinite path lengths and if this is true, create a new distance matrix with maximum size clique of reachable points
	// cleaned_index_to_original_index_mapping --> maps the indices of the cleaned distance_matrix to the original indices of the original distance_matrix
	void cleanDistanceMatrix(const cv::Mat& distance_matrix, cv::Mat& distance_matrix_cleaned, std::map<int,int>& cleaned_index_to_original_index_mapping)
	{
		// standard: use a 1:1 mapping (input = output)
		cleaned_index_to_original_index_mapping.clear();
		for (int i=0; i<distance_matrix.rows; ++i)
			cleaned_index_to_original_index_mapping[i] = i;
		distance_matrix_cleaned = distance_matrix.clone();

		if (distance_matrix.rows < 1)
			return;

		const double max_length = 1e90;

		std::vector<bool> remove_entry(distance_matrix.rows, false);	// keeps track on which entries of distance_matrix need to be removed

		// loop until all lines are marked, which need to be removed
		cv::Mat distance_matrix_temp = distance_matrix.clone();
		while (true)
		{
			// count infinite entries in each row of distance matrix
			std::vector<int> infinite_length_entries(distance_matrix_temp.rows, 0);
			for (int i=0; i<distance_matrix_temp.rows; ++i)
				for (int j=0; j<distance_matrix_temp.cols; ++j)
					if (distance_matrix_temp.at<double>(i,j)>max_length)
						infinite_length_entries[i]++;

			// sort rows by their number of infinite entries
			std::multimap<int, int> number_infinite_entries_to_row_index_mapping;	// maps number of infinite entries to the corresponding row index
			for (size_t i=0; i<infinite_length_entries.size(); ++i)
				number_infinite_entries_to_row_index_mapping.insert(std::pair<int,int>(infinite_length_entries[i], (int)i));

			// if any row has at least one infinite entry, mark the row with most infinity entries for deletion
			bool mark_line = false;
			int mark_index = -1;
			std::multimap<int, int>::reverse_iterator number_infinite_entries_to_row_index_mapping_last = number_infinite_entries_to_row_index_mapping.rbegin();
			if (number_infinite_entries_to_row_index_mapping_last->first > 0)
			{
				mark_line = true;
				mark_index = number_infinite_entries_to_row_index_mapping_last->second;
				remove_entry[mark_index] = true;
			}
			if (mark_line == true)
			{
				for (int j=0; j<distance_matrix_temp.cols; ++j)			// mark row: mark_index
					distance_matrix_temp.at<double>(mark_index, j) = -1.;
				for (int i=0; i<distance_matrix_temp.rows; ++i)			// mark col: mark_index
					distance_matrix_temp.at<double>(i, mark_index) = -1.;
			}
			else
				break;
		}

		// count entries to remove
		int number_entries_to_be_removed = 0;
		for (size_t i=0; i<remove_entry.size(); ++i)
			if (remove_entry[i] == true)
				number_entries_to_be_removed++;

		// remove elements from distance matrix if necessary
		if (number_entries_to_be_removed > 0)
		{
			std::cout << "  DistanceMatrix::cleanDistanceMatrix: Need to remove " << number_entries_to_be_removed << " elements out of " << distance_matrix.rows << " elements from the distance matrix." << std::endl;

			// setup new distance_matrix
			const int new_size = distance_matrix.rows - number_entries_to_be_removed;
			if (new_size == 0)
			{
				std::cout << "  DistanceMatrix::cleanDistanceMatrix: Warning: Would need to remove all elements of distance_matrix. Aborting." << std::endl;
				return;
			}
			distance_matrix_cleaned.create(new_size, new_size, CV_64F);
			cleaned_index_to_original_index_mapping.clear();

			// fill new distance_matrix
			int new_index = 0;
			for (size_t i=0; i<remove_entry.size(); ++i)
			{
				if (remove_entry[i] == false)
				{
					// mapping from new to old indices
					cleaned_index_to_original_index_mapping[new_index] = (int)i;

					// copy values
					int new_j = 0;
					for (size_t j=0; j<remove_entry.size(); ++j)
					{
						if (remove_entry[j] == false)
						{
							distance_matrix_cleaned.at<double>(new_index, new_j) = distance_matrix.at<double>(i,j);
							new_j++;
						}
					}
					new_index++;
				}
			}
			if (new_index != new_size)
				std::cout << "##################################################\nDistanceMatrix::cleanDistanceMatrix: Warning: new_index != new_size.\n##################################################" << std::endl;
		}
	}

	// calculate the distance matrix and check whether distance matrix contains infinite path lengths and if this is true,
	// create a new distance matrix with maximum size clique of reachable points
	// start_node --> provide the original start node to the function, it writes the new start node mapped to the new coordinates into it
	// cleaned_index_to_original_index_mapping --> maps the indices of the cleaned distance_matrix to the original indices of the original distance_matrix
	void computeCleanedDistanceMatrix(const cv::Mat& original_map, const std::vector<cv::Point>& points,
			double downsampling_factor, double robot_radius, double map_resolution, AStarPlanner& path_planner,
			cv::Mat& distance_matrix, std::map<int,int>& cleaned_index_to_original_index_mapping, int& start_node)
	{
		std::cout << "DistanceMatrix::computeCleanedDistanceMatrix: Constructing distance matrix..." << std::endl;
		// calculate the distance matrix
		cv::Mat distance_matrix_raw;
		constructDistanceMatrix(distance_matrix_raw, original_map, points, downsampling_factor, robot_radius, map_resolution, path_planner);

		// check whether distance matrix contains infinite path lengths and if this is true, create a new distance matrix with maximum size clique of reachable points
		cleanDistanceMatrix(distance_matrix_raw, distance_matrix, cleaned_index_to_original_index_mapping);

		// re-assign the start node to cleaned indices (use 0 if the original start node was removed from distance_matrix_cleaned)
		int new_start_node = 0;
		for (std::map<int,int>::iterator it=cleaned_index_to_original_index_mapping.begin(); it!=cleaned_index_to_original_index_mapping.end(); ++it)
			if (it->second == start_node)
				new_start_node = it->first;
		start_node = new_start_node;
	}
};

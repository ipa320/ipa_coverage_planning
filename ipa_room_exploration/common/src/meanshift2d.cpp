/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_segmentation
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 22.07.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "ipa_room_exploration/meanshift2d.h"
#include "ipa_room_exploration/fast_math.h"

void MeanShift2D::filter(const std::vector<cv::Vec2d>& data, std::vector<cv::Vec2d>& filtered_data, const double bandwidth, const int maximumIterations)
{
	//  prepare mean shift set
	std::vector<cv::Vec2d> mean_shift_set(data.size());
	filtered_data = data;

	//  mean shift iteration
	for (int iter=0; iter<maximumIterations; iter++)
	{
		for (int i=0; i<(int)filtered_data.size(); i++)
		{
			cv::Vec2d nominator(0., 0.);
			double denominator = 0.;
			for (int j=0; j<(int)filtered_data.size(); j++)
			{
				cv::Vec2d diff = filtered_data[j]-filtered_data[i];
				double weight = fasterexp(-bandwidth * (diff[0]*diff[0] + diff[1]*diff[1]));
				nominator += weight*filtered_data[j];
				denominator += weight;
			}
			mean_shift_set[i] = nominator/denominator;
		}
		filtered_data = mean_shift_set;
	}
//	for (int i=0; i<(int)filtered_data.size(); i++)
//		std::cout << "  meanshift[" << i << "] = (" << filtered_data[i][0] << ", " << filtered_data[i][1] << ")" << std::endl;
}


void MeanShift2D::computeConvergencePoints(const std::vector<cv::Vec2d>& filtered_data, std::vector<cv::Vec2d>& convergence_points, std::vector< std::vector<int> >& convergence_sets, const double sensitivity)
{
	//  cluster data according to convergence points
	convergence_sets.resize(1, std::vector<int>(1, 0));
	convergence_points.resize(1, filtered_data[0]);
	for (int i=1; i<(int)filtered_data.size(); i++)
	{
		bool create_new_set = true;
		for (int j=0; j<(int)convergence_points.size(); j++)
		{
			if (cv::norm(filtered_data[i]-convergence_points[j]) < sensitivity)
			{
				convergence_sets[j].push_back(i);
				convergence_points[j] = (convergence_points[j]*(convergence_sets[j].size()-1.) + filtered_data[i]) / (double)convergence_sets[j].size();	// update mean of convergence point
				create_new_set = false;
				break;
			}
		}
		if (create_new_set == true)
		{
			convergence_sets.push_back(std::vector<int>(1, i));
			convergence_points.push_back(filtered_data[i]);
		}
	}
}

cv::Vec2d MeanShift2D::findRoomCenter(const cv::Mat& room_image, const std::vector<cv::Vec2d>& room_cells, double map_resolution)
{
	// downsample data if too big
	std::vector<cv::Vec2d> room_cells_sampled;
	if (room_cells.size() > 1000)
	{
		const int factor = room_cells.size()/1000;
		for (size_t i=0; i<room_cells.size(); ++i)
			if ((int)i % factor == 0)
				room_cells_sampled.push_back(room_cells[i]);
	}
	else
		room_cells_sampled = room_cells;

	// mean shift filter
	const double bandwidth = map_resolution/2.;		// parameter
	std::vector<cv::Vec2d> filtered_room_cells;
	filter(room_cells_sampled, filtered_room_cells, bandwidth, 100);

	// take mean of filtering result in simple rooms, i.e. if no obstacles is located at the computed cell
	cv::Scalar mean_coordinates = cv::mean(filtered_room_cells);
	cv::Vec2d room_center(mean_coordinates[0], mean_coordinates[1]);
	if (room_image.at<uchar>(room_center[1], room_center[0]) == 255)
		return room_center;

	// otherwise compute convergence sets of filtering results and return mean of largest convergence set
	// determine convergence points
	std::vector<cv::Vec2d> convergence_points;
	std::vector< std::vector<int> > convergence_sets;
	computeConvergencePoints(filtered_room_cells, convergence_points, convergence_sets, 1./bandwidth*0.1);

	// select convergence point with largest point support
	size_t max_index = 0;
	size_t max_size = 0;
	for (size_t i=0; i<convergence_sets.size(); ++i)
	{
		if (convergence_sets[i].size() > max_size)
		{
			max_size = convergence_sets[i].size();
			max_index = i;
		}
	}
	return convergence_points[max_index];
}

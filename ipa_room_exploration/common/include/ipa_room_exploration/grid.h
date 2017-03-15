/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
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
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 03.2017
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

#pragma once
#include <vector>
#include <opencv/cv.h>


class GridGenerator
{
public:
	GridGenerator()
	{
	}

	// generates a standard grid with fixed spacing starting at the upper left accessible cell (because it is assumed that the map is provided
	// with walls grown by the size of half robot radius, i.e. the map is eroded by the inaccessible areas). If the map is provided in its normal
	// appearance, then the start_grid_at_first_free_pixel parameter should be set to false and the first grid center will be placed with appropriate
	// distance to the walls.
	// room_map = (optionally eroded) map of the room in [pixels] (erosion = growing of walls by robot radius --> inaccessible areas are already excluded)
	// cell_centers = the computed grid cell center points in [pixels]
	// cell_size = the grid spacing in [pixels]
	// complete_cell_test = if this is set false, cells are only inserted to the grid if their centers are accessible. If set to true, then the cell
	//                      is created if there are any accessible cells within the cell area. The cell center is however set to one of the accessible pixels.
	// start_grid_at_first_free_pixel = if set to true, it is assumed that room_map is eroded by the inaccessible areas and hence the first cell center
	//                                  should be placed in the upper left pixel. If this is false, the cell center will be placed off the wall with
	//                                  cell_size spacing.
	void generateStandardGrid(const cv::Mat& room_map, std::vector<cv::Point>& cell_centers, const int cell_size,
			const bool complete_cell_test=true, const bool start_grid_at_first_free_pixel=true)
	{
		// determine min and max coordinates of the room
		int min_y = 1000000, max_y = 0, min_x = 1000000, max_x = 0;
		for (int y=0; y<room_map.rows; y++)
		{
			for (int x=0; x<room_map.cols; x++)
			{
				//find not reachable regions and make them black
				if (room_map.at<unsigned char>(y,x)==255)
				{
					if(x<min_x)
						min_x = x;
					if(x>max_x)
						max_x = x;
					if(y<min_y)
						min_y = y;
					if(y>max_y)
						max_y = y;
				}
			}
		}

		// if the map was not eroded, the grid cell centers might be placed with cell_size/2 distance from the wall
		const uint16_t half_cell_size = (uint16_t)cell_size/(uint16_t)2;		// just ensure that the rounding is always reproducible
		if (start_grid_at_first_free_pixel == false)
		{
			min_x += half_cell_size;
			min_y += half_cell_size;
		}

		// create the grid
		if (complete_cell_test == true)
		{
			for(int y=min_y; y<=max_y; y+=cell_size)
			{
				for(int x=min_x; x<=max_x; x+=cell_size)
				{
					cv::Point cell_center(x,y);
					if (completeCellTest(room_map, cell_center, cell_size) == true)
						cell_centers.push_back(cell_center);
				}
			}
		}
		else
		{
			// only create cells where the cell center is accessible
			for (int y=min_y; y<=max_y; y+=cell_size)
				for (int x=min_x; x<=max_x; x+=cell_size)
					if (room_map.at<unsigned char>(y,x)==255)
						cell_centers.push_back(cv::Point(x,y));
		}
	}

	// checks the whole cell for accessible areas and sets cell_center to the cell-center-most accessible point in the cell
	// room_map = the map with inaccessible areas = 0 and accessible areas = 255
	// cell_center = the provided cell center point to check, is updated with a new cell center if the original cell_center is not accessible but some other pixels in the cell around
	// cell_size = the grid spacing in [pixels]
	// returns true if any accessible cell was found in the cell area and then cell_center is returned with an updated value. If the cell does not contain
	//         any accessible pixel, the return value is false.
	bool completeCellTest(const cv::Mat& room_map, cv::Point& cell_center, const int cell_size)
	{
		const int x = cell_center.x;
		const int y = cell_center.y;
		if (room_map.at<unsigned char>(y,x)==255)
		{
			// just take cell center if accessible
			return true;
		}
		else
		{
			const uint16_t half_cell_size = (uint16_t)cell_size/(uint16_t)2;		// just ensure that the rounding is always reproducible
			const bool even_grid_size = ((cell_size%2)==0);

			// check whether there are accessible pixels within the cell
			const int upper_bound = even_grid_size==true ? half_cell_size-1 : half_cell_size;	// adapt the neighborhood accordingly for even and odd grid sizes
			cv::Mat cell_pixels = cv::Mat::zeros(cell_size, cell_size, CV_8UC1);
			int accessible_pixels = 0;
			for (int dy=-half_cell_size; dy<=upper_bound; ++dy)
			{
				for (int dx=-half_cell_size; dx<=upper_bound; ++dx)
				{
					const int nx = x+dx;
					const int ny = y+dy;
					if (nx<0 || nx>=room_map.cols || ny<0 || ny>=room_map.rows)
						continue;
					if (room_map.at<unsigned char>(ny,nx)==255)
					{
						++accessible_pixels;
						cell_pixels.at<unsigned char>(half_cell_size+dy, half_cell_size+dx) = 255;
					}
				}
			}
			// if there are accessible pixels within the cell, find their center and use this as cell center
			if (accessible_pixels>0)
			{
				// use distance transform to find the pixels with maximum distance to obstacles, take from the maximum distance pixels the one
				// closest to the original cell center
				cv::Mat distances;
				cv::distanceTransform(cell_pixels, distances, CV_DIST_L2, 5);
				double max_distance = 0.;
				cv::Point cell_center;
				cv::minMaxLoc(distances, 0, &max_distance, 0, &cell_center);
				cell_center.x += x-half_cell_size;
				cell_center.y += y-half_cell_size;
				// if there are multiple candidates with same max distance, take the one closest to the center
				double min_squared_center_distance = (x-cell_center.x)*(x-cell_center.x) + (y-cell_center.y)*(y-cell_center.y);
				for (int v=0; v<distances.rows; ++v)
				{
					for (int u=0; u<distances.cols; ++u)
					{
						if ((double)distances.at<float>(v,u)==max_distance)
						{
							const double squared_center_distance = (u-half_cell_size)*(u-half_cell_size)+(v-half_cell_size)*(v-half_cell_size);
							if (squared_center_distance < min_squared_center_distance)
							{
								cell_center = cv::Point(x-half_cell_size+u, y-half_cell_size+v);
								min_squared_center_distance = squared_center_distance;
							}
						}
					}
				}
				return true;
			}
		}
		return false;
	}
};

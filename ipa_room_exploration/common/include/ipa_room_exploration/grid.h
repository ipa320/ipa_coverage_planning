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
	GridGenerator()
	{
	}

	// generates a standard grid with fixed spacing starting at the upper left accessible cell (because it is assumed that the map is provided
	// with walls grown by the size of half robot radius, i.e. the map is eroded by the inaccessible areas). If the map is provided in its normal
	// appearance, then the start_grid_at_first_free_pixel parameter should be set to false and the first grid center will be placed with appropriate
	// distance to the walls.
	// room_map = (optionally eroded) map of the room in [pixels] (erosion = growing of walls by robot radius --> inaccessible areas are already excluded)
	// cell_size = the grid spacing in [pixels]
	// cell_centers = the computed grid cell center points in [pixels]
	// complete_cell_test = if this is set false, cells are only inserted to the grid if their centers are accessible. If set to true, then the cell
	//                      is created if there are any accessible cells within the cell area. The cell center is however set to one of the accessible pixels.
	// start_grid_at_first_free_pixel = if set to true, it is assumed that room_map is eroded by the inaccessible areas and hence the first cell center
	//                                  should be placed in the upper left pixel. If this is false, the cell center will be placed off the wall with
	//                                  cell_size spacing.
	void generateStandardGrid(const cv::Mat& room_map, const int cell_size, std::vector<cv::Point>& cell_centers,
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
					if(y<min_y)
						min_y = y;
					if(y>max_y)
						max_y = y;
					if(x<min_x)
						min_x = x;
					if(x>max_x)
						max_x = x;
				}
			}
		}

		// if the map was not eroded, the grid cell centers might be placed with cell_size/2 distance from the wall
		if (start_grid_at_first_free_pixel == false)
		{
			min_x += cell_size/2;
			min_y += cell_size/2;
		}

		// create the grid
		if (complete_cell_test == true)
		{
			for(size_t y=min_y; y<=max_y; y+=cell_size)
			{
				for(size_t x=min_x; x<=max_x; x+=cell_size)
				{

				}
			}
		}
		else
		{
			// only create cells where the cell center is accessible
			for(size_t y=min_y; y<=max_y; y+=cell_size)
				for(size_t x=min_x; x<=max_x; x+=cell_size)
					if(room_map.at<unsigned char>(y,x)==255)
						cell_centers.push_back(cv::Point(x,y));
		}
	}
};

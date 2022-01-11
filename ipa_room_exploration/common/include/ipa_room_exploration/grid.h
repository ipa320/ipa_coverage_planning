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
#include <opencv2/opencv.hpp>

class BoustrophedonLine
{
public:
	std::vector<cv::Point> upper_line;		// points of the upper part of the line (always filled first)
	std::vector<cv::Point> lower_line;		// points of the potentially existing lower part of the line
	bool has_two_valid_lines;		// true if both lines, upper and lower, concurrently provide valid alternative points at some locations (i.e. only if this is true, there are two individual lines of places to visit)

	BoustrophedonLine()
	: has_two_valid_lines(false)
	{
	}
};

class BoustrophedonGrid : public std::vector<BoustrophedonLine>
{
};

class GridGenerator
{
public:
	GridGenerator()
	{
	}

	// generates a standard grid with fixed spacing starting at the upper left accessible cell (if the map is provided
	// with walls grown by the size of half robot radius, i.e. the map is eroded by the inaccessible areas) or starting at half grid distance
	// from the walls (if the original map is provided). If the map is provided in its normal appearance, then the start_grid_at_first_free_pixel
	// parameter should be set to false and the first grid center will be placed with appropriate distance to the walls.
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
	static bool completeCellTest(const cv::Mat& room_map, cv::Point& cell_center, const int cell_size)
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
#if CV_MAJOR_VERSION<=3
				cv::distanceTransform(cell_pixels, distances, CV_DIST_L2, 5);
#else
				cv::distanceTransform(cell_pixels, distances, cv::DIST_L2, 5);
#endif
				double max_distance = 0.;
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

				// display new center point
//				cv::Mat disp = distances.clone();
//				cv::Mat disp2;
//				cv::normalize(disp, disp, 0, 1., cv::NORM_MINMAX);
//				cv::resize(disp, disp2, cv::Size(), 10., 10., cv::INTER_AREA);
//				cv::circle(disp2, 10*cv::Point(cell_center.x-x+half_cell_size, cell_center.y-y+half_cell_size), 2, cv::Scalar(0), CV_FILLED);
//				cv::imshow("distance transform", disp2);
//				cv::waitKey();

				return true;
			}
		}
		return false;
	}


	// Generates a grid with fixed vertical spacing and variable distance between the points in a line (parameter), s.t. the algorithm may create
	// ordinary grid points or horizontal lines with defined vertical spacing. The generator considers obstacle safety distances and only places
	// grid cells with sufficient distance to obstacles (parameter). If a grid point cannot be placed because it would be located in an obstacle
	// or in the inaccessible area around an obstacle, then the algorithm tries to shift the point vertically up and down up to grid_spacing
	// (parameter) units. As soon as it finds free space, the grid point is set there. It can happen that such a free point exists above and below
	// the original placement of the grid point. Then the two options are store in an upper and lower trajectory and the user must later chose how
	// to deal with these option.
	// room_map = original map of the room as a CV_8UC1 map with 0=obstacles, 255=free space, in [pixels]
	// inflated_room_map = map of the room with inflated obstacles (can be provided, if cv::Mat() is provided, it is computed here with map_inflation_radius)
	// map_inflation_radius = the number of pixels obstacles shall be inflated if no precomputed inflated_room_map is provided (map_inflation_radius can be -1 otherwise), in [pixels]
	// grid_points = a vector of BoustrophedonLine objects, each of them containing line information in upper_line and optionally another line in lower_line if two_valid_lines is true, in [pixels]
	// min_max_map_coordinates = optionally precomputed min/max coordinates (min_x, max_x, min_y, max_y) of the free space in inflated_room_map, if cv::Vec4i(-1,-1,-1,-1) is provided, min/max map coordinates are computed by this function, in [pixels]
	// grid_spacing = the basic distance between two grid cell centers, is used for vertical grid spacing, in [pixels]
	// half_grid_spacing = the rounded half distance between two grid cell centers (the user shall define how it is rounded), in [pixels]
	// grid_spacing_horizontal = this value allows to specify the horizontal basic distance between two grid cell centers, it can be set to grid_spacing if the basic horizontal spacing shall be identical to the vertical spacing, in [pixels]
	// max_deviation_from_track = maximal allowed shift off the track to both sides for avoiding obstacles on track, setting max_deviation_from_track=grid_spacing is usually a good choice, for negative values max_deviation_from_track is set to grid_spacing, in [pixels]
	static void generateBoustrophedonGrid(const cv::Mat& room_map, cv::Mat& inflated_room_map, const int map_inflation_radius,
			BoustrophedonGrid& grid_points, const cv::Vec4i& min_max_map_coordinates, const int grid_spacing, const int half_grid_spacing,
			const int grid_spacing_horizontal, int max_deviation_from_track = -1)
	{
		if (max_deviation_from_track < 0)
			max_deviation_from_track = grid_spacing;

		// compute inflated_room_map if not provided
		if (inflated_room_map.rows!=room_map.rows || inflated_room_map.cols!=room_map.cols)
			cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), map_inflation_radius);

		// compute min/max map coordinates if necessary
		int min_x=inflated_room_map.cols, max_x=-1, min_y=inflated_room_map.rows, max_y=-1;
		if (min_max_map_coordinates[0]==-1 && min_max_map_coordinates[1]==-1 && min_max_map_coordinates[2]==-1 && min_max_map_coordinates[3]==-1)
		{
			for (int v=0; v<inflated_room_map.rows; ++v)
			{
				for (int u=0; u<inflated_room_map.cols; ++u)
				{
					if (inflated_room_map.at<uchar>(v,u) == 255)
					{
						if (min_x > u)
							min_x = u;
						if (max_x < u)
							max_x = u;
						if (min_y > v)
							min_y = v;
						if (max_y < v)
							max_y = v;
					}
				}
			}
		}
		else
		{
			min_x = min_max_map_coordinates[0];
			max_x = min_max_map_coordinates[1];
			min_y = min_max_map_coordinates[2];
			max_y = min_max_map_coordinates[3];
		}
		// if the room has no accessible cells, hence no min/max coordinates, return
		if ((min_x==inflated_room_map.cols) || (max_x==-1) || (min_y==inflated_room_map.rows) || (max_y==-1))
			return;

		// create grid
		const int squared_grid_spacing_horizontal = grid_spacing_horizontal*grid_spacing_horizontal;
		//std::cout << "((max_y - min_y) <= grid_spacing): min_y=" << min_y << "   max_y=" << max_y << "   grid_spacing=" << grid_spacing << std::endl;
		int y=min_y;
		// loop through the vertical grid lines with regular grid spacing
		for (; y<=max_y+half_grid_spacing; y += grid_spacing)		// we use max_y+half_grid_spacing as upper bound to cover the bottom-most line as well
		{
			if (y > max_y)	// this should happen at most once for the bottom line
				y = max_y;

			BoustrophedonLine line;
			const cv::Point invalid_point(-1,-1);
			cv::Point last_added_grid_point_above(-10000,-10000), last_added_grid_point_below(-10000,-10000);	// for keeping the horizontal grid distance
			cv::Point last_valid_grid_point_above(-1,-1), last_valid_grid_point_below(-1,-1);	// for adding the rightmost possible point
			// loop through the horizontal grid points with horizontal grid spacing length
			for (int x=min_x; x<=max_x; x+=1)
			{
				// points are added to the grid line as follows:
				//   1. if the original point is accessible --> point is added to upper_line, invalid point (-1,-1) is added to lower_line
				//   2. if the original point is not accessible:
				//      a) and no other point in the y-vicinity --> upper_line and lower_line are not updated
				//      b) but some point above and none below --> valid point is added to upper_line, invalid point (-1,-1) is added to lower_line
				//      c) but some point below and none above --> valid point is added to lower_line, invalid point (-1,-1) is added to upper_line
				//      d) but some point below and above are --> valid points are added to upper_line and lower_line, respectively

				// 1. check accessibility on regular location
				if (inflated_room_map.at<uchar>(y,x)==255)
				{
					if (squaredPointDistance(last_added_grid_point_above,cv::Point(x,y))>=squared_grid_spacing_horizontal)
					{
						line.upper_line.push_back(cv::Point(x,y));
						line.lower_line.push_back(invalid_point);
						last_added_grid_point_above = cv::Point(x,y);
					}
					else
						last_valid_grid_point_above = cv::Point(x,y);	// store this point and add it to the upper line if it was the rightmost point
				}
				// todo: add parameter to switch else branch off
				else // 2. check accessibility above or below the targeted point
				{
					// check accessibility above the target location
					bool found_above = false;
					int dy = -1;
					for (; dy>-max_deviation_from_track; --dy)
					{
						if (y+dy>=0 && inflated_room_map.at<uchar>(y+dy,x)==255)
						{
							found_above = true;
							break;
						}
					}
					if (found_above == true)
					{
						if (squaredPointDistance(last_added_grid_point_above,cv::Point(x,y+dy))>=squared_grid_spacing_horizontal)
						{
							line.upper_line.push_back(cv::Point(x,y+dy));
							line.lower_line.push_back(invalid_point);		// can be overwritten below if this point also exists
							last_added_grid_point_above = cv::Point(x,y+dy);
						}
						else
							last_valid_grid_point_above = cv::Point(x,y+dy);	// store this point and add it to the upper line if it was the rightmost point
					}

					// check accessibility below the target location
					bool found_below = false;
					dy = 1;
					for (; dy<max_deviation_from_track; ++dy)
					{
						if (y+dy<inflated_room_map.rows && inflated_room_map.at<uchar>(y+dy,x)==255)
						{
							found_below = true;
							break;
						}
					}
					if (found_below == true)
					{
						if (squaredPointDistance(last_added_grid_point_below,cv::Point(x,y+dy))>=squared_grid_spacing_horizontal)
						{
							if (found_above == true)	// update the existing entry
							{
								line.has_two_valid_lines = true;
								line.lower_line.back().x = x;
								line.lower_line.back().y = y+dy;
							}
							else	// create a new entry
							{
								line.upper_line.push_back(invalid_point);
								line.lower_line.push_back(cv::Point(x,y+dy));
							}
							last_added_grid_point_below = cv::Point(x,y+dy);
						}
						else
							last_valid_grid_point_below = cv::Point(x,y+dy);	// store this point and add it to the lower line if it was the rightmost point
					}
				}
			}
			// add the rightmost points if available
			if (last_valid_grid_point_above.x > -1 && last_valid_grid_point_above.x > last_added_grid_point_above.x)
			{
				// upper point is valid
				line.upper_line.push_back(last_valid_grid_point_above);
				if (last_valid_grid_point_below.x > -1 && last_valid_grid_point_below.x > last_added_grid_point_below.x)
					line.lower_line.push_back(last_valid_grid_point_below);
				else
					line.lower_line.push_back(invalid_point);
			}
			else
			{
				// upper point is invalid
				if (last_valid_grid_point_below.x > -1 && last_valid_grid_point_below.x > last_added_grid_point_below.x)
				{
					// lower point is valid
					line.upper_line.push_back(invalid_point);
					line.lower_line.push_back(last_valid_grid_point_below);
				}
			}

			// clean the grid line data
			// 1. if there are no valid points --> do not add the line
			// 2. if two_valid_lines is true, there are two individual lines available with places to visit
			// 3. else there is just one valid line with data potentially distributed over upper_line and lower_line
			BoustrophedonLine cleaned_line;
			if (line.upper_line.size()>0 && line.lower_line.size()>0)	// 1. check that there is valid data in the lines
			{
				// 2. if two_valid_lines is true, create two individual lines with places to visit
				if (line.has_two_valid_lines == true)
				{
					cleaned_line.has_two_valid_lines = true;
					for (size_t i=0; i<line.upper_line.size(); ++i)
					{
						if (line.upper_line[i]!=invalid_point)
							cleaned_line.upper_line.push_back(line.upper_line[i]);
						if (line.lower_line[i]!=invalid_point)
							cleaned_line.lower_line.push_back(line.lower_line[i]);
					}
				}
				else	// 3. there is just one valid line that needs to be merged from lower_line and upper_line
				{
					for (size_t i=0; i<line.upper_line.size(); ++i)
					{
						if (line.upper_line[i]!=invalid_point)
							cleaned_line.upper_line.push_back(line.upper_line[i]);		// keep the upper_line as is
						else	// the upper_line does not contain a valid point
							if (line.lower_line[i]!=invalid_point)		// move the valid point from lower to upper line
								cleaned_line.upper_line.push_back(line.lower_line[i]);
					}
				}

				// add cleaned line to the grid
				grid_points.push_back(cleaned_line);
			}
		}
	}

	static double squaredPointDistance(const cv::Point& p1, const cv::Point& p2)
	{
		return (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);
	}
};

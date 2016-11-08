#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include <ipa_room_exploration/concorde_TSP.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

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
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 11.2016
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

// Class that is used to store cells and obstacles in a certain manner. For this the vertexes are stored as points and
// the edges are stored as vectors in a counter-clockwise manner. The constructor becomes a set of respectively sorted
// points and computes the vectors out of them. Additionally the weighted center of all points gets computed, to
// simplify the visiting order later.
class generalizedPolygon
{
protected:
	// vertexes
	std::vector<geometry_msgs::Point32> vertexes_;

	// edges
	std::vector<geometry_msgs::Point32> edges_;

	// center
	geometry_msgs::Point32 center_;
public:
	// constructor
	generalizedPolygon(std::vector<geometry_msgs::Point32> vertexes)
	{
		// save given vertexes
		vertexes_ = vertexes;

		// compute vector to represent edges and compute sum of coordinates
		float sum_x = 0, sum_y = 0;
		for(size_t point = 0; point < vertexes.size()-1; ++point)
		{
			geometry_msgs::Point32 current_vector;
			current_vector.x = vertexes[point+1].x - vertexes[point].x;
			current_vector.y= vertexes[point+1].y - vertexes[point].y;
			sum_x += vertexes[point].x;
			sum_y += vertexes[point].y;
			edges_.push_back(current_vector);
		}
		geometry_msgs::Point32 last_vector;
		last_vector.x = vertexes[0].x - vertexes.back().x;
		last_vector.y = vertexes[0].y - vertexes.back().y;
		sum_x += vertexes.back().x;
		sum_y += vertexes.back().y;
		edges_.push_back(last_vector);

		// compute weighted center
		center_.x = sum_x/vertexes.size();
		center_.y = sum_y/vertexes.size();
	}
};

// Class that generates a room exploration path by using the morse cellular decomposition method, proposed by
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method decomposes the given environment into several cells which don't have any obstacle in it. For each of this
// cell the boustrophedon path is generated, which goes up and down in each cell, see the upper paper for reference.
// This class only produces a static path, regarding the given map in form of a point series. To react on dynamic
// obstacles, one has to do this in upper algorithms.
//
class boustrophedonExplorer
{
protected:

public:
	// constructor
	boustrophedonExplorer();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
				const float map_resolution, const geometry_msgs::Pose2D starting_position,
				const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin,
				const float fow_fitting_circle_radius, const float min_resample_dist);
};

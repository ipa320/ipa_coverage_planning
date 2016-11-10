#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>

#include <ipa_room_exploration/concorde_TSP.h>
#include <ipa_room_exploration/meanshift2d.h>

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
// points and computes the vectors out of them. Additionally the visible center of the polygon gets computed, to
// simplify the visiting order later, by using a meanshift algorithm.
class generalizedPolygon
{
protected:
	// vertexes
	std::vector<cv::Point> vertexes_;

	// edges
	std::vector<cv::Point> edges_;

	// center
	cv::Point center_;
public:
	// constructor
	generalizedPolygon(std::vector<cv::Point> vertexes)
	{
		// save given vertexes
		vertexes_ = vertexes;

		// compute vector to represent edges and compute sum of coordinates
		for(size_t point = 0; point < vertexes.size()-1; ++point)
		{
			cv::Point current_vector;
			current_vector.x = vertexes[point+1].x - vertexes[point].x;
			current_vector.y= vertexes[point+1].y - vertexes[point].y;
			edges_.push_back(current_vector);
		}
		cv::Point last_vector;
		last_vector.x = vertexes[0].x - vertexes.back().x;
		last_vector.y = vertexes[0].y - vertexes.back().y;
		edges_.push_back(last_vector);

		// get max x/y coordinates
		int max_x = 0, max_y = 0;
		for(size_t i=0; i<vertexes.size(); ++i)
		{
			if(vertexes[i].x > max_x)
				max_x = vertexes[i].x;
			if(vertexes[i].y > max_y)
				max_y = vertexes[i].y;
		}

		// compute visible center
		MeanShift2D ms;
		cv::Mat room = cv::Mat::zeros(max_y+10, max_x+10, CV_8UC1);
		cv::drawContours(room, std::vector<std::vector<cv::Point> >(1,vertexes), -1, cv::Scalar(255), CV_FILLED);
		cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
		// find point set with largest distance to obstacles
		double min_val = 0., max_val = 0.;
		cv::minMaxLoc(distance_map, &min_val, &max_val);
		std::vector<cv::Vec2d> room_cells;
		for (int v = 0; v < distance_map.rows; ++v)
			for (int u = 0; u < distance_map.cols; ++u)
				if (distance_map.at<float>(v, u) > max_val * 0.95f)
					room_cells.push_back(cv::Vec2d(u, v));
		// use meanshift to find the modes in that set
		cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, 0.05);
		// save found center
		center_.x = room_center[0];
		center_.y = room_center[1];
	}

	cv::Point getCenter()
	{
		return center_;
	}

	void drawPolygon(cv::Mat& image, const cv::Scalar& color)
	{
		// get needed size of the image
		int max_x=0, max_y=0;
		for(size_t point=0; point<vertexes_.size(); ++point)
		{
			if(vertexes_[point].x > max_x)
				max_x = vertexes_[point].x;
			if(vertexes_[point].y > max_y)
				max_y = vertexes_[point].y;
		}

		// draw polygon in an black image with necessary size
		cv::Mat black_image = cv::Mat(max_y+10, max_x+10, CV_8UC1, cv::Scalar(0));
		cv::drawContours(black_image, std::vector<std::vector<cv::Point> >(1,vertexes_), -1, color, CV_FILLED);

		// assign drawn map
		image = black_image.clone();
	}

	void getMinMaxCoordinates(int& min_x, int& max_x, int& min_y, int& max_y)
	{
		max_x=0, max_y=0, min_x=1e3, min_y=1e3;
		for(size_t point=0; point<vertexes_.size(); ++point)
		{
			if(vertexes_[point].x > max_x)
				max_x = vertexes_[point].x;
			if(vertexes_[point].y > max_y)
				max_y = vertexes_[point].y;
			if(vertexes_[point].x < min_x)
				min_x = vertexes_[point].x;
			if(vertexes_[point].y < min_y)
				min_y = vertexes_[point].y;

		}
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
				const float fow_fitting_circle_radius, const int path_eps);
};

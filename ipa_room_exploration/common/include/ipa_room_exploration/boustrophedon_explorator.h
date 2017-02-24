#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <ipa_room_exploration/concorde_TSP.h>
#include <ipa_room_exploration/meanshift2d.h>
#include <ipa_room_exploration/fow_to_robot_mapper.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265359

/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
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

#pragma once

// Class that is used to store cells and obstacles in a certain manner. For this the vertexes are stored as points and
// the edges are stored as vectors in a counter-clockwise manner. The constructor becomes a set of respectively sorted
// points and computes the vectors out of them. Additionally the visible center of the polygon gets computed, to
// simplify the visiting order later, by using a meanshift algorithm.
class GeneralizedPolygon
{
protected:
	// vertexes
	std::vector<cv::Point> vertices_;

	// edges
	std::vector<cv::Point2d> edges_;

	// longest edge of the current cell
	cv::Point2d longest_edge_;
	double longest_edge_angle_;

	// center
	cv::Point center_;

	// min/max coordinates
	int max_x_, min_x_, max_y_, min_y_;

public:
	// constructor
	GeneralizedPolygon(const std::vector<cv::Point>& vertices)
	{
		// save given vertexes
		vertices_ = vertices;

		// get max/min x/y coordinates
		max_x_ = 0;
		min_x_ = 100000;
		max_y_ = 0;
		min_y_ = 100000;
		for(size_t point=0; point<vertices_.size(); ++point)
		{
			if(vertices_[point].x > max_x_)
				max_x_ = vertices_[point].x;
			if(vertices_[point].y > max_y_)
				max_y_ = vertices_[point].y;
			if(vertices_[point].x < min_x_)
				min_x_ = vertices_[point].x;
			if(vertices_[point].y < min_y_)
				min_y_ = vertices_[point].y;
		}

		// compute vector to represent edges and find longest edge
		double longest_squared_edge_length = 0.0;
		for(size_t point = 0; point < vertices.size(); ++point)
		{
			// construct edge
			cv::Point2d current_vector;
			current_vector.x = vertices[(point+1)%vertices.size()].x - vertices[point].x;
			current_vector.y= vertices[(point+1)%vertices.size()].y - vertices[point].y;
			edges_.push_back(current_vector);

			// check if current edge is longer than the previous edges
			double current_length = current_vector.x*current_vector.x+current_vector.y*current_vector.y;
			if(current_length > longest_squared_edge_length)
			{
				longest_edge_ = current_vector;
				longest_squared_edge_length = current_length;
			}
		}

		// generate matrices for gradient in x/y direction
		cv::Mat gradient_x, gradient_y, room_map;
		drawPolygon(room_map, cv::Scalar(255));
		// compute gradient in x direction
		cv::Sobel(room_map, gradient_x, CV_64F, 1, 0, 5, 1.0, 0.0, cv::BORDER_DEFAULT);
		// compute gradient in y direction
		cv::Sobel(room_map, gradient_y, CV_64F, 0, 1, 5, 1.0, 0.0, cv::BORDER_DEFAULT);
		// compute the direction of the gradient for each pixel and save the occurring gradients
		std::vector<double> gradient_directions;
		for(size_t y=0; y<room_map.rows; ++y)
		{
			for(size_t x=0; x<room_map.cols; ++x)
			{
				// check if the gradient has a value larger than zero, to only take the edge-gradients into account
				int dx = gradient_x.at<double>(y,x);
				int dy = gradient_y.at<double>(y,x);
				if(dy*dy+dx*dx > 0.0)
				{
					double current_gradient = std::atan2(dy, dx);
					gradient_directions.push_back(0.1*(double)((int)((current_gradient*10)+0.5)));	// round to one digit
				}
			}
		}
		// find the gradient that occurs most often, this direction is used to rotate the map
		int max_number = 0;
		longest_edge_angle_ = 0.0;
		std::set<double> done_gradients;
		for(std::vector<double>::iterator grad=gradient_directions.begin(); grad!=gradient_directions.end(); ++grad)
		{
			// if gradient has been done, don't check it again
			if(done_gradients.find(*grad)==done_gradients.end())
			{
				int current_count = std::count(gradient_directions.begin(), gradient_directions.end(), *grad);
	//			std::cout << "current gradient: " << *grad << ", occurs " << current_count << " times." << std::endl;
				if(current_count > max_number)
				{
					max_number = current_count;
					longest_edge_angle_ = *grad;
				}
				done_gradients.insert(*grad);
			}
		}
		longest_edge_angle_ -= PI/2;

		// compute visible center
		MeanShift2D ms;
		cv::Mat room = cv::Mat::zeros(max_y_+10, max_x_+10, CV_8UC1);
		cv::drawContours(room, std::vector<std::vector<cv::Point> >(1,vertices), -1, cv::Scalar(255), CV_FILLED);
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

	std::vector<cv::Point> getVertexes()
	{
		return vertices_;
	}

	std::vector<cv::Point2d> getEdges()
	{
		return edges_;
	}

	cv::Point2d getLongestEdge()
	{
		return longest_edge_;
	}

	double getLongestEdgeAngle()
	{
		return longest_edge_angle_;
	}

	void drawPolygon(cv::Mat& image, const cv::Scalar& color)
	{
		// draw polygon in an black image with necessary size
		cv::Mat black_image = cv::Mat(max_y_+10, max_x_+10, CV_8UC1, cv::Scalar(0));
		cv::drawContours(black_image, std::vector<std::vector<cv::Point> >(1,vertices_), -1, color, CV_FILLED);

		// assign drawn map
		image = black_image.clone();
	}

	void getMinMaxCoordinates(int& min_x, int& max_x, int& min_y, int& max_y)
	{
		min_x = min_x_;
		max_x = max_x_;
		min_y = min_y_;
		max_y = max_y_;
	}
};

// Structure to save edges of a path on one row, that allows to easily get the order of the edges when planning the
// boustrophedon path.
struct BoustrophedonHorizontalLine
{
	cv::Point left_edge_, right_edge_;
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
	// pathplanner to check for the next nearest loocations
	AStarPlanner path_planner_;
public:
	// constructor
	boustrophedonExplorer();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
				const cv::Point starting_position, const cv::Point2d map_origin,
				const float coverage_radius, const double path_eps, const bool plan_for_footprint,
				const Eigen::Matrix<float, 2, 1> robot_to_fow_vector, const double min_cell_area);
};

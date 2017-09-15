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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <ipa_building_navigation/concorde_TSP.h>
#include <ipa_building_navigation/genetic_TSP.h>
#include <ipa_room_exploration/meanshift2d.h>
#include <ipa_room_exploration/fov_to_robot_mapper.h>
#include <ipa_room_exploration/room_rotator.h>
#include <ipa_room_exploration/grid.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#define PI 3.14159265359



// Class that is used to store cells and obstacles in a certain manner. For this the vertexes are stored as points and
// the edges are stored as vectors in a counter-clockwise manner. The constructor becomes a set of respectively sorted
// points and computes the vectors out of them. Additionally the visible center of the polygon gets computed, to
// simplify the visiting order later, by using a meanshift algorithm.
class GeneralizedPolygon
{
protected:
	// vertexes
	std::vector<cv::Point> vertices_;

	// center
	cv::Point center_;

	// min/max coordinates
	int max_x_, min_x_, max_y_, min_y_;

public:
	// constructor
	GeneralizedPolygon(const std::vector<cv::Point>& vertices, const double map_resolution)
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
		cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, map_resolution);
		// save found center
		center_.x = room_center[0];
		center_.y = room_center[1];
	}

	cv::Point getCenter() const
	{
		return center_;
	}

	std::vector<cv::Point> getVertices() const
	{
		return vertices_;
	}

	void drawPolygon(cv::Mat& image, const cv::Scalar& color) const
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
	cv::Point left_corner_, right_corner_;
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
class BoustrophedonExplorer
{
protected:
	// pathplanner to check for the next nearest locations
	AStarPlanner path_planner_;

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// the functions tries two axis alignments with 90deg rotation difference and chooses the one with the lower number of cells
	void findBestCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// rotates the original map for a good axis alignment and divides it into Morse cells
	// @param rotation_offset can be used to put an offset to the computed rotation for good axis alignment, in [rad]
	void computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// divides the provided map into Morse cells
	void computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
			std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers);

	// this function corrects obstacles that are one pixel width at 45deg angle, i.e. a 2x2 pixel neighborhood with [0, 255, 255, 0] or [255, 0, 0, 255]
	void correctThinWalls(cv::Mat& room_map);

	// computes the Boustrophedon path pattern for a single cell
	void computeBoustrophedonPath(const cv::Mat& room_map, const float map_resolution, const GeneralizedPolygon& cell,
			std::vector<cv::Point>& fov_middlepoint_path, cv::Point& robot_pos,
			const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps);

	// downsamples a given path original_path to waypoint distances of path_eps and appends the resulting path to downsampled_path
	void downsamplePath(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
			cv::Point& cell_robot_pos, const double path_eps);

	// downsamples a given path original_path to waypoint distances of path_eps in reverse order as given in original_path
	// and appends the resulting path to downsampled_path
	void downsamplePathReverse(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
			cv::Point& robot_pos, const double path_eps);

public:
	// constructor
	BoustrophedonExplorer();

	// Function that creates an exploration path for a given room. The room has to be drawn in a cv::Mat (filled with Bit-uchar),
	// with free space drawn white (255) and obstacles as black (0). It returns a series of 2D poses that show to which positions
	// the robot should drive at.
	void getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
				const cv::Point starting_position, const cv::Point2d map_origin,
				const double grid_spacing_in_pixel, const double path_eps, const bool plan_for_footprint,
				const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, const double min_cell_area);
};

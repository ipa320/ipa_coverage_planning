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
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 10.2015
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

#include <ipa_room_segmentation/voronoi_random_field_segmentation.h> // header

#include <ipa_room_segmentation/wavefront_region_growing.h> // some useful functions defined for all segmentations
#include <ipa_room_segmentation/contains.h>

class testfunc // this class overloads the () operator, so that it can be passed to the dlib library, finding the min of the implemented f(x)
{
public:
	testfunc()
	{

	}

    double operator()(const column_vector& arg) const
    {
    	const double x = arg(0);
    	const double y = arg(1);

    	return 100.0*std::pow(y - x*x, 2.0) + std::pow(1 - x, 2.0);
    }
};

VoronoiRandomFieldSegmentation::VoronoiRandomFieldSegmentation()
{

}

double rosen(const column_vector& m)
{
	const double x = m(0);
	const double y = m(1);

	return 100.0*std::pow(y - x*x, 2.0) + std::pow(1 - x, 2.0);
}

column_vector VoronoiRandomFieldSegmentation::find_min_value()
{
	column_vector starting_point(2);

	starting_point = 4,8;

	dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), testfunc(), starting_point, -1);

	return starting_point;
}




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
 * Author: Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 02.2017
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

#include <stddef.h>
#include <vector>

template <typename T>
class Histogram
{
public:
	Histogram(const T lower_bound, const T upper_bound, const size_t histogram_bins)
	{
		lower_bound_ = lower_bound;
		upper_bound_ = upper_bound;
		range_inverse_ = 1.0/(upper_bound_-lower_bound_);
		histogram_bins_ = histogram_bins;
		data_.resize(histogram_bins);
		for (size_t i=0; i<data_.size(); ++i)
			data_[i] = 0.;
		raw_data_.resize(histogram_bins);
	}

	void addData(const T val, const double weight=1.0)
	{
		const size_t bin = std::max((size_t)0, std::min(histogram_bins_-1, (size_t)((val - lower_bound_) * range_inverse_ * (T)histogram_bins_)));
		data_[bin] += weight;
		raw_data_[bin].push_back(std::pair<T, double>(val, weight));
	}

	size_t getMaxBin()
	{
		double max_val = 0.;
		size_t max_bin = 0;
		for (size_t i=0; i<data_.size(); ++i)
		{
			if (max_val < data_[i])
			{
				max_val = data_[i];
				max_bin = i;
			}
		}

		return max_bin;
	}

	T getMaxBinPreciseVal()
	{
		if (raw_data_.size() == 0 || raw_data_.size() != data_.size())
			return 0.;

		const size_t max_bin = getMaxBin();
		T sum = 0.;
		T weight_sum = 0.;
		RawData& data = raw_data_[max_bin];
		for (size_t i=0; i<data.size(); ++i)
		{
			sum += data[i].first*data[i].second;
			weight_sum += data[i].second;
		}
		if (weight_sum==0)
			weight_sum = 1;
		return sum/weight_sum;
	}

protected:
	typedef std::vector< std::pair< T, double> > RawData;

	std::vector<double> data_;	// stores the histogram
	std::vector<RawData> raw_data_;	// stores all entered data pairs (data, weight) for each histogram bin
	T lower_bound_;		// lowest possible value
	T upper_bound_;		// highest possible value
	T range_inverse_;	// = 1.0/(upper_bound_-lower_bound_)
	size_t histogram_bins_;		// number of histogram bins
};

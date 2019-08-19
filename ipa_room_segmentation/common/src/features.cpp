#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>

#include <ipa_room_segmentation/features.h>

#define PI 3.14159265

//get the number of implemented features. Needs to be changed to the new value if you change it
int LaserScannerFeatures::get_feature_count()
{
	return 23;
}

void LaserScannerFeatures::resetCachedData()
{
	features_.clear();
	features_.resize(get_feature_count(), 0.);
	features_computed_.clear();
	features_computed_.resize(get_feature_count(), false);

	polygon_.clear();
	polygon_computed_ = false;

	centroid_computed_ = false;
}

//**********************see features.h for a better overview of what is calculated and needed*************************
//Method for calculating the feature for the classifier
double LaserScannerFeatures::get_feature(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point point, const int feature)
{
	switch (feature)
	{
	case 1:
		return calc_feature1(beams);
	case 2:
		return calc_feature2(beams);
	case 3:
		return calc_feature3(beams, 10);
	case 4:
		return calc_feature4(beams, 10);
	case 5:
		return calc_feature5(beams);
	case 6:
		return calc_feature6(beams);
	case 7:
		return calc_feature7(beams);
	case 8:
		return calc_feature8(beams, angles);
	case 9:
		return calc_feature9(beams, angles);
	case 10:
		return calc_feature10(beams);
	case 11:
		return calc_feature11(beams);
	case 12:
		return calc_feature12(beams);
	case 13:
		return calc_feature13(beams);
	case 14:
		return calc_feature14(beams, angles, point);
	case 15:
		return calc_feature15(beams, angles, point);
	case 16:
		return calc_feature16(beams, angles, point);
	case 17:
		return calc_feature17(beams, angles, point);
	case 18:
		return calc_feature18(beams, angles, point);
	case 19:
		return calc_feature19(beams, angles, point);
	case 20:
		return calc_feature20(beams, angles, point);
	case 21:
		return calc_feature21(beams, angles, point);
	case 22:
		return calc_feature22(beams);
	case 23:
		return calc_feature23(beams);
	}
}

void LaserScannerFeatures::get_features(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point point, cv::Mat& features)
{
	// reset internal data storage
	resetCachedData();

	// compute features
	calc_feature1(beams);
	calc_feature2(beams);
	calc_feature3(beams, 10);
	calc_feature4(beams, 10);
	calc_feature5(beams);
	calc_feature6(beams);
	calc_feature7(beams);
	calc_feature8(beams, angles);
	calc_feature9(beams, angles);
	calc_feature10(beams);
	calc_feature11(beams);
	calc_feature12(beams);
	calc_feature13(beams);
	calc_feature14(beams, angles, point);
	calc_feature15(beams, angles, point);
	calc_feature16(beams, angles, point);
	calc_feature17(beams, angles, point);
	calc_feature18(beams, angles, point);
	calc_feature19(beams, angles, point);
	calc_feature20(beams, angles, point);
	calc_feature21(beams, angles, point);
	calc_feature22(beams);
	calc_feature23(beams);

	// write features
	features.create(1, get_feature_count(), CV_32FC1);
	for (int i=0; i<features.cols; ++i)
		features.at<float>(0,i) = features_[i];
}

//Calculation of Feature 1: average difference of the beams
double LaserScannerFeatures::calc_feature1(const std::vector<double>& beams)
{
	if (features_computed_[0])
		return features_[0];

	double differences_sum;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		differences_sum += abs(beams[b] - beams[b + 1]);
	}
	//get the difference betweeen the last and the first beam
	differences_sum += abs(beams[beams.size() - 1] - beams[0]);
	//calculate the average difference and return it
	features_computed_[0] = true;
	features_[0] = (differences_sum / (double)beams.size());

	if (features_[0]!=features_[0])
		std::cout << "   features_[0]="<<features_[0]<<std::endl;

	return features_[0];
}

//Calculation of Feature 2: standard deviation of the difference of the beams
double LaserScannerFeatures::calc_feature2(const std::vector<double>& beams)
{
	if (features_computed_[1])
		return features_[1];

	double mean; //mean-value of the difference, calculated with calc_feature1
	double sum; //helping variable
	//initialise
	mean = calc_feature1(beams);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += (beams[b] - mean)*(beams[b] - mean);//std::pow((beams[b] - mean), 2.0);
	}
	sum = sum / (double)(beams.size() - 1);
	features_computed_[1] = true;
	features_[1] = std::sqrt(sum);

	if (features_[1]!=features_[1])
		std::cout << "   features_[1]="<<features_[1]<<std::endl;

	return features_[1];
}

//Calculation of Feature 3: average difference of the to a max_value limited beams
double LaserScannerFeatures::calc_feature3(const std::vector<double>& beams, double maxval)
{
	if (features_computed_[2])
		return features_[2];

	double differences_sum;
	double val1, val2;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		//reset saved beamlenghts
		val1 = maxval;
		val2 = maxval;
		if (beams[b] < maxval)
		{
			val1 = beams[b];
		}
		if (beams[b + 1] < maxval)
		{
			val2 = beams[b + 1];
		}
		differences_sum += abs(val1 - val2);
	}
	//get the difference betweeen the last and the first beam
	val1 = maxval;
	val2 = maxval;
	if (beams[beams.size()-1] < maxval)
	{
		val1 = beams[beams.size()-1];
	}
	if (beams[0] < maxval)
	{
		val2 = beams[0];
	}
	differences_sum += abs(val1 - val2);
	//calculate the average difference and return it
	features_computed_[2] = true;
	features_[2] = (differences_sum / (double)beams.size());

	if (features_[2]!=features_[2])
		std::cout << "   features_[2]="<<features_[2]<<std::endl;

	return features_[2];
}

//Calculation of Feature 4: The Standard Deviation of the difference of limited beams
double LaserScannerFeatures::calc_feature4(const std::vector<double>& beams, double maxval)
{
	if (features_computed_[3])
		return features_[3];

	double mean; //mean-value of the difference, calculated with calc_feature1
	double v, w, difference, sum; //helping variables
	//initialize
	mean = calc_feature3(beams, maxval);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size() - 1; b++)
	{
		//reset value of current beam
		v = maxval;
		w = maxval;
		if (beams[b] < maxval)
		{
			v = beams[b];
		}
		if (beams[b + 1] < maxval)
		{
			w = beams[b + 1];
		}
		difference = abs(v - w);
		sum += (difference - mean)*(difference - mean);	//std::pow((difference - mean), 2.0);
	}
	//add the difference from last to first point
	v = maxval;
	w = maxval;
	if (beams[beams.size()-1] < maxval)
	{
		v = beams[beams.size()-1];
	}
	if (beams[0] < maxval)
	{
		w = beams[0];
	}
	difference = abs(v - w);
	sum += (difference - mean)*(difference - mean);	//std::pow((difference - mean), 2.0);
	sum = sum / (beams.size() - 1);
	features_computed_[3] = true;
	features_[3] = std::sqrt(sum);

	if (features_[3]!=features_[3])
		std::cout << "   features_[3]="<<features_[3]<<std::endl;

	return features_[3];
}

//Calculation of Feature 5: The average beamlength
double LaserScannerFeatures::calc_feature5(const std::vector<double>& beams)
{
	if (features_computed_[4])
		return features_[4];

	double sum;
	//get the sum of the beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		sum += beams[b];
	}
	//divide by number of beams and return value
	features_computed_[4] = true;
	features_[4] = (sum / (double)beams.size());

	if (features_[4]!=features_[4])
		std::cout << "   features_[4]="<<features_[4]<<std::endl;

	return features_[4];
}

//Calculation of Feature 6: The standard deviation of the beamlenghts
double LaserScannerFeatures::calc_feature6(const std::vector<double>& beams)
{
	if (features_computed_[5])
		return features_[5];

	double mean; //mean-value of the beamlenghts, calculated with calc_feature5
	double sum; //helping variable
	//initialize
	mean = calc_feature5(beams);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += (beams[b] - mean)*(beams[b] - mean);	//std::pow((beams[b] - mean), 2);
	}
	sum = sum / (beams.size() - 1);
	features_computed_[5] = true;
	features_[5] = std::sqrt(sum);

	if (features_[5]!=features_[5])
		std::cout << "   features_[5]="<<features_[5]<<std::endl;

	return features_[5];
}

//Calculation of Feature 7: The number of gaps between the beams, a gap is when the difference of the lenghts is larger
//than a specified threshold
double LaserScannerFeatures::calc_feature7(const std::vector<double>& beams)
{
	if (features_computed_[6])
		return features_[6];

	double threshold = 0.5; //[m], see "Semantic labeling of places"
	double gaps = 0;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		if (abs(beams[b] - beams[b + 1]) > threshold)
		{
			gaps++;
		}
	}
	if (abs(beams[beams.size() - 1] - beams[0]) > threshold)
	{
		gaps++;
	}
	features_computed_[6] = true;
	features_[6] = gaps;
	return features_[6];
}

//Calculation of feature 8: The distance between two Endpoints of local minima of beamlenghts
double LaserScannerFeatures::calc_feature8(const std::vector<double>& beams, const std::vector<double>& angles)
{
	if (features_computed_[7])
		return features_[7];

	//Remark: angles are relatively to the robot
	double length_1 = 10000000;
	double length_2 = 10000000;
	double angle_1, angle_2;
	//get the two Points corresponding to minimal beamlength
	for (int b = 0; b < beams.size(); b++)
	{
		if (beams[b] < length_1 && beams[b] > length_2)
		{
			length_1 = beams[b];
			angle_1 = angles[b];
		}
		else if (beams[b] < length_2)
		{
			length_2 = beams[b];
			angle_2 = angles[b];
		}
	}
	//calculate the x/y-values of the Points
	double x1_x2 = std::cos(angle_1 * PI / 180) * length_1 - std::cos(angle_2 * PI / 180) * length_2;
	double y1_y2 = std::sin(angle_1 * PI / 180) * length_1 - std::sin(angle_2 * PI / 180) * length_2;
	//calculate and return the euclidean distance between the Points
	features_computed_[7] = true;
	features_[7] = std::sqrt(x1_x2*x1_x2 + y1_y2*y1_y2);

	if (features_[7]!=features_[7])
		std::cout << "   features_[7]="<<features_[7]<<std::endl;

	return features_[7];	//std::pow((x_1 - x_2), 2) + std::pow((y_1 - y_2), 2));
}

//Calculate Feature 9: The Angle between two Endpoints of local minima of beamlengths
double LaserScannerFeatures::calc_feature9(const std::vector<double>& beams, const std::vector<double>& angles)
{
	if (features_computed_[8])
		return features_[8];

	//Remark: angles are relative to the robot
	double length_1 = beams[0];
	double length_2 = beams[1];
	double angle_1 = angles[0];
	double angle_2 = angles[1];
	double x_1, y_1, x_2, y_2;
	//get the two Points corresponding to minimal beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		if (beams[b] < length_1 && beams[b] > length_2)
		{
			length_1 = beams[b];
			angle_1 = angles[b];
		}
		else if (beams[b] <= length_2)
		{
			length_2 = beams[b];
			angle_2 = angles[b];
		}
	}
	//calculate the x/y-values of the Points
	const double pi_to_degree = PI / 180;
	x_1 = std::cos(angle_1 * pi_to_degree) * length_1;
	y_1 = std::sin(angle_1 * pi_to_degree) * length_1;
	x_2 = std::cos(angle_2 * pi_to_degree) * length_2;
	y_2 = std::sin(angle_2 * pi_to_degree) * length_2;
	//calculate and return the angle between the Points
	const double coordvec = (x_1 * x_2) + (y_1 * y_2);
	const double absvec = (length_1 * length_2);
	const double quot = std::max(-1., std::min(1., coordvec / absvec));
	features_computed_[8] = true;
	features_[8] = std::acos(quot) * 180.0 / PI;
	return features_[8];
}

//Calculate Feature 10: The average of the relations (b_i/b_(i+1)) between two neighboring beams
double LaserScannerFeatures::calc_feature10(const std::vector<double>& beams)
{
	if (features_computed_[9])
		return features_[9];

	double length_1, length_2;
	double sum_relation = 0;
	//calculate the relations and add it to the sum
	for (int b = 0; b < beams.size() - 1; b++)
	{
		length_1 = beams[b];
		length_2 = beams[b + 1];
		if (length_1 < length_2)
		{
			sum_relation += (length_1 / length_2);
		}
		else
		{
			sum_relation += (length_2 / length_1);
		}
	}
	length_1 = beams[beams.size() - 1];
	length_2 = beams[0];
	if (length_1 < length_2)
	{
		sum_relation += (length_1 / length_2);
	}
	else
	{
		sum_relation += (length_2 / length_1);
	}
	//calculate and return the average of the relations
	features_computed_[9] = true;
	features_[9] = (sum_relation / beams.size());

	if (features_[9]!=features_[9])
		std::cout << "   features_[9]="<<features_[9]<<std::endl;

	return features_[9];
}

//Calculate Feature 11: The standard deviation of the relations (b_i/b_(i+1)) between two neighboring beams
double LaserScannerFeatures::calc_feature11(const std::vector<double>& beams)
{
	if (features_computed_[10])
		return features_[10];

	//calculate the mean of the relations by using Feature 10
	double mean = calc_feature10(beams);
	double sum = 0;
	//calculate the standard_deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += (beams[b] - mean);	//std::pow((beams[b] - mean), 2);
	}
	sum = sum / (beams.size() - 1);
	features_computed_[10] = true;
	features_[10] = std::sqrt(sum);

	if (features_[10]!=features_[10])
		std::cout << "   features_[10]="<<features_[10]<<std::endl;

	return features_[10];
}

//Calculate Feature 12: The number of relative gaps. A relative gap is when the relation (b_i/b_(i+1)) is smaller than a
//specified threshold
double LaserScannerFeatures::calc_feature12(const std::vector<double>& beams)
{
	if (features_computed_[11])
		return features_[11];

	double threshold = 0.5; //[m] see "Semantic labeling of places"
	double gaps, length_1, length_2;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		length_1 = beams[b];
		length_2 = beams[b + 1];
		if (length_1 < length_2)
		{
			if ((length_1 / length_2) < threshold)
			{
				gaps++;
			}
		}
		else
		{
			if ((length_2 / length_1) < threshold)
			{
				gaps++;
			}
		}
	}
	length_1 = beams[0];
	length_2 = beams[beams.size() - 1];
	if (length_1 < length_2)
	{
		if ((length_1 / length_2) < threshold)
		{
			gaps++;
		}
	}
	else
	{
		if ((length_2 / length_1) < threshold)
		{
			gaps++;
		}
	}
	features_computed_[11] = true;
	features_[11] = gaps;
	return features_[11];
}

//Calculate Feature 13: The Kurtosis, which is given by:
//(Sum((x - mean)^4))/sigma^4) - 3, where mean is the mean-value and sigma is the standard deviation
double LaserScannerFeatures::calc_feature13(const std::vector<double>& beams)
{
	if (features_computed_[12])
		return features_[12];

	double sum = 0;
	//get the standard deviation and the mean by using previous functions
	double sigma = calc_feature6(beams);
	double mean = calc_feature5(beams);
	//calculate the Kurtosis
	for (int b = 0; b < beams.size(); b++)
	{
		double v=(beams[b] - mean);
		sum += v*v*v*v;	//std::pow((beams[b] - mean), 4);
	}
	features_computed_[12] = true;
	features_[12] = ((sum / std::pow(sigma, 4)) - 3);

	if (features_[12]!=features_[12])
		std::cout << "   features_[12]="<<features_[12]<<std::endl;

	return features_[12];
}

//Calc Feature 22: The average of the beam lengths divided by the maximal length
double LaserScannerFeatures::calc_feature22(const std::vector<double>& beams)
{
	if (features_computed_[21])
		return features_[21];

	double sum;
	double maxval = 0.;
	//find maximal value of the beams
	for (int b = 0; b < beams.size(); b++)
		if (beams[b] > maxval)
			maxval = beams[b];
	if (maxval == 0.)
		maxval = 1.;
	//get the average of the beams/maxval
	//get the sum of the beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		sum += (beams[b] / maxval);
	}
	//divide by number of beams and return value
	features_computed_[21] = true;
	features_[21] = (sum / (double)beams.size());

	if (features_[21]!=features_[21])
		std::cout << "   features_[21]="<<features_[21]<<std::endl;

	return features_[21];
}

//Calculate Feature 23: The standard deviation of the beam lengths divided by the maximal length
double LaserScannerFeatures::calc_feature23(const std::vector<double>& beams)
{
	if (features_computed_[22])
		return features_[22];

	double sum = 0;
	double mean = calc_feature22(beams);
	double maxval = 0;
	//find maximal value of the beams
	for (int b = 0; b < beams.size(); b++)
		if (beams[b] > maxval)
			maxval = beams[b];
	if (maxval == 0.)
		maxval = 1.;
	const double maxvalinv = 1./maxval;
	//get the standard deviation
	for (int b = 0; b < beams.size(); b++)
	{
		const double v = (beams[b] * maxvalinv) - mean;
		sum += v*v;
	}
	sum = sum / (beams.size() - 1);
	features_computed_[22] = true;
	features_[22] = std::sqrt(sum);

	if (features_[22]!=features_[22])
		std::cout << "   features_[22]="<<features_[22]<<std::endl;

	return features_[22];
}

//*******************Features based on a polygonal approximation of the beams*******************
//Calculate the polygonal approximation
std::vector<cv::Point> LaserScannerFeatures::calc_polygonal_approx(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (polygon_computed_ == true)
		return polygon_;

	polygon_.clear();
	double x, y;
	//calculate the endpoint for every beam and add it to the polygon
	for (int b = 0; b < beams.size(); b++)
	{ //calculate the x/y-values
	  //Remark: angles in radiant
		double pi_to_degree = PI / 180;
		x = std::cos(angles[b] * pi_to_degree) * beams[b];
		y = std::sin(angles[b] * pi_to_degree) * beams[b];
		polygon_.push_back(cv::Point(location.x + x, location.y + y));
	}
	polygon_computed_ = true;
	return polygon_;
}

//Calculate the centroid of the polygonal approximation
cv::Point LaserScannerFeatures::calc_centroid(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (centroid_computed_ == true)
		return centroid_;

	double x, y;
	double sumX = 0;
	double sumY = 0;
	//get every Point by using the polygonal approximation
	std::vector<cv::Point> polygon = calc_polygonal_approx(beams, angles, location);
	for (int p = 0; p < polygon.size(); p++)
	{
		sumX += polygon[p].x;
		sumY += polygon[p].y;
	}
	centroid_.x = sumX / polygon.size();
	centroid_.y = sumY / polygon.size();
	centroid_computed_ = true;
	return centroid_;
}

//Calculate Feature 14: The area of the polygonal approximation of the beams
double LaserScannerFeatures::calc_feature14(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[13])
		return features_[13];

	double map_resolution = 0.05000;
	std::vector<cv::Point> polygon = calc_polygonal_approx(beams, angles, location);
	features_computed_[13] = true;
	features_[13] = map_resolution * map_resolution * cv::contourArea(polygon);

	if (features_[13]!=features_[13])
		std::cout << "   features_[13]="<<features_[13]<<std::endl;

	return features_[13];
}

//Calculate Feature 15: The perimeter of the polygonal approximation of the beams
double LaserScannerFeatures::calc_feature15(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[14])
		return features_[14];

	std::vector<cv::Point> polygon = calc_polygonal_approx(beams, angles, location);
	features_computed_[14] = true;
	features_[14] = cv::arcLength(polygon, true);

	if (features_[14]!=features_[14])
		std::cout << "   features_[14]="<<features_[14]<<std::endl;

	return features_[14];
}

//Calculate Feature 16: The quotient of area divided by perimeter of the polygonal approximation of the beams
double LaserScannerFeatures::calc_feature16(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[15])
		return features_[15];

	features_computed_[15] = true;
	features_[15] = (calc_feature14(beams, angles, location) / calc_feature15(beams, angles, location));

	if (features_[15]!=features_[15])
		std::cout << "   features_[15]="<<features_[15]<<std::endl;

	return features_[15];
}

//Calculate Feature 17: The average of the distance between the centroid and the boundary-Points of the polygonal approximation
double LaserScannerFeatures::calc_feature17(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[16])
		return features_[16];

	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point centroid = calc_centroid(beams, angles, location);
	double sum = 0;
	double delta_x, delta_y;
	//calculate the distance between the centroid and the boundary and add it to the sum
	for (int p = 0; p < polygon.size(); p++)
	{
		delta_x = polygon[p].x - centroid.x;
		delta_y = polygon[p].y - centroid.y;
		sum += std::sqrt(delta_x*delta_x + delta_y*delta_y);	//std::pow(delta_x, 2) + std::pow(delta_y, 2));
	}
	//calculate and return the average of the distances
	features_computed_[16] = true;
	features_[16] = (sum / polygon.size());

	if (features_[16]!=features_[16])
		std::cout << "   features_[16]="<<features_[16]<<std::endl;

	return features_[16];
}

//Calculate Feature 18: The standard deviation of the distance between the centroid and the boundary-Points
double LaserScannerFeatures::calc_feature18(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[17])
		return features_[17];

	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point centroid = calc_centroid(beams, angles, location);
	//get the mean of the distances by using Feature 17
	double mean = calc_feature17(beams, angles, location);
	double current_distance;
	double sum = 0;
	//calculate the standard_deviation
	for (int p = 0; p < polygon.size(); p++)
	{
		double delta_x = polygon[p].x - centroid.x;
		double delta_y = polygon[p].y - centroid.y;
		current_distance = std::sqrt(delta_x*delta_x + delta_y*delta_y);	//std::pow(delta_x, 2) + std::pow(delta_y, 2));
		sum += (current_distance - mean)*(current_distance - mean);	//std::pow(current_distance - mean, 2);
	}
	sum = sum / (beams.size() - 1);
	features_computed_[17] = true;
	features_[17] = std::sqrt(sum);

	if (features_[17]!=features_[17])
		std::cout << "   features_[17]="<<features_[17]<<std::endl;

	return features_[17];
}

//Calculate Feature 19: The half major axis of the bounding ellipse, calculatet with openCV
double LaserScannerFeatures::calc_feature19(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[18])
		return features_[18];

	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point centroid = calc_centroid(beams, angles, location);
	cv::Point2f points[4];
	std::vector < cv::Point2f > edge_points;
	double distance = 0;
	//saving-variable for the Points of the ellipse
	cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(polygon));
	//get the edge-points of the ellipse
	ellipse.points(points);
	//saving the Points of the ellipse in a vector
	for (int i = 0; i < 4; i++)
	{
		edge_points.push_back(points[i]);
	}
	//calculate the distance between the Points and take the largest one
	for (int p = 0; p < edge_points.size(); p++)
	{
		for (int np = 0; np < edge_points.size(); np++)
		{
			//if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) > distance)
			const float a = (edge_points[p].x - edge_points[np].x);
			const float b = (edge_points[p].y - edge_points[np].y);
			const double sqr = a*a + b*b;
			if (sqr > distance)
			{
				//distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2));
				distance = sqr;
			}
		}
	}
	features_computed_[18] = true;
	features_[18] = (std::sqrt(distance) / 2);

	if (features_[18]!=features_[18])
		std::cout << "   features_[18]="<<features_[18]<<std::endl;

	return features_[18];
}

//Calculate Feature 20: The half minor axis of the bounding ellipse, calculated with openCV
double LaserScannerFeatures::calc_feature20(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[19])
		return features_[19];

	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point2f points[4];
	std::vector < cv::Point2f > edge_points;
	double distance = 1e6*1e6;
	//saving-variable for the Points of the ellipse
	cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(polygon));
	//get the edge-points of the ellipse
	ellipse.points(points);
	//saving the Points of the ellipse in a vector
	for (int i = 0; i < 4; i++)
	{
		edge_points.push_back(points[i]);
	}
	//calculate the distance between the Points and take the largest one
	for (int p = 0; p < edge_points.size(); p++)
	{
		for (int np = 0; np < edge_points.size(); np++)
		{
			if (p==np)
				continue;
//			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) < distance
//			        && std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) > 0 && p != np)
			const float a = (edge_points[p].x - edge_points[np].x);
			const float b = (edge_points[p].y - edge_points[np].y);
			const double sqr = a*a + b*b;
			if (sqr < distance)	// && sqr > 0) // && p != np)
			{
				distance = sqr;
			}
		}
	}
	features_computed_[19] = true;
	features_[19] = (std::sqrt(distance) / 2);

	if (features_[19]!=features_[19])
		std::cout << "   features_[19]="<<features_[19]<<std::endl;

	return features_[19];
}

//Calculate Feature 21: The Quotient of half the major axis and half the minor axis
double LaserScannerFeatures::calc_feature21(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[20])
		return features_[20];

	features_computed_[20] = true;
	features_[20] = (calc_feature19(beams, angles, location) / (0.0001+calc_feature20(beams, angles, location)));

	if (features_[20]!=features_[20])
		std::cout << "   features_[20]="<<features_[20]<<std::endl;

	return features_[20];
}

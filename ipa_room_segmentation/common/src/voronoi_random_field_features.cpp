#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <opencv/cv.h>

#include <ipa_room_segmentation/voronoi_random_field_features.h>

#define PI 3.14159265

//get the number of implemented features. Needs to be changed to the new value if you change it
int get_feature_count()
{
	return 23;
}
//**********************see features.h for a better overview of what is calculated and needed*************************
//Method for calculating the feature for the classifier
double get_feature(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point point, const int feature)
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

//Calculation of Feature 1: average difference of the beams
double calc_feature1(const std::vector<double>& beams)
{
	double differences_sum;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		differences_sum += abs(beams[b] - beams[b + 1]);
	}
	//get the difference betweeen the last and the first beam
	differences_sum += abs(beams[beams.size() - 1] - beams[0]);
	//calculate the average difference and return it
	return (differences_sum / beams.size());
}

//Calculation of Feature 2: standard deviation of the difference of the beams
double calc_feature2(const std::vector<double>& beams)
{
	double feature2_value;
	double mean; //mean-value of the difference, calculated with calc_feature1
	double sum; //helping variable
	//initialise
	mean = calc_feature1(beams);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2.0);
	}
	sum = sum / (beams.size() - 1);
	feature2_value = std::sqrt(sum);
	return feature2_value;
}

//Calculation of Feature 3: average difference of the to a max_value limited beams
double calc_feature3(const std::vector<double>& beams, double maxval)
{
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
	if (beams[beams.size()] < maxval)
	{
		val1 = beams[beams.size() - 1];
	}
	if (beams[0] < maxval)
	{
		val2 = beams[0];
	}
	differences_sum += abs(val1 - val2);
	//calculate the average difference and return it
	return (differences_sum / beams.size());
}

//Calculation of Feature 4: The Standard Deviation of the difference of limited beams
double calc_feature4(const std::vector<double>& beams, double maxval)
{
	double feature4_value;
	double mean; //mean-value of the difference, calculated with calc_feature1
	double v, w, difference, sum; //helping variables
	//initialise
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
		sum += std::pow((difference - mean), 2.0);
	}
	//add the difference from last to first point
	v = maxval;
	w = maxval;
	if (beams[beams.size()] < maxval)
	{
		v = beams[beams.size() - 1];
	}
	if (beams[0] < maxval)
	{
		w = beams[0];
	}
	difference = abs(v - w);
	sum += std::pow((difference - mean), 2.0);
	sum = sum / (beams.size() - 1);
	feature4_value = std::sqrt(sum);
	return feature4_value;
}

//Calculation of Feature 5: The average beamlength
double calc_feature5(const std::vector<double>& beams)
{
	double sum;
	//get the sum of the beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		sum += beams[b];
	}
	//divide by number of beams and return value
	return (sum / beams.size());
}

//Calculation of Feature 6: The standard deviation of the beamlenghts
double calc_feature6(const std::vector<double>& beams)
{
	double mean; //mean-value of the beamlenghts, calculated with calc_feature5
	double sum; //helping variable
	//initialise
	mean = calc_feature5(beams);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2);
	}
	sum = sum / (beams.size() - 1);
	return std::sqrt(sum);
}

//Calculation of Feature 7: The number of gaps between the beams, a gap is when the difference of the lenghts is larger
//than a specified threshold
double calc_feature7(const std::vector<double>& beams)
{
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
	return gaps;
}

//Calculation of feature 8: The distance between two Endpoints of local minima of beamlenghts
double calc_feature8(const std::vector<double>& beams, const std::vector<double>& angles)
{
	//Remark: angles are relatively to the robot
	double length_1 = 10000000;
	double length_2 = 10000000;
	double angle_1, angle_2;
	double x_1, y_1, x_2, y_2;
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
	x_1 = std::cos(angle_1 * PI / 180) * length_1;
	y_1 = std::sin(angle_1 * PI / 180) * length_1;
	x_2 = std::cos(angle_2 * PI / 180) * length_2;
	y_2 = std::sin(angle_2 * PI / 180) * length_2;
	//calculate and return the euclidean distance between the Points
	return std::sqrt(std::pow((x_1 - x_2), 2) + std::pow((y_1 - y_2), 2));
}

//Calculate Feature 9: The Angle between two Endpoints of local minima of beamlengths
double calc_feature9(const std::vector<double>& beams, const std::vector<double>& angles)
{
	//Remark: angles are relatively to the robot
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
	double pi_to_degree = PI / 180;
	x_1 = std::cos(angle_1 * pi_to_degree) * length_1;
	y_1 = std::sin(angle_1 * pi_to_degree) * length_1;
	x_2 = std::cos(angle_2 * pi_to_degree) * length_2;
	y_2 = std::sin(angle_2 * pi_to_degree) * length_2;
	//calculate and return the angle between the Points
	double coordvec = (x_1 * x_2) + (y_1 * y_2);
	double absvec = (length_1 * length_2);
	return std::acos(coordvec / absvec) * 180.0 / PI;
}

//Calculate Feature 10: The average of the relations (b_i/b_(i+1)) between two neighboring beams
double calc_feature10(const std::vector<double>& beams)
{
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
	return (sum_relation / beams.size());
}

//Calculate Feature 11: The standard deviation of the relations (b_i/b_(i+1)) between two neighboring beams
double calc_feature11(const std::vector<double>& beams)
{
	//calculate the mean of the relations by using Feature 10
	double mean = calc_feature10(beams);
	double sum = 0;
	//calculate the standard_deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2);
	}
	sum = sum / (beams.size() - 1);
	return std::sqrt(sum);
}

//Calculate Feature 12: The number of relative gaps. A relative gap is when the relation (b_i/b_(i+1)) is smaller than a
//specified threshold
double calc_feature12(const std::vector<double>& beams)
{
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
	return gaps;
}

//Calculate Feature 13: The Kurtosis, which is given by:
//(Sum((x - mean)^4))/sigma^4) - 3, where mean is the mean-value and sigma is the standard deviation
double calc_feature13(const std::vector<double>& beams)
{
	double sum = 0;
	//get the standard deviation and the mean by using previous functions
	double sigma = calc_feature6(beams);
	double mean = calc_feature5(beams);
	//calculate the Kurtosis
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 4);
	}
	return ((sum / std::pow(sigma, 4)) - 3);
}

//Calc Feature 22: The average of the beam lengths divided by the maximal length
double calc_feature22(const std::vector<double>& beams)
{
	double sum;
	double maxval = 0;
	//find maximal value of the beams
	for (int b = 0; b < beams.size(); b++)
	{
		if (beams[b] > maxval)
		{
			maxval = beams[b];
		}
	}
	//get the average of the beams/maxval
	//get the sum of the beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		sum += (beams[b] / maxval);
	}
	//divide by number of beams and return value
	return (sum / beams.size());
}

//Calculate Feature 23: The standard deviation of the beam lengths divided by the maximal length
double calc_feature23(const std::vector<double>& beams)
{
	double sum = 0;
	double mean = calc_feature22(beams);
	double maxval = 0;
	//find maximal value of the beams
	for (int b = 0; b < beams.size(); b++)
	{
		if (beams[b] > maxval)
		{
			maxval = beams[b];
		}
	}
	//get the standard deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow(((beams[b] / maxval) - mean), 2);
	}
	sum = sum / (beams.size() - 1);
	return std::sqrt(sum);
}

//*******************Features based on a polygonal approximation of the beams*******************
//Calculate the polygonal approximation
std::vector<cv::Point> calc_polygonal_approx(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	std::vector < cv::Point > polygon;
	double x, y;
	//calculate the endpoint for every beam and add it to the polygon
	for (int b = 0; b < beams.size(); b++)
	{ //calculate the x/y-values
	  //Remark: angles in radian
		double pi_to_degree = PI / 180;
		x = std::cos(angles[b] * pi_to_degree) * beams[b];
		y = std::sin(angles[b] * pi_to_degree) * beams[b];
		polygon.push_back(cv::Point(location.x + x, location.y + y));
	}
	return polygon;
}

//Calculate the centroid of the polygonal approximation
cv::Point calc_centroid(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	double x, y;
	double sumX = 0;
	double sumY = 0;
	//get every Point by using the polygonal approximation
	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	for (int p = 0; p < polygon.size(); p++)
	{
		sumX += polygon[p].x;
		sumY += polygon[p].y;
	}
	x = sumX / polygon.size();
	y = sumY / polygon.size();
	return cv::Point(x, y);
}

//Calculate Feature 14: The area of the polygonal approximation of the beams
double calc_feature14(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	double map_resolution = 0.05000;
	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	return map_resolution * map_resolution * cv::contourArea(polygon);
}

//Calculate Feature 15: The perimeter of the polygonal approximation of the beams
double calc_feature15(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	return cv::arcLength(polygon, true);
}

//Calculate Feature 16: The quotient of area divided by perimeter of the polygonal approximation of the beams
double calc_feature16(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	return (calc_feature14(beams, angles, location) / calc_feature15(beams, angles, location));
}

//Calculate Feature 17: The average of the distance between the centroid and the boundary-Points of the polygonal approximation
double calc_feature17(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point centroid = calc_centroid(beams, angles, location);
	double sum = 0;
	double delta_x, delta_y;
	//calculate the distance between the centroid and the boundary and add it to the sum
	for (int p = 0; p < polygon.size(); p++)
	{
		delta_x = polygon[p].x - centroid.x;
		delta_y = polygon[p].y - centroid.y;
		sum += std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
	}
	//calculate and return the average of the distances
	return (sum / polygon.size());
}

//Calculate Feature 18: The standard deviation of the distance between the centroid and the boundary-Points
double calc_feature18(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
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
		current_distance = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));
		sum += std::pow(current_distance - mean, 2);
	}
	sum = sum / (beams.size() - 1);
	return std::sqrt(sum);
}

//Calculate Feature 19: The half major axis of the bounding ellipse, calculatet with openCV
double calc_feature19(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
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
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) > distance)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2));
			}
		}
	}
	return (distance / 2);
}

//Calculate Feature 20: The half minor axis of the bounding ellipse, calculated with openCV
double calc_feature20(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	std::vector < cv::Point > polygon = calc_polygonal_approx(beams, angles, location);
	cv::Point2f points[4];
	std::vector < cv::Point2f > edge_points;
	double distance = 1000000;
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
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) < distance
			        && std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) > 0 && p != np)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2));
			}
		}
	}
	return (distance / 2);
}

//Calculate Feature 21: The Quotient of half the major axis and half the minor axis
double calc_feature21(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	return (calc_feature19(beams, angles, location) / calc_feature20(beams, angles, location));
}

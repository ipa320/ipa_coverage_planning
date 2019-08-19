#include <ipa_room_segmentation/voronoi_random_field_features.h>

#define PI 3.14159265

// structure to perform breadth-first-search to detect minimal loops

//get the number of implemented features. Needs to be changed to the new value if you change it
int voronoiRandomFieldFeatures::getFeatureCount()
{
	return 28;
}

// reset the saved features
void voronoiRandomFieldFeatures::resetCachedData()
{
	features_.clear();
	features_.resize(getFeatureCount(), 0.);
	features_computed_.clear();
	features_computed_.resize(getFeatureCount(), false);

	polygon_.clear();
	polygon_computed_ = false;

	centroid_computed_ = false;
}

//**********************see features.h for a better overview of what is calculated and needed*************************
//Method for calculating the feature for the classifier
double voronoiRandomFieldFeatures::getFeature(const std::vector<double>& beams, const std::vector<double>& angles,
		const std::vector<cv::Point>& clique_points, std::vector<unsigned int>& labels_for_clique_points,
		std::vector<unsigned int>& possible_labels, cv::Point point, const int feature)
{
	switch (feature)
	{
	case 1:
		return calcFeature1(beams);
	case 2:
		return calcFeature2(beams);
	case 3:
		return calcFeature3(beams, 30);
	case 4:
		return calcFeature4(beams, 30);
	case 5:
		return calcFeature5(beams);
	case 6:
		return calcFeature6(beams);
	case 7:
		return calcFeature7(beams);
	case 8:
		return calcFeature8(beams, angles);
	case 9:
		return calcFeature9(beams, angles);
	case 10:
		return calcFeature10(beams);
	case 11:
		return calcFeature11(beams);
	case 12:
		return calcFeature12(beams);
	case 13:
		return calcFeature13(beams);
	case 14:
		return calcFeature14(beams, angles, point);
	case 15:
		return calcFeature15(beams, angles, point);
	case 16:
		return calcFeature16(beams, angles, point);
	case 17:
		return calcFeature17(beams, angles, point);
	case 18:
		return calcFeature18(beams, angles, point);
	case 19:
		return calcFeature19(beams, angles, point);
	case 20:
		return calcFeature20(beams, angles, point);
	case 21:
		return calcFeature21(beams, angles, point);
	case 22:
		return calcFeature22(beams);
	case 23:
		return calcFeature23(beams);
	case 24:
		return calcFeature24(clique_points);
	case 25:
		return calcFeature25(possible_labels, labels_for_clique_points);
	case 26:
		return calcFeature26(beams, 22);
	case 27:
		return calcFeature27(beams, angles, 8, point);
	case 28:
		return calcFeature28(beams, 5);
	default:
		return -1;
	}
}

void voronoiRandomFieldFeatures::getFeatures(const std::vector<double>& beams, const std::vector<double>& angles, const std::vector<cv::Point>& clique_points, std::vector<unsigned int>& labels_for_clique_points,
			std::vector<unsigned int>& possible_labels, cv::Point point, std::vector<double>& features)
{
	// reset internal data storage
	resetCachedData();

	// compute features
	calcFeature1(beams);
	calcFeature2(beams);
	calcFeature3(beams, 30);
	calcFeature4(beams, 30);
	calcFeature5(beams);
	calcFeature6(beams);
	calcFeature7(beams);
	calcFeature8(beams, angles);
	calcFeature9(beams, angles);
	calcFeature10(beams);
	calcFeature11(beams);
	calcFeature12(beams);
	calcFeature13(beams);
	calcFeature14(beams, angles, point);
	calcFeature15(beams, angles, point);
	calcFeature16(beams, angles, point);
	calcFeature17(beams, angles, point);
	calcFeature18(beams, angles, point);
	calcFeature19(beams, angles, point);
	calcFeature20(beams, angles, point);
	calcFeature21(beams, angles, point);
	calcFeature22(beams);
	calcFeature23(beams);
	calcFeature24(clique_points);
	calcFeature25(possible_labels, labels_for_clique_points);
	calcFeature26(beams, 22);
	calcFeature27(beams, angles, 8, point);
	calcFeature28(beams, 5);

	// write features
	features.clear();
	features = features_;
}

//Calculation of Feature 1: average difference of the beams
double voronoiRandomFieldFeatures::calcFeature1(const std::vector<double>& beams)
{
	if (features_computed_[0])
		return features_[0];

	double differences_sum = 0;
	for (int b = 0; b < beams.size() - 1; b++)
	{
		differences_sum += abs(beams[b] - beams[b + 1]);
	}
	//get the difference between the last and the first beam
	differences_sum += abs(beams[beams.size() - 1] - beams[0]);

	// set the cache of the calculated feature
	features_computed_[0] = true;
	features_[0] = (differences_sum / (double)beams.size());

	//calculate the average difference and return it
	return features_[0];
}

//Calculation of Feature 2: standard deviation of the difference of the beams
double voronoiRandomFieldFeatures::calcFeature2(const std::vector<double>& beams)
{
	if (features_computed_[1])
		return features_[1];

	double mean; //mean-value of the difference, calculated with calcFeature1
	double sum = 0; //helping variable
	//initialize
	mean = calcFeature1(beams);
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2.0);
	}
//	std::cout << "f2: (" << sum << ") ";
	sum = sum / (double) (beams.size() - 1);

	features_computed_[1] = true;
	features_[1] = std::sqrt(sum);

	return features_[1];
}

//Calculation of Feature 3: average difference of the to a max_value limited beams
double voronoiRandomFieldFeatures::calcFeature3(const std::vector<double>& beams, double maxval)
{
	if (features_computed_[2])
		return features_[2];

	double differences_sum = 0;
	double val1, val2;
	for (int b = 0; b < beams.size()-1; b++)
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
	//get the difference between the last and the first beam
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

	return features_[2];
}

//Calculation of Feature 4: The Standard Deviation of the difference of limited beams
double voronoiRandomFieldFeatures::calcFeature4(const std::vector<double>& beams, double maxval)
{
	if (features_computed_[3])
		return features_[3];

	double mean; //mean-value of the difference, calculated with calcFeature1
	double v, w, difference, sum; //helping variables
	//initialise
	mean = calcFeature3(beams, maxval);
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
	if (beams[beams.size()-1] < maxval)
	{
		v = beams[beams.size()-1];
	}
	if (beams[0] < maxval)
	{
		w = beams[0];
	}
	difference = abs(v - w);
	sum += std::pow((difference - mean), 2.0);
	sum = sum / (beams.size() - 1);

	features_computed_[3] = true;
	features_[3] = std::sqrt(sum);

	return features_[3];
}

//Calculation of Feature 5: The average beamlength
double voronoiRandomFieldFeatures::calcFeature5(const std::vector<double>& beams)
{
	if (features_computed_[4])
		return features_[4];

	double sum = 0;
	//get the sum of the beamlengths
	for (int b = 0; b < beams.size(); b++)
	{
		sum += beams[b];
	}
	//divide by number of beams and return value
	features_computed_[4] = true;
	features_[4] = (sum / (double)beams.size());

	return features_[4];
}

//Calculation of Feature 6: The standard deviation of the beamlenghts
double voronoiRandomFieldFeatures::calcFeature6(const std::vector<double>& beams)
{
	if (features_computed_[5])
		return features_[5];

	double mean; //mean-value of the beamlenghts, calculated with calcFeature5
	double sum, res; //helping variables
	//initialize
	mean = calcFeature5(beams);
	sum = 0;
	//calculate deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2.0);
	}
	res = sum / (double)(beams.size() - 1);
//	std::cout << "f6: (" << res << ") ";

	features_computed_[5] = true;
	features_[5] = std::sqrt(res);

	return features_[5];
}

//Calculation of Feature 7: The number of gaps between the beams, a gap is when the difference of the lenghts is larger
//than a specified threshold
double voronoiRandomFieldFeatures::calcFeature7(const std::vector<double>& beams)
{
	if (features_computed_[6])
		return features_[6];

	double threshold = 10; //[pixel], see "Semantic labeling of places"
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

	return gaps;
}

//Calculation of feature 8: The distance between two Endpoints of local minima of beamlenghts
double voronoiRandomFieldFeatures::calcFeature8(const std::vector<double>& beams, const std::vector<double>& angles)
{
	if (features_computed_[7])
		return features_[7];

	//Remark: angles are relatively to the robot
	double length_1 = 10000000;
	double length_2 = 10000000;
	double angle_1 = angles[0], angle_2 = angles[1];
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
	features_computed_[7] = true;
	features_[7] = std::sqrt(std::pow((x_1 - x_2), 2) + std::pow((y_1 - y_2), 2));

	return features_[7];
}

//Calculate Feature 9: The Angle between two Endpoints of local minima of beamlengths
double voronoiRandomFieldFeatures::calcFeature9(const std::vector<double>& beams, const std::vector<double>& angles)
{
	if (features_computed_[8])
		return features_[8];

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
	const double quot = std::max(-1., std::min(1., coordvec / absvec));
	// if the raycasting-algorithm only found zero-beams return 0
	features_computed_[8] = true;
	features_[8] = std::acos(quot) * 180.0 / PI;

	return features_[8];
}

//Calculate Feature 10: The average of the relations (b_i/b_(i+1)) between two neighboring beams
double voronoiRandomFieldFeatures::calcFeature10(const std::vector<double>& beams)
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
	features_[9] = (sum_relation / (double)beams.size());

	return features_[9];
}

//Calculate Feature 11: The standard deviation of the relations (b_i/b_(i+1)) between two neighboring beams
double voronoiRandomFieldFeatures::calcFeature11(const std::vector<double>& beams)
{
	if (features_computed_[10])
		return features_[10];

	//calculate the mean of the relations by using Feature 10
	double mean = calcFeature10(beams);
	double sum = 0;
	//calculate the standard_deviation
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 2);
	}
	sum = sum / (double)(beams.size() - 1);
	features_computed_[10] = true;
	features_[10] = std::sqrt(sum);

	return features_[10];
}

//Calculate Feature 12: The number of relative gaps. A relative gap is when the relation (b_i/b_(i+1)) is smaller than a
//specified threshold
double voronoiRandomFieldFeatures::calcFeature12(const std::vector<double>& beams)
{
	if (features_computed_[11])
		return features_[11];

	double threshold = 0.85;
	double gaps = 0;
	double length_1, length_2;
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
double voronoiRandomFieldFeatures::calcFeature13(const std::vector<double>& beams)
{
	if (features_computed_[12])
		return features_[12];

	double sum = 0;
	//get the standard deviation and the mean by using previous functions
	double sigma = calcFeature6(beams);
	double mean = calcFeature5(beams);
	//calculate the Kurtosis
	for (int b = 0; b < beams.size(); b++)
	{
		sum += std::pow((beams[b] - mean), 4);
	}
	features_computed_[12] = true;
	features_[12] = ((sum / std::pow(sigma, 4)) - 3);

	return features_[12];
}

//Calc Feature 22: The average of the beam lengths divided by the maximal length
double voronoiRandomFieldFeatures::calcFeature22(const std::vector<double>& beams)
{
	if (features_computed_[21])
		return features_[21];

	double sum = 0;
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
	features_computed_[21] = true;
	features_[21] = (sum / (double)beams.size());

	return features_[21];
}

//Calculate Feature 23: The standard deviation of the beam lengths divided by the maximal length
double voronoiRandomFieldFeatures::calcFeature23(const std::vector<double>& beams)
{
	if (features_computed_[22])
		return features_[22];

	double sum = 0;
	double mean = calcFeature22(beams);
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
	sum = sum / (double) (beams.size() - 1);

	features_computed_[22] = true;
	features_[22] = std::sqrt(sum);

	return features_[22];
}

//*******************Features based on a polygonal approximation of the beams*******************
//Calculate the polygonal approximation
std::vector<cv::Point> voronoiRandomFieldFeatures::calcPolygonalApprox(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (polygon_computed_ == true)
		return polygon_;

	polygon_.clear();
	double x, y;
	//calculate the endpoint for every beam and add it to the polygon
	for (int b = 0; b < beams.size(); b++)
	{ //calculate the x/y-values
	  //Remark: angles in radian
		double pi_to_degree = PI / 180;
		x = std::cos(angles[b] * pi_to_degree) * beams[b];
		y = std::sin(angles[b] * pi_to_degree) * beams[b];
		polygon_.push_back(cv::Point(location.x + x, location.y + y));
	}

	polygon_computed_ = true;
	return polygon_;
}

//Calculate the centroid of the polygonal approximation
cv::Point voronoiRandomFieldFeatures::calcCentroid(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (centroid_computed_ == true)
		return centroid_;

	double x, y;
	double sumX = 0;
	double sumY = 0;
	//get every Point by using the polygonal approximation
	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
	for (int p = 0; p < polygon.size(); p++)
	{
		sumX += polygon[p].x;
		sumY += polygon[p].y;
	}

	centroid_.x = sumX / (double) polygon.size();
	centroid_.y = sumY / (double) polygon.size();
	centroid_computed_ = true;
	return centroid_;
}

//Calculate Feature 14: The area of the polygonal approximation of the beams
double voronoiRandomFieldFeatures::calcFeature14(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[13])
		return features_[13];

	double map_resolution = 0.05000;
	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);

	features_computed_[13] = true;
	features_[13] = map_resolution * map_resolution * cv::contourArea(polygon);

	return features_[13];
}

//Calculate Feature 15: The perimeter of the polygonal approximation of the beams
double voronoiRandomFieldFeatures::calcFeature15(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[14])
		return features_[14];

	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
	features_computed_[14] = true;
	features_[14] = cv::arcLength(polygon, true);

	return features_[14];
}

//Calculate Feature 16: The quotient of area divided by perimeter of the polygonal approximation of the beams
double voronoiRandomFieldFeatures::calcFeature16(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[15])
		return features_[15];

	features_computed_[15] = true;
	features_[15] = (calcFeature14(beams, angles, location) / calcFeature15(beams, angles, location));

	return features_[15];
}

//Calculate Feature 17: The average of the distance between the centroid and the boundary-Points of the polygonal approximation
double voronoiRandomFieldFeatures::calcFeature17(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[16])
		return features_[16];

	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
	cv::Point centroid = calcCentroid(beams, angles, location);
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
	features_computed_[16] = true;
	features_[16] = (sum / (double) polygon.size());

	return features_[16];
}

//Calculate Feature 18: The standard deviation of the distance between the centroid and the boundary-Points
double voronoiRandomFieldFeatures::calcFeature18(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[17])
		return features_[17];

	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
	cv::Point centroid = calcCentroid(beams, angles, location);
	//get the mean of the distances by using Feature 17
	double mean = calcFeature17(beams, angles, location);
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
	sum = sum / (double) (beams.size() - 1);

	features_computed_[17] = true;
	features_[17] = std::sqrt(sum);

	return features_[17];
}

//Calculate Feature 19: The half major axis of the bounding ellipse, calculated with openCV
double voronoiRandomFieldFeatures::calcFeature19(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[18])
		return features_[18];

	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
	cv::Point centroid = calcCentroid(beams, angles, location);
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

	features_computed_[18] = true;
	features_[18] = (distance / 2);

	return features_[18];
}

//Calculate Feature 20: The half minor axis of the bounding ellipse, calculated with openCV
double voronoiRandomFieldFeatures::calcFeature20(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[19])
		return features_[19];

	std::vector < cv::Point > polygon = calcPolygonalApprox(beams, angles, location);
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

	features_computed_[19] = true;
	features_[19] = (distance / 2);

	return features_[19];
}

//Calculate Feature 21: The Quotient of half the major axis and half the minor axis
double voronoiRandomFieldFeatures::calcFeature21(const std::vector<double>& beams, const std::vector<double>& angles, cv::Point location)
{
	if (features_computed_[20])
		return features_[20];

	features_computed_[20] = true;
	features_[20] = (calcFeature19(beams, angles, location) / calcFeature20(beams, angles, location));

	return features_[20];
}

// Calculate Feature 24: The curvature of the voronoi graph approximated by the points of the clique. The curvature of a graph
//						 is given by k = 1/r, with r as the radius of the approximate circle at this position.
double voronoiRandomFieldFeatures::calcFeature24(std::vector<cv::Point> clique_points)
{
	if (features_computed_[23])
		return features_[23];

	float radius = 0;
	cv::Point2f center;

	// Get the circle that approaches the points of the clique. If the clique has more than three neighbors it is a voronoi-node
	// clique, in which case the mean of the curvatures gets calculated.
	if(clique_points.size() <= 3)
	{
		// get the enclosing circle together with the radius of it
		cv::minEnclosingCircle(clique_points, center, radius);
	}
	else
	{
		// go trough each 3-point combination and calculate the sum of the radius
		for(unsigned int curvature = 1; curvature < clique_points.size(); ++curvature)
		{
			float temporary_radius;

			std::vector<cv::Point> temporary_point_vector(3); // vector that stores the current points

			// fill vector
			temporary_point_vector[0] = clique_points[0];
			temporary_point_vector[1] = clique_points[curvature];
			// Check if i+1 would be larger than last index. If so take the first point that isn't zero as second point.
			if((curvature+1)%clique_points.size() == 0)
				temporary_point_vector[2] = clique_points[1];
			else
				temporary_point_vector[2] = clique_points[(curvature+1)%clique_points.size()];

			// calculate the enclosing circle for this combination
			cv::minEnclosingCircle(temporary_point_vector, center, temporary_radius);

			// add the current radius to sum
			radius += temporary_radius;
		}

		// get the mean
		radius = radius/((double) clique_points.size()-1);
	}

	// return the curvature
	features_computed_[23] = true;
	features_[23] = 1.0/(double)radius;

	return features_[23];
}

// Calculate Feature 25: The relation between the labels of Points from the central point to the other points in the clique.
//						 If two neighboring points have the labels hallway-hallway this feature gets very high and if they
//						 are differing from each other it gets small. To do this the possible_labels-vector stores all the
//						 labels for the different classes that can occur.
//						 !!!!!! Important: !!!!!!
//							The possible_lables-vector stores in this program the labels in the order
//										room-hallway-doorway
double voronoiRandomFieldFeatures::calcFeature25(std::vector<unsigned int>& possible_labels, std::vector<unsigned int>& labels_for_points)
{
	if (features_computed_[24])
		return features_[24];

	// count how often each possible label occurs in the given labels
	int maximal_amount = -1; // integer to save the maximal amount one label occurs

	for(size_t label = 0; label < possible_labels.size(); ++label)
	{
		// check for one label how often it appears
		int label_count = std::count(labels_for_points.begin(), labels_for_points.end(), possible_labels[label]);

		// check if current label appears more often than the previously checked
		if(label_count > maximal_amount)
			maximal_amount = label_count;
	}

	// set the initial value for the feature (40 * maximal_amount, so the feature is always >= 0, even if later a lot of the value gets subtracted)
	double feature_value = 40 * maximal_amount;

	// ***Check for possibility that two neighboring labels can occur. For example it is very unlikely that a hallway appears right
	// after a room, without a doorway between, but it is very likely that a hallway adds to a hallway.

	// Create a map to get a factor to add or subtract from the inital-value for a feature. The key for this map is the sum of
	// two labels, so shows the relation between two points, and the Data for a key is  the factor.
	std::map<unsigned int, double> label_mapping;

	// create each possible key and data (done by hand because in this case all possible configurations are known before --> room,hallway,doorway)
	label_mapping[possible_labels[0] + possible_labels[0]] = 10; // room-room
	label_mapping[possible_labels[0] + possible_labels[1]] = -10; // room-hallway
	label_mapping[possible_labels[0] + possible_labels[2]] = 5; // room-doorway
	label_mapping[possible_labels[1] + possible_labels[1]] = 10; // hallway-hallway
	label_mapping[possible_labels[1] + possible_labels[2]] = 5; // hallway-doorway
	label_mapping[possible_labels[2] + possible_labels[2]] = 10; // doorway-doorway

	// increase or decrease the feature-value
	for(std::vector<unsigned int>::iterator current_point = labels_for_points.begin(); current_point != labels_for_points.end(); ++current_point)
	{
		// check each neighbor that isn't the point itself
		for(std::vector<unsigned int>::iterator current_neighbor = labels_for_points.begin(); current_neighbor != labels_for_points.end(); ++current_neighbor)
		{
			// check if the two labels are not from the same point by calculating the distance in the vector between these two
			if(std::distance(current_point, current_neighbor) != 0)
			{
				// get the key for the mapping
				unsigned int current_sum = *current_point + *current_neighbor;

				feature_value += label_mapping[current_sum];
			}
		}
	}

	features_computed_[24] = true;
	features_[24] = feature_value;

	return features_[24];
}

// Feature 26: number of beams that are shorter than a defined maxval
double voronoiRandomFieldFeatures::calcFeature26(const std::vector<double>& beams, double maxval)
{
	if(features_computed_[25] == true)
		return features_[25];

	int number_of_short_beams = 0;

	for(size_t beam = 0; beam < beams.size(); ++beam)
		if(beams[beam] <= maxval)
			++number_of_short_beams;

	features_computed_[25] = true;
	features_[25] = (double) number_of_short_beams;

	return features_[25];
}

// Feature 27: The are of the bounding box that is calculated for the endpoints of all beams that are limited to a maxval
double voronoiRandomFieldFeatures::calcFeature27(const std::vector<double>& beams, const std::vector<double>& angles, double epsilon, cv::Point location)
{
	if(features_computed_[26] == true)
		return features_[26];

	std::vector<cv::Point> short_points;

	// get minimal value of the beamlengths
	double min_length = 1e5;
	for(size_t beam = 0; beam < beams.size(); ++beam)
		if(beams[beam] <= min_length)
			min_length = beams[beam];

	// search for beams which lengths are in the defined epsilon neighborhood
	double pi_to_degree = PI / 180;
	for(size_t beam = 0; beam < beams.size(); ++beam)
	{
		if(beams[beam] <= (min_length + epsilon) || beams[beam] > (min_length - epsilon))
		{
			double x = std::cos(angles[beam] * pi_to_degree) * beams[beam] + location.x;
			double y = std::sin(angles[beam] * pi_to_degree) * beams[beam] + location.y;
			short_points.push_back(cv::Point(x, y));
		}
	}

	// calculate the bounding box area
	cv::RotatedRect bounding_box = cv::minAreaRect(short_points);

	features_computed_[26] = true;
	features_[26] = bounding_box.size.area();

	return features_[26];
}

// Feature 28: Ratio of the average lengths of n longest and n smalles beams.
double voronoiRandomFieldFeatures::calcFeature28(const std::vector<double>& beams, double number_of_beams)
{
	if(features_computed_[27] == true)
		return features_[27];

	// find n longest and shortest beams
	std::vector<double> longest_beams (number_of_beams);
	std::vector<double> shortest_beams (number_of_beams);
	std::vector<double> sorted_beams = beams;

	std::sort(sorted_beams.begin(), sorted_beams.end());

	int index = 0;
	for(std::vector<double>::iterator beam = sorted_beams.begin(); beam != sorted_beams.begin()+number_of_beams; ++beam)
	{
		longest_beams[index] = *beam;
		++index;
	}

	index = 0;
	for(std::vector<double>::reverse_iterator beam = sorted_beams.rbegin(); beam != sorted_beams.rbegin()+number_of_beams; ++beam)
	{
		shortest_beams[index] = *beam;
		++index;
	}

	// get average beamlengths
	double average_longest = std::accumulate(longest_beams.begin(), longest_beams.end(), 0) / (double) longest_beams.size();
	double average_shortest = std::accumulate(shortest_beams.begin(), shortest_beams.end(), 0) / (double) shortest_beams.size();

	// calculate ratio
	features_computed_[27] = true;
	features_[27] = average_shortest / average_longest;

	return features_[27];

}

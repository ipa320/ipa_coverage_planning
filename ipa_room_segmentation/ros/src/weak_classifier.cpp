#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ipa_room_segmentation/features.h>
#include <ipa_room_segmentation/weak_classifier.h>

WeakClassifier::WeakClassifier()
{
}

WeakClassifier::WeakClassifier(int feature, std::vector<double> weights)
{
	this->feature = feature;
	this->weights = weights;
}

WeakClassifier::WeakClassifier(double theta, int p, int feature)
{
	this->theta = theta;
	this->p = p;
	this->feature = feature;
}

//Training Method for the weak classifiers, so theta and p are calculated for best results
void WeakClassifier::train(std::vector<int> labels, std::vector<cv::Point> points, std::vector<std::vector<double> > beams,
        std::vector<std::vector<double> > angles)
{
	int saved_direction, points_to_sort;
	double current_e;
	double saved_e;
	std::vector<double> saved_features, sorted_features, sorted_weights;
	std::vector<int> sorted_labels, labels_from_trainingfile;
	double maxval;
	int maxval_position;
	//the values of the sum of weights from positive/negativ examples right/left of the threshold
	double pos_right_value = 0.0;
	double neg_right_value = 0.0;
	double pos_left_value = 0.0;
	double neg_left_value = 0.0;
	//the position of the best threshold
	int threshold_position = -1;
	//*********************go trough the given examples and get the threshold******************************
	//1. Calculate the features for every given Point and sort it in a vector of ascending order. Also sort the
	//   labels corresponding to the sorting of the features.
	for (int point = 0; point < points.size(); point++)
	{
		saved_features.push_back(get_feature(beams[point], angles[point], points[point], feature));
		labels_from_trainingfile.push_back(labels[point]);
	}
	points_to_sort = saved_features.size();
	for (int point = 0; point < points_to_sort; point++)
	{
		maxval = -1;
		for (int current_feature_position = 0; current_feature_position < saved_features.size(); current_feature_position++)
		{
			if (saved_features[current_feature_position] >= maxval)
			{
				maxval = saved_features[current_feature_position];
				maxval_position = current_feature_position;
			}
		}
		sorted_features.insert(sorted_features.begin(), maxval);
		saved_features.erase(saved_features.begin() + maxval_position);
		sorted_weights.insert(sorted_weights.begin(), weights[maxval_position]);
		weights.erase(weights.begin() + maxval_position);
		sorted_labels.insert(sorted_labels.begin(), labels_from_trainingfile[maxval_position]);
		labels_from_trainingfile.erase(labels_from_trainingfile.begin() + maxval_position);
	}
	//2. Set the threshold to the first value. The sum of the right examples need to be set to all values then.
	for (int weight = 0; weight < sorted_weights.size(); weight++)
	{
		if (sorted_labels[weight] == 1)
		{
			pos_right_value += sorted_weights[weight];
		}
		else
		{
			neg_right_value += sorted_weights[weight];
		}
	}
	//set inital value for missclassify-variable
	saved_e = pos_right_value + neg_right_value;
	//3. Go trough the sorted examples and find the threshold and direction p that best classifies the given examples
	for (int sorted_feature = 0; sorted_feature < sorted_features.size(); sorted_feature++)
	{
		//check in first direction if number of missclassified Points at this threshold is smaller than saved one
		current_e = neg_right_value + pos_left_value;
		if (current_e < saved_e)
		{
			saved_e = current_e;
			threshold_position = sorted_feature;
			saved_direction = 1;
		}
		//check in second direction if number of missclassified Points at this threshold is smaller than saved one
		current_e = neg_left_value + pos_right_value;
		if (current_e < saved_e)
		{
			saved_e = current_e;
			threshold_position = sorted_feature;
			saved_direction = -1;
		}
		//update the values from left and right
		if (sorted_labels[sorted_feature] == 1)
		{
			pos_right_value -= sorted_weights[sorted_feature];
			pos_left_value += sorted_weights[sorted_feature];
		}
		else if (sorted_labels[sorted_feature] == -1)
		{
			neg_right_value -= sorted_weights[sorted_feature];
			neg_left_value += sorted_weights[sorted_feature];
		}
	}
	//4. Save the best threshold and the direction
	if (threshold_position == 0)
	{
		theta = sorted_features[threshold_position];
	}
	else if (threshold_position == sorted_features.size() - 1)
	{
		theta = sorted_features[threshold_position];
	}
	else
	{
		theta = (sorted_features[threshold_position - 1] + sorted_features[threshold_position]) / 2.0;
	}
	p = saved_direction;
	std::cout << theta << " " << p << std::endl;
}

int WeakClassifier::classify(std::vector<double> beams, std::vector<double> angles, cv::Point point)
{
	if (p == 1)
	{
		if (get_feature(beams, angles, point, feature) >= theta)
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	else if (p == -1)
	{
		if (get_feature(beams, angles, point, feature) < theta)
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	return 9001;
}

int WeakClassifier::classify(double feature_value)
{
	if (p == 1)
	{
		if (feature_value >= theta)
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	else if (p == -1)
	{
		if (feature_value < theta)
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	return 9001;
}

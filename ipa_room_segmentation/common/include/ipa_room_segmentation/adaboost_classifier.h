#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

#include <ctime>
#include <stdlib.h>

#include <ipa_room_segmentation/features.h>
#include <ipa_room_segmentation/watershed_region_spreading.h>
#include <ipa_room_segmentation/contains.h>
#include <ipa_room_segmentation/raycasting.h>

class adaboost_classifier
{
private:
	//variables for calculating-purpose
	double map_resolution_from_subscription_;
	double room_area_factor_lower_limit_;
	double room_area_factor_upper_limit_;
	//variable that shows if the classifiers has already been trained
	bool trained_;
	//angle-vector used to calculate the features for this algorithm
	std::vector<double> angles_for_simulation_;
	//saving-vector for the already used coloures
	std::vector < cv::Scalar > already_used_colours_;
	//Parameters for the classifiers
	CvBoostParams params_;
	//the AdaBoost-classifiers for rooms and hallways
	CvBoost hallway_boost_, room_boost_;
public:
	adaboost_classifier(cv::Mat original_map_from_subscription, double map_resolution_from_subscription, double room_area_factor_lower_limit,
	        double room_area_factor_upper_limit);
	//training-method for the classifier
	void trainClassifiers(cv::Mat first_room_training_map, cv::Mat second_room_training_map, cv::Mat first_hallway_training_map,
	        cv::Mat second_hallway_training_map);
	//labeling-algorithm after the training
	cv::Mat semanticLabeling(cv::Mat map_to_be_labeled);
};

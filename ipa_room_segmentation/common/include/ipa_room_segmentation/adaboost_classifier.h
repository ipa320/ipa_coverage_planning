#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

#include <ctime>
#include <stdlib.h>

#include <ipa_room_segmentation/features.h>
#include <ipa_room_segmentation/raycasting.h>

class AdaboostClassifier
{
protected:

	bool trained_; // variable that shows if the classifiers has already been trained

	std::vector<double> angles_for_simulation_; // angle-vector used to calculate the features for this algorithm

#if CV_MAJOR_VERSION == 2
	CvBoostParams params_; // Parameters for the classifiers
	CvBoost hallway_boost_, room_boost_; // the AdaBoost-classifiers for rooms and hallways
#else
	cv::Ptr<cv::ml::Boost> hallway_boost_, room_boost_; // the AdaBoost-classifiers for rooms and hallways
#endif

	LaserScannerRaycasting raycasting_;

public:


	AdaboostClassifier();


	//training-method for the classifier
	void trainClassifiers(const std::vector<cv::Mat>& room_training_maps, const std::vector<cv::Mat>& hallway_training_maps,
			const std::string& classifier_storage_path, bool load_features=false);


	//labeling-algorithm after the training
	void segmentMap(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
			double room_area_factor_lower_limit, double room_area_factor_upper_limit,
			const std::string& classifier_storage_path, const std::string& classifier_default_path, bool display_results=false);
};

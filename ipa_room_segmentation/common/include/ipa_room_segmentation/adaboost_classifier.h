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


class AdaboostClassifier
{
protected:

	bool trained_; //variable that shows if the classifiers has already been trained

	std::vector<double> angles_for_simulation_; //angle-vector used to calculate the features for this algorithm

	CvBoostParams params_; //Parameters for the classifiers

	CvBoost hallway_boost_, room_boost_; //the AdaBoost-classifiers for rooms and hallways

public:


	AdaboostClassifier();


	//training-method for the classifier
	void trainClassifiers(const cv::Mat& first_room_training_map, const cv::Mat& second_room_training_map, const cv::Mat& first_hallway_training_map, const cv::Mat& second_hallway_training_map, const std::string& classifier_storage_path);


	//labeling-algorithm after the training
	void semanticLabeling(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription, double room_area_factor_lower_limit, double room_area_factor_upper_limit, const std::string& classifier_storage_path);
};

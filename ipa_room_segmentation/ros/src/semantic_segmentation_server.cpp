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

#define PI 3.14159265


bool contains(std::vector<cv::Scalar> vector, cv::Scalar element)
{
	//this functions checks, if the given element is in the given vector (in this case for cv::Sclar elements)
	if (!vector.empty())
	{
		return vector.end() != std::find(vector.begin(), vector.end(), element);
	}
	else
	{
		return false;
	}
}

std::vector<double> bresenham_raycasting(cv::Mat map, cv::Point location)
{
	std::vector<double> beams;
	int x_destination, y_destination;
	int double_x, double_y;
	int dx, dy, xstep, ystep;
	int x_current, y_current;
	int x_start = location.x;
	int y_start = location.y;
	int error, previous_error;
	bool hit_black_pixel;
	//go trough every angle from 0:1:359
	for (double angle = 0; angle < 360; angle++)
	{
		//set destination Point and difference of the coordinates
		x_destination = x_start + std::cos(angle * PI / 180) * 10000;
		y_destination = y_start + std::sin(angle * PI / 180) * 10000;
		dx = x_destination - x_start;
		dy = y_destination - y_start;
		//reset the went distance
		hit_black_pixel = false;
		x_current = 0;
		y_current = 0;
		//check for quadrant in which the line goes
		if (dy < 0)
		{
			ystep = -1;
			dy = -dy;
		}
		else
		{
			ystep = 1;
		}

		if (dx < 0)
		{
			xstep = -1;
			dx = -dx;
		}
		else
		{
			xstep = 1;
		}
		//set the doubled differences
		double_x = 2 * dx;
		double_y = 2 * dy;
		if (double_x >= double_y) //first octant (0 <= slope <= 1) --> favour x
		{
			error = dx;
			previous_error = error;
			//go in direction of the current angle
			do
			{
				x_current += xstep;
				error += double_y;
				if (error > double_x)
				{
					y_current += ystep;
					error -= double_x;
					//check Pixels that vary from bresenham line --> supercover line
					if (error + previous_error < double_x)
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_x)
					{
						if (map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0
						        || map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(x_start + x_current, y_start + y_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			beams.push_back(std::sqrt(std::pow(x_current, 2.0) + std::pow(y_current, 2.0)));
		}
		else // favour y
		{
			error = dy;
			previous_error = error;
			//go in direction of the current angle
			do
			{
				y_current += ystep;
				error += double_x;
				if (error > double_y)
				{
					x_current += xstep;
					error -= double_y;
					//check Pixels that vary from bresenham line --> supercover line
					if (error + previous_error < double_y)
					{
						if (map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_y)
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0
						        || map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(x_start + x_current, y_start + y_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			beams.push_back(std::sqrt(std::pow(x_current, 2.0) + std::pow(y_current, 2.0)));
		}
	}
	return beams;
}

std::vector<double> raycasting(cv::Mat map, cv::Point location)
{
	//Raycasting Algorithm. It simulates the laser measurment at the given location and returns the lengths
	//of the simulated beams
	double simulated_x, simulated_y, simulated_cos, simulated_sin;
	double temporary_distance;
	std::vector<double> distances;
	double delta_x, delta_y;
	for (double angle = 0; angle < 360; angle++)
	{
		simulated_cos = std::cos(angle * PI / 180);
		simulated_sin = std::sin(angle * PI / 180);
		temporary_distance = 90000001;
		for (double distance = 0; distance < 1000000; distance++)
		{
			simulated_x = simulated_cos * distance;
			simulated_y = simulated_sin * distance;
			//make sure the simulated Point isn't out of the boundaries of the map
			if (location.x + simulated_x > 0 && location.x + simulated_x < map.rows && location.y + simulated_y > 0 && location.y + simulated_y < map.cols)
			{
				if (map.at<unsigned char>(location.x + simulated_x, location.y + simulated_y) == 0 && distance < temporary_distance)
				{
					temporary_distance = distance;
					break;
				}
			}
		}
		if (temporary_distance > 90000000)
		{
			temporary_distance = 10;
		}
		distances.push_back(temporary_distance);
	}
	return distances;
}

cv::Mat watershed_region_spread(cv::Mat spreading_image)
{
	//This function spreads the coloured regions of the given Map to the neighboring white Pixels a large enough number of times.
	cv::Mat spreading_map = spreading_image.clone();
	cv::Mat temporary_map_to_fill_white_pixels_ = spreading_image.clone();
	for (int loop_counter = 0; loop_counter < 730; loop_counter++)
	{
		for (int column = 0; column < spreading_map.cols; column++)
		{
			for (int row = 0; row < spreading_map.rows; row++)
			{
				if (spreading_map.at<unsigned char>(row, column) == 255)
				{
					//check 3x3 area around white pixel for fillcolour, if filled Pixel around fill white pixel with that colour
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (temporary_map_to_fill_white_pixels_.at<unsigned char>(row + row_counter, column + column_counter) != 0
							        && temporary_map_to_fill_white_pixels_.at<unsigned char>(row + row_counter, column + column_counter) != 255)
							{
								spreading_map.at<unsigned char>(row, column) = spreading_map.at<unsigned char>(row + row_counter, column + column_counter);
							}
						}
					}
				}
			}
		}
		temporary_map_to_fill_white_pixels_ = spreading_map.clone();
	}
	return temporary_map_to_fill_white_pixels_;
}

int Boost_training(cv::Mat first_room_training_map, cv::Mat second_room_training_map, cv::Mat first_hallway_training_map, cv::Mat second_hallway_training_map)
{
	std::vector<float> labels_for_hallways, labels_for_rooms;
	std::vector<std::vector<float> > hallway_features, room_features;
	std::vector<double> angles_for_simulation, temporary_beams;
	std::vector<float> temporary_features;
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation.push_back(angle);
	}
	//Get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it belongs to a hallway
	//first room training map:
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second room training map:
	for (int y = 0; y < second_room_training_map.cols; y++)
	{
		for (int x = 0; x < second_room_training_map.rows; x++)
		{
			if (second_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//first hallway training map:
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second hallway training map:
	for (int y = 0; y < second_hallway_training_map.cols; y++)
	{
		for (int x = 0; x < second_hallway_training_map.rows; x++)
		{
			if (second_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 400, 0, 2, false, 0);
	//*********hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_hallways.size(), 1, CV_32FC1);
	cv::Mat hallway_features_Mat(hallway_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_hallways.size(); i++)
	{
		hallway_labels_Mat.at<float>(i, 0) = labels_for_hallways[i];
		for (int f = 0; f < 23; f++)
		{
			hallway_features_Mat.at<float>(i, f) = (float) hallway_features[i][f];
		}
	}
	// Train a boost classifier
	CvBoost hallway_boost;
	hallway_boost.train(hallway_features_Mat, CV_ROW_SAMPLE, hallway_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params);
	//save the trained booster
	// todo: do not use fixed paths! use a relative path --> data will end up in ~/.ros/...
	//hallway_boost.save("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_boost.xml", "boost");
	hallway_boost.save("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_boost.xml", "boost");
	ROS_INFO("Done hallway classifiers.");
	//*********room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
	cv::Mat room_features_Mat(room_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_rooms.size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
		for (int f = 0; f < 23; f++)
		{
			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
		}
	}
	// Train a boost classifier
	CvBoost room_boost;
	room_boost.train(room_features_Mat, CV_ROW_SAMPLE, room_labels_Mat, cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), params);
	//save the trained booster
	//room_boost.save("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_boost.xml", "boost");
	room_boost.save("src/autopnp/ipa_room_segmentation/training_results/trained_room_boost.xml", "boost");
	ROS_INFO("Done room classifiers.");

	return 0;
}
int SVM_training(cv::Mat first_room_training_map, cv::Mat second_room_training_map, cv::Mat first_hallway_training_map, cv::Mat second_hallway_training_map)
{
	std::vector<float> labels_for_hallways, labels_for_rooms;
	std::vector<std::vector<float> > hallway_features, room_features;
	std::vector<double> angles_for_simulation, temporary_beams;
	std::vector<float> temporary_features;
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation.push_back(angle);
	}
	//get the labels for every training point. 1.0 means it belongs to a room and -1.0 means it belongs to a hallway
	//first room training map:
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second room training map:
	for (int y = 0; y < second_room_training_map.cols; y++)
	{
		for (int x = 0; x < second_room_training_map.rows; x++)
		{
			if (second_room_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_room_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_rooms.push_back(1.0);
				}
				else
				{
					labels_for_rooms.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_room_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back((float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				room_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//first hallway training map:
	for (int y = 82; y < 357; y++)
	{
		for (int x = 84; x < 361; x++)
		{
			if (first_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (first_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(first_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	//second hallway training map:
	for (int y = 0; y < second_hallway_training_map.cols; y++)
	{
		for (int x = 0; x < second_hallway_training_map.rows; x++)
		{
			if (second_hallway_training_map.at<unsigned char>(x, y) != 0)
			{
				//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
				if (second_hallway_training_map.at<unsigned char>(x, y) != 255)
				{
					labels_for_hallways.push_back(1.0);
				}
				else
				{
					labels_for_hallways.push_back(-1.0);
				}
				//simulate the beams and features for every position and save it
				temporary_beams = raycasting(second_hallway_training_map, cv::Point(x, y));
				for (int f = 1; f <= 23; f++)
				{
					temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f));
				}
				hallway_features.push_back(temporary_features);
				temporary_features.clear();

			}
		}
	}
	// Set up SVM parameters
	CvSVMParams params;
	params.svm_type = CvSVM::ONE_CLASS;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.nu = 0.6;
	//*********hallway***************
	//save the found labels and features in Matrices
	cv::Mat hallway_labels_Mat(labels_for_hallways.size(), 1, CV_32FC1);
	cv::Mat hallway_features_Mat(hallway_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_hallways.size(); i++)
	{
		hallway_labels_Mat.at<float>(i, 0) = labels_for_hallways[i];
		for (int f = 0; f < 23; f++)
		{
			hallway_features_Mat.at<float>(i, f) = (float) hallway_features[i][f];
		}
	}
	// Train a SVM classifier
	CvSVM hallway_SVM;
	hallway_SVM.train(hallway_features_Mat, hallway_labels_Mat, cv::Mat(), cv::Mat(), params);
	//save the trained classifier
	//hallway_SVM.save("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_SVM.xml");
	hallway_SVM.save("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_SVM.xml");
	ROS_INFO("Done hallway classifiers.");
	//*********room***************
	//save the found labels and features in Matrices
	cv::Mat room_labels_Mat(labels_for_rooms.size(), 1, CV_32FC1);
	cv::Mat room_features_Mat(room_features.size(), 23, CV_32FC1);
	for (int i = 0; i < labels_for_rooms.size(); i++)
	{
		room_labels_Mat.at<float>(i, 0) = labels_for_rooms[i];
		for (int f = 0; f < 23; f++)
		{
			room_features_Mat.at<float>(i, f) = (float) room_features[i][f];
		}
	}
	// Train a SVM classifier
	CvSVM room_SVM;
	room_SVM.train(hallway_features_Mat, hallway_labels_Mat, cv::Mat(), cv::Mat(), params);
	//save the trained classifier
	//room_SVM.save("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_SVM.xml");
	room_SVM.save("src/autopnp/ipa_room_segmentation/training_results/trained_room_SVM.xml");
	ROS_INFO("Done room classifiers.");

	return 0;
}

int classify_Points_boost(cv::Mat map_to_be_labeled)
{
	CvBoost strong_room_classifier, strong_hallway_classifier;
	cv::Mat original_Map_to_be_labeled = map_to_be_labeled.clone();
	std::vector < cv::Scalar > already_used_colours;

//	strong_room_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_boost.xml");
//	strong_hallway_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_boost.xml");
	strong_room_classifier.load("src/autopnp/ipa_room_segmentation/training_results/trained_room_boost.xml");
	strong_hallway_classifier.load("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_boost.xml");
	ROS_INFO("Loaded training results.");
	std::vector<double> angles_for_simulation;
	//get the angles-vector for calculating the features
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation.push_back(angle);
	}
	//go trough each Point and label it
	for (int x = 0; x < original_Map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_Map_to_be_labeled.cols; y++)
		{
			if (original_Map_to_be_labeled.at<unsigned char>(x, y) == 255)
			{
				std::vector<double> temporary_beams = raycasting(original_Map_to_be_labeled, cv::Point(x, y));
				std::vector<float> temporary_features;
				cv::Mat featuresMat(1, 23, CV_32FC1);
				for (int f = 1; f <= 23; f++)
				{
					featuresMat.at<float>(0, f - 1) = (float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f);
				}
				//classify each Point
				float room_sum = strong_room_classifier.predict(featuresMat, cv::Mat(), cv::Range::all(), false, true);
				float hallway_sum = strong_hallway_classifier.predict(featuresMat, cv::Mat(), cv::Range::all(), false, true);
				//get the certanity-values for each class (it shows the probability that it belongs to the given class)
				double room_certanity = (std::exp((double) room_sum)) / (std::exp(-1 * (double) room_sum) + std::exp((double) room_sum));
				double hallway_certanity = (std::exp((double) hallway_certanity))
				        / (std::exp(-1 * (double) hallway_certanity) + std::exp((double) hallway_certanity));
				//make a decision-list and check which class the Point belongs to
				double probability_for_room = room_certanity;
				double probability_for_hallway = hallway_certanity * (1.0 - probability_for_room);
				if (probability_for_room > probability_for_hallway)
				{
					original_Map_to_be_labeled.at<unsigned char>(x, y) = 150; //label it as room
				}
				else
				{
					original_Map_to_be_labeled.at<unsigned char>(x, y) = 100; //label it as hallway
				}
			}
		}
	}
	//apply a median filter over the image to smooth the results
	cv::Mat temporary_map = original_Map_to_be_labeled.clone();
	cv::medianBlur(temporary_map, temporary_map, 3);
	for (int x = 0; x < original_Map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < original_Map_to_be_labeled.cols; y++)
		{
			if (original_Map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				temporary_map.at<unsigned char>(x, y) = 0;
			}
		}
	}
	cv::Mat labeling_Map = map_to_be_labeled.clone();
	cv::Mat blured_image_for_thresholding = temporary_map.clone();
	// todo: no absolute paths
	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/opencvboost.png", temporary_map);
	//fill the large enough rooms with a random color and split the hallways into smaller regions
	std::vector<std::vector<cv::Point> > contours, temporary_contours, saved_room_contours, saved_hallway_contours;
	std::vector < cv::Vec4i > hierarchy;
	double map_resolution = 0.0500;

	//find the contours, which are labeled as a room
	cv::threshold(temporary_map, temporary_map, 120, 255, cv::THRESH_BINARY); //find rooms (value = 150)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(blured_image_for_thresholding, contours, -1, cv::Scalar(0), CV_FILLED); //make the found regions at the original map black, because they have been looked at already

	//only take rooms that are large enough and that are not a hole-contour
	for (int c = 0; c < contours.size(); c++)
	{
		if (map_resolution * map_resolution * cv::contourArea(contours[c]) > 1.0 && hierarchy[c][3] != 1)
		{
			saved_room_contours.push_back(contours[c]);
		}
	}

	//find the contours, which are labeled as a hallway
	temporary_map = blured_image_for_thresholding.clone();
	cv::threshold(temporary_map, temporary_map, 90, 255, cv::THRESH_BINARY); //find hallways (value = 100)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	//if the hallway-contours are too big split them into smaller regions, also don't take too small regions
	for (int contour_counter = 0; contour_counter < contours.size(); contour_counter++)
	{
		if (map_resolution * map_resolution * cv::contourArea(contours[contour_counter]) > 40.0)
		{
			//Generate a black map to draw the hallway-contour in. Then use this map to ckeck if the generated random Points
			// are inside the contour.
			cv::Mat contour_Map = cv::Mat::zeros(temporary_map.cols, temporary_map.rows, CV_8UC1);
			cv::drawContours(contour_Map, contours, contour_counter, cv::Scalar(255), CV_FILLED);
			//center-counter so enough centers could be found
			int center_counter = 0;
			//saving-vector for watershed centers
			std::vector < cv::Point > temporary_watershed_centers;
			//find random watershed centers
			do
			{
				int random_x = rand() % temporary_map.rows;
				int random_y = rand() % temporary_map.cols;
				if (contour_Map.at<unsigned char>(random_y, random_x) == 255)
				{
					temporary_watershed_centers.push_back(cv::Point(random_x, random_y));
					center_counter++;
				}
			} while (center_counter <= (map_resolution * map_resolution * cv::contourArea(contours[contour_counter])) / 8);
			//draw the centers as white circles into a black map and give the center-map and the contour-map to the opencv watershed-algorithm
			for (int current_center = 0; current_center < temporary_watershed_centers.size(); current_center++)
			{
				bool coloured = false;
				do
				{
					cv::Scalar fill_colour(rand() % 200 + 53);
					if (!contains(already_used_colours, fill_colour))
					{
						cv::circle(contour_Map, temporary_watershed_centers[current_center], 2, fill_colour, CV_FILLED);
						already_used_colours.push_back(fill_colour);
						coloured = true;
					}
				} while (!coloured);
			}
			//make sure all previously black Pixels are still black
			for (int x = 0; x < map_to_be_labeled.rows; x++)
			{
				for (int y = 0; y < map_to_be_labeled.cols; y++)
				{
					if (map_to_be_labeled.at<unsigned char>(x, y) == 0)
					{
						contour_Map.at<unsigned char>(x, y) = 0;
					}
				}
			}
			cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/contour.png", contour_Map);
			temporary_map = watershed_region_spread(contour_Map);
			cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/spreading.png", temporary_map);
			//draw the seperated contour into the map, which should be labeled
			for (int x = 0; x < labeling_Map.rows; x++)
			{
				for (int y = 0; y < labeling_Map.cols; y++)
				{
					if (temporary_map.at<unsigned char>(x, y) != 0)
					{
						labeling_Map.at<unsigned char>(x, y) = temporary_map.at<unsigned char>(x, y);
					}
				}
			}
		}
		else if (map_resolution * map_resolution * cv::contourArea(contours[contour_counter]) > 1.0)
		{
			saved_hallway_contours.push_back(contours[contour_counter]);
		}
	}
	//draw every last room and hallway contour witch a random colour into the map, which should be labeled
	for (int room = 0; room < saved_room_contours.size(); room++)
	{
		bool coloured = false;
		do
		{
			cv::Scalar fill_colour(rand() % 200 + 53);
			if (!contains(already_used_colours, fill_colour))
			{
				cv::drawContours(labeling_Map, saved_room_contours, room, fill_colour, CV_FILLED);
				already_used_colours.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	for (int hallway = 0; hallway < saved_hallway_contours.size(); hallway++)
	{
		bool coloured = false;
		do
		{
			cv::Scalar fill_colour(rand() % 200 + 53);
			if (!contains(already_used_colours, fill_colour))
			{
				cv::drawContours(labeling_Map, saved_hallway_contours, hallway, fill_colour, CV_FILLED);
				already_used_colours.push_back(fill_colour);
				coloured = true;
			}
		} while (!coloured);
	}
	//spread the coloured regions to regions, which were too small and aren't drawn into the map
	labeling_Map = watershed_region_spread(labeling_Map);
	//make sure previously black Pixels are still black
	for (int x = 0; x < map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < map_to_be_labeled.cols; y++)
		{
			if (map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				labeling_Map.at<unsigned char>(x, y) = 0;
			}
		}
	}
	cv::Mat tmp = labeling_Map.clone();
	for(int t = 0; t < 255; t++)
	{
		cv::threshold(labeling_Map, tmp, t, 255, cv::THRESH_BINARY);
	}
	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/boost_labeled.png", labeling_Map);
	return 0;
}

int classify_Points_SVM(cv::Mat map_to_be_labeled)
{
	CvSVM strong_room_classifier, strong_hallway_classifier;

//	strong_room_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_SVM.xml");
//	strong_hallway_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_SVM.xml");
	strong_room_classifier.load("src/autopnp/ipa_room_segmentation/training_results/trained_room_SVM.xml");
	strong_hallway_classifier.load("src/autopnp/ipa_room_segmentation/training_results/trained_hallway_SVM.xml");
	ROS_INFO("Loaded training results.");
	std::vector<double> angles_for_simulation;
	//get the angles-vector for calculating the features
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation.push_back(angle);
	}
	//go trough each Point and label it
	for (int x = 0; x < map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < map_to_be_labeled.cols; y++)
		{
			if (map_to_be_labeled.at<unsigned char>(x, y) == 255)
			{
				std::vector<double> temporary_beams = raycasting(map_to_be_labeled, cv::Point(x, y));
				std::vector<float> temporary_features;
				cv::Mat featuresMat(1, 23, CV_32FC1);
				for (int f = 1; f <= 23; f++)
				{
					featuresMat.at<float>(0, f - 1) = (float) get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), f);
				}
				//classify each Point
				float room_sum = strong_room_classifier.predict(featuresMat, true);
				float hallway_sum = strong_hallway_classifier.predict(featuresMat, true);
				//get the certanity-values for each class (it shows the probability that it belongs to the given class)
				double room_certanity = (std::exp((double) room_sum)) / (std::exp(-1 * (double) room_sum) + std::exp((double) room_sum));
				double hallway_certanity = (std::exp((double) hallway_certanity))
				        / (std::exp(-1 * (double) hallway_certanity) + std::exp((double) hallway_certanity));
				//make a decision-list and check which class the Point belongs to
				double probability_for_room = room_certanity;
				double probability_for_hallway = hallway_certanity * (1.0 - probability_for_room);
				if (probability_for_room > probability_for_hallway)
				{
					map_to_be_labeled.at<unsigned char>(x, y) = 150; //label it as room
				}
				else
				{
					map_to_be_labeled.at<unsigned char>(x, y) = 100; //label it as hallway
				}
			}
		}
	}
	//apply a median filter over the image to smooth the results
	cv::Mat temporary_map = map_to_be_labeled.clone();
	cv::medianBlur(temporary_map, temporary_map, 3);
	for (int x = 0; x < map_to_be_labeled.rows; x++)
	{
		for (int y = 0; y < map_to_be_labeled.cols; y++)
		{
			if (map_to_be_labeled.at<unsigned char>(x, y) == 0)
			{
				temporary_map.at<unsigned char>(x, y) = 0;
			}
		}
	}
	// todo: no absolute paths
	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/opencvSVM.png", temporary_map);
	return 0;
}

int main(int argc, char **argv)
{
//	srand(time(NULL));
	ros::init(argc, argv, "semantic_segmentation_server");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO("Semantic labeling of places using the generalized AdaBoost-Algorithm from OpenCV. Detects rooms and hallways.");
//	ros::spin();
	// todo: no absolute paths
	cv::Mat first_room_training_map = cv::imread("/home/rmb-fj/Pictures/maps/room_training_map.png", 0);
	cv::Mat second_room_training_map = cv::imread("/home/rmb-fj/Pictures/maps/lab_d_room_training_map.png", 0);
	cv::Mat first_hallway_training_map = cv::imread("/home/rmb-fj/Pictures/maps/hallway_training_map.png", 0);
	cv::Mat second_hallway_training_map = cv::imread("/home/rmb-fj/Pictures/maps/lab_a_hallway_training_map.png", 0);
//	cv::Mat map_to_be_labeled = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	cv::Mat map_to_be_labeled = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	for (int y_coordinate = 0; y_coordinate < map_to_be_labeled.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < map_to_be_labeled.rows; x_coordinate++)
		{
			//find not reachable regions and make them black
			if (map_to_be_labeled.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				map_to_be_labeled.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}

	std::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	int done, classified;

	ROS_INFO("Starting training the algorithm.");
	done = Boost_training(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);
	if (done == 0)
	{
		ROS_INFO("Finished training the algorithm.");
	}
	else
	{
		std::cout << "something was wrong, check training-algortihm" << std::endl;
	}
	cv::waitKey(1000);
	ROS_INFO("Starting labeling the map.");
	classified = classify_Points_boost(map_to_be_labeled);
	if (classified == 0)
	{
		ROS_INFO("Finished labeling the map.");
	}
	else
	{
		std::cout << "something was wrong, check labeling-algortihm" << std::endl;
	}
//	ROS_INFO("Starting training the SVM algorithm.");
//	done = SVM_training(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);
//	if (done == 0)
//	{
//		ROS_INFO("Finished training the algorithm.");
//	}
//	else
//	{
//		std::cout << "something was wrong, check training-algortihm" << std::endl;
//	}
//	cv::waitKey(500);
//	ROS_INFO("Starting labeling the map.");
//	classified = classify_Points_SVM(map_to_be_labeled);
//	if (classified == 0)
//	{
//		ROS_INFO("Finished labeling the map.");
//	}
//	else
//	{
//		std::cout << "something was wrong, check labeling-algortihm" << std::endl;
//	}

	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);

	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;
	//************raycasting test**********************
//	std::vector<double> angles;
//	for(double a = 0; a < 360; a++)
//	{
//		angles.push_back(a);
//	}
//	for(int x = 150; x < map_to_be_labeled.rows; x++)
//	{
//		for(int y = 100; y < map_to_be_labeled.cols; y++)
//		{
//			if(map_to_be_labeled.at<unsigned char>(x,y) == 255)
//			{
//				std::vector<double> temporary_beams = bresenham_raycasting(map_to_be_labeled, cv::Point(x,y));
//				cv::Mat temporary_map = map_to_be_labeled.clone();
//				for(int i = 0; i < temporary_beams.size(); i++)
//				{
//					double deltax = std::cos(angles[i] * PI / 180) * temporary_beams[i];
//					double deltay = std::sin(angles[i] * PI / 180) * temporary_beams[i];
//					cv::line(temporary_map, cv::Point(y,x), cv::Point(y+deltay, x+deltax), cv::Scalar(120));
//					cv::circle(temporary_map, cv::Point(y,x), 2, cv::Scalar(90));
//				}
//				cv::imshow("test", temporary_map);
//				cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/raycasting_bresenham.png", temporary_map);
//				cv::waitKey(100000);
//			}
//		}
//	}

	return 0;
}

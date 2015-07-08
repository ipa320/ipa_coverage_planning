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

#include <ipa_room_segmentation/features.h>

#define PI 3.14159265

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
	// Set up boosting parameters
	CvBoostParams params(CvBoost::DISCRETE, 120, 0, 1, false, 0);
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
	hallway_boost.save("ipa_room_segmentation/training_results/trained_hallway_boost.xml", "boost");
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
	room_boost.save("ipa_room_segmentation/training_results/trained_room_boost.xml", "boost");
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
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-7);
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
	hallway_SVM.save("ipa_room_segmentation/training_results/trained_hallway_SVM.xml");
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
	room_SVM.save("ipa_room_segmentation/training_results/trained_room_SVM.xml");
	ROS_INFO("Done room classifiers.");

	return 0;
}

int classify_Points_boost(cv::Mat map_to_be_labeled)
{
	CvBoost strong_room_classifier, strong_hallway_classifier;

//	strong_room_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_boost.xml");
//	strong_hallway_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_boost.xml");
	strong_room_classifier.load("ipa_room_segmentation/training_results/trained_room_boost.xml");
	strong_hallway_classifier.load("ipa_room_segmentation/training_results/trained_hallway_boost.xml");
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
	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/opencvboost.png", temporary_map);
	//fill the large enough rooms with a random color and split the hallways into smaller regions
	std::vector<std::vector<cv::Point> > contours, saved_contours;
	std::vector < cv::Vec4i > hierarchy;
	double map_resolution = 0.0500;
	temporary_map = map_to_be_labeled.clone();

	cv::threshold(temporary_map, temporary_map, 120, 254, cv::THRESH_BINARY); //find rooms (value = 150)
	cv::findContours(temporary_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::drawContours(map_to_be_labeled, contours, -1, cv::Scalar(0), CV_FILLED); //make the found regions at the original map black, because they have been looked at already

	//only take rooms that are large enough
	for (int c = 0; c < contours.size(); c++)
	{
		if (map_resolution * map_resolution * cv::contourArea(contours[c]) > 1.0)
		{
			saved_contours.push_back(contours[c]);
		}
	}

	cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/opencv/boost_labeled.png", temporary_map);
	return 0;
}

int classify_Points_SVM(cv::Mat map_to_be_labeled)
{
	CvSVM strong_room_classifier, strong_hallway_classifier;

//	strong_room_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_room_SVM.xml");
//	strong_hallway_classifier.load("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/trained_hallway_SVM.xml");
	strong_room_classifier.load("ipa_room_segmentation/training_results/trained_room_SVM.xml");
	strong_hallway_classifier.load("ipa_room_segmentation/training_results/trained_hallway_SVM.xml");
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
	ros::init(argc, argv, "opencv_semantic");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO("Semantic labeling of places using the generalized AdaBoost-Algorithm from opencv. Detects rooms and hallways.");
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
//	ROS_INFO("Starting training the algorithm.");
//	done = Boost_training(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);
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
//	classified = classify_Points_boost(map_to_be_labeled);
//	if (classified == 0)
//	{
//		ROS_INFO("Finished labeling the map.");
//	}
//	else
//	{
//		std::cout << "something was wrong, check labeling-algortihm" << std::endl;
//	}
	ROS_INFO("Starting training the SVM algorithm.");
	done = SVM_training(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);
	if (done == 0)
	{
		ROS_INFO("Finished training the algorithm.");
	}
	else
	{
		std::cout << "something was wrong, check training-algortihm" << std::endl;
	}
	cv::waitKey(500);
	ROS_INFO("Starting labeling the map.");
	classified = classify_Points_SVM(map_to_be_labeled);
	if (classified == 0)
	{
		ROS_INFO("Finished labeling the map.");
	}
	else
	{
		std::cout << "something was wrong, check labeling-algortihm" << std::endl;
	}

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

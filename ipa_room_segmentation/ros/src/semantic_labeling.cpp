#include "ros/ros.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

#include <time.h>

#include <ipa_room_segmentation/features.h>
#include <ipa_room_segmentation/weak_classifier.h>

#define PI 3.14159265

//***********************How To********************************
//1. If you don't already trained the algortihm you first need to label a gridmap at your own, telling him where a
//   room/hallway/door/... is by making it a different color
//2. Change the paths of your saving files for training results
//3. Make sure your training-map is loaded into the program.
//4. Run the segmentation_algorithm with bool train = true
//5. If you have training data: make sure they are loaded correctly into the program by adding the files to the second
//	 part of the segmentation-algorithm
// IMPORTANT: change the paths for every loading/saving/...
//6. Run the segmentation_algorithm with bool train = false
//7. Pray that the results are good enough

std::vector<double> raycasting(cv::Mat map, cv::Point location)
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
		x_destination = x_start + std::cos(angle * PI / 180) * 1000;
		y_destination = y_start + std::sin(angle * PI / 180) * 1000;
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

int train_algorithm(std::vector<int> labels, std::vector<cv::Point> points, std::vector<std::vector<double> > beams, std::vector<std::vector<double> > angles,
        int T, std::vector<WeakClassifier> *classifiers, std::vector<double> *at)
{
	double pos_ex = 0, neg_ex = 0;
	std::vector<double> weights, rj, rt;
	std::vector<WeakClassifier> temporary_classifiers;
	double sum, r;
	WeakClassifier current_hj;
	//*****************Training of the generalized AdaBoost-Algorithm*******************
	//1.Initialization
	//count positive and negative examples for the weights
	for (int y = 0; y < labels.size(); y++)
	{
		if (labels[y] > 0)
		{
			pos_ex++;
		}
		else
		{
			neg_ex++;
		}
	}
	//initialize weights
	for (int y = 0; y < labels.size(); y++)
	{
		if (labels[y] > 0)
		{
			weights.push_back(1 / (2 * pos_ex));
		}
		else
		{
			weights.push_back(1 / (2 * neg_ex));
		}
	}
	//2. T training steps
	for (int t = 1; t <= T; t++)
	{
		sum = 0;
		//normalize the weights
		for (int w = 0; w < weights.size(); w++)
		{
			sum += weights[w];
		}
		for (int w = 0; w < weights.size(); w++)
		{
			weights[w] = weights[w] / sum;
		}
		//for each feature train a classifier that classifies the examples with minimal error
		//Remark: currently only 23 Features declared. If more or less later, change the max. value of f
		for (int f = 1; f <= 23; f++)
		{
			WeakClassifier current_classifier(f, weights);
			current_classifier.train(labels, points, beams, angles);
			temporary_classifiers.push_back(current_classifier);
			std::cout << "trained one classifier: " << f << std::endl;
		}
		//for each classifier hj calculate rj
		for (int h = 0; h < temporary_classifiers.size(); h++)
		{
			r = 0;
			for (int p = 0; p < points.size(); p++)
			{
				double ui = (double) labels[p] * temporary_classifiers[h].classify(beams[p], angles[p], points[p]);
				r += weights[p] * ui;
			}
			rj.push_back(r);
		}
		//save the classifier hj and the corresponding rj that maximises |rj|
		r = 0;
		for (int rt = 0; rt < rj.size(); rt++)
		{
			if (std::abs(rj[rt]) > std::abs(r))
			{
				r = rj[rt];
				current_hj = temporary_classifiers[rt];
			}
		}
		temporary_classifiers.clear();
		rj.clear();
		classifiers->push_back(current_hj);
		//calculate weighting-factor and update weights
		at->push_back(0.5 * std::log10((1 + r) / (1 - r)));
		for (int w = 0; w < weights.size(); w++)
		{
			weights[w] = weights[w] * std::exp(-0.5 * std::log10((1 + r) / (1 - r)) * labels[w] * current_hj.classify(beams[w], angles[w], points[w]));
		}
		std::cout << "one step done. T currently: " << t << std::endl;
	}
	return 0;
}

std::vector<double> strong_classifier(std::vector<WeakClassifier> classifiers, std::vector<double> at, std::vector<double> calculated_features)
{
	//classify the Position the robot currently is. It returns a vector that shows if it belongs to the class or not.
	//The vector consists of two elements: 1. -1 if location doesn't belong to class or 1 if it does
	//									   2. The confidence-value that the location belongs to the class
	double F = 0;
	double C;
	std::vector<double> classifying_parameters;
	//get the Strong classifier F that shows if Point belongs to class
	for (int c = 0; c < classifiers.size(); c++)
	{
		F += at[c] * classifiers[c].classify(calculated_features[classifiers[c].feature - 1]);
	}
	if (F < 0)
	{
		classifying_parameters.push_back(-1.0);
	}
	else
	{
		classifying_parameters.push_back(1.0);
	}
	//Calculate the confidence-value
	C = (std::exp(F)) / (std::exp(-F) + std::exp(F));
	classifying_parameters.push_back(C);
	return classifying_parameters;
}

int segmentation_algorithm(bool train, cv::Mat originalmap, cv::Mat secondmap)
{
	std::vector<double> angles_for_simulation;
	for (double angle = 0; angle < 360; angle++)
	{
		angles_for_simulation.push_back(angle);
	}
	//Segmenting algorithm
	if (train) //only training should be done --> map is a training map
	{
		std::vector<WeakClassifier> door_classifiers;
		std::vector<double> door_weighting_factors;
		std::vector < std::vector<double> > simulated_beams;
		std::vector < cv::Point > points;
		std::vector<int> labels_for_points;
		std::vector < std::vector<double> > simulated_angles;
		int counter = 0;
		//get the angle-vector, which is similiar for every simulation

		//get the points, labels and simulated beams for non-black Pixels in the map
		//first map:
		for (int y = 81; y < 278; y++) //(int y = 185; y < 206; y++)
		{
			for (int x = 92; x < 361; x++) //(int x = 236; x < 261; x++)
			{
				if (originalmap.at<unsigned char>(x, y) != 0)
				{
					counter++;
					//put Point into the points vector
					points.push_back(cv::Point(x, y));
					//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
					if (originalmap.at<unsigned char>(x, y) == 255)
					{
						labels_for_points.push_back(-1);
					}
					else
					{
						labels_for_points.push_back(1);
					}
					//simulate the beams for every position and save it
					simulated_beams.push_back(raycasting(originalmap, cv::Point(x, y)));
					simulated_angles.push_back(angles_for_simulation);
				}
			}
		} //second map:
		for (int y = 27; y < 283; y++)
		{
			for (int x = 29; x < 603; x++)
			{
				if (secondmap.at<unsigned char>(x, y) != 0)
				{
					counter++;
					//put Point into the points vector
					points.push_back(cv::Point(x, y));
					//check for label of each Pixel (if it belongs to doors the label is 1, otherwise it is -1)
					if (secondmap.at<unsigned char>(x, y) == 255)
					{
						labels_for_points.push_back(-1);
					}
					else
					{
						labels_for_points.push_back(1);
					}
					//simulate the beams for every position and save it
					simulated_beams.push_back(raycasting(secondmap, cv::Point(x, y)));
					simulated_angles.push_back(angles_for_simulation);
				}
			}
		}
		std::cout << "found and simulated all Points. Points to look at: " << counter << std::endl;
		//train the algorithm for the given map and save the resulting weak classifier parameters and weighting factors
		//in a .txt file
		train_algorithm(labels_for_points, points, simulated_beams, simulated_angles, 70, &door_classifiers, &door_weighting_factors);
		std::cout << door_classifiers.size() << " " << door_weighting_factors.size() << std::endl;
		std::vector<double> testclassify;
		//std::ofstream saving_file("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/saver.txt");
		std::ofstream saving_file("ipa_room_segmentation/training_results/saver.txt");
		if (saving_file.is_open())
		{
			std::cout << "starting saving classifier parameters" << std::endl;
			for (unsigned int i = 0; i < door_classifiers.size(); i++)
			{
				saving_file << door_classifiers[i].theta << " " << door_classifiers[i].p << " " << door_classifiers[i].feature << " ";
//				saving_file << "test1 " << "test2 " << "test3 ";
				saving_file << std::endl;
			}
			std::cout << "finished saving" << std::endl;
			saving_file.close();
		}
		else
		{
			std::cout << "nicht geöffnet1" << std::endl;
		}
		//std::ofstream at_saver("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/weightings.txt");
		std::ofstream at_saver("ipa_room_segmentation/training_results/weightings.txt");
		if (at_saver.is_open())
		{
			std::cout << "starting saving weighting factors" << std::endl;
			for (unsigned int i = 0; i < door_classifiers.size(); i++)
			{
				at_saver << door_weighting_factors[i] << " ";
//				at_saver << "test1 " << "test2 " << "test3";
				at_saver << std::endl;
			}
			std::cout << "finished saving" << std::endl;
			at_saver.close();
		}
		else
		{
			std::cout << "nicht geöffnet2" << std::endl;
		}
	}
	else //only labeling should be done --> map is a map to be segmented
	{
		std::string line;
		double value, i;
		std::vector<double> temporary_classifier_parameters;
		std::vector < std::vector<double> > classifiers_out_of_file;
		std::vector<WeakClassifier> loaded_room_classifiers, loaded_hallway_classifiers;
		std::vector<double> room_weighting_factors_from_file, hallway_weighting_factors_from_file;
		//1. Load the saved classifiers and weighting-factors, calculated by the training algorithm
		//room-classifiers
		//std::ifstream classifiers_reading_file("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/saver_room.txt");
		std::ifstream classifiers_reading_file("ipa_room_segmentation/training_results/saver_room.txt");
		if (classifiers_reading_file.is_open())
		{
			while (getline(classifiers_reading_file, line))
			{
				std::istringstream iss(line);
				i = 0;
				while (iss >> value)
				{
					i++;
					temporary_classifier_parameters.push_back(value);
					if (i == 3)
					{
						classifiers_out_of_file.push_back(temporary_classifier_parameters);
						temporary_classifier_parameters.clear();
						i = 0;
					}
				}
			}
			classifiers_reading_file.close();
		}
		else
		{
			std::cout << "nicht geöffnet" << std::endl;
		}
		//create classifiers from the saved parameters
		std::cout << "number of opened room-classifier: " << classifiers_out_of_file.size() << std::endl;
		if (classifiers_out_of_file.size() == 0)
		{
			//something is wrong, don't go further
			return -1;
		}
		for (int i = 0; i < classifiers_out_of_file.size(); i++)
		{
			WeakClassifier temporary_classifier(classifiers_out_of_file[i][0], classifiers_out_of_file[i][1], classifiers_out_of_file[i][2]);
			loaded_room_classifiers.push_back(temporary_classifier);
		}
		temporary_classifier_parameters.clear();
		classifiers_out_of_file.clear();
		//hallway-classifiers
		//std::ifstream hallway_classifiers_reading_file("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/saver_hallway.txt");
		std::ifstream hallway_classifiers_reading_file("ipa_room_segmentation/training_results/saver_hallway.txt");
		if (hallway_classifiers_reading_file.is_open())
		{
			while (getline(hallway_classifiers_reading_file, line))
			{
				std::istringstream iss(line);
				i = 0;
				while (iss >> value)
				{
					i++;
					temporary_classifier_parameters.push_back(value);
					if (i == 3)
					{
						classifiers_out_of_file.push_back(temporary_classifier_parameters);
						temporary_classifier_parameters.clear();
						i = 0;
					}
				}
			}
			hallway_classifiers_reading_file.close();
		}
		else
		{
			std::cout << "nicht geöffnet" << std::endl;
		}
		//create classifiers from the saved parameters
		std::cout << "number of opened hallway-classifier: " << classifiers_out_of_file.size() << std::endl;
		if (classifiers_out_of_file.size() == 0)
		{
			//something is wrong, don't go further
			return -1;
		}
		for (int i = 0; i < classifiers_out_of_file.size(); i++)
		{
			WeakClassifier temporary_classifier(classifiers_out_of_file[i][0], classifiers_out_of_file[i][1], classifiers_out_of_file[i][2]);
			loaded_hallway_classifiers.push_back(temporary_classifier);
		}
		temporary_classifier_parameters.clear();
		classifiers_out_of_file.clear();
		//*********weighting-factors**************
		//room-weighting factors
		//std::ifstream weightings_reading_file("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/weightings_room.txt");
		std::ifstream weightings_reading_file("ipa_room_segmentation/training_results/weightings_room.txt");
		if (weightings_reading_file.is_open())
		{
			while (getline(weightings_reading_file, line))
			{
				std::istringstream iss(line);
				i = 0;
				while (iss >> value)
				{
					room_weighting_factors_from_file.push_back(value);
				}
			}
			weightings_reading_file.close();
		}
		else
		{
			std::cout << "nicht geöffnet" << std::endl;
		}
		std::cout << "number of opened room-weighting-factors: " << room_weighting_factors_from_file.size() << std::endl;
		if (room_weighting_factors_from_file.size() == 0)
		{
			//something is wrong, don't go further
			return -1;
		}
		//hallway-weighting factors
		//std::ifstream hallway_weightings_reading_file("/home/rmb-fj/roomsegmentation/src/segmentation/src/training_results/weightings_hallway.txt");
		std::ifstream hallway_weightings_reading_file("ipa_room_segmentation/training_results/weightings_hallway.txt");
		if (hallway_weightings_reading_file.is_open())
		{
			while (getline(hallway_weightings_reading_file, line))
			{
				std::istringstream iss(line);
				i = 0;
				while (iss >> value)
				{
					hallway_weighting_factors_from_file.push_back(value);
				}
			}
			hallway_weightings_reading_file.close();
		}
		else
		{
			std::cout << "nicht geöffnet" << std::endl;
		}
		std::cout << "number of opened hallway-weighting-factors: " << hallway_weighting_factors_from_file.size() << std::endl;
		if (hallway_weighting_factors_from_file.size() == 0)
		{
			//something is wrong, don't go further
			return -1;
		}
		//2. Classify each Pixel by calculating the confidence-value for room/hallway and checking if it is more sure a
		//	 room or a hallway
		for (int y = 0; y < originalmap.cols; y++)
		{
			for (int x = 0; x < originalmap.rows; x++)
			{
				if (originalmap.at<unsigned char>(x, y) == 255)
				{
					//raycast at given Position and calculate the Feature-values for every position
					std::vector<double> temporary_beams = raycasting(originalmap, cv::Point(x, y));
					std::vector<double> temporary_features;
					WeakClassifier temporary_classifier;
					for (int feature = 1; feature <= 23; feature++)
					{
						temporary_features.push_back(get_feature(temporary_beams, angles_for_simulation, cv::Point(x, y), feature));
					}
					std::vector<double> room_classify = strong_classifier(loaded_room_classifiers, room_weighting_factors_from_file, temporary_features);
					std::vector<double> hallway_classify = strong_classifier(loaded_hallway_classifiers, hallway_weighting_factors_from_file,
					        temporary_features);
					//calculate the probability for each room if it belongs to a room or a hallway as a decision-list
					double probability_for_room = room_classify[1];
					double probability_for_hallway = hallway_classify[1] * (1.0 - probability_for_room);
					//check for label of each Point
					if (probability_for_room >= probability_for_hallway) //Point is a room
					{
						originalmap.at<unsigned char>(x, y) = 150; //label it as a room
					}
					else //if it is no room, it is a hallway
					{
						originalmap.at<unsigned char>(x, y) = 100; //label it as hallway
					}
				}
			}
		}
		//apply a median filter on the image, so the regions are more compact together
		cv::Mat temporary_map = originalmap.clone();
		cv::medianBlur(temporary_map, temporary_map, 3);
		for (int x = 0; x < originalmap.rows; x++)
		{
			for (int y = 0; y < originalmap.cols; y++)
			{
				if (originalmap.at<unsigned char>(x, y) == 0)
				{
					temporary_map.at<unsigned char>(x, y) = 0;
				}
			}
		}
		cv::imwrite("/home/rmb-fj/Pictures/maps/semantic/testlabeling.png", temporary_map);
	}
	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "semantic");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO("Semantic labeling of places using the generalized AdaBoost-Algorithm. Detects rooms and hallways.");
//	ros::spin();
	// todo: no absolute paths
	cv::Mat first_training_map = cv::imread("/home/rmb-fj/Pictures/maps/hallway_training_map.png", 0);
	cv::Mat second_training_map = cv::imread("/home/rmb-fj/Pictures/maps/lab_a_hallway_training_map.png", 0);
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
//	std::cout << "starting training" << std::endl;
//	segmentation_algorithm(true, first_training_map, second_training_map);
//	std::cout << "finished training" << std::endl;
	int done = segmentation_algorithm(false, map_to_be_labeled, second_training_map);
	if (done == 0)
	{
		std::cout << "finished segmenting the map" << std::endl;
	}
	else
	{
		std::cout << "something was wrong, check labeling-algortihm" << std::endl;
	}
	return 0;
}

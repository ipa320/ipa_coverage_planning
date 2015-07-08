#include "ros/ros.h"
#include "ipa_room_segmentation/seg.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include <ctime>

cv::Mat distance_segmentation_algorithm(cv::Mat map)
{

	std::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	//variables that come from ROS
	double map_resolution_factor_from_subscription_ = 0.050000;
	double room_area_factor_lower_limit_ = 1.0; // eigentlich =3.0, aber dann wird oberster RechterRaum nicht als Raum erkannt, da area==2.0
	double room_area_factor_upper_limit_ = 45.0;
	//variables for distance transformation
	cv::Mat temporary_map = map.clone();
	cv::Mat distance_map;
	//variables for thresholding
	cv::Mat threshmap;
	std::vector < std::vector<cv::Point> > contours;
	std::vector < cv::Vec4i > hierarchy;
	std::vector < std::vector<cv::Point> > saved_contours;
	std::vector < std::vector<cv::Point> > temporary_contours;
	//variables for filling the map
	cv::Mat temporary_map_to_fill_white_Pixels;

	//
	//Segmentation of a gridmap into roomlike areas based on the distancetransformation of the map
	//

	//1. Get the distancetransformed map and make it an 8-bit single-channel image
	cv::erode(temporary_map, temporary_map, cv::Mat());
	cv::distanceTransform(temporary_map, distance_map, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map, distance_map);
	cv::imwrite("/home/rmb-fj/Pictures/maps/distance_segmentation/distancemap.jpg", distance_map);

	//2. Threshold the map and find the contours of the rooms. Change the threshold and repeat steps until last possible threshold.
	//Then take the contours from the threshold with the most contours between the roomfactors and draw it in the map with a random color.
	for (double tr = 255.0; tr > 0.0; tr--)
	{ //change the threshold for the grayscale-image from largest possible value to smallest
	  //reset number of rooms
		temporary_contours.clear();
		contours.clear();
		cv::threshold(distance_map, threshmap, tr, 255, cv::THRESH_BINARY);
		cv::findContours(threshmap, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		//get the number of large enough regions to be a room
		for (int c = 0; c < contours.size(); c++)
		{
			if (hierarchy[c][3] == -1)
			{
				double room_area = map_resolution_factor_from_subscription_ * map_resolution_factor_from_subscription_ * cv::contourArea(contours[c]);
				if (room_area >= room_area_factor_lower_limit_ && room_area <= room_area_factor_upper_limit_)
				{
					temporary_contours.push_back(contours[c]);
				}
			}
		}
		//check if current step has more rooms than the saved one
		if (temporary_contours.size() > saved_contours.size())
		{
			saved_contours.clear();
			saved_contours = temporary_contours;
		}
	}
	//Draw the contours in the map
	for (int sc = 0; sc < saved_contours.size(); sc++)
	{
		cv::Scalar fill_colour(rand() % 200 + 53);
		cv::drawContours(map, saved_contours, sc, fill_colour, CV_FILLED);
	}
	//spread the colors to the white Pixels
	temporary_map_to_fill_white_Pixels = map.clone();
	for (int loopcounter = 0; loopcounter < 300; loopcounter++)
	{
		for (int column = 0; column < temporary_map.cols; column++)
		{
			for (int row = 0; row < temporary_map.rows; row++)
			{
				if (map.at<unsigned char>(row, column) == 255)
				{
					//check 3x3 area around white pixel for fillcolor, if filled Pixel around fill white pixel with that color
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (map.at<unsigned char>(row + row_counter, column + column_counter) != 0
							        && map.at<unsigned char>(row + row_counter, column + column_counter) != 255)
							{
								temporary_map_to_fill_white_Pixels.at<unsigned char>(row, column) = map.at<unsigned char>(row + row_counter,
								        column + column_counter);
							}
						}
					}
				}
			}
		}
		map = temporary_map_to_fill_white_Pixels.clone();
	}

	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);

	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;
	return map;
}

int main(int argc, char **argv)
{
	cv::Mat original_map = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", 0);
	for (int y_coordinate = 0; y_coordinate < original_map.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < original_map.rows; x_coordinate++)
		{
			//find not reachable regions andmake them black
			if (original_map.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				original_map.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	cv::Mat segmented_map;
	ros::init(argc, argv, "distance_transform_segmentation");
	ros::NodeHandle n;
	ROS_INFO("Raumsegmentierung auf Basis einer distanztransformierten Karte");
	segmented_map = distance_segmentation_algorithm(original_map);
	ROS_INFO("Done segmenting the map");
	cv::imwrite("/home/rmb-fj/Pictures/maps/distance_segmentation/distance_segmented_map.png", segmented_map);

	return 0;
}

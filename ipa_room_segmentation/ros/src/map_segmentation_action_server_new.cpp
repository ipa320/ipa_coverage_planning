#include "ros/ros.h"
#include "ipa_room_segmentation/seg.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>

#include <ctime>
/*This segmentation algorithm does:
 * 1. collect the map data
 * 2. erode the map to extract contours
 * 3. find the extracted contures and save them
 * 4. draw and fill the saved contoures in a clone of the map from 1. with a random colour
 * 5. get the obstacle information from the original map and draw them in the clone from 4.
 * 6. find the last white pixels and make them the colour of the nearest contour 
 * 
 */

bool segmentationAlgorithm()
{
	std::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	cv::Mat original_map_from_subscription_ = cv::imread("/home/rmb-fj/Pictures/maps/black_map.png", -1);
	if (original_map_from_subscription_.empty())
	{
		ROS_INFO("Fehler Bildladen");
		return false;
	}
	double map_resolution_from_subscription_ = 0.050000;
	double room_area_factor_lower_limit_ = 2.0;
	double room_area_factor_upper_limit_ = 40.0;
	//double map_resolution = map_resolution_from_subscription;
	//make non-white pixels black
	for (int y_coordinate = 0; y_coordinate < original_map_from_subscription_.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < original_map_from_subscription_.rows; x_coordinate++)
		{
			//find not reachable regions and make them black
			if (original_map_from_subscription_.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				original_map_from_subscription_.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	cv::imwrite("/home/rmb-fj/Pictures/maps/black_map.png", original_map_from_subscription_);
	std::vector < std::vector<cv::Point> > saved_contours;
	//make two mapclones to work with
	cv::Mat temporary_map_to_find_rooms_ = original_map_from_subscription_.clone();
	cv::Mat new_map = original_map_from_subscription_.clone();
	//**************erode temporary_map until last possible room found****************
	//erode map a spedified amount of times
	ROS_INFO("starting eroding");
	for (int counter = 0; counter < 50; counter++)
	{
		cv::Mat eroded_map;
		cv::Point anchor(-1, -1);
		cv::erode(temporary_map_to_find_rooms_, eroded_map, cv::Mat(), anchor, 1);
		//save the more eroded map
		temporary_map_to_find_rooms_ = eroded_map;
		cv::Mat contour_map = eroded_map.clone();
		//find Contours in the more eroded map
		std::vector < std::vector<cv::Point> > contours;
		std::vector < cv::Vec4i > hierarchy;
		cv::findContours(contour_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		if (contours.size() != 0)
		{
			for (int idx = 0; idx < contours.size(); idx++)
			{ //only take first level contours --> second level contours belong to holes and doesn't need to be looked at
				if (hierarchy[idx][3] == -1)
				{
					//check if contour fulfills criteria of a room
					double room_area = map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[idx]);

					if (room_area_factor_lower_limit_ < room_area && room_area < room_area_factor_upper_limit_)
					{
						//save contour for later drawing in map
						saved_contours.push_back(contours[idx]);
						//make region black if room found --> region doesn't need to be looked at anymore
						cv::drawContours(temporary_map_to_find_rooms_, contours, idx, cv::Scalar(0), CV_FILLED, 8, hierarchy, 2);
					}
				}
			}
		}
	}
	//*******************draw contures in new map***********************
	std::cout << "Segmentation Found " << saved_contours.size() << " rooms." << std::endl;
	//draw filled contoures in new_map with random colour
	for (int idx = 0; idx < saved_contours.size(); idx++)
	{
		cv::Scalar fill_colour(rand() % 253 + 1);
		cv::drawContours(new_map, saved_contours, idx, fill_colour, CV_FILLED);
	}
	cv::imwrite("/home/rmb-fj/Pictures/maps/map_with_contours_drawn_in.jpg", new_map);
	//*************************obstacles***********************
	//get obstacle informations
	ROS_INFO("starting getting obstacle information");
	std::vector < cv::Point > black_pixels;
	std::vector < cv::Point > white_pixels;
	for (int x_coordinate = 0; x_coordinate < original_map_from_subscription_.cols; x_coordinate++)
	{
		for (int y_coordinate = 0; y_coordinate < original_map_from_subscription_.rows; y_coordinate++)
		{
			//find obstacles = black pixels
			if (original_map_from_subscription_.at<unsigned char>(y_coordinate, x_coordinate) == 0)
			{
				black_pixels.push_back(cv::Point(x_coordinate, y_coordinate));
			}
		}
	}
	//draw obstacles in new_map
	for (unsigned int i = 0; i < black_pixels.size(); i++)
	{
		new_map.at<unsigned char>(black_pixels[i]) = 0;
	}
	ROS_INFO("drawn obstacles in map");
	cv::imwrite("/home/rmb-fj/Pictures/maps/obstacle_map.jpg", new_map);

	//**************spread the colored region by making white pixel around a contour their color****************
	cv::Mat temporary_map_to_fill_white_pixels_ = new_map.clone();
	for (int loop_counter = 0; loop_counter < 1000; loop_counter++)
	{ //check a specified amount of times for white Pixels, so every Pixel is colored
		for (int column = 0; column < original_map_from_subscription_.cols; column++)
		{
			for (int row = 0; row < original_map_from_subscription_.rows; row++)
			{
				if (new_map.at<unsigned char>(row, column) == 255)
				{ //only look at white Pixels
				  //check 3x3 area around white pixel for fillcolor, if filled Pixel around fill white pixel with that color
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (new_map.at<unsigned char>(row + row_counter, column + column_counter) != 0
							        && new_map.at<unsigned char>(row + row_counter, column + column_counter) != 255)
							{
								temporary_map_to_fill_white_pixels_.at<unsigned char>(row, column) = new_map.at<unsigned char>(row + row_counter,
								        column + column_counter);
							}
						}
					}
				}
			}
		}
		//fill in temporary map colored white pixels in the new map with color
		new_map = temporary_map_to_fill_white_pixels_.clone();
	}
	ROS_INFO("filled white pixels in new_map");
	cv::imwrite("/home/rmb-fj/Pictures/maps/outfilled_map.png", new_map);
	//***********************Find min/max x and y coordinate and center of each found room********************
	temporary_map_to_find_rooms_ = new_map.clone();
	//min/max y/x-values vector for each room
	std::vector<int> min_y_value_of_the_room(255, 100000000);
	std::vector<int> max_y_value_of_the_room(255, 0);
	std::vector<int> min_x_value_of_the_room(255, 100000000);
	std::vector<int> max_x_value_of_the_room(255, 0);
	//vector of the central Point for each room, firstly filled with Point out of the map
	cv::Point center_out_of_bounds(-1, -1);
	std::vector < cv::Point > room_center_for_each_room(255, center_out_of_bounds);
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int column = 0; column <= original_map_from_subscription_.cols; column++)
	{
		for (int row = 0; row <= original_map_from_subscription_.rows; row++)
		{
			//if Pixel is white or black it is no room --> doesn't need to be checked
			if (temporary_map_to_find_rooms_.at<unsigned char>(row, column) != 0 && temporary_map_to_find_rooms_.at<unsigned char>(row, column) != 255)
			{
				min_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)] = std::min(row,
				        min_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column) - 1]);
				max_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)] = std::max(row,
				        max_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column) - 1]);
				min_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)] = std::min(column,
				        min_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column) - 1]);
				max_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)] = std::max(column,
				        max_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column) - 1]);

			}
		}
	}
	//get center of room for each room
	for (int idx = 0; idx < room_center_for_each_room.size(); idx++)
	{
		room_center_for_each_room[idx].x = max_x_value_of_the_room[idx] / 2;
		room_center_for_each_room[idx].y = max_y_value_of_the_room[idx] / 2;
	}
	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);

	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "segmentation_server");
	ros::NodeHandle n;
	segmentationAlgorithm();
	ROS_INFO("Bitte Eingabe machen");
	//ros::spin();

	return 0;
}

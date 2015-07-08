#include "ros/ros.h"
#include "ipa_room_segmentation/seg.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <list>
#include <vector>
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
	// todo: no absolute paths
	cv::Mat original_map_from_subscription_ = cv::imread("/home/rmb-fj/Pictures/maps/radish/radish_2.png", CV_LOAD_IMAGE_GRAYSCALE);
	if (original_map_from_subscription_.empty())
	{
		ROS_INFO("Fehler Bildladen");
		return false;
	}
	double map_resolution_from_subscription_ = 0.050000; //resolution of testimage
	int map_sampling_factor_from_subscription_ = 1.5;
	int room_area_factor_lower_limit_ = 2.0; // eigentlich =3.0, aber dann wird oberster RechterRaum nicht als Raum erkannt, da area==2.0
	int room_area_factor_upper_limit_ = 40.0;
	//double map_resolution = map_resolution_from_subscription;
	//create two mapclones to work with
	//make non-white pixels black
	for (int y_coordinate = 0; y_coordinate < original_map_from_subscription_.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < original_map_from_subscription_.rows; x_coordinate++)
		{
			//find not reachable regions andmake them black
			if (original_map_from_subscription_.at<unsigned char>(x_coordinate, y_coordinate) != 255)
			{
				original_map_from_subscription_.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	std::vector < std::vector<cv::Point> > saved_contours;
	cv::Mat temporary_map_to_find_rooms_ = original_map_from_subscription_.clone();
	cv::Mat new_map = original_map_from_subscription_.clone();
	//**************erode temporary_map until last possible room found****************
	//erode map until eroding not possible anymore
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
		//cv::drawContours(contour_map, contours, -1, cv::Scalar(128, 128, 128, 128), 2);
		ROS_INFO("one more eroding and finding contours finished");
		std::cout << "got " << contours.size() << "rooms." << std::endl;
		if (contours.size() != 0)
		{
			for (int idx = 0; idx < contours.size(); idx++)
			{//only take first level contours --> second level contours belong to holes and doesn't need to be looked at
				if(hierarchy[idx][3] == -1)
				{
					//check if contour fulfills criteria of a room
					int room_area = map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[idx]);
					std::cout << "calculated room_area: " << room_area << std::endl;

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
	} //exit loop if there are no more rooms to find
	  //*******************draw contures in new map***********************
	std::cout << "Segmentation Found " << saved_contours.size() << " rooms." << std::endl;
	//draw filled contoures in new_map with random colour
	for (int idx = 0; idx < saved_contours.size(); idx++)
	{
		cv::Scalar fill_colour(rand() % 254 + 1);
		cv::drawContours(new_map, saved_contours, idx, fill_colour, CV_FILLED);
		//cv::imshow("current map",new_map);
		//cv::waitKey(1000);
	}
	cv::imwrite("/home/rmb-fj/Pictures/maps/radish/map_with_contours_drawn_in.jpg", new_map);
	std::cout << "finding rooms done" << std::endl;
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
	std::cout << "found all obstacles" << std::endl;
	//draw obstacles in new_map_obstacles
	for (unsigned int i = 0; i < black_pixels.size(); i++)
	{
		new_map.at<unsigned char>(black_pixels[i]) = 0;
	}
	ROS_INFO("drawing obstacles in map done");
	cv::imwrite("/home/rmb-fj/Pictures/maps/radish/obstacle_map.jpg", new_map);

	//**************make white pixels the colour of the nearest contour****************
	cv::Mat temporary_map_to_fill_white_pixels_= new_map.clone();
	for(int loop_counter = 0; loop_counter < 1000; loop_counter++)
	{
		for(int column = 0; column < original_map_from_subscription_.cols; column++)
		{
			for(int row = 0; row < original_map_from_subscription_.rows; row++)
			{
				if(new_map.at<unsigned char>(row, column) == 255)
				{
					//check 3x3 area around white pixel for fillcolor, if filled Pixel around fill white pixel with that color
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if(new_map.at<unsigned char>(row+row_counter, column+column_counter) != 0
									&& new_map.at<unsigned char>(row+row_counter, column+column_counter) != 255)
							{
								temporary_map_to_fill_white_pixels_.at<unsigned char>(row, column)
										= new_map.at<unsigned char>(row+row_counter, column+column_counter);
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
	cv::imwrite("/home/rmb-fj/Pictures/maps/radish/outfilled_map.jpg", new_map);
	//***********************Find min/max x and y coordinate and center of each found room********************
	temporary_map_to_find_rooms_ = new_map.clone();
	//min/max y/x-values vector for each room
	std::vector<int> min_y_value_of_the_room(254, 100000000);
	std::vector<int> max_y_value_of_the_room(254, 0);
	std::vector<int> min_x_value_of_the_room(254, 100000000);
	std::vector<int> max_x_value_of_the_room(254, 0);
	//vector of the central Point for each room, firstly filled with Point out of the map
	cv::Point center_out_of_bounds(-1,-1);
	std::vector<cv::Point> room_center_for_each_room(255, center_out_of_bounds);
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for(int column = 0; column <= original_map_from_subscription_.cols; column++)
	{
		for(int row = 0; row <= original_map_from_subscription_.rows; row++)
		{
			//if Pixel is white or black it is no room --> doesn't need to be checked
			if(temporary_map_to_find_rooms_.at<unsigned char>(row, column) != 0 &&
					temporary_map_to_find_rooms_.at<unsigned char>(row, column) != 255)
			{
				min_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)]= std::min(row,
						min_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)-1]);
				max_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)]= std::max(row,
						max_y_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)-1]);
				min_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)]= std::min(column,
						min_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)-1]);
				max_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)]= std::max(column,
						max_x_value_of_the_room[temporary_map_to_find_rooms_.at<unsigned char>(row, column)-1]);

			}
		}
	}
	//get center of room for each room
	for(int idx = 0; idx < room_center_for_each_room.size(); idx++)
	{
		room_center_for_each_room[idx].x = max_x_value_of_the_room[idx] / 2;
		room_center_for_each_room[idx].y = max_y_value_of_the_room[idx] / 2;
	}

	//***********************Find min/max x and y coordinate and center of each found room********************
	//return new_map;
	//cv::imwrite("/home/rmb-fj/Test.jpg", new_map);
	//cv::imshow("originalMap", original_map_from_subscription_);
	//cv::imshow("FinalMap", new_map);
	//cv::waitKey(0);
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "segmentation_test");
	ros::NodeHandle n;
	//ros::ServiceServer service = n.advertiseService("roomsegmentation", segmentation_algorithm);
	ROS_INFO("Segmentierungsalgorithmus IPA test");
	segmentationAlgorithm();
	//ros::spin();

	return 0;
}


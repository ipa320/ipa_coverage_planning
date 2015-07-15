#include <ipa_room_segmentation/morphological_segmentation.h>

/*This segmentation algorithm does:
 * 1. collect the map data
 * 2. erode the map to extract contours
 * 3. find the extracted contures and save them if they fullfill the room-area criterion
 * 4. draw and fill the saved contoures in a clone of the map from 1. with a random colour
 * 5. get the obstacle information from the original map and draw them in the clone from 4.
 * 6. spread the coloured regions to the white Pixels
 * 7. get the min/max x/y-coordinates and the coordinates of the roomcenters
 */

morphological_segmentation::morphological_segmentation(cv::Mat original_map_from_subscription, double map_resolution_from_subscription,
        double room_area_factor_lower_limit, double room_area_factor_upper_limit)
{
	//set the variables for limits and map resoulution
	map_resolution_from_subscription_ = map_resolution_from_subscription;
	room_area_factor_lower_limit_ = room_area_factor_lower_limit;
	room_area_factor_upper_limit_ = room_area_factor_upper_limit;
	//start segmenting the map
	segmentationAlgorithm(original_map_from_subscription);
}

void morphological_segmentation::segmentationAlgorithm(cv::Mat map_to_be_labeled)
{
	//uncomment this if you want to have different random-values in every step
	//(rand() only gives pseudo-random-numbers and srand() sets a different inital-value for rand())
//	srand (time(NULL));
	//set variables for time measurement and start measuring
	std	::time_t start_t, ende_t;
	float sek_t;
	std::time(&start_t);

	//make two mapclones to work with
	temporary_map_to_find_rooms_ = map_to_be_labeled.clone();
	new_map_to_draw_contours_ = map_to_be_labeled.clone();
	//**************erode temporary_map until last possible room found****************
	//erode map a spedified amount of times
	ROS_INFO("starting eroding");
	for (int counter = 0; counter < 50; counter++)
	{
		//erode the map one time
		cv::Mat eroded_map;
		cv::Point anchor(-1, -1);//needed for opencv erode
		cv::erode(temporary_map_to_find_rooms_, eroded_map, cv::Mat(), anchor, 1);
		//save the more eroded map
		temporary_map_to_find_rooms_ = eroded_map;
		//Save the eroded map in a second map, which is used to find the contours. This is neccesarry, because
		//the function findContours changes the given map and would make it impossible to work any further with it
		cv::Mat contour_map = eroded_map.clone();
		//find Contours in the more eroded map
		std::vector < std::vector<cv::Point> > temporary_contours;//temporary saving-variable
		//hierarchy saves if the contours are hole-contours:
		//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
		//child-contour = 1 if it has one, = 1 if not, same for parent_contour
		std::vector < cv::Vec4i > hierarchy;
		cv::findContours(contour_map, temporary_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		if (temporary_contours.size() != 0)
		{
			//check every contour if it fullfills the criteria of a room
			for (int current_contour = 0; current_contour < temporary_contours.size(); current_contour++)
			{ //only take first level contours --> second level contours belong to holes and doesn't need to be looked at
				if (hierarchy[current_contour][3] == -1)
				{
					//check if contour is large/small enough for a room
					double room_area = map_resolution_from_subscription_ * map_resolution_from_subscription_
					* cv::contourArea(temporary_contours[current_contour]);
					if (room_area_factor_lower_limit_ < room_area && room_area < room_area_factor_upper_limit_)
					{
						//save contour for later drawing in map
						saved_contours.push_back(temporary_contours[current_contour]);
						//make region black if room found --> region doesn't need to be looked at anymore
						cv::drawContours(temporary_map_to_find_rooms_, temporary_contours, current_contour, cv::Scalar(0), CV_FILLED, 8, hierarchy, 2);
					}
				}
			}
		}
	}
	//*******************draw contures in new map***********************
	std::cout << "Segmentation Found " << saved_contours.size() << " rooms." << std::endl;
	//draw filled contoures in new_map_to_draw_contours_ with random colour if this colour hasn't been used yet
	for (int idx = 0; idx < saved_contours.size(); idx++)
	{
		bool drawn = false; //checking-variable if contour has been drawn
		cv::Scalar fill_colour(rand() % 253 + 1);
		do
		{
			if (!contains(already_used_coloures_, fill_colour))
			{
				//if colour is unique draw Contour in map
				cv::drawContours(new_map_to_draw_contours_, saved_contours, idx, fill_colour, CV_FILLED);
				already_used_coloures_.push_back(fill_colour);//add colour to used coloures
				drawn = true;
			}
		}while (!drawn);
	}
	cv::imwrite("/home/rmb-fj/Pictures/maps/map_with_contours_drawn_in.jpg", new_map_to_draw_contours_);
	//*************************obstacles***********************
	//get obstacle informations and draw them into the new map
	ROS_INFO("starting getting obstacle information");
	for (int x_coordinate = 0; x_coordinate < map_to_be_labeled.rows; x_coordinate++)
	{
		for (int y_coordinate = 0; y_coordinate < map_to_be_labeled.cols; y_coordinate++)
		{
			//find obstacles = black pixels
			if (map_to_be_labeled.at<unsigned char>(x_coordinate, y_coordinate) == 0)
			{
				new_map_to_draw_contours_.at<unsigned char>(x_coordinate, y_coordinate) = 0;
			}
		}
	}
	ROS_INFO("drawn obstacles in map");
	cv::imwrite("/home/rmb-fj/Pictures/maps/obstacle_map.jpg", new_map_to_draw_contours_);

	//**************spread the colored region by making white pixel around a contour their color****************
	temporary_map_to_fill_white_pixels = new_map_to_draw_contours_.clone();
	//spread the coloured regions to the white Pixels
	temporary_map_to_fill_white_pixels = watershed_region_spreading(temporary_map_to_fill_white_pixels);
	//save the spreaded map
	new_map_to_draw_contours_ = temporary_map_to_fill_white_pixels.clone();
	ROS_INFO("filled white pixels in new_map_to_draw_contours_");
	cv::imwrite("/home/rmb-fj/Pictures/maps/outfilled_map.png", new_map_to_draw_contours_);

	//***********************Find min/max x and y coordinate and center of each found room********************
	temporary_map_to_find_room_values_ = new_map_to_draw_contours_.clone();
	//min/max y/x-values vector for each room. Initialized with extreme values
	std::vector<int> min_y_value_of_the_room(255, 100000000);
	std::vector<int> max_y_value_of_the_room(255, 0);
	std::vector<int> min_x_value_of_the_room(255, 100000000);
	std::vector<int> max_x_value_of_the_room(255, 0);
	//vector of the central Point for each room, initially filled with Points out of the map
	cv::Point center_out_of_bounds(-1, -1);
	std::vector < cv::Point > room_center_for_each_room(255, center_out_of_bounds);
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int column = 0; column <= map_to_be_labeled.cols; column++)
	{
		for (int row = 0; row <= map_to_be_labeled.rows; row++)
		{
			//if Pixel is white or black it is no room --> doesn't need to be checked
			if (temporary_map_to_find_room_values_.at<unsigned char>(row, column) != 0
					&& temporary_map_to_find_room_values_.at<unsigned char>(row, column) != 255)
			{
				min_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::min(row,
						min_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column) - 1]);
				max_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::max(row,
						max_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column) - 1]);
				min_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::min(column,
						min_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column) - 1]);
				max_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::max(column,
						max_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column) - 1]);

			}
		}
	}
	//get centers for each room
	for (int idx = 0; idx < room_center_for_each_room.size(); idx++)
	{
		room_center_for_each_room[idx].x = max_x_value_of_the_room[idx] / 2;
		room_center_for_each_room[idx].y = max_y_value_of_the_room[idx] / 2;
	}
	//stop and show running-time of the algorithm
	std::time(&ende_t);
	sek_t = (float) (ende_t - start_t);
	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;
}

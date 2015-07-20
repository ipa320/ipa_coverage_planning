#include <ipa_room_segmentation/morphological_segmentation.h>

/*This segmentation algorithm does:
 * 1. collect the map data
 * 2. erode the map to extract contours
 * 3. find the extracted contures and save them if they fullfill the room-area criterion
 * 4. draw and fill the saved contoures in a clone of the map from 1. with a random colour
 * 5. get the obstacle information from the original map and draw them in the clone from 4.
 * 6. spread the coloured regions to the white Pixels
 */

morphological_segmentation::morphological_segmentation()
{

}

cv::Mat morphological_segmentation::segmentationAlgorithm(cv::Mat map_to_be_labeled)
{
	//make two mapclones to work with
	temporary_map_to_find_rooms_ = map_to_be_labeled.clone();
	new_map_to_draw_contours_ = map_to_be_labeled.clone();
	//**************erode temporary_map until last possible room found****************
	//erode map a specified amount of times
	ROS_INFO("starting eroding");
	for (int counter = 0; counter < 73; counter++)
	{
		//erode the map one time
		cv::Mat eroded_map;
		cv::Point anchor(-1, -1); //needed for opencv erode
		cv::erode(temporary_map_to_find_rooms_, eroded_map, cv::Mat(), anchor, 1);
		//save the more eroded map
		temporary_map_to_find_rooms_ = eroded_map;
		//Save the eroded map in a second map, which is used to find the contours. This is neccesarry, because
		//the function findContours changes the given map and would make it impossible to work any further with it
		cv::Mat contour_map = eroded_map.clone();
		//find Contours in the more eroded map
		std::vector < std::vector<cv::Point> > temporary_contours; //temporary saving-variable
		//hierarchy saves if the contours are hole-contours:
		//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
		//child-contour = 1 if it has one, = -1 if not, same for parent_contour
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
		int draw_counter = 0; //counter to exit loop if it gets into an endless-loop (e.g. when there are more than 250 rooms)
		cv::Scalar fill_colour(rand() % 253 + 1);
		do
		{
			draw_counter++;
			if (!contains(already_used_coloures_, fill_colour) || draw_counter > 250)
			{
				//if colour is unique draw Contour in map
				cv::drawContours(new_map_to_draw_contours_, saved_contours, idx, fill_colour, CV_FILLED);
				already_used_coloures_.push_back(fill_colour); //add colour to used coloures
				drawn = true;
			}
		} while (!drawn);
	}
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
	//**************spread the colored region by making white pixel around a contour their color****************
	temporary_map_to_fill_white_pixels = new_map_to_draw_contours_.clone();
	//spread the coloured regions to the white Pixels
	temporary_map_to_fill_white_pixels = watershed_region_spreading(temporary_map_to_fill_white_pixels);
	//save the spreaded map
	new_map_to_draw_contours_ = temporary_map_to_fill_white_pixels.clone();
	ROS_INFO("filled white pixels in new map");

	return new_map_to_draw_contours_;
}

void morphological_segmentation::clear_all_vectors()
{
	saved_contours.clear();
	already_used_coloures_.clear();
}

int morphological_segmentation::get_size_color()
{
	return already_used_coloures_.size();
}

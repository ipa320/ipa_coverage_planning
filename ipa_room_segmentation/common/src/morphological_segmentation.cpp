#include <ipa_room_segmentation/morphological_segmentation.h>

#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>

MorphologicalSegmentation::MorphologicalSegmentation()
{

}

void MorphologicalSegmentation::segmentMap(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription,
        double room_area_factor_lower_limit, double room_area_factor_upper_limit)
{
	/*This segmentation algorithm does:
	 * 1. collect the map data
	 * 2. erode the map to extract contours
	 * 3. find the extracted contures and save them if they fullfill the room-area criterion
	 * 4. draw and fill the saved contoures in a clone of the map from 1. with a random colour
	 * 5. get the obstacle information from the original map and draw them in the clone from 4.
	 * 6. spread the coloured regions to the white Pixels
	 */

	//make two map clones to work with
	cv::Mat temporary_map_to_find_rooms = map_to_be_labeled.clone(); //map to find the rooms and for eroding
	//**************erode temporary_map until last possible room found****************
	//erode map a specified amount of times
	std::vector < std::vector<cv::Point> > saved_contours; //saving variable for every contour that is between the upper and the lower limit
	ROS_INFO("starting eroding");
	for (int counter = 0; counter < 73; counter++)
	{
		//erode the map one time
		cv::Mat eroded_map;
		cv::Point anchor(-1, -1); //needed for opencv erode
		cv::erode(temporary_map_to_find_rooms, eroded_map, cv::Mat(), anchor, 1);
		//save the more eroded map
		temporary_map_to_find_rooms = eroded_map;
		//Save the eroded map in a second map, which is used to find the contours. This is neccesarry, because
		//the function findContours changes the given map and would make it impossible to work any further with it
		cv::Mat contour_map = eroded_map.clone();
		//find Contours in the more eroded map
		std::vector < std::vector<cv::Point> > temporary_contours; //temporary saving-variable
		//hierarchy saves if the contours are hole-contours:
		//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
		//child-contour = 1 if it has one, = -1 if not, same for parent_contour
		std::vector < cv::Vec4i > hierarchy;
#if CV_MAJOR_VERSION<=3
		cv::findContours(contour_map, temporary_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
#else
		cv::findContours(contour_map, temporary_contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
#endif
		if (temporary_contours.size() != 0)
		{
			//check every contour if it fullfills the criteria of a room
			for (int current_contour = 0; current_contour < temporary_contours.size(); current_contour++)
			{ //only take first level contours --> second level contours belong to holes and doesn't need to be looked at
				if (hierarchy[current_contour][3] == -1)
				{
					//check if contour is large/small enough for a room
					double room_area = map_resolution_from_subscription * map_resolution_from_subscription
					        * cv::contourArea(temporary_contours[current_contour]);
					//subtract the area from the hole contours inside the found contour, because the contour area grows extremly large if it is a closed loop
					for (int hole = 0; hole < temporary_contours.size(); hole++)
					{
						if (hierarchy[hole][3] == current_contour) //check if the parent of the hole is the current looked at contour
						{
							room_area -= map_resolution_from_subscription * map_resolution_from_subscription
									* cv::contourArea(temporary_contours[hole]);
						}
					}
					if (room_area_factor_lower_limit < room_area && room_area < room_area_factor_upper_limit)
					{
						//save contour for later drawing in map
						saved_contours.push_back(temporary_contours[current_contour]);
						//make region black if room found --> region doesn't need to be looked at anymore
#if CV_MAJOR_VERSION<=3
						cv::drawContours(temporary_map_to_find_rooms, temporary_contours, current_contour, cv::Scalar(0), CV_FILLED, 8, hierarchy, 2);
#else
						cv::drawContours(temporary_map_to_find_rooms, temporary_contours, current_contour, cv::Scalar(0), cv::FILLED, 8, hierarchy, 2);
#endif
					}
				}
			}
		}
	}
	//*******************draw contures in new map***********************
	std::cout << "Segmentation Found " << saved_contours.size() << " rooms." << std::endl;
	//draw filled contoures in new_map_to_draw_contours_ with random colour if this colour hasn't been used yet
	cv::Mat new_map_to_draw_contours; //map for drawing the found contours
	map_to_be_labeled.convertTo(segmented_map, CV_32SC1, 256, 0);
	std::vector < cv::Scalar > already_used_coloures; //vector for saving the already used coloures
	for (int idx = 0; idx < saved_contours.size(); idx++)
	{
		bool drawn = false; //checking-variable if contour has been drawn
		int draw_counter = 0; //counter to exit loop if it gets into an endless-loop (e.g. when there are more rooms than possible)
		do
		{
			draw_counter++;
			cv::Scalar fill_colour(rand() % 52224 + 13056);
			if (!contains(already_used_coloures, fill_colour) || draw_counter > 250)
			{
				//if colour is unique draw Contour in map
#if CV_MAJOR_VERSION<=3
				cv::drawContours(segmented_map, saved_contours, idx, fill_colour, CV_FILLED);
#else
				cv::drawContours(segmented_map, saved_contours, idx, fill_colour, cv::FILLED);
#endif
				already_used_coloures.push_back(fill_colour); //add colour to used coloures
				drawn = true;
			}
		} while (!drawn);
	}
	//*************************obstacles***********************
	//get obstacle informations and draw them into the new map
	ROS_INFO("starting getting obstacle information");
	for (int row = 0; row < map_to_be_labeled.rows; ++row)
	{
		for (int col = 0; col < map_to_be_labeled.cols; ++col)
		{
			//find obstacles = black pixels
			if (map_to_be_labeled.at<unsigned char>(row, col) == 0)
			{
				segmented_map.at<int>(row, col) = 0;
			}
		}
	}
	ROS_INFO("drawn obstacles in map");
	//**************spread the colored region by making white pixel around a contour their color****************
	//spread the coloured regions to the white Pixels
	wavefrontRegionGrowing(segmented_map);
	ROS_INFO("filled white pixels in new map");

}

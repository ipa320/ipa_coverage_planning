#include <ipa_room_segmentation/distance_segmentation.h>

distance_segmentation::distance_segmentation()
{

}

cv::Mat distance_segmentation::segmentationAlgorithm(cv::Mat map_to_be_labeled)
{
	//variables for distance transformation
	cv::Mat temporary_map = map_to_be_labeled.clone();
	//variables for thresholding and finding the room-areas
	cv::Mat thresh_map;
	std::vector < std::vector<cv::Point> > contours;
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy;
	std::vector < std::vector<cv::Point> > temporary_contours;
	//
	//Segmentation of a gridmap into roomlike areas based on the distance-transformation of the map
	//

	//1. Get the distance-transformed map and make it an 8-bit single-channel image
	cv::erode(temporary_map, temporary_map, cv::Mat());
	cv::distanceTransform(temporary_map, distance_map_, CV_DIST_L2, 5);
	cv::convertScaleAbs(distance_map_, distance_map_);

	//2. Threshold the map and find the contours of the rooms. Change the threshold and repeat steps until last possible threshold.
	//Then take the contours from the threshold with the most contours between the roomfactors and draw it in the map with a random color.
	for (double current_trhreshold = 255.0; current_trhreshold > 0.0; current_trhreshold--)
	{ //change the threshold for the grayscale-image from largest possible value to smallest
	  //reset number of rooms
		temporary_contours.clear();
		contours.clear();
		cv::threshold(distance_map_, thresh_map, current_trhreshold, 255, cv::THRESH_BINARY);
		cv::findContours(thresh_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		//Get the number of large enough regions to be a room. Only check non-holes.
		for (int c = 0; c < contours.size(); c++)
		{
			if (hierarchy[c][3] == -1)
			{
				double room_area = map_resolution_from_subscription_ * map_resolution_from_subscription_ * cv::contourArea(contours[c]);
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
	//Draw the found contours from the step with most areas in the map with a random colour, that hasn't been used yet
	temporary_map = map_to_be_labeled.clone();
	for (int current_contour = 0; current_contour < saved_contours.size(); current_contour++)
	{
		bool drawn = false; //variable to check if contour has been drawn
		int loop_counter = 0;//loop counter for ending the loop if it gets into a endless loop
		do
		{
			loop_counter++;
			cv::Scalar fill_colour(rand() % 200 + 53);
			if (!contains(already_used_coloures_, fill_colour) || loop_counter > 250)
			{
				cv::drawContours(temporary_map, saved_contours, current_contour, fill_colour, CV_FILLED);
				already_used_coloures_.push_back(fill_colour); //add used colour to the saving-vector
				drawn = true;
			}
		} while (!drawn);
	}
	//spread the colors to the white Pixels
	temporary_map = watershed_region_spreading(temporary_map);
	return temporary_map;
}

void distance_segmentation::clear_all_vectors()
{
	saved_contours.clear();
	already_used_coloures_.clear();
}

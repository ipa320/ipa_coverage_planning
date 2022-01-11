#include <ipa_room_segmentation/distance_segmentation.h>

#include <ipa_room_segmentation/wavefront_region_growing.h>
#include <ipa_room_segmentation/contains.h>

DistanceSegmentation::DistanceSegmentation()
{

}

void DistanceSegmentation::segmentMap(const cv::Mat& map_to_be_labeled, cv::Mat& segmented_map, double map_resolution_from_subscription, double room_area_factor_lower_limit, double room_area_factor_upper_limit)
{
	//variables for energy maximization
	double optimal_room_area = 50; //variable that sets the desired optimal room area
	double constant_additional_value = optimal_room_area * optimal_room_area; //variable that sets the energy function higher so that it is 0 for the lower limit
	//variables for distance transformation
	cv::Mat temporary_map = map_to_be_labeled.clone();
	//variables for thresholding and finding the room-areas
	cv::Mat thresh_map;
	std::vector < std::vector<cv::Point> > contours;
	//hierarchy saves if the contours are hole-contours:
	//hierarchy[{0,1,2,3}]={next contour (same level), previous contour (same level), child contour, parent contour}
	//child-contour = 1 if it has one, = -1 if not, same for parent_contour
	std::vector < cv::Vec4i > hierarchy, hierarchy_saver;
	std::vector < std::vector<cv::Point> > temporary_contours;
	//
	//Segmentation of a gridmap into roomlike areas based on the distance-transformation of the map
	//

	//1. Get the distance-transformed map and make it an 8-bit single-channel image
	cv::erode(temporary_map, temporary_map, cv::Mat());
	cv::Mat distance_map;	//variable for the distance-transformed map, type: CV_32FC1
#if CV_MAJOR_VERSION<=3
	cv::distanceTransform(temporary_map, distance_map, CV_DIST_L2, 5);
#else
	cv::distanceTransform(temporary_map, distance_map, cv::DIST_L2, 5);
#endif
	cv::convertScaleAbs(distance_map, distance_map);	// conversion to 8 bit image

	//2. Threshold the map and find the contours of the rooms. Change the threshold and repeat steps until last possible threshold.
	//Then take the contours from the threshold with the most contours between the roomfactors and draw it in the map with a random color.
	std::vector<std::vector<cv::Point> > saved_contours, hole_contour_saver;	//saving-vector for the found contours
	double saved_energy = 0;
	for (int current_threshold = 255; current_threshold > 0; current_threshold--)
	{ //change the threshold for the grayscale-image from largest possible value to smallest
	  //reset number of rooms
		temporary_contours.clear();
		contours.clear();
		hierarchy.clear();
		cv::threshold(distance_map, thresh_map, current_threshold, 255, cv::THRESH_BINARY);
#if CV_MAJOR_VERSION<=3
		cv::findContours(thresh_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
#else
		cv::findContours(thresh_map, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
#endif

		//Get the number of large enough regions to be a room. Only check non-holes.
		//Energy function: -(x-a)^2 + b, where x is the current area, a is the optimal area and b is a factor to make the function zero at x=0
		for (int c = 0; c < contours.size(); c++)
		{
			if (hierarchy[c][3] == -1)
			{
				double room_area = map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[c]);
				//subtract the area from the hole contours inside the found contour, because the contour area grows extremly large if it is a closed loop
				for(int hole = 0; hole < contours.size(); hole++)
				{
					if(hierarchy[hole][3] == c)//check if the parent of the hole is the current looked at contour
					{
						room_area -= map_resolution_from_subscription * map_resolution_from_subscription * cv::contourArea(contours[hole]);
					}
				}
				if (room_area >= room_area_factor_lower_limit && room_area <= room_area_factor_upper_limit)
				{
					temporary_contours.push_back(contours[c]);
					//update the energy of the current configuration
				}
			}
		}
		//check if current step has a better energy than the saved one
		if (temporary_contours.size() >= saved_contours.size())//(current_energy >= saved_energy)
		{
			saved_contours.clear();
			hole_contour_saver.clear();
			hierarchy_saver.clear();
			saved_contours = temporary_contours;
			hole_contour_saver = contours;
			hierarchy_saver = hierarchy;
		}
	}
	//Draw the found contours from the step with most areas in the map with a random colour, that hasn't been used yet
	std::vector<cv::Scalar> already_used_colors;	//saving-variable for already used fill-colours
	map_to_be_labeled.convertTo(segmented_map, CV_32SC1, 256, 0);		// rescale to 32 int, 255 --> 255*256 = 65280
	for (int current_contour = 0; current_contour < saved_contours.size(); current_contour++)
	{
		bool drawn = false; //variable to check if contour has been drawn
		int loop_counter = 0;//loop counter for ending the loop if it gets into an endless loop
		do
		{
			loop_counter++;
			cv::Scalar fill_colour(rand() % 52224 + 13056);
			if (!contains(already_used_colors, fill_colour) || loop_counter > 250)
			{
				cv::drawContours(segmented_map, saved_contours, current_contour, fill_colour, 7);
				already_used_colors.push_back(fill_colour); //add used colour to the saving-vector
				drawn = true;
			}
		} while (!drawn);
	}
	//draw the hole contours black into the new map
	for(int current_hole = 0; current_hole < hole_contour_saver.size(); current_hole++)
	{
		if(hierarchy_saver[current_hole][3] == 1)
		{
#if CV_MAJOR_VERSION<=3
			cv::drawContours(segmented_map, hole_contour_saver, current_hole, cv::Scalar(0), CV_FILLED);
#else
			cv::drawContours(segmented_map, hole_contour_saver, current_hole, cv::Scalar(0), cv::FILLED);
#endif
		}
	}
	//spread the colors to the white pixels
	wavefrontRegionGrowing(segmented_map);
}

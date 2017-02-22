#include <ipa_room_segmentation/wavefront_region_growing.h>

// spreading image is supposed to be of type CV_32SC1
void wavefrontRegionGrowing(cv::Mat& image)
{
	//This function spreads the colored regions of the given map to the neighboring white pixels
	if (image.type()!=CV_32SC1)
	{
		std::cout << "Error: wavefrontRegionGrowing: provided image is not of type CV_32SC1." << std::endl;
		return;
	}

	cv::Mat spreading_map = image.clone();
	bool finished = false;
	while (finished == false)
	{
		finished = true;
		for (int row = 1; row < spreading_map.rows-1; ++row)
		{
			for (int column = 1; column < spreading_map.cols-1; ++column)
			{
				if (spreading_map.at<int>(row, column) > 65279)		// unassigned pixels
				{
					//check 3x3 area around white pixel for fillcolour, if filled Pixel around fill white pixel with that colour
					bool set_value = false;
					for (int row_counter = -1; row_counter <= 1 && set_value==false; ++row_counter)
					{
						for (int column_counter = -1; column_counter <= 1 && set_value==false; ++column_counter)
						{
							int value = image.at<int>(row + row_counter, column + column_counter);
							if (value != 0 && value <= 65279)
							{
								spreading_map.at<int>(row, column) = value;
								set_value = true;
								finished = false;	// keep on iterating the wavefront propagation until no more changes occur
							}
						}
					}
				}
			}
		}
		image = spreading_map.clone();
	}
}

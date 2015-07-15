#include <ipa_room_segmentation/watershed_region_spreading.h>

cv::Mat watershed_region_spreading(cv::Mat spreading_image)
{
	//This function spreads the coloured regions of the given Map to the neighboring white Pixels a large enough number of times.
	cv::Mat spreading_map = spreading_image.clone();
	cv::Mat temporary_map_to_fill_white_pixels_ = spreading_image.clone();
	for (int loop_counter = 0; loop_counter < 730; loop_counter++)
	{
		for (int column = 0; column < spreading_map.cols; column++)
		{
			for (int row = 0; row < spreading_map.rows; row++)
			{
				if (spreading_map.at<unsigned char>(row, column) == 255)
				{
					//check 3x3 area around white pixel for fillcolour, if filled Pixel around fill white pixel with that colour
					for (int row_counter = -1; row_counter <= 1; row_counter++)
					{
						for (int column_counter = -1; column_counter <= 1; column_counter++)
						{
							if (temporary_map_to_fill_white_pixels_.at<unsigned char>(row + row_counter, column + column_counter) != 0
							        && temporary_map_to_fill_white_pixels_.at<unsigned char>(row + row_counter, column + column_counter) != 255)
							{
								spreading_map.at<unsigned char>(row, column) = spreading_map.at<unsigned char>(row + row_counter, column + column_counter);
							}
						}
					}
				}
			}
		}
		temporary_map_to_fill_white_pixels_ = spreading_map.clone();
	}
	return temporary_map_to_fill_white_pixels_;
}

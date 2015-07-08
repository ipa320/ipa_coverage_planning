#include <autopnp_scenario/map_segmentation_algorithm.h>
//#define __DEBUG_DISPLAYS__
segmentation_algorithm::segmentation_algorithm(std::string name_of_the_action) :
map_segmentation_action_server_(nh_, name_of_the_action, boost::bind(&segmentation_algorithm::execute_map_segmentation_server, this, _1), false), action_name_(
name_of_the_action)
{
	//Start action server
	map_segmentation_action_server_.start();
	//Initialize the map resolution
	map_resolution_ = 0.0;
	room_area_ = 0.0;
	ros::param::param("/map_segmentation_algorithm_parameter/map_sampling_factor_check_", map_sampling_factor_, 1.5);
	ros::param::param("/map_segmentation_algorithm_parameter/room_area_factor_lower_limit_check_", room_area_factor_lower_limit_, 3.0);
	ros::param::param("/map_segmentation_algorithm_parameter/room_area_factor_upper_limit_check_", room_area_factor_upper_limit_, 40.0);	
}
/* This is the map segmentation algorithm,does the following steps:
* 1. collect the navigation data from analyze map action client
* 2. erode the image until get the last room
* 3. find contours
* 4. for each contour check if contour fulfills criteria of a room
* 5. if contour fulfill the criteria save the contour somewhere else
* 6. exit the loop if there are no more contours
* 7. copy original image
* 8. copy the stored contours into the map and fill each with a unique
* id number repeat until convergence(i.e there are no more white pixels)
*/
cv::Mat segmentation_algorithm::Image_Segmentation_method(cv::Mat &Original_Map_from_subscription, double map_resolution_data_from_subscription)
{
	map_resolution_ = map_resolution_data_from_subscription;
	//1. collect the navigation data from analyze map action client
	temporary_map_to_get_the_contours_of_the_room_ = Original_Map_from_subscription.clone();
	new_map_to_draw_the_saved_contours_of_the_room_ = Original_Map_from_subscription.clone();
	//2. erode the image until get the last room
	for (int for_loop_counter = 0; for_loop_counter < map_sampling_factor_ / map_resolution_; for_loop_counter++)
	{	
		cv::erode(temporary_map_to_get_the_contours_of_the_room_, expanded_map_, cv::Mat(), cv::Point(-1, -1), 1);
		temporary_map_to_get_the_contours_of_the_room_ = expanded_map_;
		contour_map_ = expanded_map_.clone();
		//*********************** Find contours and Draw Contours*******************
		//3. find contours
		cv::findContours(contour_map_, temporary_contours_, hierarchy_, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		cv::drawContours(contour_map_, temporary_contours_, -1, cv::Scalar(128, 128, 128, 128), 2);
		for (int idx = 0; idx >= 0; idx = hierarchy_[idx][0])
		{
			room_area_ = map_resolution_ * map_resolution_ * cv::contourArea(temporary_contours_[idx]);
			//4. for each contour check if contour fulfills criteria of a room
			if (room_area_factor_lower_limit_ < room_area_ && room_area_ < room_area_factor_upper_limit_ && temporary_contours_.size() != 0)
			{
				//5. if contour fulfill the criteria save the contour somewhere else
				saved_contours_.push_back(temporary_contours_[idx]);
				//************Remove the Saved Contour or make the region Black************
				cv::drawContours(temporary_map_to_get_the_contours_of_the_room_, temporary_contours_, idx, cv::Scalar(0), CV_FILLED, 8, hierarchy_, 2);
				//************Remove the Saved Contour or make the region Black************
			}
		}
	} //6. exit the loop if there are no more contours
	//**********************Draw the saved Contours in the clone version of original image***************
	for (unsigned int idx = 0; idx < saved_contours_.size(); idx++)
	{
		std::cout << "Segmentation found " << saved_contours_.size() << " rooms." << std::endl;
		//0-for obstacles and it's a black pixel
		//255-for white value and accessible area
		cv::Scalar color_to_fill(rand() % 253 + 1);
		cv::drawContours(new_map_to_draw_the_saved_contours_of_the_room_, saved_contours_, idx, color_to_fill, -1);
	}
	//**********************Draw the saved Contours in the clone version of original image***************
	//*********** To draw the Obstacle in the modified map*********************
	new_map_with_obstacles_info_ = new_map_to_draw_the_saved_contours_of_the_room_.clone();
	for (int y_coordinate = 0; y_coordinate < Original_Map_from_subscription.cols; y_coordinate++)
	{
		for (int x_coordinate = 0; x_coordinate < Original_Map_from_subscription.rows; x_coordinate++)
		{
			if (Original_Map_from_subscription.at<unsigned char>(x_coordinate, y_coordinate) == 0)
			{
				black_pixel_.push_back(cv::Point(y_coordinate, x_coordinate));
			}
		}
	}
	for (unsigned int idx = 0; idx < black_pixel_.size(); idx++)
	{
		new_map_with_obstacles_info_.at<unsigned char>(black_pixel_[idx]) = 0;
	}
	//*********** To draw the Obstacle in the modified map*********************
	//************Replica-Padding to the Image region*************
	//7. copy original image
	complete_map_after_contour_extraction_and_labelling = new_map_with_obstacles_info_.clone();
	int map_column_length, map_row_length = 0;
	temporary_map_for_replica_padding_purpose = new_map_with_obstacles_info_.clone();
	// 8. copy the stored contours into the map and fill each with a unique
	// id number repeat until convergence(i.e there are no more white pixels)
	for (int loop_counter = 0; loop_counter < 1000; loop_counter++)
	{
		for (map_column_length = 0; map_column_length < Original_Map_from_subscription.cols; map_column_length++)
		{
			for (map_row_length = 0; map_row_length < Original_Map_from_subscription.rows; map_row_length++)
			{
				if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length) != 0
				&& complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length) != 255)
				{
					//Check every Pixel where its neighborhood is already replaced or not
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length - 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length - 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length - 1));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length, map_row_length - 1));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length - 1, map_column_length + 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length - 1, map_column_length + 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length - 1));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length - 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length - 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length + 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length + 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length - 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length - 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length - 1, map_row_length + 1));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length, map_row_length + 1));
					}
					if (complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length + 1, map_column_length + 1) == 255
					&& temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length + 1, map_column_length + 1)
					!= complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length))
					{
						neighbourhood_pixel_.push_back(cv::Point(map_column_length + 1, map_row_length + 1));
					}
				}
				for (unsigned int idx = 0; idx < neighbourhood_pixel_.size(); idx++)
				{
					if (neighbourhood_pixel_.size() != 0)
						temporary_map_for_replica_padding_purpose.at<unsigned char>(neighbourhood_pixel_[idx]) = complete_map_after_contour_extraction_and_labelling.at<unsigned char>(map_row_length, map_column_length);
				}
				neighbourhood_pixel_.clear();
			}
		}
		//Go for Next Check where the Replaced Pixel are now set as original
		complete_map_after_contour_extraction_and_labelling = temporary_map_for_replica_padding_purpose.clone();
	}
	//************Replica-Padding to the Image region*************
	//************Extracting Data from segmented map and Bounding Box Technique**********************************
	bounding_box_map_to_extract_room_info = temporary_map_for_replica_padding_purpose.clone();
	cv::Point point_at_the_min_end_of_xy_coordinate, point_at_the_max_end_of_xy_coordinate, center_of_the_individual_room;
	std::vector<int> min_y_value_of_the_room(255, 100000000);
	std::vector<int> max_y_value_of_the_room(255, 0);
	std::vector<int> min_x_value_of_the_room(255, 100000000);
	std::vector<int> max_x_value_of_the_room(255, 0);
	for (map_column_length = 0; map_column_length < Original_Map_from_subscription.cols; map_column_length++)
	{
		for (map_row_length = 0; map_row_length < Original_Map_from_subscription.rows; map_row_length++)
		{
			if (bounding_box_map_to_extract_room_info.at<unsigned char>(map_row_length, map_column_length) != 0
			&& bounding_box_map_to_extract_room_info.at<unsigned char>(map_row_length, map_column_length) != 255)
			{
				min_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::min(map_column_length,
				min_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				max_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::max(map_column_length,
				max_y_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				min_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::min(map_row_length,
				min_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
				max_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)] = std::max(map_row_length,
				max_x_value_of_the_room[temporary_map_for_replica_padding_purpose.at<unsigned char>(map_row_length, map_column_length)]);
			}
		}
	}
	for (unsigned int loop_counter = 0; loop_counter < min_y_value_of_the_room.size(); loop_counter++)
	{
		if (min_y_value_of_the_room[loop_counter] != 100000000 && min_x_value_of_the_room[loop_counter] != 100000000 && max_y_value_of_the_room[loop_counter] != 0
		&& max_x_value_of_the_room[loop_counter] != 0)
		{
			point_at_the_min_end_of_xy_coordinate.x = min_y_value_of_the_room[loop_counter];
			point_at_the_min_end_of_xy_coordinate.y = min_x_value_of_the_room[loop_counter];
			point_at_the_max_end_of_xy_coordinate.x = max_y_value_of_the_room[loop_counter];
			point_at_the_max_end_of_xy_coordinate.y = max_x_value_of_the_room[loop_counter];
			center_of_the_individual_room.x = min_y_value_of_the_room[loop_counter] + (max_y_value_of_the_room[loop_counter] - min_y_value_of_the_room[loop_counter]) / 2;
			center_of_the_individual_room.y = min_x_value_of_the_room[loop_counter] + (max_x_value_of_the_room[loop_counter] - min_x_value_of_the_room[loop_counter]) / 2;
			#ifdef __DEBUG_DISPLAYS__
			ROS_INFO("Center of the bounding Box: [ %d , %d ]", center_of_the_individual_room.x , center_of_the_individual_room.y);
			#endif
			minimum_x_coordinate_value_of_the_room_.push_back(min_x_value_of_the_room[loop_counter]);
			minimum_y_coordinate_value_of_the_room_.push_back(min_y_value_of_the_room[loop_counter]);
			maximum_x_coordinate_value_of_the_room_.push_back(max_x_value_of_the_room[loop_counter]);
			maximum_y_coordinate_value_of_the_room_.push_back(max_y_value_of_the_room[loop_counter]);
			center_of_room_.push_back(center_of_the_individual_room);
			x_coordinate_value_of_the_room_center_.push_back(center_of_the_individual_room.x);
			y_coordinate_value_of_the_room_center_.push_back(center_of_the_individual_room.y);
			cv::rectangle(bounding_box_map_to_extract_room_info, point_at_the_min_end_of_xy_coordinate, point_at_the_max_end_of_xy_coordinate, cv::Scalar(255), 1);
			cv::circle(bounding_box_map_to_extract_room_info, center_of_the_individual_room, 3, cv::Scalar(255), -1);
		}
	}
	#ifdef __DEBUG_DISPLAYS__
	cv::Mat Debug_image = temporary_map_for_replica_padding_purpose.clone();
	cv::imshow( "segmented map", Debug_image );
	cv::waitKey(100);
	cv::imshow("bounding box", bounding_box_map_to_extract_room_info);
	cv::waitKey(100);
	#endif
	//************Extracting Data from segmented map and Bounding Box Technique**********************************
	return temporary_map_for_replica_padding_purpose.clone();
}

void segmentation_algorithm::execute_map_segmentation_server(const autopnp_scenario::MapSegmentationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("111111111111 map segmentation action server 1111111111111");
	ROS_INFO("map resolution is : %f", goal->map_resolution);
	ROS_INFO("map sampling factor is : %f", map_sampling_factor_);
	ROS_INFO("room area factor lower limit is : %f", room_area_factor_lower_limit_);
	ROS_INFO("room area factor upper limit is : %f", room_area_factor_upper_limit_);
	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr_obj->image;
	cv::Mat Segmented_map;
	Segmented_map = Image_Segmentation_method(original_img, goal->map_resolution);
	ROS_INFO("111111111111 map segmentation action server 1111111111111\n");
	looping_rate.sleep();
	//Publish Result message:
	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = Segmented_map;
	cv_image.toImageMsg(action_result_.output_map);
	//cv::imshow("segmentation", Segmented_map);
	//cv::waitKey();
	//setting value to the action msgs to publish
	action_result_.map_resolution = goal->map_resolution;
	action_result_.map_origin_x = goal->map_origin_x;
	action_result_.map_origin_y = goal->map_origin_y;
	//setting massages in pixel value
	if (goal->return_format_in_pixel == true)
	{
		action_result_.room_center_x_in_pixel = x_coordinate_value_of_the_room_center_;
		action_result_.room_center_y_in_pixel = y_coordinate_value_of_the_room_center_;
		action_result_.room_min_x_in_pixel = minimum_x_coordinate_value_of_the_room_;
		action_result_.room_min_y_in_pixel = minimum_y_coordinate_value_of_the_room_;
		action_result_.room_max_x_in_pixel = maximum_x_coordinate_value_of_the_room_;
		action_result_.room_max_y_in_pixel = maximum_y_coordinate_value_of_the_room_;
	}
	//setting massages in meter
	if (goal->return_format_in_meter == true)
	{
		for (unsigned int loop_counter = 0; loop_counter < x_coordinate_value_of_the_room_center_.size(); loop_counter++)
		{
			action_result_.room_center_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(x_coordinate_value_of_the_room_center_[loop_counter]));
			action_result_.room_center_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(y_coordinate_value_of_the_room_center_[loop_counter]));
			action_result_.room_min_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(minimum_x_coordinate_value_of_the_room_[loop_counter]));
			action_result_.room_min_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(minimum_y_coordinate_value_of_the_room_[loop_counter]));
			action_result_.room_max_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(maximum_x_coordinate_value_of_the_room_[loop_counter]));
			action_result_.room_max_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(maximum_y_coordinate_value_of_the_room_[loop_counter]));
		}
	}
	//publish result
	map_segmentation_action_server_.setSucceeded(action_result_);
	//clearing the action msgs container
	action_result_.room_center_x_in_meter.clear();
	action_result_.room_center_y_in_meter.clear();
	action_result_.room_min_x_in_meter.clear();
	action_result_.room_min_y_in_meter.clear();
	action_result_.room_max_x_in_meter.clear();
	action_result_.room_max_y_in_meter.clear();
	//clearing the memory container used
	room_number_.clear();
	minimum_x_coordinate_value_of_the_room_.clear();
	maximum_x_coordinate_value_of_the_room_.clear();
	minimum_y_coordinate_value_of_the_room_.clear();
	maximum_y_coordinate_value_of_the_room_.clear();
	center_of_room_.clear();
	x_coordinate_value_of_the_room_center_.clear();
	y_coordinate_value_of_the_room_center_.clear();
	temporary_contours_.clear();
	saved_contours_.clear();
	hierarchy_.clear();
	black_pixel_.clear();
	neighbourhood_pixel_.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "segment_map");
	segmentation_algorithm Segmentation_Algorithm_obj(ros::this_node::getName());
	ROS_INFO("map segmentation action server is initialized.....");
	ros::spin();
	return 0;
}

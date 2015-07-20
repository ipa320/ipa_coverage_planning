#include <ipa_room_segmentation/morphological_segmentation_server.h>

morphological_segmentation_algorithm::morphological_segmentation_algorithm(std::string name_of_the_action) :
		morphological_segmentation_server_(nh_, name_of_the_action,
		        boost::bind(&morphological_segmentation_algorithm::execute_morph_segmentation_server, this, _1), false), action_name_(name_of_the_action)
{
	//Start action server
	morphological_segmentation_server_.start();
	//Initialize the map resolution
	map_resolution_ = 0.0;
	ros::param::param("/map_segmentation_algorithm_parameter/map_sampling_factor_check_", map_sampling_factor_, 1.5);
	ros::param::param("/map_segmentation_algorithm_parameter/room_area_factor_lower_limit_check_", room_lower_limit_, 1.0);
	ros::param::param("/map_segmentation_algorithm_parameter/room_area_factor_upper_limit_check_", room_upper_limit_, 40.0);
}

void morphological_segmentation_algorithm::execute_morph_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Segmentation action server with morphological Method********");
	ROS_INFO("map resolution is : %f", goal->map_resolution);
	ROS_INFO("map sampling factor is : %f", map_sampling_factor_);
	ROS_INFO("room area factor lower limit is : %f", room_lower_limit_);
	ROS_INFO("room area factor upper limit is : %f", room_upper_limit_);
	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr_obj->image;
	//set the resolution and the limits for the actual goal and the Map origin
	segmenter_.map_resolution_from_subscription_ = goal->map_resolution;
	segmenter_.room_area_factor_lower_limit_ = room_lower_limit_;
	segmenter_.room_area_factor_upper_limit_ = room_upper_limit_;
	map_origin_ = cv::Point2d(goal->map_origin_x, goal->map_origin_y);
	//segment the given map
	segmented_map_ = segmenter_.segmentationAlgorithm(original_img);
	ROS_INFO("********Segmented the map************");
	looping_rate.sleep();
	//get the min/max-values and the room-centers
	//min/max y/x-values vector for each room. Initialized with extreme values
	std::vector<int> min_y_value_of_the_room(255, 100000000);
	std::vector<int> max_y_value_of_the_room(255, 0);
	std::vector<int> min_x_value_of_the_room(255, 100000000);
	std::vector<int> max_x_value_of_the_room(255, 0);
	//vector of the central Point for each room, initially filled with Points out of the map
	std::vector<int> room_centers_x_values(255, -1);
	std::vector<int> room_centers_y_values(255, -1);
	//***********************Find min/max x and y coordinate and center of each found room********************
	cv::Mat temporary_map_to_find_room_values_ = segmented_map_.clone();
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int column = 0; column < temporary_map_to_find_room_values_.cols; column++)
	{
		for (int row = 0; row < temporary_map_to_find_room_values_.rows; row++)
		{
			//if Pixel is white or black it is no room --> doesn't need to be checked
			if (temporary_map_to_find_room_values_.at<unsigned char>(row, column) != 0
			        && temporary_map_to_find_room_values_.at<unsigned char>(row, column) != 255)
			{
				min_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::min(row,
				        min_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)]);
				max_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::max(row,
				        max_x_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)]);
				max_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::max(column,
				        max_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)]);
				min_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)] = std::min(column,
				        min_y_value_of_the_room[temporary_map_to_find_room_values_.at<unsigned char>(row, column)]);
			}
		}
	}
	//get centers for each room
	for (int idx = 0; idx < room_centers_x_values.size(); idx++)
	{
		if (max_x_value_of_the_room[idx] != 0 && max_y_value_of_the_room[idx] != 0
				&& min_x_value_of_the_room[idx] != 100000000 && min_y_value_of_the_room[idx] != 100000000)
		{
			room_centers_x_values[idx] = min_x_value_of_the_room[idx] + ((max_x_value_of_the_room[idx] - min_x_value_of_the_room[idx]) / 2);
			room_centers_y_values[idx] = min_y_value_of_the_room[idx] + ((max_y_value_of_the_room[idx] - min_y_value_of_the_room[idx]) / 2);
		}
	}
	//****************publish the results**********************
	//converting the cv format in map msg format
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = segmented_map_;
	cv_image.toImageMsg(action_result_.output_map);
	//setting value to the action msgs to publish
	action_result_.map_resolution = goal->map_resolution;
	action_result_.map_origin_x = goal->map_origin_x;
	action_result_.map_origin_y = goal->map_origin_y;
	//setting massages in pixel value
	if (goal->return_format_in_pixel == true)
	{
		action_result_.room_center_x_in_pixel = room_centers_x_values;
		action_result_.room_center_y_in_pixel = room_centers_y_values;
		action_result_.room_min_x_in_pixel = min_x_value_of_the_room;
		action_result_.room_min_y_in_pixel = min_y_value_of_the_room;
		action_result_.room_max_x_in_pixel = max_x_value_of_the_room;
		action_result_.room_max_y_in_pixel = max_y_value_of_the_room;
	}
	//setting massages in meter
	if (goal->return_format_in_meter == true)
	{
		for (unsigned int loop_counter = 0; loop_counter < room_centers_x_values.size(); loop_counter++)
		{
			action_result_.room_center_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(room_centers_x_values[loop_counter]));
			action_result_.room_center_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(room_centers_y_values[loop_counter]));
			action_result_.room_min_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(min_x_value_of_the_room[loop_counter]));
			action_result_.room_min_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(min_y_value_of_the_room[loop_counter]));
			action_result_.room_max_x_in_meter.push_back(convert_pixel_to_meter_for_x_coordinate_(max_x_value_of_the_room[loop_counter]));
			action_result_.room_max_y_in_meter.push_back(convert_pixel_to_meter_for_y_coordinate_(max_y_value_of_the_room[loop_counter]));
		}
	}

	//publish result
	morphological_segmentation_server_.setSucceeded(action_result_);
	//*****************clear the values for the next time***********************
	//clearing the action msgs container
	action_result_.room_center_x_in_meter.clear();
	action_result_.room_center_y_in_meter.clear();
	action_result_.room_min_x_in_meter.clear();
	action_result_.room_min_y_in_meter.clear();
	action_result_.room_max_x_in_meter.clear();
	action_result_.room_max_y_in_meter.clear();
	//clearing the memory container used
	min_x_value_of_the_room.clear();
	max_x_value_of_the_room.clear();
	min_y_value_of_the_room.clear();
	max_y_value_of_the_room.clear();
	room_centers_x_values.clear();
	room_centers_y_values.clear();
	segmenter_.clear_all_vectors();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "morphological_segmentation_server");
	morphological_segmentation_algorithm segmentationAlgorithmObj(ros::this_node::getName());
	ROS_INFO("Action Server for morphological segmentation has been initalized......");
	ros::spin();

	return 0;
}

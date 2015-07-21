#include <ipa_room_segmentation/room_segmentation_server.h>

RoomSegmentationServer::RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action) :
		node_handle_(nh), room_segmentation_server_(node_handle_, name_of_the_action,
		        boost::bind(&RoomSegmentationServer::execute_segmentation_server, this, _1), false), action_name_(name_of_the_action)
{
	//Start action server
	room_segmentation_server_.start();

	//Initialize the map resolution
	map_resolution_ = 0.0;

	//set the parameter to check if the algorithm needs to be trained
	train_the_algorithm_ = true;

	// Parameters
	std::cout << "\n--------------------------\nRoom Segmentation Parameters:\n--------------------------\n";
	node_handle_.param("map_sampling_factor_check", map_sampling_factor_check_, 1.5);
	std::cout << "room_segmentation/map_sampling_factor_check = " << map_sampling_factor_check_ << std::endl;
	node_handle_.param("room_area_factor_lower_limit_check", room_lower_limit_check_, 1.0);
	std::cout << "room_segmentation/room_area_factor_lower_limit_check = " << room_lower_limit_check_ << std::endl;
	node_handle_.param("room_area_factor_upper_limit_check", room_upper_limit_check_, 45.0);
	std::cout << "room_segmentation/room_area_factor_upper_limit_check = " << room_upper_limit_check_ << std::endl;
	node_handle_.param("room_segmentation_algorithm", room_segmentation_algorithm_, 1);
	std::cout << "room_segmentation/room_segmentation_algorithm = " << room_segmentation_algorithm_ << std::endl;
}

void RoomSegmentationServer::execute_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Segmentation action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);
	ROS_INFO("map sampling factor is : %f", map_sampling_factor_check_);
	ROS_INFO("room area factor lower limit is : %f", room_lower_limit_check_);
	ROS_INFO("room area factor upper limit is : %f", room_upper_limit_check_);

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img;
	original_img = cv_ptr_obj->image;

	//set the resolution and the limits for the actual goal and the Map origin
	map_origin_ = cv::Point2d(goal->map_origin_x, goal->map_origin_y);

	//segment the given map
	if (room_segmentation_algorithm_ == 1)
	{
		morphological_segmentation_.segmentationAlgorithm(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_);
	}
	else if (room_segmentation_algorithm_ == 2)
	{
		distance_segmentation_.segmentationAlgorithm(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_);
	}
	else if (room_segmentation_algorithm_ == 3)
	{
		voronoi_segmentation_.segmentationAlgorithm(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_);
	}
	else if (room_segmentation_algorithm_ == 4)
	{
		if (train_the_algorithm_)
		{
			//load the training maps
			//TODO: no absolute paths!!
			cv::Mat first_room_training_map = cv::imread("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/training_maps/room_training_map.png", 0);
			cv::Mat second_room_training_map = cv::imread("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/training_maps/lab_d_room_training_map.png", 0);
			cv::Mat first_hallway_training_map = cv::imread("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/training_maps/hallway_training_map.png", 0);
			cv::Mat second_hallway_training_map = cv::imread("/home/rmb-fj/git/care-o-bot-indigo/src/autopnp/ipa_room_segmentation/training_maps/lab_a_hallway_training_map.png", 0);
			//train the algorithm
			semantic_segmentation_.trainClassifiers(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map);
		}
		semantic_segmentation_.semanticLabeling(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_);
	}
	else
	{
		ROS_ERROR("Undefined algorithm selected.");
		return;
	}

	cv::imwrite("/home/rmb-fj/Pictures/maps/action_tests/one_server.png", segmented_map_);

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
		if (max_x_value_of_the_room[idx] != 0 && max_y_value_of_the_room[idx] != 0 && min_x_value_of_the_room[idx] != 100000000
		        && min_y_value_of_the_room[idx] != 100000000)
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
	room_segmentation_server_.setSucceeded(action_result_);
//	//*****************clear the values for the next time***********************
//	//clearing the action msgs container
//	action_result_.room_center_x_in_meter.clear();
//	action_result_.room_center_y_in_meter.clear();
//	action_result_.room_min_x_in_meter.clear();
//	action_result_.room_min_y_in_meter.clear();
//	action_result_.room_max_x_in_meter.clear();
//	action_result_.room_max_y_in_meter.clear();
//	//clearing the memory container used
//	min_x_value_of_the_room.clear();
//	max_x_value_of_the_room.clear();
//	min_y_value_of_the_room.clear();
//	max_y_value_of_the_room.clear();
//	room_centers_x_values.clear();
//	room_centers_y_values.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_segmentation_server_");

	ros::NodeHandle nh;

	RoomSegmentationServer segmentationAlgorithmObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room segmentation has been initalized......");
	ros::spin();

	return 0;
}

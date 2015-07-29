#include <ipa_room_segmentation/room_segmentation_server.h>

#include <ros/package.h>
#include <ipa_room_segmentation/meanshift2d.h>

RoomSegmentationServer::RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action) :
		node_handle_(nh), room_segmentation_server_(node_handle_, name_of_the_action,
		        boost::bind(&RoomSegmentationServer::execute_segmentation_server, this, _1), false), action_name_(name_of_the_action)
{
	//Start action server
	room_segmentation_server_.start();

	//Initialize the map resolution
	map_resolution_ = 0.0;

	//set the parameter to check if the algorithm needs to be trained
	train_the_algorithm_ = false;

	// Parameters
	std::cout << "\n--------------------------\nRoom Segmentation Parameters:\n--------------------------\n";
	node_handle_.param("room_segmentation_algorithm", room_segmentation_algorithm_, 1);
	std::cout << "room_segmentation/room_segmentation_algorithm = " << room_segmentation_algorithm_ << std::endl << std::endl;
	if (room_segmentation_algorithm_ == 1)
		ROS_INFO("You have chosen the morphological segmentation method.");
	else if (room_segmentation_algorithm_ == 2)
		ROS_INFO("You have chosen the distance segmentation method.");
	else if (room_segmentation_algorithm_ == 3)
		ROS_INFO("You have chosen the voronoi segmentation method.");
	else if (room_segmentation_algorithm_ == 4)
		ROS_INFO("You have chosen the semantic segmentation method.");
	std::cout << std::endl;
	//Info to remind the user of changing all important values.
	ROS_INFO("----> Important Server announcement: <-----");
	ROS_INFO(
	        "Make sure you have set all Parameters to the right ones, especially the roomarea borders. If not unusual behavior and results of the algorithms and the robot may occure.");
	ROS_INFO("----> Announcement over. <----");
	std::cout << std::endl;
	node_handle_.param("map_sampling_factor_check", map_sampling_factor_check_, 1.5);
	std::cout << "room_segmentation/map_sampling_factor_check = " << map_sampling_factor_check_ << std::endl;
	node_handle_.param("room_area_factor_lower_limit_check", room_lower_limit_check_, 1.0);
	std::cout << "room_segmentation/room_area_factor_lower_limit_check = " << room_lower_limit_check_ << std::endl;
	node_handle_.param("room_area_factor_upper_limit_check", room_upper_limit_check_, 45.0);
	std::cout << "room_segmentation/room_area_factor_upper_limit_check = " << room_upper_limit_check_ << std::endl;
	//set voronoi Parameters if the chosen algorithm is the Voronoi segmentation
	if (room_segmentation_algorithm_ == 3)
	{
		node_handle_.param("voronoi_neighborhood_index", voronoi_neighborhood_index_, 310);
		std::cout << "room_segmentation/voronoi_neighborhood_index = " << voronoi_neighborhood_index_ << std::endl;
		node_handle_.param("voronoi_max_neighborhood_size", voronoi_max_neighborhood_size_, 150);
		std::cout << "room_segmentation/voronoi_max_neighborhood_size_check = " << voronoi_max_neighborhood_size_ << std::endl;
		node_handle_.param("min_critical_Point_distance", min_critical_Point_distance_, 27.0);
		std::cout << "room_segmentation/min_critical_Point_distance = " << min_critical_Point_distance_ << std::endl;

	}

}

void RoomSegmentationServer::execute_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Segmentation action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);
	ROS_INFO("map sampling factor is : %f", map_sampling_factor_check_);
	ROS_INFO("room area factor lower limit is : %f", room_lower_limit_check_);
	ROS_INFO("room area factor upper limit is : %f", room_upper_limit_check_);
	if (room_segmentation_algorithm_ == 3)
	{
		ROS_INFO("voronoi_neighborhood_index is : %d", voronoi_neighborhood_index_);
		ROS_INFO("voronoi_max_neighborhood_size is %d: ", voronoi_max_neighborhood_size_);
		ROS_INFO("min_critical_Point_distance is %f: ", min_critical_Point_distance_);
	}

	//converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat original_img = cv_ptr_obj->image;

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
		voronoi_segmentation_.segmentationAlgorithm(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_,
		        voronoi_neighborhood_index_, voronoi_max_neighborhood_size_, min_critical_Point_distance_);
	}
	else if (room_segmentation_algorithm_ == 4)
	{
		const std::string package_path = ros::package::getPath("ipa_room_segmentation");
		const std::string classifier_path = package_path + "/common/files/training_results/";
		if (train_the_algorithm_)
		{
			//load the training maps, change to your maps when you want to train different ones
			cv::Mat first_room_training_map = cv::imread(package_path + "/common/files/training_maps/room_training_map.png", 0);
			cv::Mat second_room_training_map = cv::imread(package_path + "/common/files/training_maps/lab_d_room_training_map.png", 0);
			cv::Mat first_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/hallway_training_map.png", 0);
			cv::Mat second_hallway_training_map = cv::imread(package_path + "/common/files/training_maps/lab_a_hallway_training_map.png", 0);
			//train the algorithm
			semantic_segmentation_.trainClassifiers(first_room_training_map, second_room_training_map, first_hallway_training_map, second_hallway_training_map,
			        classifier_path);
		}
		semantic_segmentation_.semanticLabeling(original_img, segmented_map_, goal->map_resolution, room_lower_limit_check_, room_upper_limit_check_,
		        classifier_path);
	}
	else
	{
		ROS_ERROR("Undefined algorithm selected.");
		return;
	}

	ROS_INFO("********Segmented the map************");
//	looping_rate.sleep();

// get the min/max-values and the room-centers
// compute room label codebook
	std::map<int, size_t> label_vector_index_codebook; // maps each room label to a position in the rooms vector
	size_t vector_index = 0;
	for (int v = 0; v < segmented_map_.rows; ++v)
	{
		for (int u = 0; u < segmented_map_.cols; ++u)
		{
			const int label = segmented_map_.at<int>(v, u);
			if (label > 0 && label < 65280) // do not count walls/obstacles or free space as label
			{
				if (label_vector_index_codebook.find(label) == label_vector_index_codebook.end())
				{
					label_vector_index_codebook[label] = vector_index;
					vector_index++;
				}
			}
		}
	}
	//min/max y/x-values vector for each room. Initialized with extreme values
	std::vector<int> min_x_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_x_value_of_the_room(label_vector_index_codebook.size(), 0);
	std::vector<int> min_y_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_y_value_of_the_room(label_vector_index_codebook.size(), 0);
	//vector of the central Point for each room, initially filled with Points out of the map
	std::vector<int> room_centers_x_values(label_vector_index_codebook.size(), -1);
	std::vector<int> room_centers_y_values(label_vector_index_codebook.size(), -1);
	//***********************Find min/max x and y coordinate and center of each found room********************
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int y = 0; y < segmented_map_.rows; ++y)
	{
		for (int x = 0; x < segmented_map_.cols; ++x)
		{
			const int label = segmented_map_.at<int>(y, x);
			if (label > 0 && label < 65280) //if Pixel is white or black it is no room --> doesn't need to be checked
			{
				const int index = label_vector_index_codebook[label];
				min_x_value_of_the_room[index] = std::min(x, min_x_value_of_the_room[index]);
				max_x_value_of_the_room[index] = std::max(x, max_x_value_of_the_room[index]);
				max_y_value_of_the_room[index] = std::max(y, max_y_value_of_the_room[index]);
				min_y_value_of_the_room[index] = std::min(y, min_y_value_of_the_room[index]);
			}
		}
	}
	//get centers for each room
//	for (size_t idx = 0; idx < room_centers_x_values.size(); ++idx)
//	{
//		if (max_x_value_of_the_room[idx] != 0 && max_y_value_of_the_room[idx] != 0 && min_x_value_of_the_room[idx] != 100000000 && min_y_value_of_the_room[idx] != 100000000)
//		{
//			room_centers_x_values[idx] = (min_x_value_of_the_room[idx] + max_x_value_of_the_room[idx]) / 2;
//			room_centers_y_values[idx] = (min_y_value_of_the_room[idx] + max_y_value_of_the_room[idx]) / 2;
//			cv::circle(segmented_map_, cv::Point(room_centers_x_values[idx], room_centers_y_values[idx]), 2, cv::Scalar(200*256), CV_FILLED);
//		}
//	}
	MeanShift2D ms;
	for (std::map<int, size_t>::iterator it = label_vector_index_codebook.begin(); it != label_vector_index_codebook.end(); ++it)
	{
		// compute distance transform for each room
		const int label = it->first;
		cv::Mat room = cv::Mat::zeros(segmented_map_.rows, segmented_map_.cols, CV_8UC1);
		for (int v = 0; v < segmented_map_.rows; ++v)
			for (int u = 0; u < segmented_map_.cols; ++u)
				if (segmented_map_.at<int>(v, u) == label)
					room.at < uchar > (v, u) = 255;
		cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
		cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
		// find point set with largest distance to obstacles
		double min_val = 0., max_val = 0.;
		cv::minMaxLoc(distance_map, &min_val, &max_val);
		std::vector < cv::Vec2d > room_cells;
		for (int v = 0; v < distance_map.rows; ++v)
			for (int u = 0; u < distance_map.cols; ++u)
				if (distance_map.at<float>(v, u) > max_val * 0.95f)
					room_cells.push_back(cv::Vec2d(u, v));
		// use meanshift to find the modes in that set
		cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, goal->map_resolution);
		const int index = it->second;
		room_centers_x_values[index] = room_center[0];
		room_centers_y_values[index] = room_center[1];
		cv::circle(segmented_map_, cv::Point(room_centers_x_values[index], room_centers_y_values[index]), 2, cv::Scalar(200 * 256), CV_FILLED);
	}

	cv::imshow("segmentation", segmented_map_);
	cv::waitKey();

	//cv::imwrite("/home/rmb-fj/Pictures/maps/action_tests/one_server.png", segmented_map_);

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
	//*****************clear the values for the next time***********************
	//clearing the action msgs container
	action_result_.room_center_x_in_meter.clear();
	action_result_.room_center_y_in_meter.clear();
	action_result_.room_min_x_in_meter.clear();
	action_result_.room_min_y_in_meter.clear();
	action_result_.room_max_x_in_meter.clear();
	action_result_.room_max_y_in_meter.clear();
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

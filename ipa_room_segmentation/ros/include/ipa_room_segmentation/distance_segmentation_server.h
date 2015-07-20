#include "ros/ros.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <actionlib/server/simple_action_server.h>


#include <iostream>
#include <list>
#include <string>
#include <vector>


#include <ipa_room_segmentation/MapSegmentationAction.h>

#include <ipa_room_segmentation/distance_segmentation.h>

class distance_segmentation_algorithm
{
private:
	//resolution of the given map [m/cell]
	double map_resolution_;
	//sampling-factor of the map
	double map_sampling_factor_;
	//limits for the room-areas
	double room_upper_limit_, room_lower_limit_;
	//map that has been segmented
	cv::Mat segmented_map_;
	//Map origin which comes from the message
	cv::Point2d map_origin_;
	//converter-> Pixel to meter for X coordinate
	double convert_pixel_to_meter_for_x_coordinate_(int pixel_valued_object_x)
	{
		double meter_value_obj_x = (pixel_valued_object_x * map_resolution_) + map_origin_.x;
		return meter_value_obj_x;
	}
	//converter-> Pixel to meter for Y coordinate
	double convert_pixel_to_meter_for_y_coordinate_(int pixel_valued_object_y)
	{
		double meter_value_obj_y = (pixel_valued_object_y * map_resolution_) + map_origin_.y;
		return meter_value_obj_y;
	}
	//This is the execution function used by action server
	void execute_dist_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal);
	//segmentation-object that segments the map
	distance_segmentation segmenter_;
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<ipa_room_segmentation::MapSegmentationAction> distance_segmentation_server_;
	std::string action_name_;
	ipa_room_segmentation::MapSegmentationFeedback action_feedback_;
	ipa_room_segmentation::MapSegmentationResult action_result_;
public:
	//initialize the action-server
	distance_segmentation_algorithm(std::string name_of_the_action);

	//Default destructor for the class
	~distance_segmentation_algorithm(void)
	{
	}
};

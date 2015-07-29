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

#include <ipa_room_segmentation/morphological_segmentation.h>

#include <ipa_room_segmentation/voronoi_segmentation.h>

#include <ipa_room_segmentation/adaboost_classifier.h>

class RoomSegmentationServer
{
private:

	// parameters
	double map_sampling_factor_check_;	//sampling-factor of the map
	double room_upper_limit_check_;	//limits for the room-areas
	double room_lower_limit_check_;
	int room_segmentation_algorithm_;	// this variable selects the algorithm for room segmentation,
										// 1 = morphological segmentation
										// 2 = distance segmentation
										// 3 = Voronoi segmentation
										// 4 = semantic segmentation
	double map_resolution_; //resolution of the given map [m/cell]

	bool train_the_algorithm_; //Boolean to say if the algorithm needs to be trained

	int voronoi_neighborhood_index_; //Variable for the Voronoi method that specify the neighborhood that is looked at for critical Point extraction
	int max_iterations_; //number of iterations for search of neighborhood in voronoi method
	double min_critical_Point_distance_factor_; //Variable that sets the minimal distance between two critical Points before one gets eliminated

	//maps and map-parameters
	cv::Mat segmented_map_; //map that has been segmented
	cv::Point2d map_origin_; //Map origin which comes from the message


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
	void execute_segmentation_server(const ipa_room_segmentation::MapSegmentationGoalConstPtr &goal);


	//segmentation-objects that segment the map
	DistanceSegmentation distance_segmentation_; //distance segmentation method

	MorphologicalSegmentation morphological_segmentation_; //morphological segmentation method

	VoronoiSegmentation voronoi_segmentation_; //voronoi segmentation method

	AdaboostClassifier semantic_segmentation_; //semantic segmentation method

protected:
	ros::NodeHandle node_handle_;
	actionlib::SimpleActionServer<ipa_room_segmentation::MapSegmentationAction> room_segmentation_server_;
	std::string action_name_;
	ipa_room_segmentation::MapSegmentationFeedback action_feedback_;
	ipa_room_segmentation::MapSegmentationResult action_result_;
public:
	//initialize the action-server
	RoomSegmentationServer(ros::NodeHandle nh, std::string name_of_the_action);

	//Default destructor for the class
	~RoomSegmentationServer(void)
	{
	}
};

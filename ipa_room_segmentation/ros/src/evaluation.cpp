#include "ros/ros.h"
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa_room_segmentation/MapSegmentationAction.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <list>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

//calculate the compactness of the rooms. Compactness factor is given by area/perimeter
std::vector<double> calculate_compactness(std::vector<std::vector<cv::Point> > rooms)
{
	double current_area, current_perimeter;
	double map_resolution = 0.05000;
	std::vector<double> compactness_factors;
	//calculate the area and perimeter for each room using opencv
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		current_area = map_resolution * map_resolution * cv::contourArea(rooms[current_room]);
		current_perimeter = map_resolution * cv::arcLength(rooms[current_room], true);
		compactness_factors.push_back(current_area / (current_perimeter * current_perimeter));
	}
	return compactness_factors;
}

//calculate too much area of the bounding box
std::vector<double> calculate_bounding_error(std::vector<std::vector<cv::Point> > rooms)
{
	std::vector<double> space_errors;
	double bounding_box_area, room_area;
	double map_resolution = 0.05000;
	cv::RotatedRect current_bounding_box;
	//calculate the rotated bounding box for each room and subtract the roomarea from it
	for (int current_room = 0; current_room < rooms.size(); current_room++) {
		current_bounding_box = minAreaRect(rooms[current_room]);
		bounding_box_area = map_resolution * map_resolution * current_bounding_box.size.area();
		room_area = map_resolution * map_resolution * cv::contourArea(rooms[current_room]);
		//put the difference in the error vector
		space_errors.push_back(bounding_box_area - room_area);
	}
	return space_errors;
}

//calculate area for every room
std::vector<double> calculate_areas(std::vector<std::vector<cv::Point> > rooms)
{
	std::vector<double> calculated_areas;
	double map_resolution = 0.0500;
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		calculated_areas.push_back(map_resolution * map_resolution * cv::contourArea(rooms[current_room]));
	}
	return calculated_areas;
}

//calculate perimeter for every room
std::vector<double> calculate_perimeters(std::vector<std::vector<cv::Point> > rooms)
{
	std::vector<double> calculated_perimeters;
	double map_resoultion = 0.0500;
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		calculated_perimeters.push_back(map_resoultion * cv::arcLength(rooms[current_room], true));
	}
	return calculated_perimeters;
}

//check if every roomcenter is reachable
bool check_reachability(const std::vector<std::vector<cv::Point> >& rooms, const cv::Mat& map)
{
	bool reachable = true;
	cv::RotatedRect current_bounding_box;
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		current_bounding_box = minAreaRect(rooms[current_room]);
		if (map.at<unsigned char>(current_bounding_box.center) == 0)
		{
			reachable = false;
		}
	}
	return reachable;
}

//Calculate the length of the major axis of the minimum bounding ellipse
double calc_major_axis(std::vector<cv::Point> room)
{
	cv::Point2f points[4];
	std::vector<cv::Point2f> edge_points;
	double distance = 0;
	double map_resoultion = 0.05;
	//saving-variable for the Points of the ellipse
	cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(room));
	//get the edge-points of the ellipse
	ellipse.points(points);
	//saving the Points of the ellipse in a vector
	for (int i = 0; i < 4; i++)
	{
		edge_points.push_back(points[i]);
	}
	//calculate the distance between the Points and take the largest one
	for (int p = 0; p < edge_points.size(); p++)
	{
		for (int np = 0; np < edge_points.size(); np++)
		{
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2)+ std::pow((edge_points[p].y - edge_points[np].y),2)) > distance)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2)
								+ std::pow((edge_points[p].y - edge_points[np].y),2));
			}
		}
	}
	return map_resoultion * distance;
}

//Calculate the length of the minor axis of the minimum bounding ellipse
double calc_minor_axis(std::vector<cv::Point> room) {
	cv::Point2f points[4];
	std::vector<cv::Point2f> edge_points;
	double distance = 10000000;
	double map_resoultion = 0.05;
	//saving-variable for the Points of the ellipse
	cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(room));
	//get the edge-points of the ellipse
	ellipse.points(points);
	//saving the Points of the ellipse in a vector
	for (int i = 0; i < 4; i++)
	{
		edge_points.push_back(points[i]);
	}
	//calculate the distance between the Points and take the largest one
	for (int p = 0; p < edge_points.size(); p++)
	{
		for (int np = 0; np < edge_points.size(); np++)
		{
			//np != p: make sure the distance is nor calculated to the Point itself
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2)
							+ std::pow((edge_points[p].y - edge_points[np].y),2)) < distance && np != p)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2)
								+ std::pow((edge_points[p].y - edge_points[np].y),2));
			}
		}
	}
	return map_resoultion * distance;
}

//Calculate the Quotient of the langths of the major axis and the minor axis from the fitting ellipse for each room
std::vector<double> calc_Ellipse_axis(std::vector<std::vector<cv::Point> > rooms)
{
	std::vector<double> quotients;
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		quotients.push_back(calc_major_axis(rooms[current_room]) / calc_minor_axis(rooms[current_room]));
	}
	return quotients;
}

//Calculate the average distance between room-centers
double calc_average_distance(std::vector<std::vector<cv::Point> > rooms)
{
	double mean = 0.0;
	double dx, dy;
	double map_resoultion = 0.05;
	std::vector<cv::Point2f> centers;
	cv::RotatedRect current_bounding_box;
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		current_bounding_box = minAreaRect(rooms[current_room]);
		centers.push_back(current_bounding_box.center);
	}
	//calculate the sum of distances
	for (int current_center = 0; current_center < centers.size() - 1; current_center++)
	{
		dx = centers[current_center].x - centers[current_center + 1].x;
		dy = centers[current_center].y - centers[current_center + 1].y;
		mean += std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
	}
	return map_resoultion * (mean / centers.size());
}

//Calculate standard deviation of room-areas
double calc_area_deviation(std::vector<std::vector<cv::Point> > rooms)
{
	double sigma = 0.0;
	double mean = 0.0;
	std::vector<double> areas = calculate_areas(rooms);
	//calculate the average room-area
	for (int current_room = 0; current_room < areas.size(); current_room++) {
		mean += areas[current_room];
	}
	mean = mean / areas.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < areas.size(); current_room++) {
		sigma += std::pow(areas[current_room] - mean, 2.0);
	}
	sigma = sigma / (areas.size() - 1);
	return std::sqrt(sigma);
}

//Calculate standard deviation of room-perimeters
double calc_perimeter_deviation(std::vector<std::vector<cv::Point> > rooms)
{
	double sigma = 0.0;
	double mean = 0.0;
	std::vector<double> perimeters = calculate_perimeters(rooms);
	//calculate the average room-area
	for (int current_room = 0; current_room < perimeters.size();current_room++)
	{
		mean += perimeters[current_room];
	}
	mean = mean / perimeters.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < perimeters.size();current_room++)
	{
		sigma += std::pow(perimeters[current_room] - mean, 2.0);
	}
	sigma = sigma / (perimeters.size() - 1);
	return std::sqrt(sigma);
}

//Calculate standard deviation of ellipsis-quotients
double calc_quotients_deviation(std::vector<std::vector<cv::Point> > rooms)
{
	double sigma = 0.0;
	double mean = 0.0;
	std::vector<double> quotients = calc_Ellipse_axis(rooms);
	//calculate the average room-area
	for (int current_room = 0; current_room < quotients.size();
			current_room++) {
		mean += quotients[current_room];
	}
	mean = mean / quotients.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < quotients.size();
			current_room++) {
		sigma += std::pow(quotients[current_room] - mean, 2.0);
	}
	sigma = sigma / (quotients.size() - 1);
	return std::sqrt(sigma);
}

//Calculate standard deviation of bounding-box-errors
double calc_errors_deviation(std::vector<std::vector<cv::Point> > rooms)
{
	double sigma = 0.0;
	double mean = 0.0;
	std::vector<double> errors = calculate_bounding_error(rooms);
	//calculate the average room-area
	for (int current_room = 0; current_room < errors.size(); current_room++) {
		mean += errors[current_room];
	}
	mean = mean / errors.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < errors.size(); current_room++) {
		sigma += std::pow(errors[current_room] - mean, 2.0);
	}
	sigma = sigma / (errors.size() - 1);
	return std::sqrt(sigma);
}

int segmentationNameToNumber(const std::string name)
{
	if (name.compare("morphological") == 0)
		return 1;
	else if (name.compare("distance") == 0)
		return 2;
	else if (name.compare("voronoi") == 0)
		return 3;
	else if (name.compare("semantic") == 0)
		return 4;
	return 1;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "evaluation");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO("Evaluation of the segmented maps. Calculates some Parameters describing the rooms.");
//	ros::spin();

	double map_resolution = 0.0500;

	std::vector<std::string> segmentation_names;
//	segmentation_names.push_back("morphological");
//	segmentation_names.push_back("distance");
	segmentation_names.push_back("voronoi");
//	segmentation_names.push_back("semantic");

//	std::string map_name = "NLB";
////		"lab_ipa" //done
////		"lab_c_scan" //done
////		"Freiburg52_scan" //done
////		"Freiburg79_scan" //done
////		"lab_b_scan" //done
////		"lab_intel" //done
////		"Freiburg101_scan" //done
////		"lab_d_scan" //done
////		"lab_f_scan" //done
////		"lab_a_scan" //done
////		"NLB" //done
	std::vector< std::string > map_names;
//	map_names.push_back("lab_ipa.png");
	map_names.push_back("lab_c_scan.png");
//	map_names.push_back("Freiburg52_scan.png");
//	map_names.push_back("Freiburg79_scan.png");
//	map_names.push_back("lab_b_scan.png");
//	map_names.push_back("lab_intel.png");
//	map_names.push_back("Freiburg101_scan.png");
//	map_names.push_back("lab_d_scan.png");
	map_names.push_back("lab_f_scan.png");
//	map_names.push_back("lab_a_scan.png");
//	map_names.push_back("NLB.png");
//	map_names.push_back("lab_ipa_furnitures.png");
//	map_names.push_back("lab_c_scan_furnitures.png");
//	map_names.push_back("Freiburg52_scan_furnitures.png");
//	map_names.push_back("Freiburg79_scan_furnitures.png");
//	map_names.push_back("lab_b_scan_furnitures.png");
//	map_names.push_back("lab_intel_furnitures.png");
//	map_names.push_back("Freiburg101_scan_furnitures.png");
//	map_names.push_back("lab_d_scan_furnitures.png");
//	map_names.push_back("lab_f_scan_furnitures.png");
//	map_names.push_back("lab_a_scan_furnitures.png");
//	map_names.push_back("NLB_furnitures.png");

	//define vectors to save the parameters
	std::vector<double> av_compactness_vector(segmentation_names.size()), max_compactness_vector(segmentation_names.size()), min_compactness_vector(segmentation_names.size());
	std::vector<double> av_bb_vector(segmentation_names.size()), max_bb_vector(segmentation_names.size()), min_bb_vector(segmentation_names.size()), dev_bb_vector(segmentation_names.size());
	std::vector<double> av_area_vector(segmentation_names.size()), max_area_vector(segmentation_names.size()), min_area_vector(segmentation_names.size()), dev_area_vector(segmentation_names.size());
	std::vector<double> av_per_vector(segmentation_names.size()), max_per_vector(segmentation_names.size()), min_per_vector(segmentation_names.size()), dev_per_vector(segmentation_names.size());
	std::vector<bool> reachable(segmentation_names.size());
	std::vector<double> av_quo_vector(segmentation_names.size()), max_quo_vector(segmentation_names.size()), min_quo_vector(segmentation_names.size()), dev_quo_vector(segmentation_names.size());
	std::vector<int> segments_number_vector(segmentation_names.size());

	std::stringstream output;
	const std::string segmented_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/segmented_maps/";
	const std::string command = "mkdir -p " + segmented_map_path;
	int return_value = system(command.c_str());

	// loop through map files
	for (size_t image_index = 0; image_index<map_names.size(); ++image_index)
	{
		std::string map_name = map_names[image_index];
		std::string image_filename = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/" + map_name;
		std::cout << "map: " << image_filename << std::endl;
		cv::Mat map = cv::imread(image_filename.c_str(), 0);
		//make non-white pixels black
		for (int y = 0; y < map.rows; y++)
		{
			for (int x = 0; x < map.cols; x++)
			{
				//find not reachable regions and make them black
				if (map.at<unsigned char>(y, x) >= 250)
					map.at<unsigned char>(y, x) = 255;
				else
					map.at<unsigned char>(y, x) = 0;
			}
		}

		//calculate parameters for each segmentation and save it
		for (size_t segmentation_index = 0; segmentation_index < segmentation_names.size();++segmentation_index)
		{
			sensor_msgs::Image labeling;
			cv_bridge::CvImage cv_image;
		//	cv_image.header.stamp = ros::Time::now();
			cv_image.encoding = "mono8";
			cv_image.image = map;
			cv_image.toImageMsg(labeling);
			// create the action client --> "name of server"
			// true causes the client to spin its own thread
			actionlib::SimpleActionClient<ipa_room_segmentation::MapSegmentationAction> ac("/room_segmentation/room_segmentation_server", true);

			ROS_INFO("Waiting for action server to start.");
			// wait for the action server to start
			ac.waitForServer(); //will wait for infinite time

			ROS_INFO("Action server started, sending goal.");
			// send a goal to the action
			ipa_room_segmentation::MapSegmentationGoal goal;
			goal.input_map = labeling;
			goal.map_origin.position.x = 0;
			goal.map_origin.position.y = 0;
			goal.map_resolution = 0.05;
			goal.return_format_in_meter = false;
			goal.return_format_in_pixel = true;
			goal.room_segmentation_algorithm = segmentationNameToNumber(segmentation_names[segmentation_index]);
			ac.sendGoal(goal);

			//wait for the action to return
			bool finished_before_timeout = ac.waitForResult(ros::Duration(600.0));

			if (!finished_before_timeout)
				continue;

			ROS_INFO("Finished successfully!");
			ipa_room_segmentation::MapSegmentationResultConstPtr result = ac.getResult();
			cv_bridge::CvImagePtr cv_ptr_seq = cv_bridge::toCvCopy(result->segmented_map, sensor_msgs::image_encodings::TYPE_32SC1);
			cv::Mat segmented_map = cv_ptr_seq->image;

			std::string image_filename = segmented_map_path + map_name + "_segmented_" + segmentation_names[segmentation_index] + ".png";

			// images: segmented_map, sequence_map
			cv::Mat color_segmented_map = segmented_map.clone();
			color_segmented_map.convertTo(color_segmented_map, CV_8U);
			cv::cvtColor(color_segmented_map, color_segmented_map, CV_GRAY2BGR);
			for(size_t i = 1; i <= result->room_information_in_pixel.size(); ++i)
			{
				//choose random color for each room
				const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
				for(size_t v = 0; v < segmented_map.rows; ++v)
					for(size_t u = 0; u < segmented_map.cols; ++u)
						if(segmented_map.at<int>(v,u) == i)
							color_segmented_map.at<cv::Vec3b>(v,u) = color;
			}
			cv::imwrite(image_filename, color_segmented_map);

//			cv::Mat map = cv::imread(image_filename.c_str(), 0);
//			cv::imshow("segmented map", color_segmented_map);
//			cv::waitKey();

			// retrieve room contours
			cv::Mat temporary_map = segmented_map.clone();
			std::vector<std::vector<cv::Point> > contours, saved_contours;
			std::vector<cv::Vec4i> hierarchy;
			for(size_t i = 1; i <= result->room_information_in_pixel.size(); ++i)
			{
				cv::Mat saving_map = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_8UC1);
				for(size_t v = 0; v < segmented_map.rows; ++v)
					for(size_t u = 0; u < segmented_map.cols; ++u)
						if(segmented_map.at<int>(v,u) == i)
							saving_map.at<uchar>(v,u) = 255;
				cv::findContours(saving_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
				cv::drawContours(temporary_map, contours, -1, cv::Scalar(0), CV_FILLED);
				for (int c = 0; c < contours.size(); c++)
				{
					if (map_resolution * map_resolution * cv::contourArea(contours[c]) > 1.0)
					{
						saved_contours.push_back(contours[c]);
					}
				}
			}

			//compactnesses
			std::vector<double> compactness = calculate_compactness(saved_contours);
			double average = 0.0;
			double max_compactness = 0;
			double min_compactness = 100000000;
			for (int cmp = 0; cmp < compactness.size(); cmp++) {
				average += compactness[cmp];
				if (compactness[cmp] > max_compactness)
				{
					max_compactness = compactness[cmp];
				}
				if (compactness[cmp] < min_compactness)
				{
					min_compactness = compactness[cmp];
				}
			}
			average = average / compactness.size();
			av_compactness_vector[segmentation_index] = average;
			max_compactness_vector[segmentation_index] = max_compactness;
			min_compactness_vector[segmentation_index] = min_compactness;

			//Bounding Box
			std::vector<double> bounding_errors = calculate_bounding_error(saved_contours);
			average = 0.0;
			double max_error = 0;
			double min_error = 10000000;
			for (int e = 0; e < bounding_errors.size(); e++) {
				average += bounding_errors[e];
				if (bounding_errors[e] > max_error) {
					max_error = bounding_errors[e];
				}
				if (bounding_errors[e] < min_error) {
					min_error = bounding_errors[e];
				}
			}
			average = average / bounding_errors.size();
			av_bb_vector[segmentation_index] = average;
			max_bb_vector[segmentation_index] = max_error;
			min_bb_vector[segmentation_index] = min_error;
			dev_bb_vector[segmentation_index] = calc_errors_deviation(saved_contours);

			//area
			std::vector<double> areas = calculate_areas(saved_contours);
			average = 0.0;
			double max_area = 0.0;
			double min_area = 100000000;
			for (int e = 0; e < areas.size(); e++) {
				average += areas[e];
				if (areas[e] > max_area) {
					max_area = areas[e];
				}
				if (areas[e] < min_area) {
					min_area = areas[e];
				}
			}
			average = average / areas.size();
			av_area_vector[segmentation_index] = average;
			max_area_vector[segmentation_index] = max_area;
			min_area_vector[segmentation_index] = min_area;
			dev_area_vector[segmentation_index] = calc_area_deviation(saved_contours);

			//perimeters
			std::vector<double> perimeters = calculate_perimeters(saved_contours);
			average = 0.0;
			double max_per = 0.0;
			double min_per = 100000000;
			for (int e = 0; e < perimeters.size(); e++) {
				average += perimeters[e];
				if (perimeters[e] > max_per) {
					max_per = perimeters[e];
				}
				if (perimeters[e] < min_per) {
					min_per = perimeters[e];
				}
			}
			average = average / perimeters.size();
			av_per_vector[segmentation_index] = average;
			max_per_vector[segmentation_index] = max_per;
			min_per_vector[segmentation_index] = min_per;
			dev_per_vector[segmentation_index] = calc_perimeter_deviation(saved_contours);

			//reachability
			if (check_reachability(saved_contours, segmented_map))
			{
				reachable[segmentation_index] = true;
			}
			else
			{
				reachable[segmentation_index] = false;
			}

			//Quotient
			std::vector<double> quotients = calc_Ellipse_axis(saved_contours);
			average = 0.0;
			double max_quo = 0.0;
			double min_quo = 100000000;
			for (int e = 0; e < quotients.size(); e++) {
				average += quotients[e];
				if (quotients[e] > max_quo) {
					max_quo = quotients[e];
				}
				if (quotients[e] < min_quo) {
					min_quo = quotients[e];
				}
			}
			average = average / quotients.size();
			av_quo_vector[segmentation_index] = average;
			max_quo_vector[segmentation_index] = max_quo;
			min_quo_vector[segmentation_index] = min_quo;
			dev_quo_vector[segmentation_index] = calc_quotients_deviation(saved_contours);

			//number of segments
			segments_number_vector[segmentation_index] = saved_contours.size();
		}

		//write parameters into file
		output << "--------------Segmentierungsevaluierung----------------" << std::endl;
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << segmentation_names[i] << " & ";
		output << std::endl;
		output << "Kompaktheitsmaße: " << std::endl;
		output << "Durschnitt: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << av_compactness_vector[i] << " & ";
		output << std::endl;
		output << "Maximum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << max_compactness_vector[i] << " & ";
		output << std::endl;
		output << "Minimum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << min_compactness_vector[i] << " & ";
		output << std::endl;
		output << "****************************" << std::endl;

		output << "Überflüssige Fläche Bounding Box: " << std::endl;
		output << "Durchschnitt Bounding Fehler: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << av_bb_vector[i] << " & ";
		output << std::endl;
		output << "Maximum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << max_bb_vector[i] << " & ";
		output << std::endl;
		output << "Minimum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << min_bb_vector[i] << " & ";
		output << std::endl;
		output << "Standardabweichung: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << dev_bb_vector[i] << " & ";
		output << std::endl;
		output << "**************************************" << std::endl;

		output << "Flächenmaße: " << std::endl;
		output << "Durchschnitt: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << av_area_vector[i] << " & ";
		output << std::endl;
		output << "Maximum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << max_area_vector[i] << " & ";
		output << std::endl;
		output << "Minimum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << min_area_vector[i] << " & ";
		output << std::endl;
		output << "Standardabweichung: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << dev_area_vector[i] << " & ";
		output << std::endl;
		output << "**************************************" << std::endl;

		output << "Umfangsmaße: " << std::endl;
		output << "Durchschnitt: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << av_per_vector[i] << " & ";
		output << std::endl;
		output << "Maximum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << max_per_vector[i] << " & ";
		output << std::endl;
		output << "Minimum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << min_per_vector[i] << " & ";
		output << std::endl;
		output << "Standardabweichung: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << dev_per_vector[i] << " & ";
		output << std::endl;
		output << "**************************************" << std::endl;

		output << "Erreichbarkeit für alle Raumzentren: " << std::endl;
		for(size_t i = 0; i < segmentation_names.size(); ++i)
		{
			if(reachable[i] == true)
				output << "Alle Raumzentren erreichbar" << std::endl;
			else
				output << "Nicht alle erreichbar" << std::endl;
		}
		output << "****************************" << std::endl;

		output << "Quotienten der Ellipsenachsen: " << std::endl;
		output << "Durchschnitt: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << av_quo_vector[i] << " & ";
		output << std::endl;
		output << "Maximum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << max_quo_vector[i] << " & ";
		output << std::endl;
		output << "Minimum: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << min_quo_vector[i] << " & ";
		output << std::endl;
		output << "Standardabweichung: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << dev_quo_vector[i] << " & ";
		output << std::endl;
		output << "****************************" << std::endl;

		output << "Anzahl Räume: ";
		for(size_t i = 0; i < segmentation_names.size(); ++i)
			output << segments_number_vector[i] << " & ";
		output << std::endl;

		std::string log_filename = segmented_map_path + map_name + "_evaluation.txt";
		std::ofstream file(log_filename.c_str(), std::ios::out);
		if (file.is_open() == true)
			file << output.str();
		file.close();
	}

	return 0;
}

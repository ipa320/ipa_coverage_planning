#include "ros/ros.h"
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
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
bool check_reachability(std::vector<std::vector<cv::Point> > rooms, cv::Mat map)
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


int main(int argc, char **argv) {
	ros::init(argc, argv, "evaluation");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO(
			"Evaluation of the segmented maps. Calculates some Parameters describing the rooms.");
//	ros::spin();

	double map_resolution = 0.0500;

	std::vector<std::string> segmentation_names;
	segmentation_names.push_back("morphological");
	segmentation_names.push_back("distance");
	segmentation_names.push_back("voronoi");
	segmentation_names.push_back("semantic");

	std::string map_name = "lab_ipa";
//		"lab_ipa" //done
//		"lab_c_scan" //done
//		"Freiburg52_scan" //done
//		"Freiburg79_scan" //done
//		"lab_b_scan" //done
//		"lab_intel" //done
//		"Freiburg101_scan" //done
//		"lab_d_scan" //done
//		"lab_f_scan" //done
//		"lab_a_scan" //done
//		"NLB" //done

	//define vectors to save the parameters
	std::vector<double> av_compactness_vector(4), max_compactness_vector(4), min_compactness_vector(4);
	std::vector<double> av_bb_vector(4), max_bb_vector(4), min_bb_vector(4), dev_bb_vector(4);
	std::vector<double> av_area_vector(4), max_area_vector(4), min_area_vector(4), dev_area_vector(4);
	std::vector<double> av_per_vector(4), max_per_vector(4), min_per_vector(4), dev_per_vector(4);
	std::vector<bool> reachable(4);
	std::vector<double> av_quo_vector(4), max_quo_vector(4), min_quo_vector(4), dev_quo_vector(4);
	std::vector<int> segments_number_vector(4);

	std::stringstream output;
	const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/segmented_maps/";
	const std::string command = "mkdir -p " + test_map_path;
	int return_value = system(command.c_str());

	//calculate parameters for each segmentation and save it
	for (size_t map_index = 0; map_index < segmentation_names.size();++map_index)
	{
		std::string image_filename = test_map_path + map_name + "_segmented_" + segmentation_names[map_index] + ".png";

		cv::Mat map = cv::imread(image_filename.c_str(), 0);
		cv::imshow("loaded map", map);
		cv::waitKey();
		cv::Mat temporary_map = map.clone();
		cv::Mat saving_map;
		std::vector<std::vector<cv::Point> > contours, saved_contours;
		std::vector<cv::Vec4i> hierarchy;
		for (int t = 255; t > 0; t--)
		{
			cv::threshold(temporary_map, saving_map, t, 254, cv::THRESH_BINARY);
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
		av_compactness_vector[map_index] = average;
		max_compactness_vector[map_index] = max_compactness;
		min_compactness_vector[map_index] = min_compactness;

		//Bounding Box
		std::vector<double> bounding_errors = calculate_bounding_error(
				saved_contours);
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
		av_bb_vector[map_index] = average;
		max_bb_vector[map_index] = max_error;
		min_bb_vector[map_index] = min_error;
		dev_bb_vector[map_index] = calc_errors_deviation(saved_contours);

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
		av_area_vector[map_index] = average;
		max_area_vector[map_index] = max_area;
		min_area_vector[map_index] = min_area;
		dev_area_vector[map_index] = calc_area_deviation(saved_contours);

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
		av_per_vector[map_index] = average;
		max_per_vector[map_index] = max_per;
		min_per_vector[map_index] = min_per;
		dev_per_vector[map_index] = calc_perimeter_deviation(saved_contours);

		//reachability
		if (check_reachability(saved_contours, map))
		{
			reachable[map_index] = true;
		}
		else
		{
			reachable[map_index] = false;
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
		av_quo_vector[map_index] = average;
		max_quo_vector[map_index] = max_quo;
		min_quo_vector[map_index] = min_quo;
		dev_quo_vector[map_index] = calc_quotients_deviation(saved_contours);

		//number of segments
		segments_number_vector[map_index] = saved_contours.size();
	}

	//write parameters into file
	output << "--------------Segmentierungsevaluierung----------------" << std::endl;
	for(size_t i = 0; i < 4; ++i)
		output << segmentation_names[i] << " & ";
	output << std::endl;
	output << "Kompaktheitsmaße: " << std::endl;
	output << "Durschnitt: ";
	for(size_t i = 0; i < 4; ++i)
		output << av_compactness_vector[i] << " & ";
	output << std::endl;
	output << "Maximum: ";
	for(size_t i = 0; i < 4; ++i)
		output << max_compactness_vector[i] << " & ";
	output << std::endl;
	output << "Minimum: ";
	for(size_t i = 0; i < 4; ++i)
		output << min_compactness_vector[i] << " & ";
	output << std::endl;
	output << "****************************" << std::endl;

	output << "Überflüssige Fläche Bounding Box: " << std::endl;
	output << "Durchschnitt Bounding Fehler: ";
	for(size_t i = 0; i < 4; ++i)
		output << av_bb_vector[i] << " & ";
	output << std::endl;
	output << "Maximum: ";
	for(size_t i = 0; i < 4; ++i)
		output << max_bb_vector[i] << " & ";
	output << std::endl;
	output << "Minimum: ";
	for(size_t i = 0; i < 4; ++i)
		output << min_bb_vector[i] << " & ";
	output << std::endl;
	output << "Standardabweichung: ";
	for(size_t i = 0; i < 4; ++i)
		output << dev_bb_vector[i] << " & ";
	output << std::endl;
	output << "**************************************" << std::endl;

	output << "Flächenmaße: " << std::endl;
	output << "Durchschnitt: ";
	for(size_t i = 0; i < 4; ++i)
		output << av_area_vector[i] << " & ";
	output << std::endl;
	output << "Maximum: ";
	for(size_t i = 0; i < 4; ++i)
		output << max_area_vector[i] << " & ";
	output << std::endl;
	output << "Minimum: ";
	for(size_t i = 0; i < 4; ++i)
		output << min_area_vector[i] << " & ";
	output << std::endl;
	output << "Standardabweichung: ";
	for(size_t i = 0; i < 4; ++i)
		output << dev_area_vector[i] << " & ";
	output << std::endl;
	output << "**************************************" << std::endl;

	output << "Umfangsmaße: " << std::endl;
	output << "Durchschnitt: ";
	for(size_t i = 0; i < 4; ++i)
		output << av_per_vector[i] << " & ";
	output << std::endl;
	output << "Maximum: ";
	for(size_t i = 0; i < 4; ++i)
		output << max_per_vector[i] << " & ";
	output << std::endl;
	output << "Minimum: ";
	for(size_t i = 0; i < 4; ++i)
		output << min_per_vector[i] << " & ";
	output << std::endl;
	output << "Standardabweichung: ";
	for(size_t i = 0; i < 4; ++i)
		output << dev_per_vector[i] << " & ";
	output << std::endl;
	output << "**************************************" << std::endl;

	output << "Erreichbarkeit für alle Raumzentren: " << std::endl;
	for(size_t i = 0; i < 4; ++i)
	{
		if(reachable[i] == true)
			output << "Alle Raumzentren erreichbar" << std::endl;
		else
			output << "Nicht alle erreichbar" << std::endl;
	}
	output << "****************************" << std::endl;

	output << "Quotienten der Ellipsenachsen: " << std::endl;
	output << "Durchschnitt: ";
	for(size_t i = 0; i < 4; ++i)
		output << av_quo_vector[i] << " & ";
	output << std::endl;
	output << "Maximum: ";
	for(size_t i = 0; i < 4; ++i)
		output << max_quo_vector[i] << " & ";
	output << std::endl;
	output << "Minimum: ";
	for(size_t i = 0; i < 4; ++i)
		output << min_quo_vector[i] << " & ";
	output << std::endl;
	output << "Standardabweichung: ";
	for(size_t i = 0; i < 4; ++i)
		output << dev_quo_vector[i] << " & ";
	output << std::endl;
	output << "****************************" << std::endl;

	output << "Anzahl Räume: ";
	for(size_t i = 0; i < 4; ++i)
		output << segments_number_vector[i] << " & ";
	output << std::endl;

	std::string log_filename = test_map_path + map_name + "_evaluation.txt";
	std::ofstream file(log_filename.c_str(), std::ios::out);
	if (file.is_open() == true)
		file << output.str();
	file.close();

	return 0;
}

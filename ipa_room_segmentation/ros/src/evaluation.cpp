#include "ros/ros.h"
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
		current_perimeter = cv::arcLength(rooms[current_room], true);
		compactness_factors.push_back(current_area / current_perimeter);
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
	for (int current_room = 0; current_room < rooms.size(); current_room++)
	{
		current_bounding_box = minAreaRect(rooms[current_room]);
		bounding_box_area = current_bounding_box.size.area();
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
		calculated_perimeters.push_back(cv::arcLength(rooms[current_room], true));
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
	std::vector < cv::Point2f > edge_points;
	double distance = 0;
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
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) > distance)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2));
			}
		}
	}
	return distance;
}

//Calculate the length of the minor axis of the minimum bounding ellipse
double calc_minor_axis(std::vector<cv::Point> room)
{
	cv::Point2f points[4];
	std::vector < cv::Point2f > edge_points;
	double distance = 10000000;
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
			if (std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2)) < distance && np != p)
			{
				distance = std::sqrt(std::pow((edge_points[p].x - edge_points[np].x), 2) + std::pow((edge_points[p].y - edge_points[np].y), 2));
			}
		}
	}
	return distance;
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
	std::vector < cv::Point2f > centers;
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
	return (mean / centers.size());
}

//Calculate standard deviation of room-areas
double calc_area_deviation(std::vector<std::vector<cv::Point> > rooms)
{
	double sigma = 0.0;
	double mean = 0.0;
	std::vector<double> areas = calculate_areas(rooms);
	//calculate the average room-area
	for (int current_room = 0; current_room < areas.size(); current_room++)
	{
		mean += areas[current_room];
	}
	mean = mean / areas.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < areas.size(); current_room++)
	{
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
	for (int current_room = 0; current_room < perimeters.size(); current_room++)
	{
		mean += perimeters[current_room];
	}
	mean = mean / perimeters.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < perimeters.size(); current_room++)
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
	for (int current_room = 0; current_room < quotients.size(); current_room++)
	{
		mean += quotients[current_room];
	}
	mean = mean / quotients.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < quotients.size(); current_room++)
	{
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
	for (int current_room = 0; current_room < errors.size(); current_room++)
	{
		mean += errors[current_room];
	}
	mean = mean / errors.size();
	//calculate the standard deviation
	for (int current_room = 0; current_room < errors.size(); current_room++)
	{
		sigma += std::pow(errors[current_room] - mean, 2.0);
	}
	sigma = sigma / (errors.size() - 1);
	return std::sqrt(sigma);
}

//Calculate number of sides of the polygon
int calc_sides_number(std::vector<std::vector<cv::Point> > rooms)
{
	int average = 0;
	for (int r = 0; r < rooms.size(); r++)
	{
		average += rooms[r].size() - 1;
	}
	return (average / rooms.size());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "evaluation");
	ros::NodeHandle n;
//	ros::Subscriber semantic_labeler = n.Subscribe("Laser_scanner", 1000, segmentation_algorithm);
	ROS_INFO("Evaluation of the segmented maps. Calculates some Parameters describing the rooms.");
//	ros::spin();
	std::vector<std::vector<cv::Point> > contours, saved_contours;
	std::vector < cv::Vec4i > hierarchy;
	cv::Mat map = cv::imread("/home/rmb-fj/Pictures/maps/morphological/outfilled_map.png", 0);
	cv::Mat temporary_map = map.clone();
	cv::Mat saving_map;
	for (int t = 255; t > 0; t--)
	{
		cv::threshold(temporary_map, saving_map, t, 254, cv::THRESH_BINARY);
		cv::findContours(saving_map, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
		cv::drawContours(temporary_map, contours, -1, cv::Scalar(0), CV_FILLED);
		for (int c = 0; c < contours.size(); c++)
		{
			double map_resolution = 0.0500;
			if (map_resolution * map_resolution * cv::contourArea(contours[c]) > 1.0)
			{
				saved_contours.push_back(contours[c]);
			}
		}
	}

//	//Zeitmessung:
//	#include <ctime>
//	std::time_t start_t, ende_t;
//	float sek_t;
//	std::time(&start_t);
//
//	std::time(&ende_t);
//	sek_t = (float) (ende_t - start_t);
//
//	std::cout << "Berechnungszeit: " << sek_t << "s" << std::endl;

	std::cout << "Kompaktheitsmaße: " << std::endl;
	std::vector<double> compactness = calculate_compactness(saved_contours);
	double average = 0.0;
	double max_compactness = 0;
	double min_compactness = 100000000;
	for (int cmp = 0; cmp < compactness.size(); cmp++)
	{
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
	average = average / saved_contours.size();
	std::cout << "Durschnitt: " << average << std::endl;
	std::cout << "Maximum: " << max_compactness << std::endl;
	std::cout << "Minimum: " << min_compactness << std::endl;
	std::cout << "****************************" << std::endl;
	std::cout << "Überflüssige Fläche Bounding Box: " << std::endl;
	std::vector<double> bounding_errors = calculate_bounding_error(saved_contours);
	average = 0.0;
	double max_error = 0;
	double min_error = 10000000;
	for (int e = 0; e < bounding_errors.size(); e++)
	{
		average += bounding_errors[e];
		if (bounding_errors[e] > max_error)
		{
			max_error = bounding_errors[e];
		}
		if (bounding_errors[e] < min_error)
		{
			min_error = bounding_errors[e];
		}
	}
	average = average / bounding_errors.size();
	std::cout << "Durchschnitt Bounding Fehler: " << average << std::endl;
	std::cout << "Maximum: " << max_error << std::endl;
	std::cout << "Minimum: " << min_error << std::endl;
	std::cout << "Standardabweichung: " << calc_errors_deviation(saved_contours) << std::endl;
	std::cout << "**************************************" << std::endl;
	std::cout << "Flächenmaße: " << std::endl;
	std::vector<double> areas = calculate_areas(saved_contours);
	average = 0.0;
	double max_area = 0.0;
	double min_area = 100000000;
	for (int e = 0; e < areas.size(); e++)
	{
		average += areas[e];
		if (areas[e] > max_area)
		{
			max_area = areas[e];
		}
		if (areas[e] < min_area)
		{
			min_area = areas[e];
		}
	}
	average = average / areas.size();
	std::cout << "Durchschnitt: " << average << std::endl;
	std::cout << "Maximum: " << max_area << std::endl;
	std::cout << "Minimum: " << min_area << std::endl;
	std::cout << "Standardabweichung: " << calc_area_deviation(saved_contours) << std::endl;
	std::cout << "**************************************" << std::endl;
	std::cout << "Umfangsmaße: " << std::endl;
	std::vector<double> perimeters = calculate_perimeters(saved_contours);
	average = 0.0;
	double max_per = 0.0;
	double min_per = 100000000;
	for (int e = 0; e < perimeters.size(); e++)
	{
		average += perimeters[e];
		if (perimeters[e] > max_per)
		{
			max_per = perimeters[e];
		}
		if (perimeters[e] < min_per)
		{
			min_per = perimeters[e];
		}
	}
	average = average / perimeters.size();
	std::cout << "Durchschnitt: " << average << std::endl;
	std::cout << "Maximum: " << max_per << std::endl;
	std::cout << "Minimum: " << min_per << std::endl;
	std::cout << "Standardabweichung: " << calc_perimeter_deviation(saved_contours) << std::endl;
	std::cout << "**************************************" << std::endl;
	std::cout << "Erreichbarkeit für alle Raumzentren: " << std::endl;
	if (check_reachability(saved_contours, map))
	{
		std::cout << "Alle Raumzentren erreichbar" << std::endl;
	}
	else
	{
		std::cout << "Nicht alle erreichbar" << std::endl;
	}
	std::cout << "****************************" << std::endl;
	std::cout << "Quotienten der Ellipsenachsen: " << std::endl;
	std::vector<double> quotients = calc_Ellipse_axis(saved_contours);
	average = 0.0;
	double max_quo = 0.0;
	double min_quo = 100000000;
	for (int e = 0; e < quotients.size(); e++)
	{
		average += quotients[e];
		if (quotients[e] > max_quo)
		{
			max_quo = quotients[e];
		}
		if (quotients[e] < min_quo)
		{
			min_quo = quotients[e];
		}
	}
	average = average / quotients.size();
	std::cout << "Durchschnitt: " << average << std::endl;
	std::cout << "Maximum: " << max_quo << std::endl;
	std::cout << "Minimum: " << min_quo << std::endl;
	std::cout << "Standardabweichung: " << calc_quotients_deviation(saved_contours) << std::endl;
	std::cout << "****************************" << std::endl;
	std::cout << "Durchschnittlicher Abstand der Raumzentren: " << calc_average_distance(saved_contours) << std::endl;
	std::cout << "***********************************" << std::endl;
	std::cout << "Durchschnittliche Anzahl an Seiten der Räume: " << calc_sides_number(saved_contours) << std::endl;
	std::cout << "***************************************** " << std::endl;
	std::cout << "Anzahl Räume: " << saved_contours.size() << std::endl;

	return 0;
}

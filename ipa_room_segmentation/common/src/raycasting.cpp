#include <ipa_room_segmentation/raycasting.h>

std::vector<double> raycasting(const cv::Mat& map, const cv::Point& location)
{
//	cv::Mat test_map = map.clone();
	//Raycasting Algorithm. It simulates the laser measurment at the given location and returns the lengths
	//of the simulated beams
	double simulated_x, simulated_y, simulated_cos, simulated_sin;
	double temporary_distance;
	std::vector<double> distances(360, 0);
	double delta_x, delta_y;
	double pi_to_rad = PI / 180;
	for (double angle = 0; angle < 360; angle++)
	{
		simulated_cos = std::cos(angle * pi_to_rad);
		simulated_sin = std::sin(angle * pi_to_rad);
		temporary_distance = 90000001;
		for (double distance = 0; distance < 1000000; ++distance)
		{
			simulated_x = simulated_cos * distance;
			simulated_y = simulated_sin * distance;
			//make sure the simulated Point isn't out of the boundaries of the map
			if (location.x + simulated_x > 0 && location.x + simulated_x < map.rows && location.y + simulated_y > 0 && location.y + simulated_y < map.cols)
			{
				if (map.at<unsigned char>(location.x + simulated_x, location.y + simulated_y) == 0 && distance < temporary_distance)
				{
					temporary_distance = distance;
//					cv::line(test_map, cv::Point(location.y, location.x), cv::Point(location.y + simulated_y, location.x + simulated_x), cv::Scalar(127), 1);
					break;
				}
			}
		}
		if (temporary_distance > 90000000)
		{
			temporary_distance = 10;
		}
		distances[angle] = temporary_distance;
	}

//	cv::circle(test_map, cv::Point(location.y, location.x), 3, cv::Scalar(50), CV_FILLED);
//	cv::imshow("simulated angles", test_map);
//	cv::waitKey(5000);

	return distances;
}

std::vector<double> bresenham_raycasting(const cv::Mat& map, const cv::Point& location)
{
	std::vector<double> beams(360);
	int x_destination, y_destination;
	int double_x, double_y;
	int dx, dy, xstep, ystep;
	int x_current, y_current;
	int x_start = location.x;
	int y_start = location.y;
	int error, previous_error;
	bool hit_black_pixel;
	//go trough every angle from 0:1:359
	for (double angle = 0; angle < 360; angle++)
	{
		//set destination Point and difference of the coordinates
		x_destination = x_start + std::cos(angle * PI / 180) * 1000;
		y_destination = y_start + std::sin(angle * PI / 180) * 1000;
		dx = x_destination - x_start;
		dy = y_destination - y_start;
		//reset the went distance
		hit_black_pixel = false;
		x_current = 0;
		y_current = 0;
		//check for quadrant in which the line goes
		if (dy < 0)
		{
			ystep = -1;
			dy = -dy;
		}
		else
		{
			ystep = 1;
		}

		if (dx < 0)
		{
			xstep = -1;
			dx = -dx;
		}
		else
		{
			xstep = 1;
		}
		//set the doubled differences
		double_x = 2 * dx;
		double_y = 2 * dy;
		if (double_x >= double_y) //first octant (0 <= slope <= 1) --> favour x
		{
			error = dx;
			previous_error = error;
			//go in direction of the current angle
			do
			{
				x_current += xstep;
				error += double_y;
				if (error > double_x)
				{
					y_current += ystep;
					error -= double_x;
					//check Pixels that vary from bresenham line --> supercover line
					if (error + previous_error < double_x)
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_x)
					{
						if (map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0
						        || map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(x_start + x_current, y_start + y_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			beams[angle] = std::sqrt(std::pow(x_current, 2.0) + std::pow(y_current, 2.0));
		}
		else // favour y
		{
			error = dy;
			previous_error = error;
			//go in direction of the current angle
			do
			{
				y_current += ystep;
				error += double_x;
				if (error > double_y)
				{
					x_current += xstep;
					error -= double_y;
					//check Pixels that vary from bresenham line --> supercover line
					if (error + previous_error < double_y)
					{
						if (map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_y)
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(x_start + x_current, y_start + y_current - ystep) == 0
						        || map.at<unsigned char>(x_start + x_current - xstep, y_start + y_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(x_start + x_current, y_start + y_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			beams[angle] = std::sqrt(std::pow(x_current, 2.0) + std::pow(y_current, 2.0));
		}
	}
	return beams;
}

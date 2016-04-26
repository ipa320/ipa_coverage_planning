#include <ipa_room_segmentation/raycasting.h>

LaserScannerRaycasting::LaserScannerRaycasting()
: precomputed_cos_(360), precomputed_sin_(360)
{
	double pi_to_rad = PI / 180.;
	for (int angle = 0; angle < 360; angle++)
	{
		precomputed_cos_[angle] = std::cos(angle * pi_to_rad);
		precomputed_sin_[angle] = std::sin(angle * pi_to_rad);
	}
}

void LaserScannerRaycasting::raycasting(const cv::Mat& map, const cv::Point& location, std::vector<double>& distances)
{
//	cv::Mat test_map = map.clone();
	//Raycasting Algorithm. It simulates the laser measurment at the given location and returns the lengths
	//of the simulated beams
	double simulated_y, simulated_x, simulated_cos, simulated_sin;
	double temporary_distance;
	distances.resize(360, 0);
	double delta_y, delta_x;
	for (int angle = 0; angle < 360; angle++)
	{
		simulated_cos = precomputed_cos_[angle];
		simulated_sin = precomputed_sin_[angle];
		temporary_distance = 90000001;
		for (double distance = 1; distance < 1000000; ++distance)
		{
			const int ny = location.y + simulated_sin * distance;
			const int nx = location.x + simulated_cos * distance;
			//make sure the simulated point isn't out of the boundaries of the map
			if (ny < 0 || ny >= map.rows || nx < 0 || nx >= map.cols)
				break;
			if (map.at<unsigned char>(ny, nx) == 0 && distance < temporary_distance)
			{
				temporary_distance = distance;
//				cv::line(test_map, cv::Point(location.x, location.y), cv::Point(nx, ny), cv::Scalar(127), 1);
				break;
			}
		}
		if (temporary_distance > 90000000)
		{
			temporary_distance = 10;
		}
		distances[angle] = temporary_distance;
	}

//	cv::circle(test_map, cv::Point(location.x, location.y), 3, cv::Scalar(50), CV_FILLED);
//	cv::imshow("simulated angles", test_map);
//	cv::waitKey();
//	return distances;
}

void LaserScannerRaycasting::bresenham_raycasting(const cv::Mat& map, const cv::Point& location, std::vector<double>& distances)
{
	distances.resize(360);
	int y_destination, x_destination;
	int double_y, double_x;
	int dy, dx, ystep, xstep;
	int y_current, x_current;
	int y_start = location.y;
	int x_start = location.x;
	int error, previous_error;
	bool hit_black_pixel;
	//go trough every angle from 0:1:359
	for (int angle = 0; angle < 360; angle++)
	{
		//set destination Point and difference of the coordinates
		y_destination = y_start + std::sin((double)angle * PI / 180.) * 1000;
		x_destination = x_start + std::cos((double)angle * PI / 180.) * 1000;
		dy = y_destination - y_start;
		dx = x_destination - x_start;
		//reset the went distance
		hit_black_pixel = false;
		y_current = 0;
		x_current = 0;
		//check for quadrant in which the line goes
		if (dx < 0)
		{
			xstep = -1;
			dx = -dx;
		}
		else
		{
			xstep = 1;
		}

		if (dy < 0)
		{
			ystep = -1;
			dy = -dy;
		}
		else
		{
			ystep = 1;
		}
		//set the doubled differences
		double_y = 2 * dy;
		double_x = 2 * dx;
		if (double_y >= double_x) //first octant (0 <= slope <= 1) --> favour y
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
						if (map.at<unsigned char>(y_start + y_current, x_start + x_current - xstep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_y)
					{
						if (map.at<unsigned char>(y_start + y_current - ystep, x_start + x_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(y_start + y_current, x_start + x_current - xstep) == 0
						        || map.at<unsigned char>(y_start + y_current - ystep, x_start + x_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(y_start + y_current, x_start + x_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			distances[angle] = std::sqrt(std::pow(y_current, 2.0) + std::pow(x_current, 2.0));
		}
		else // favour x
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
						if (map.at<unsigned char>(y_start + y_current - ystep, x_start + x_current) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else if (error + previous_error > double_x)
					{
						if (map.at<unsigned char>(y_start + y_current, x_start + x_current - xstep) == 0)
						{
							hit_black_pixel = true;
						}
					}
					else
					{
						if (map.at<unsigned char>(y_start + y_current, x_start + x_current - xstep) == 0
						        || map.at<unsigned char>(y_start + y_current - ystep, x_start + x_current))
						{
							hit_black_pixel = true;
						}
					}
				}
				//check if next Pixel is a black Pixel
				if (map.at<unsigned char>(y_start + y_current, x_start + x_current) == 0)
				{
					hit_black_pixel = true;
				}
				previous_error = error;
			} while (!hit_black_pixel);
			distances[angle] = std::sqrt(std::pow(y_current, 2.0) + std::pow(x_current, 2.0));
		}
	}
}

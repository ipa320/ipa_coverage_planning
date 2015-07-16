#include <ipa_room_segmentation/raycasting.h>

std::vector<double> raycasting(cv::Mat map, cv::Point location)
{
	//Raycasting Algorithm. It simulates the laser measurment at the given location and returns the lengths
	//of the simulated beams
	double simulated_x, simulated_y, simulated_cos, simulated_sin;
	double temporary_distance;
	std::vector<double> distances;
	double delta_x, delta_y;
	for (double angle = 0; angle < 360; angle++)
	{
		simulated_cos = std::cos(angle * PI / 180);
		simulated_sin = std::sin(angle * PI / 180);
		temporary_distance = 90000001;
		for (double distance = 0; distance < 1000000; distance++)
		{
			simulated_x = simulated_cos * distance;
			simulated_y = simulated_sin * distance;
			//make sure the simulated Point isn't out of the boundaries of the map
			if (location.x + simulated_x > 0 && location.x + simulated_x < map.rows && location.y + simulated_y > 0 && location.y + simulated_y < map.cols)
			{
				if (map.at<unsigned char>(location.x + simulated_x, location.y + simulated_y) == 0 && distance < temporary_distance)
				{
					temporary_distance = distance;
					break;
				}
			}
		}
		if (temporary_distance > 90000000)
		{
			temporary_distance = 10;
		}
		distances.push_back(temporary_distance);
	}
	return distances;
}

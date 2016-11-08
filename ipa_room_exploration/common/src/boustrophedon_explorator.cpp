#include <ipa_room_exploration/boustrophedon_explorator.h>

// Constructor
boustrophedonExplorer::boustrophedonExplorer()
{

}

// Function that creates a room exploration path for the given map, by using the morse cellular decomposition method proposed in
//
// "H. Choset, E. Acar, A. A. Rizzi and J. Luntz,
// "Exact cellular decompositions in terms of critical points of Morse functions," Robotics and Automation, 2000. Proceedings.
// ICRA '00. IEEE International Conference on, San Francisco, CA, 2000, pp. 2270-2277 vol.3."
//
// This method takes the given map and separates it into several cells. Each cell is obstacle free and so allows an
// easier path planning. For each cell then a boustrophedon path is planned, which goes up, down and parallel to the
// upper and lower boundaries of the cell, see the referenced paper for details. This function does the following steps:
//	I.	Sweep a slice (a morse function) trough the given map and check for connectivity of this line,
//		i.e. how many connected segments there are. If the connectivity increases, i.e. more segments appear,
//		an IN event occurs that opens new separate cells, if it decreases, i.e. segments merge, an OUT event occurs that
//		merges two cells together. If an event occurs, the algorithm checks along the current line for critical points,
//		that are points that trigger the events. From these the boundary of the cells are drawn, starting from the CP
//		and going left/right until a black pixel is hit.
//	II.	After all cells have been determined by the sweeping slice, the algorithm finds these by using cv::findContours().
//		This gives a set of points for each cell, that are used to create a generalizedPolygon out of each cell.
void boustrophedonExplorer::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float robot_radius,
		const float map_resolution, const geometry_msgs::Pose2D starting_position,
		const geometry_msgs::Polygon room_min_max_coordinates, const cv::Point2d map_origin,
		const float fow_fitting_circle_radius, const float min_resample_dist)
{
	// *********************** I. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// create a map copy to mark the cell boundaries
	cv::Mat cell_map = room_map.clone();

	// find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
	size_t y_start = 0;
	int n_start = 0;
	bool found = false, obstacle = false;
	for(size_t y=0; y<room_map.cols; ++y)
	{
		for(size_t x=0; x<room_map.rows; ++x)
		{
			if(room_map.at<uchar>(x,y) == 255 && found == false)
			{
				y_start = y;
				found = true;
			}
			else if(found == true && obstacle == false && room_map.at<uchar>(x,y) == 0)
			{
				++n_start;
				obstacle = true;
			}
			else if(found == true && obstacle == true && room_map.at<uchar>(x,y) == 255)
			{
				obstacle = false;
			}
		}

		if(found == true)
			break;
	}

	// swipe trough the map and detect critical points
	int previous_number_of_segments = n_start;
	for(size_t y=y_start+1; y<room_map.cols; ++y) // start at y_start+1 because we know number of segments at y_start
	{
		int number_of_segments = 0; // int to count how many segments at the current slice are
		bool obstacle_hit = false; // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
		bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel
		for(size_t x=0; x<room_map.rows; ++x)
		{
			if(room_map.at<uchar>(x,y) == 255)
				hit_white_pixel = true;

			else if(hit_white_pixel == true)
			{
				if(obstacle_hit == false && room_map.at<uchar>(x,y) == 0) // check for event
				{
					++number_of_segments;
					obstacle_hit = true;
				}
				else if(obstacle_hit == true && room_map.at<uchar>(x,y) == 255) // check for leaving obstacle
				{
					obstacle_hit = false;
				}
			}
		}

		// reset hit_white_pixel to use this Boolean later
		hit_white_pixel = false;

		// check if number of segments has changed --> event occurred
		if(previous_number_of_segments < number_of_segments) // IN event
		{
			// check the current slice again for critical points
			for(size_t x=0; x < room_map.rows; ++x)
			{
				if(room_map.at<uchar>(x,y) == 255)
					hit_white_pixel = true;

				else if(hit_white_pixel == true && room_map.at<uchar>(x,y) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(x+dx, y-1) == 0)
							critical_point = false;

					// TODO: if critical point mark the separation
				}
			}
		}
	}
}

#include <ipa_building_navigation/A_star_pathplanner.h>

const int dir = 8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir] =
{ 1, 1, 0, -1, -1, -1, 0, 1 };
static int dy[dir] =
{ 0, 1, 1, 1, 0, -1, -1, -1 };

static int expanding_counter = 0;

// Determine priority (in the priority queue)
bool operator<(const nodeAstar& a, const nodeAstar& b)
{
	return a.getPriority() > b.getPriority();
}

AStarPlanner::AStarPlanner()
{
	n = 1;
	m = 1;
}

void AStarPlanner::downsampleMap(const cv::Mat& map, cv::Mat& downsampled_map, const double downsampling_factor, const double robot_radius, const double map_resolution)
{
	//erode the map so the planner doesn't go near the walls
	//	--> calculate the number of times for eroding from Robot Radius [m]
	cv::Mat eroded_map;
	int number_of_erosions = (robot_radius / map_resolution);
	cv::erode(map, eroded_map, cv::Mat(), cv::Point(-1, -1), number_of_erosions);
	//downsampling of the map to reduce calculationtime
	if (downsampling_factor != 1.)
		cv::resize(eroded_map, downsampled_map, cv::Size(0, 0), downsampling_factor, downsampling_factor, cv::INTER_LINEAR);
	else
		downsampled_map = eroded_map;
}

// A-star algorithm.
// The route returned is a string of direction digits.
std::string AStarPlanner::pathFind(const int & xStart, const int & yStart, const int & xFinish, const int & yFinish, const cv::Mat& map)
{
	static std::priority_queue<nodeAstar> pq[2]; // list of open (not-yet-tried) nodes
	static int pqi; // pq index
	static nodeAstar* n0;
	static nodeAstar* m0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;

	cv::Mat map_to_calculate_path(cv::Size(m, n), CV_32S);

	// create map from the given eroded map
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			if (map.at<unsigned char>(y, x) == 255)
			{
				map_to_calculate_path.at<int>(x, y) = 0;
			}
			else
			{
				map_to_calculate_path.at<int>(x, y) = 1;
			}
		}
	}

	cv::Mat closed_nodes_map(cv::Size(m, n), CV_32S); //map of already tried nodes
	cv::Mat open_nodes_map(cv::Size(m, n), CV_32S); // map of open (not-yet-tried) nodes
	cv::Mat dir_map(cv::Size(m, n), CV_32S); // map of directions

	// initialize the node maps
	for (y = 0; y < closed_nodes_map.rows; y++)
	{
		for (x = 0; x < closed_nodes_map.cols; x++)
		{
			closed_nodes_map.at<int>(y, x) = 0;
			open_nodes_map.at<int>(y, x) = 0;
		}
	}

	// create the start node and push into list of open nodes
	n0 = new nodeAstar(xStart, yStart, 0, 0);
	n0->updatePriority(xFinish, yFinish);
	pq[pqi].push(*n0);
	open_nodes_map.at<int>(xStart, yStart) = n0->getPriority(); // mark it on the open nodes map

	//garbage collection
	delete n0;

	// A* search
	while (!pq[pqi].empty())
	{
		// get the current node w/ the highest priority
		// from the list of open nodes
		n0 = new nodeAstar(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

		x = n0->getxPos();
		y = n0->getyPos();

		pq[pqi].pop(); // remove the node from the open list
		open_nodes_map.at<int>(x, y) = 0;
		// mark it on the closed nodes map
		closed_nodes_map.at<int>(x, y) = 1;

		// quit searching when the goal state is reached
		//if((*n0).estimate(xFinish, yFinish) == 0)
		if (x == xFinish && y == yFinish)
		{
			// generate the path from finish to start
			// by following the directions
			std::string path = "";
			while (!(x == xStart && y == yStart))
			{
				j = dir_map.at<int>(x, y);
				c = '0' + (j + dir / 2) % dir;
				path = c + path;
				x += dx[j];
				y += dy[j];
			}

			delete n0;// garbage collection
			// empty the leftover nodes
			while (!pq[pqi].empty())
				pq[pqi].pop();
			return path;
		}

		// generate moves (child nodes) in all possible directions
		for (i = 0; i < dir; i++)
		{
			xdx = x + dx[i];
			ydy = y + dy[i];

			expanding_counter++;

			if (!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 || map_to_calculate_path.at<int>(xdx, ydy) == 1 || closed_nodes_map.at<int>(xdx, ydy) == 1))
			{
				// generate a child node
				m0 = new nodeAstar(xdx, ydy, n0->getLevel(), n0->getPriority());
				m0->nextLevel(i);
				m0->updatePriority(xFinish, yFinish);

				// if it is not in the open list then add into that
				if (open_nodes_map.at<int>(xdx, ydy) == 0)
				{
					open_nodes_map.at<int>(xdx, ydy) = m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					dir_map.at<int>(xdx, ydy) = (i + dir / 2) % dir;
				}
				else if (open_nodes_map.at<int>(xdx, ydy) > m0->getPriority())
				{
					// update the priority info
					open_nodes_map.at<int>(xdx, ydy) = m0->getPriority();
					// update the parent direction info
					dir_map.at<int>(xdx, ydy) = (i + dir / 2) % dir;

					// replace the node
					// by emptying one pq to the other one
					// except the node to be replaced will be ignored
					// and the new node will be pushed in instead
					while (!(pq[pqi].top().getxPos() == xdx && pq[pqi].top().getyPos() == ydy))
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop(); // remove the wanted node

					// empty the larger size pq to the smaller one
					if (pq[pqi].size() > pq[1 - pqi].size())
						pqi = 1 - pqi;
					while (!pq[pqi].empty())
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				delete m0; // garbage collection
			}
		}
		delete n0; // garbage collection
	}
	return ""; // no route found
}

//This is the path planning algorithm for this class. It downsamples the map with the given factor (0 < factor < 1) so the
//map gets reduced and calculation time gets better. If it is set to 1 the map will have original size, if it is 0 the algorithm
//won't work, so make sure to not set it to 0. The algorithm also needs the Robot radius [m] and the map resolution [m²/pixel] to
//calculate the needed amount of erosions to include the radius in the planning.
double AStarPlanner::planPath(const cv::Mat& map, const cv::Point& start_point, const cv::Point& end_point,
		const double downsampling_factor, const double robot_radius, const double map_resolution)
{
	expanding_counter = 0;

	//length of the planned path
	double path_length = 0;

	if(start_point.x == end_point.x && start_point.y == end_point.y)//if the start and end-point are the same return 0
	{
		return path_length;
	}

	cv::Mat downsampled_map;
	downsampleMap(map, downsampled_map, downsampling_factor, robot_radius, map_resolution);

	//transform the Pixel values to the downsampled ones
	int start_x = downsampling_factor * start_point.x;
	int start_y = downsampling_factor * start_point.y;
	int end_x = downsampling_factor * end_point.x;
	int end_y = downsampling_factor * end_point.y;

	//set the sizes of the map
	m = downsampled_map.rows;// horizontal size of the map
	n = downsampled_map.cols;// vertical size size of the map

	// get the route
	clock_t start = clock();
	std::string route = pathFind(start_x, start_y, end_x, end_y, downsampled_map);
	if (route == "")
	{
		std::cout << "An empty route generated!" << std::endl;
		return 9002.; //return extremely large distance as path length if the rout could not be generated
	}
	clock_t end = clock();
	double time_elapsed = double(end - start);

	// follow the route on the map and update the path length
	if (route.length() > 0)
	{
		int j;
		char c;
		int x = start_x;
		int y = start_y;
		for (int i = 0; i < route.length(); i++)
		{
			//get the next char of the string and make it an integer, which shows the direction
			c = route.at(i);
			j=c-'0';
			x = x + dx[j];
			y = y + dy[j];
			//Update the pathlength with the directions of the path. When the path goes vertical or horizontal add length 1.
			//When it goes diagonal add sqrt(2)
			if (j == 0 || j == 2 || j == 4 || j == 6)
			{
				path_length += (1 / downsampling_factor);
			}
			if (j == 1 || j == 3 || j == 5 || j == 7)
			{
				path_length += (std::sqrt(2) / downsampling_factor);
			}
		}
	}

	return path_length;
}

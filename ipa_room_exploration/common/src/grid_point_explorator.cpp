#include <ipa_room_exploration/grid_point_explorator.h>

// Constructor
gridPointExplorator::gridPointExplorator(int grid_line_length)
{
	grid_line_length_ = grid_line_length;
}

// Function to create a static pose series that has the goal to inspect the complete floor of the given room.
void gridPointExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path)
{

}

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <ipa_room_exploration/RoomExplorationAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_client");

	actionlib::SimpleActionClient<ipa_room_exploration::RoomExplorationAction> ac("room_exploration/room_exploration_server", true);

	// read in test map
	cv::Mat map = cv::imread("/home/florianj/git/care-o-bot-indigo/src/autopnp/ipa_room_exploration/maps/map.png", 0);
	//make non-white pixels black
	int min_y = 1e5, max_y = 0, min_x = 1e5, max_x = 0;
	for (int y = 0; y < map.rows; y++)
	{
		for (int x = 0; x < map.cols; x++)
		{
			//find not reachable regions and make them black
			if (map.at<unsigned char>(y, x) < 250)
			{
				map.at<unsigned char>(y, x) = 0;
			}
			//else make it white
			else
			{
				map.at<unsigned char>(y, x) = 255;

				if(y < min_y)
					min_y = y;
				if(y > max_y)
					max_y = y;
				if(x < min_x)
					min_x = x;
				if(x > max_x)
					max_x = x;
			}
		}
	}

	std::cout << "map-size: " << map.rows << "x" << map.cols << std::endl;

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

//	cv::Point2f src_center(map.cols/2.0F, map.rows/2.0F);
//	cv::Mat rot_mat = getRotationMatrix2D(src_center, 180, 1.0);
//	cv::Mat dst;
//	cv::warpAffine(map, dst, rot_mat, map.size());
//	cv::flip(dst, map, 1);
//	cv::imshow("map", map);
//	cv::waitKey();
	sensor_msgs::Image labeling;

	cv_bridge::CvImage cv_image;
//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(labeling);

	geometry_msgs::Polygon min_max_points;
	geometry_msgs::Point32 min_point, max_point;
	min_point.x = min_x+9;
	min_point.y = min_y;
	max_point.x = max_x;
	max_point.y = max_y;
	min_max_points.points;
	min_max_points.points.push_back(min_point);
	min_max_points.points.push_back(max_point);

	std::cout << min_max_points.points[0] << " " << min_max_points.points[1] << std::endl;


	geometry_msgs::Pose2D map_origin;
	map_origin.x = 0.0;
	map_origin.y = 0.0;
	map_origin.theta = 0.0;

	geometry_msgs::Pose2D starting_position;
	starting_position.x = 1.0;
	starting_position.y = 1.0;
	starting_position.theta = 0.0;

	ipa_room_exploration::RoomExplorationGoal goal;
	goal.input_map = labeling;
	goal.map_origin = map_origin;
	goal.starting_position = starting_position;
	goal.map_resolution = 0.05;
	goal.robot_radius = 0.3; // turtlebot, used for sim 0.177
	goal.room_min_max = min_max_points;
	ac.sendGoal(goal);
	return 0;
}

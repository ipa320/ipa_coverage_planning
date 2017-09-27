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
#include <nav_msgs/OccupancyGrid.h>

#include <ipa_navigation_utils/utils.h>
#include <ipa_building_msgs/RoomExplorationAction.h>
#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_room_exploration/timer.h>
#include <Eigen/Dense>

// overload of << operator for geometry_msgs::Pose2D to wanted format
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose2D& obj)
{
	std::stringstream ss;
	ss <<  "[" << obj.x << ", " << obj.y << ", " << obj.theta << "]";
	os << ss.rdbuf();
	return os;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "room_exploration_client");
  ros::NodeHandle priv_nh("~");

	actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac("room_exploration_server", true);


  // read params
  std::string env_pack_path;
  ipa_utils::getRosParam(priv_nh, "env_pack", env_pack_path, "");
  std::string map_name;
  ipa_utils::getRosParam(priv_nh, "robot_env", map_name, "ipa-apartment");
  std::string file_name;
  ipa_utils::getRosParam(priv_nh, "image", file_name, "map.pgm");
  double resolution;
  ipa_utils::getRosParam(priv_nh, "resolution", resolution, 0.05);
  std::vector<double> origin (3,0);
  ipa_utils::getRosParam(priv_nh, "origin", origin, origin);
  double robot_radius;
  ipa_utils::getRosParam(priv_nh, "robot_radius", robot_radius, 1.0);


  const std::string image_path = env_pack_path + "/envs/" + map_name + file_name;

  cv::Mat map = cv::imread(image_path, 0);
	//make non-white pixels black
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
			}
		}
	}
	std::cout << "map-size: " << map.rows << "x" << map.cols << std::endl;

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	sensor_msgs::Image labeling;
	cv_bridge::CvImage cv_image;
//	cv_image.header.stamp = ros::Time::now();
	cv_image.encoding = "mono8";
	cv_image.image = map;
	cv_image.toImageMsg(labeling);

  geometry_msgs::Pose map_origin;
  map_origin.position = origin;

	geometry_msgs::Pose2D starting_position;
  starting_position.x = 0.0;
  starting_position.y = 0.0;
  starting_position.theta = 0.0;

	std::vector<geometry_msgs::Point32> fov_points(4);
	fov_points[0].x = -0.3;		// this is the working area of a vacuum cleaner with 60 cm width
	fov_points[0].y = 0.3;
	fov_points[1].x = -0.3;
	fov_points[1].y = -0.3;
	fov_points[2].x = 0.3;
	fov_points[2].y = -0.3;
	fov_points[3].x = 0.3;
	fov_points[3].y = 0.3;
	int planning_mode = 1;	// footprint planning

	ipa_building_msgs::RoomExplorationGoal goal;
	goal.input_map = labeling;
  goal.map_resolution = resolution;
	goal.map_origin = map_origin;
  goal.robot_radius = robot_radius; // turtlebot, used for sim 0.177, 0.4
  goal.coverage_radius = 1.0;
	goal.field_of_view = fov_points;
	goal.starting_position = starting_position;
	goal.planning_mode = planning_mode;
	ac.sendGoal(goal);

	ac.waitForResult(ros::Duration());
	ipa_building_msgs::RoomExplorationResultConstPtr action_result = ac.getResult();

	std::cout << "Got a path with " << action_result->coverage_path.size() << " nodes." << std::endl;

	// display path
	const double inverse_map_resolution = 1./goal.map_resolution;
	cv::Mat path_map = map.clone();
	for (size_t point=0; point<action_result->coverage_path.size(); ++point)
	{
		const cv::Point point1((action_result->coverage_path[point].x-map_origin.position.x)*inverse_map_resolution, (action_result->coverage_path[point].y-map_origin.position.y)*inverse_map_resolution);
		cv::circle(path_map, point1, 2, cv::Scalar(128), -1);
		if (point > 0)
		{
			const cv::Point point2((action_result->coverage_path[point-1].x-map_origin.position.x)*inverse_map_resolution, (action_result->coverage_path[point-1].y-map_origin.position.y)*inverse_map_resolution);
			cv::line(path_map, point1, point2, cv::Scalar(128), 1);
		}
		std::cout << "coverage_path[" << point << "]: x=" << action_result->coverage_path[point].x << ", y=" << action_result->coverage_path[point].y << ", theta=" << action_result->coverage_path[point].theta << std::endl;
	}
	cv::imshow("path", path_map);
	cv::waitKey();

	return 0;
}

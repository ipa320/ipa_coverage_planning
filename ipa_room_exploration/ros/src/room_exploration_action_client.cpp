#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>

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
	ros::NodeHandle nh;

	actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac("room_exploration_server", true);

	// read params
	bool use_test_maps;
	priv_nh.param("use_test_maps", use_test_maps, true);
	double resolution;
	priv_nh.param("resolution", resolution, 0.05);
	std::vector<double> origin (3,0);
	priv_nh.param("origin", origin, origin);
	double robot_radius;
	priv_nh.param("robot_radius", robot_radius, 0.3);
	double coverage_radius;
	priv_nh.param("coverage_radius", coverage_radius, 1.0);
	std::vector<double> start_pos = {0, 0, 0};
	priv_nh.param("starting_position", start_pos, start_pos);

	if (start_pos.size() != 3)
	{
		ROS_FATAL("starting_position must contain 3 values");
		return -1;
	}

	std::string image_path;
	if (use_test_maps)
	{
		// read in test map
		const std::string test_map_path = ros::package::getPath("ipa_room_segmentation") + "/common/files/test_maps/";
		image_path = test_map_path + "lab_ipa.png";
	}
	else
	{
		std::string env_pack_path;
		priv_nh.param<std::string>("env_pack", env_pack_path, "ipa_room_segmentation");
		std::string file_name;
		priv_nh.param<std::string>("image", file_name, "map.pgm");
		std::string map_name;
		priv_nh.param<std::string>("robot_env", map_name, "lab_ipa");

		image_path = env_pack_path + "/envs/" + map_name + "/" + file_name;
	}

	cv::Mat map_flipped = cv::imread(image_path, 0);
	cv::Mat map;
	cv::flip(map_flipped, map, 0);
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

//	const std::string topic = "/move_base/global_costmap/costmap";
//	nav_msgs::OccupancyGrid grid;
//	grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(topic, nh));
//	ROS_INFO("got grid");
//	std::vector<signed char>& dats = grid.data;
//	std::cout << dats.size() << std::endl;
//	cv::Mat test_map = cv::Mat(grid.info.height, grid.info.width, map.type());
//	for(size_t v = 0; v < test_map.rows; ++v)
//		for(size_t u = 0; u < test_map.cols; ++u)
//			test_map.at<uchar>(v,u) = (uchar)(2.55*(double)dats[v*grid.info.width+u]);
//	cv::imshow("testtt", test_map);
//	cv::waitKey();

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	DynamicReconfigureClient drc_exp(nh, "room_exploration_server/set_parameters", "room_exploration_server/parameter_updates");
	drc_exp.setConfig("room_exploration_algorithm", 8);
	drc_exp.setConfig("execute_path", false);
//	drc_exp.setConfig("path_eps", 3);
//	drc_exp.setConfig("grid_line_length", 15);
//	drc_exp.setConfig("path_eps", 10);
//	drc_exp.setConfig("cell_size", 10);
//	drc_exp.setConfig("plan_for_footprint", true);
//	drc_exp.setConfig("goal_eps", 0.0);
//	drc_exp.setConfig("delta_theta", 0.005);

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

	geometry_msgs::Pose map_origin;
	map_origin.position.x = origin[0];
	map_origin.position.y = origin[1];
	map_origin.position.z = origin[2];

	geometry_msgs::Pose2D starting_position;
	starting_position.x = start_pos[0];
	starting_position.y = start_pos[1];
	starting_position.theta = start_pos[2];

	std::vector<geometry_msgs::Point32> fov_points(4);
	fov_points[0].x = 0.04035;		// this field of view represents the off-center iMop floor wiping device
	fov_points[0].y = -0.136;
	fov_points[1].x = 0.04035;
	fov_points[1].y = 0.364;
	fov_points[2].x = 0.54035;		// todo: this definition is mirrored on x (y-coordinates are inverted) to work properly --> check why, make it work the intuitive way
	fov_points[2].y = 0.364;
	fov_points[3].x = 0.54035;
	fov_points[3].y = -0.136;
	int planning_mode = 2;	// viewpoint planning
//	fov_points[0].x = 0.15;		// this field of view fits a Asus Xtion sensor mounted at 0.63m height (camera center) pointing downwards to the ground in a respective angle
//	fov_points[0].y = 0.35;
//	fov_points[1].x = 0.15;
//	fov_points[1].y = -0.35;
//	fov_points[2].x = 1.15;
//	fov_points[2].y = -0.65;
//	fov_points[3].x = 1.15;
//	fov_points[3].y = 0.65;
//	int planning_mode = 2;	// viewpoint planning
//	fov_points[0].x = -0.3;		// this is the working area of a vacuum cleaner with 60 cm width
//	fov_points[0].y = 0.3;
//	fov_points[1].x = -0.3;
//	fov_points[1].y = -0.3;
//	fov_points[2].x = 0.3;
//	fov_points[2].y = -0.3;
//	fov_points[3].x = 0.3;
//	fov_points[3].y = 0.3;
//	int planning_mode = 1;	// footprint planning
	geometry_msgs::Point32 fov_origin;
	fov_origin.x = 0.;
	fov_origin.y = 0.;

	ipa_building_msgs::RoomExplorationGoal goal;
	goal.input_map = labeling;
	goal.map_resolution = resolution;
	goal.map_origin = map_origin;
	goal.robot_radius = robot_radius; // turtlebot, used for sim 0.177, 0.4
	goal.coverage_radius = coverage_radius;
	goal.field_of_view = fov_points;
	goal.field_of_view_origin = fov_origin;
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

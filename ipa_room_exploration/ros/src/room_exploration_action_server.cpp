#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::exploreRoom, this, _1), false)
{
	//Start action server
	room_exploration_server_.start();

	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("room_segmentation_algorithm", path_planning_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << path_planning_algorithm_ << std::endl << std::endl;
	if (path_planning_algorithm_ == 1)
		ROS_INFO("You have chosen the grid exploration method.");

	if (path_planning_algorithm_ == 1) // get grid point exploration parameters
	{
		node_handle_.param("grid_line_length", grid_line_length_, 10);
		std::cout << "room_exploration/grid_line_length = " << grid_line_length_ << std::endl;
	}
}

// Function to publish a navigation goal for move_base. It returns true, when the goal could be reached.
// The function tracks the robot pose while moving to the goal and adds these poses to the given pose-vector. This is done
// because it allows to calculate where the robot field of view has theoretically been and identify positions of the map that
// the robot hasn't seen.
bool RoomExplorationServer::publishNavigationGoal(const geometry_msgs::Pose2D& nav_goal, const std::string map_frame,
		const std::string base_frame, std::vector<geometry_msgs::Pose2D>& robot_poses)
{
	// move base client, that sends navigation goals to a move_base action server
	MoveBaseClient mv_base_client("/move_base", true);

	// wait for the action server to come up
	while(!mv_base_client.waitForServer(ros::Duration(5.0)))
	{
	  ROS_INFO("Waiting for the move_base action server to come up");
	}

	std::cout << "navigation goal: (" << nav_goal.x << ", "  << nav_goal.y << ", " << nav_goal.theta << ")" << std::endl;

	move_base_msgs::MoveBaseGoal move_base_goal;

	// create move_base_goal
	move_base_goal.target_pose.header.frame_id = "map";
	move_base_goal.target_pose.header.stamp = ros::Time::now();

	move_base_goal.target_pose.pose.position.x = nav_goal.x;
	move_base_goal.target_pose.pose.position.y = nav_goal.y;
	move_base_goal.target_pose.pose.orientation.z = std::sin(nav_goal.theta/2);
	move_base_goal.target_pose.pose.orientation.w = std::cos(nav_goal.theta/2);

	// send goal to the move_base sever, when one is found
	ROS_INFO("Sending goal");
	mv_base_client.sendGoal(move_base_goal);

	// wait until goal is reached or the goal is aborted
//	ros::Duration sleep_rate(0.1);
	do
	{
		tf::TransformListener listener;
		tf::StampedTransform transform;

		// try to get the transformation from map_frame to base_frame, wait max 5 seconds for this transform to come up
		try
		{
			ros::Time time = ros::Time(0);
			listener.waitForTransform(map_frame, base_frame, time, ros::Duration(5.0));
			listener.lookupTransform(map_frame, base_frame, time, transform);

			ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getOrigin().y());

			// save the current pose if a transform could be found
			geometry_msgs::Pose2D current_pose;

			current_pose.x = transform.getOrigin().x();
			current_pose.y = transform.getOrigin().y();
			double roll, pitch, yaw;
			transform.getBasis().getRPY(roll, pitch, yaw);
			current_pose.theta = yaw;

			robot_poses.push_back(current_pose);
		}
		catch(tf::TransformException &ex)
		{
			ROS_INFO("Couldn't get transform!");// %s", ex.what());
		}

	}while(mv_base_client.getState() != actionlib::SimpleClientGoalState::ABORTED && mv_base_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

	// check if point could be reached or not
	if(mv_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Hooray, the base moved to the goal");
		return true;
	}
	else
	{
		ROS_INFO("The base failed to move to the goal for some reason");
		return false;
	}
}

// Function to draw the seen points into the given map, that shows the positions the robot can actually reach. This is done by
// going trough all given robot-poses and calculate where the field of view has been. The field of view is given in the relative
// not rotated case, meaning to be in the robot-frame, where x_robot shows into the direction of the front and the y_robot axis
// along its left side. The function then calculates the field_of_view_points in the global frame by using the given robot pose
// and draws a rectangle at this position.
void RoomExplorationServer::drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
			const std::vector<geometry_msgs::Point32>& field_of_view_points, const float map_resolution, const cv::Point2d map_origin)
{
	// go trough each given robot pose
	for(std::vector<geometry_msgs::Pose2D>::const_iterator current_pose = robot_poses.begin(); current_pose != robot_poses.end(); ++current_pose)
	{
		// get the rotation matrix
		float sin_theta = std::sin(current_pose->theta);
		float cos_theta = std::cos(current_pose->theta);
//		cv::Mat R = (cv::Mat_<float>(2, 2) << cos_theta, -sin_theta, sin_theta, cos_theta); // template initialization
		Eigen::Matrix<float, 2, 2> R;
		R << cos_theta, -sin_theta, sin_theta, cos_theta;

		// transform field of view points
		std::vector<cv::Point> transformed_fow_points;
		Eigen::Matrix<float, 2, 1> pose_as_matrix;
		pose_as_matrix << current_pose->x, current_pose->y;
		for(size_t point = 0; point < field_of_view_points.size(); ++point)
		{
			// transform fow-point from geometry_msgs::Point32 to Eigen::Matrix
			Eigen::Matrix<float, 2, 1> fow_point;
			fow_point << field_of_view_points[point].x, field_of_view_points[point].y;

			// linear transformation
			Eigen::Matrix<float, 2, 1> transformed_vector = pose_as_matrix + R * fow_point;

			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
			// values
			cv::Point current_point = cv::Point((transformed_vector(0, 0) - map_origin.x)/map_resolution, (transformed_vector(1, 0) - map_origin.y)/map_resolution);
			current_point.x = std::max(current_point.x, 0);
			current_point.y = std::max(current_point.y, 0);
			current_point.x = std::min(current_point.x, reachable_areas_map.cols);
			current_point.y = std::min(current_point.y, reachable_areas_map.rows);
			transformed_fow_points.push_back(current_point);
//			std::cout << current_point << std::endl;
		}
//		std::cout << std::endl;

		// print results
//		std::cout << "Pose: " << *current_pose << std::endl;
//		for(size_t i = 0; i < transformed_fow_points.size(); ++i)
//			 std::cout << "fow: " <<  transformed_fow_points[i] << std::endl;
//		std::cout << std::endl;

		// draw field of view in map for current pose
		cv::fillConvexPoly(reachable_areas_map, transformed_fow_points, cv::Scalar(127));
	}
}

// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_room_exploration::RoomExplorationGoalConstPtr &goal)
{
	ros::Rate looping_rate(1);
	ROS_INFO("*****Room Exploration action server*****");
	ROS_INFO("map resolution is : %f", goal->map_resolution);

	// read the given parameters out of the goal
	cv::Point2d map_origin;
	map_origin.x = goal->map_origin.x;
	map_origin.y = goal->map_origin.y;

	float map_resolution = goal->map_resolution;

	float robot_radius = goal->robot_radius;

	geometry_msgs::Pose2D starting_position = goal->starting_position;
	geometry_msgs::Polygon min_max_coordinates = goal->room_min_max;

	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;
	transformImageToRoomCordinates(room_map);

	// plan the path using the wanted planner
	std::vector<geometry_msgs::Pose2D> exploration_path;
	if(path_planning_algorithm_ == 1) // use grid point explorator
	{
		// set wanted grid size
		grid_point_planner.setGridLineLength(grid_line_length_);

		// plan path
		grid_point_planner.getExplorationPath(room_map, exploration_path, robot_radius, map_resolution, starting_position, min_max_coordinates, map_origin);
	}

	// after planning a path, navigate trough all points and save the robot poses to check what regions have been seen
	std::vector<geometry_msgs::Pose2D> robot_poses;
	for(size_t nav_goal = 0; nav_goal < exploration_path.size(); ++nav_goal)
	{
//		cv::Mat map_copy = room_map.clone();
//		cv::circle(map_copy, cv::Point(exploration_path[nav_goal].y / map_resolution, exploration_path[nav_goal].x / map_resolution), 3, cv::Scalar(127), CV_FILLED);
//		cv::imshow("current_goal", map_copy);
//		cv::waitKey();

		publishNavigationGoal(exploration_path[nav_goal], goal->map_frame, goal->base_frame, robot_poses);
	}

	// draw the seen positions so the server can check what points haven't been seen
	cv::Mat seen_positions_map = room_map.clone();
	drawSeenPoints(seen_positions_map, robot_poses, goal->field_of_view, map_resolution, map_origin);

	// testing purpose: print the listened robot positions
	cv::Mat map_copy = room_map.clone();
	for(size_t i = 0; i < robot_poses.size(); ++i)
	{
		cv::circle(seen_positions_map, cv::Point(robot_poses[i].x/map_resolution, robot_poses[i].y/map_resolution), 2, cv::Scalar(100), CV_FILLED);
	}
//	cv::imshow("listened positions", map_copy);
	cv::namedWindow("seen area", cv::WINDOW_NORMAL);
	cv::imshow("seen area", seen_positions_map);
	cv::resizeWindow("seen area", 600, 600);
	cv::waitKey();

	room_exploration_server_.setSucceeded();
}

// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");

	ros::NodeHandle nh;

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room exploration has been initialized......");
	ros::spin();

	return 0;
}

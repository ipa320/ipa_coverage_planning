#include <ipa_room_exploration/room_exploration_action_server.h>

// Callback function for dynamic reconfigure.
void RoomExplorationServer::dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level)
{
	// set segmentation algorithm
	std::cout << "######################################################################################" << std::endl;
	std::cout << "Dynamic reconfigure request:" << std::endl;

	path_planning_algorithm_ = config.room_exploration_algorithm;
	std::cout << "room_exploration/path_planning_algorithm_ = " << path_planning_algorithm_ << std::endl;

	// set parameters regarding the chosen algorithm
	if (path_planning_algorithm_ == 1) // set grid point exploration parameters
	{
		grid_line_length_ = config.grid_line_length;
		std::cout << "room_exploration/grid_line_length_ = " << grid_line_length_ << std::endl;
	}

	std::cout << "######################################################################################" << std::endl;
}

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::exploreRoom, this, _1), false)
{
	//Start action server
	room_exploration_server_.start();

	// dynamic reconfigure
	room_exploration_dynamic_reconfigure_server_.setCallback(boost::bind(&RoomExplorationServer::dynamic_reconfigure_callback, this, _1, _2));


	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("room_exploration_algorithm", path_planning_algorithm_, 1);
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
		const std::string camera_frame, std::vector<geometry_msgs::Pose2D>& robot_poses)
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

		// try to get the transformation from map_frame to base_frame, wait max. 5 seconds for this transform to come up
		try
		{
			ros::Time time = ros::Time(0);
			listener.waitForTransform(map_frame, camera_frame, time, ros::Duration(5.0));
			listener.lookupTransform(map_frame, camera_frame, time, transform);

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
// along its left side. The function then calculates the field_of_view_points in the global frame by using the given robot pose.
// After this the function does a raycasting to check if the field of view has been blocked by an obstacle and couldn't see
// what's behind it. This ensures that no Point is wrongly classified as seen.
void RoomExplorationServer::drawSeenPoints(cv::Mat& reachable_areas_map, const std::vector<geometry_msgs::Pose2D>& robot_poses,
			const std::vector<geometry_msgs::Point32>& field_of_view_points, const Eigen::Matrix<float, 2, 1> raycasting_corner_1,
			const Eigen::Matrix<float, 2, 1> raycasting_corner_2, const float map_resolution, const cv::Point2d map_origin)
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

		// transform corners for raycasting
		Eigen::Matrix<float, 2, 1> transformed_corner_1 = pose_as_matrix + R * raycasting_corner_1;
		Eigen::Matrix<float, 2, 1> transformed_corner_2 = pose_as_matrix + R * raycasting_corner_2;

		// convert to openCV format
		cv::Point transformed_corner_cv_1 = cv::Point((transformed_corner_1(0, 0) - map_origin.x)/map_resolution, (transformed_corner_1(1, 0) - map_origin.y)/map_resolution);
		transformed_corner_cv_1.x = std::max(transformed_corner_cv_1.x, 0);
		transformed_corner_cv_1.y = std::max(transformed_corner_cv_1.y, 0);
		transformed_corner_cv_1.x = std::min(transformed_corner_cv_1.x, reachable_areas_map.cols);
		transformed_corner_cv_1.y = std::min(transformed_corner_cv_1.y, reachable_areas_map.rows);
		cv::Point transformed_corner_cv_2 = cv::Point((transformed_corner_2(0, 0) - map_origin.x)/map_resolution, (transformed_corner_2(1, 0) - map_origin.y)/map_resolution);
		transformed_corner_cv_2.x = std::max(transformed_corner_cv_2.x, 0);
		transformed_corner_cv_2.y = std::max(transformed_corner_cv_2.y, 0);
		transformed_corner_cv_2.x = std::min(transformed_corner_cv_2.x, reachable_areas_map.cols);
		transformed_corner_cv_2.y = std::min(transformed_corner_cv_2.y, reachable_areas_map.rows);

//		std::cout << "corners: " << std::endl << transformed_corner_cv_1 << std::endl <<transformed_corner_cv_2 << std::endl;

		// raycast the field of view to look what areas actually have been seen
		// get points between the edge-points to get goals for raycasting
		cv::LineIterator border_line(reachable_areas_map, transformed_corner_cv_1, transformed_corner_cv_2, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
		std::vector<cv::Point> raycasting_goals(border_line.count);

		for(size_t i = 0; i < border_line.count; i++, ++border_line)
			raycasting_goals[i] = border_line.pos();

		// transform pose into OpenCV format
		cv::Point pose_cv((current_pose->x - map_origin.x)/map_resolution, (current_pose->y - map_origin.y)/map_resolution);

		// go trough the found raycasting goals and draw the field-of-view
		for(std::vector<cv::Point>::iterator goal = raycasting_goals.begin(); goal != raycasting_goals.end(); ++goal)
		{
			// use openCVs bresenham algorithm to find the points from the robot pose to the goal
			cv::LineIterator ray_points(reachable_areas_map, pose_cv , *goal, 8);

			// go trough the points on the ray and draw them if they are inside the fow, stop the current for-step when a black
			// pixel is hit (an obstacle stops the camera from seeing whats behind)
			for(size_t point = 0; point < ray_points.count; point++, ++ray_points)
			{
				if(reachable_areas_map.at<uchar>(ray_points.pos()) == 0)
				{
					break;
				}

				if (reachable_areas_map.at<uchar>(ray_points.pos()) > 0 && cv::pointPolygonTest(transformed_fow_points, ray_points.pos(), false) >= 0)//pointInsidePolygonCheck(ray_points.pos(), transformed_fow_points) == 1)
				{
					reachable_areas_map.at<uchar>(ray_points.pos()) = 127;
				}
			}
		}

		// draw field of view in map for current pose
//		cv::fillConvexPoly(reachable_areas_map, transformed_fow_points, cv::Scalar(127));
	}
}

// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal)
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

		publishNavigationGoal(exploration_path[nav_goal], goal->map_frame, goal->camera_frame, robot_poses);
	}

	// find the points that are used to raycast the field of view
	// find points that span biggest angle
	std::vector<Eigen::Matrix<float, 2, 1> > fow_vectors;
	for(int i = 0; i < 4; ++i)
	{
		Eigen::Matrix<float, 2, 1> current_vector;
		current_vector << goal->field_of_view[i].x, goal->field_of_view[i].y;

		fow_vectors.push_back(current_vector);
	}

	// get angles
	float dot = fow_vectors[0].transpose()*fow_vectors[1];
	float abs = fow_vectors[0].norm() * fow_vectors[1].norm();
	float angle_1 = std::acos(dot/abs);
	dot = fow_vectors[2].transpose()*fow_vectors[3];
	abs = fow_vectors[2].norm() * fow_vectors[3].norm();
	float angle_2 = std::acos(dot/abs);

	// get points that define the edge-points of the line the raycasting should go to, by computing the intersection of two
	// lines: the line defined by the robot pose and the fow-point that spans the highest angle and a line parallel to the
	// front side of the fow with an offset
	Eigen::Matrix<float, 2, 1> corner_point_1, corner_point_2;
	float border_distance = 7;
	Eigen::Matrix<float, 2, 1> pose_to_fow_edge_vector_1 = fow_vectors[0];
	Eigen::Matrix<float, 2, 1> pose_to_fow_edge_vector_2 = fow_vectors[1];
	if(angle_1 > angle_2) // do a line crossing s.t. the corners are guaranteed to be after the fow
	{
		// get vectors showing the directions for for the lines from pose to edge of fow
		Eigen::Matrix<float, 2, 1> normed_fow_vector_1 = fow_vectors[0]/fow_vectors[0].norm();
		Eigen::Matrix<float, 2, 1> normed_fow_vector_2 = fow_vectors[1]/fow_vectors[1].norm();

		// get the offset point after the end of the fow
		Eigen::Matrix<float, 2, 1> offset_point_after_fow = fow_vectors[2];
		offset_point_after_fow(1, 0) = offset_point_after_fow(1, 0) + border_distance;

		// find the parameters for the two different intersections (for each corner point)
		float first_edge_parameter = (pose_to_fow_edge_vector_1(1, 0)/pose_to_fow_edge_vector_1(0, 0) * (fow_vectors[0](0, 0) - offset_point_after_fow(0, 0)) + offset_point_after_fow(1, 0) - fow_vectors[0](1, 0))/( pose_to_fow_edge_vector_1(1, 0)/pose_to_fow_edge_vector_1(0, 0) * (fow_vectors[3](0, 0) - fow_vectors[2](0, 0)) - (fow_vectors[3](1, 0) - fow_vectors[2](1, 0)) );
		float second_edge_parameter = (pose_to_fow_edge_vector_2(1, 0)/pose_to_fow_edge_vector_2(0, 0) * (fow_vectors[1](0, 0) - offset_point_after_fow(0, 0)) + offset_point_after_fow(1, 0) - fow_vectors[1](1, 0))/( pose_to_fow_edge_vector_2(1, 0)/pose_to_fow_edge_vector_2(0, 0) * (fow_vectors[3](0, 0) - fow_vectors[2](0, 0)) - (fow_vectors[3](1, 0) - fow_vectors[2](1, 0)) );

		// use the line equation and found parameters to actually find the corners
		corner_point_1 = first_edge_parameter * (fow_vectors[3] - fow_vectors[2]) + offset_point_after_fow;
		corner_point_2 = second_edge_parameter * (fow_vectors[3] - fow_vectors[2]) + offset_point_after_fow;
	}
	else
	{
		// follow the lines to the farthest points and go a little longer, this ensures that the whole fow is covered
		corner_point_1 = 1.3 * fow_vectors[2];
		corner_point_2 = 1.3 * fow_vectors[3];
	}

//	std::cout << "relative corners: " << corner_point_1 << std::endl << corner_point_2 << std::endl;

	// draw the seen positions so the server can check what points haven't been seen
	cv::Mat seen_positions_map = room_map.clone();
	drawSeenPoints(seen_positions_map, robot_poses, goal->field_of_view, corner_point_1, corner_point_2, map_resolution, map_origin);

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
//	cv::waitKey();

	// apply a binary filter on the image, making the drawn seen areas black
	cv::threshold(seen_positions_map, seen_positions_map, 150, 255, cv::THRESH_BINARY);

	// find regions with an area that is bigger than a defined value, which have not been seen by the fow
	std::vector < std::vector<cv::Point> > left_areas;
	cv::findContours(seen_positions_map, left_areas, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	cv::Mat black_map(room_map.cols, room_map.rows, room_map.type(), cv::Scalar(0));
	cv::drawContours(black_map, left_areas, -1, cv::Scalar(255), CV_FILLED);

	cv::namedWindow("left area", cv::WINDOW_NORMAL);
	cv::imshow("left area", black_map);
	cv::resizeWindow("left area", 600, 600);
	cv::waitKey();

	//

	room_exploration_server_.setSucceeded();
}

// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");

	ros::NodeHandle nh("~");

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ROS_INFO("Action Server for room exploration has been initialized......");
	ros::spin();

	return 0;
}

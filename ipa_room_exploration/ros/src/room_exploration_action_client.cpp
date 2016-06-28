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

#include <ipa_room_exploration/timer.h>

#include <Eigen/Dense>

// crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
// This code is patterned after [Franklin, 2000]
int pointInsidePolygonCheck(const cv::Point& P, const std::vector<cv::Point>& V)
{
    int    cn = 0;    // the  crossing number counter

    // loop through all edges of the polygon
    for (int i = 0; i < V.size(); i++)  // edge from V[i]  to V[i+1]
    {
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))    // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <=  P.y))) 	// a downward crossing
       {
            // compute  the actual edge-ray intersect x-coordinate
            float vt = (float)(P.y  - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x <  V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                 ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if  odd (in)
}



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
//	ac.waitForServer(); //will wait for infinite time

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

	geometry_msgs::Point32 fow_point_1;// geometry_msgs::Point32(0.3, 0.3);
	fow_point_1.x = 0.3;
	fow_point_1.y = 0.3;
	geometry_msgs::Point32 fow_point_2;// = geometry_msgs::Point32(0.3, -0.3);
	fow_point_2.x = 0.3;
	fow_point_2.y = -0.3;
	geometry_msgs::Point32 fow_point_3;// = geometry_msgs::Point32(0.7, 0.7);
	fow_point_3.x = 0.6;
	fow_point_3.y = -0.6;
	geometry_msgs::Point32 fow_point_4;// = geometry_msgs::Point32(0.7, -0.7);
	fow_point_4.x = 0.6;
	fow_point_4.y = 0.6;
	std::vector<geometry_msgs::Point32> fow_points(4);
	fow_points[0] = fow_point_1;
	fow_points[1] = fow_point_2;
	fow_points[2] = fow_point_3;
	fow_points[3] = fow_point_4;

	ipa_room_exploration::RoomExplorationGoal goal;
	goal.input_map = labeling;
	goal.map_origin = map_origin;
	goal.starting_position = starting_position;
	goal.map_resolution = 0.05;
	goal.robot_radius = 0.3; // turtlebot, used for sim 0.177
	goal.room_min_max = min_max_points;
	goal.camera_frame = "/base_footprint";
	goal.map_frame = "/map";
	goal.field_of_view = fow_points;
//	ac.sendGoal(goal);

	// testing
	std::vector<cv::Point> fow(5);
	fow[0] = cv::Point(10, 10);
	fow[1] = cv::Point(30, 10);
	fow[2] = cv::Point(40, 40);
	fow[3] = cv::Point(0, 40);
	fow[4] = cv::Point(10, 10);

	cv::Point robot_pose(20, 5);
	cv::Point test_point(20, 50);

	cv::Mat white_map = cv::Mat(100, 100, CV_8U, cv::Scalar(255));

	Timer tim;
//	std::cout << pointInsidePolygonCheck(robot_pose, fow) << std::endl;
//	std::cout << pointInsidePolygonCheck(test_point, fow) << std::endl;

	double simulated_sin = std::sin(3.14/2.0);
	double simulated_cos = std::cos(3.14/2.0);
	double temporary_distance = 1e12;

	// find points that span biggest angle
	std::vector<Eigen::Matrix<double, 2, 1> > fow_vectors;
	for(int i = 0; i < 4; ++i)
	{
		Eigen::Matrix<double, 2, 1> current_vector;
		current_vector << fow[i].x - robot_pose.x , fow[i].y - robot_pose.y;

		fow_vectors.push_back(current_vector);

		std::cout << current_vector << std::endl << std::endl;
	}

	// get angles
	double dot = fow_vectors[0].transpose()*fow_vectors[1];
	double abs = fow_vectors[0].norm() * fow_vectors[1].norm();
	double angle_1 = std::acos(dot/abs);
	dot = fow_vectors[2].transpose()*fow_vectors[3];
	abs = fow_vectors[2].norm() * fow_vectors[3].norm();
	double angle_2 = std::acos(dot/abs);

	// get points that define the points the raycasting should go to
	double travel_distance = 1.2 * fow_vectors[2].norm(); // from current pose to most far points
	Eigen::Matrix<double, 2, 1> edge_point_1, edge_point_2, robot_pose_as_vector;
	robot_pose_as_vector << robot_pose.x, robot_pose.y;
	if(angle_1 > angle_2)
	{
		Eigen::Matrix<double, 2, 1> normed_fow_vector_1 = fow_vectors[0]/fow_vectors[0].norm();
		Eigen::Matrix<double, 2, 1> normed_fow_vector_2 = fow_vectors[1]/fow_vectors[1].norm();

		edge_point_1 = travel_distance * normed_fow_vector_1 + robot_pose_as_vector;
		edge_point_2 = travel_distance * normed_fow_vector_2 + robot_pose_as_vector;
	}
	else
	{
		edge_point_1 = 1.2 * fow_vectors[2] + robot_pose_as_vector;
		edge_point_2 = 1.2 * fow_vectors[3] + robot_pose_as_vector;
	}

	// transform to OpenCv format
	cv::Point corner_1 (edge_point_1(0, 0), edge_point_1(1, 0));
	cv::Point corner_2 (edge_point_2(0, 0), edge_point_2(1, 0));

	std::cout << "corners: " << std::endl << corner_1 << std::endl << corner_2 << std::endl;

	cv::circle(white_map, corner_1, 2, cv::Scalar(100), CV_FILLED);
	cv::circle(white_map, corner_2, 2, cv::Scalar(100), CV_FILLED);
	cv::fillConvexPoly(white_map, fow, cv::Scalar(200));

	for (double distance = 1; distance < 50; ++distance)
	{
		const int ny = robot_pose.y + simulated_sin * distance;
		const int nx = robot_pose.x + simulated_cos * distance;
		//make sure the simulated point isn't out of the boundaries of the map
		if (ny < 0 || ny >= map.rows || nx < 0 || nx >= map.cols)
			break;
		if (white_map.at<unsigned char>(ny, nx) > 0 && pointInsidePolygonCheck(cv::Point(nx, ny), fow) == 1)
		{
			white_map.at<uchar> (ny, nx) = 127;
		}
	}

	cv::circle(white_map, robot_pose, 2, cv::Scalar(100), CV_FILLED);
	cv::circle(white_map, test_point, 2, cv::Scalar(100), CV_FILLED);

	cv::namedWindow("seen area", cv::WINDOW_NORMAL);
	cv::imshow("seen area", white_map);
	cv::resizeWindow("seen area", 600, 600);
	cv::waitKey();

	return 0;
}

#include <ipa_room_exploration/coverage_check_server.h>

// The default constructors
CoverageCheckServer::CoverageCheckServer()
{
}

CoverageCheckServer::CoverageCheckServer(ros::NodeHandle nh)
:node_handle_(nh)
{
	coverage_check_server_ = node_handle_.advertiseService("coverage_check", &CoverageCheckServer::checkCoverage, this);
	ROS_INFO("Server for coverage checking initialized.....");
}

// Callback function for the server.
bool CoverageCheckServer::checkCoverage(ipa_building_msgs::CheckCoverageRequest& request, ipa_building_msgs::CheckCoverageResponse& response)
{
	// convert the map msg to cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	std::cout << "request.input_map.encoding:" << request.input_map.encoding << std::endl;
	cv_ptr_obj = cv_bridge::toCvCopy(request.input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat map = cv_ptr_obj->image;

	// convert field of view to Eigen format
	std::vector<Eigen::Matrix<float, 2, 1> > field_of_view;
	for(size_t i = 0; i < request.field_of_view.size(); ++i)
	{
		Eigen::Matrix<float, 2, 1> current_vector;
		current_vector << request.field_of_view[i].x, request.field_of_view[i].y;
		field_of_view.push_back(current_vector);
	}

	// convert field of view origin to Eigen format
	Eigen::Matrix<float, 2, 1> fov_origin;
	fov_origin << request.field_of_view_origin.x, request.field_of_view_origin.y;

	// convert path to cv format
	std::vector<cv::Point3d> path;
	for (size_t i=0; i<request.path.size(); ++i)
		path.push_back(cv::Point3d(request.path[i].x, request.path[i].y, request.path[i].theta));

	cv::Mat coverage_map, number_of_coverage_image;
	bool return_value = checkCoverage(map, request.map_resolution, cv::Point2d(request.map_origin.position.x, request.map_origin.position.y), path,
			field_of_view, fov_origin, request.coverage_radius, request.check_for_footprint, request.check_number_of_coverages, coverage_map, number_of_coverage_image);

	// convert the map with the covered area back to the sensor_msgs format
	ros::Time now = ros::Time::now();
	cv_bridge::CvImage cv_image;
	cv_image.header.stamp = now;
	cv_image.encoding = sensor_msgs::image_encodings::MONO8;	// "mono8"
	cv_image.image = coverage_map;
	cv_image.toImageMsg(response.coverage_map);

	// if needed, return the image with number of coverages drawn in
	if(request.check_number_of_coverages==true)
	{
		cv_bridge::CvImage number_image;
		number_image.header.stamp = now;
		number_image.encoding = sensor_msgs::image_encodings::TYPE_32SC1;	//"32SC1";
		number_image.image = number_of_coverage_image;
		number_image.toImageMsg(response.number_of_coverage_image);
	}

	return return_value;
}
bool CoverageCheckServer::checkCoverage(const cv::Mat& map, const float map_resolution, const cv::Point2d& map_origin, const std::vector<cv::Point3d>& path,
		const std::vector<Eigen::Matrix<float, 2, 1> >& field_of_view, const Eigen::Matrix<float, 2, 1>& fov_origin, const float coverage_radius,
		const bool check_for_footprint, const bool check_number_of_coverages, cv::Mat& coverage_map, cv::Mat& number_of_coverage_image)
{
	// create a map that stores the number of coverages during the execution, if wanted
	cv::Mat* image_pointer = NULL;
	if(check_number_of_coverages==true)
	{
		number_of_coverage_image = cv::Mat::zeros(map.rows, map.cols, CV_32SC1);
		image_pointer = &number_of_coverage_image;
		ROS_INFO("Checking number of coverages.");
	}

	// check if the coverage check should be done for the footprint or the field of view
	coverage_map = map.clone();
	if(check_for_footprint==false)
	{
		ROS_INFO("Checking coverage for FOV.");
		drawCoveredPointsPolygon(coverage_map, path, field_of_view, fov_origin, map_resolution, map_origin, image_pointer);
	}
	else
	{
		ROS_INFO("Checking coverage for footprint.");
		drawCoveredPointsCircle(coverage_map, path, coverage_radius, map_resolution, map_origin, image_pointer);
	}
	ROS_INFO("Finished coverage check.");

	return true;
}


void CoverageCheckServer::drawCoveredPointsPolygon(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
			const std::vector<Eigen::Matrix<float, 2, 1> >& field_of_view, const Eigen::Matrix<float, 2, 1>& fov_origin,
			const float map_resolution, const cv::Point2d map_origin, cv::Mat* number_of_coverages_image)
{
	const float map_resolution_inverse = 1./map_resolution;

	// go trough each given robot pose
	for(std::vector<cv::Point3d>::const_iterator current_pose = robot_poses.begin(); current_pose != robot_poses.end(); ++current_pose)
	{
		// get the rotation matrix
		float sin_theta = std::sin(current_pose->z);
		float cos_theta = std::cos(current_pose->z);
		Eigen::Matrix<float, 2, 2> R;
		R << cos_theta, -sin_theta, sin_theta, cos_theta;

		// current pose as Eigen matrix
		Eigen::Matrix<float, 2, 1> pose_as_matrix;
		pose_as_matrix << current_pose->x, current_pose->y;

		// transform field of view points
		std::vector<cv::Point> transformed_fov_points;
		for(size_t point = 0; point < field_of_view.size(); ++point)
		{
			// linear transformation
			const Eigen::Matrix<float, 2, 1> transformed_fov_point = pose_as_matrix + R * field_of_view[point];

			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel values
			transformed_fov_points.push_back(clampImageCoordinates(cv::Point((transformed_fov_point(0, 0)-map_origin.x)*map_resolution_inverse, (transformed_fov_point(1, 0)-map_origin.y)*map_resolution_inverse), reachable_areas_map.rows, reachable_areas_map.cols));
		}

		// transform field of view origin
		const Eigen::Matrix<float, 2, 1> transformed_fov_origin = pose_as_matrix + R * fov_origin;
		const cv::Point transformed_fov_origin_point = clampImageCoordinates(cv::Point((transformed_fov_origin(0, 0)-map_origin.x)*map_resolution_inverse, (transformed_fov_origin(1, 0)-map_origin.y)*map_resolution_inverse), reachable_areas_map.rows, reachable_areas_map.cols);

		// draw current field of view in map
		cv::Mat fov_mat = cv::Mat::zeros(reachable_areas_map.rows, reachable_areas_map.cols, reachable_areas_map.type());
		std::vector<std::vector<cv::Point> > contours(1, transformed_fov_points);
#if CV_MAJOR_VERSION<=3
		cv::drawContours(fov_mat, contours, 0, cv::Scalar(255), CV_FILLED);
#else
		cv::drawContours(fov_mat, contours, 0, cv::Scalar(255), cv::FILLED);
#endif

		// check visibility for each pixel of the fov area
		for (int v=0; v<fov_mat.rows; ++v)
		{
			for (int u=0; u<fov_mat.cols; ++u)
			{
				if (fov_mat.at<uchar>(v,u)==0)
					continue;

				// create a line iterator from fov_origin to current fov point and verify visibility
				bool point_visible = true;
				const cv::Point current_goal(u,v);
				cv::LineIterator ray_points(reachable_areas_map, transformed_fov_origin_point, current_goal, 8, false);
				for(size_t point = 0; point < ray_points.count; ++point, ++ray_points)
				{
					if (reachable_areas_map.at<uchar>(ray_points.pos()) == 0)
					{
						point_visible = false;
						break;
					}
				}

				// mark visible point in map
				if (point_visible == true)
				{
					reachable_areas_map.at<uchar>(current_goal) = 127;

					// if wanted, count the coverage
					if(number_of_coverages_image!=NULL)
					{
						number_of_coverages_image->at<int>(current_goal) = number_of_coverages_image->at<int>(current_goal)+1;
					}
				}
			}
		}
	}
}


void CoverageCheckServer::drawCoveredPointsCircle(cv::Mat& reachable_areas_map, const std::vector<cv::Point3d>& robot_poses,
			const double coverage_radius, const float map_resolution,
			const cv::Point2d map_origin, cv::Mat* number_of_coverages_image)
{
	const float map_resolution_inverse = 1./map_resolution;

	cv::Mat map_copy = reachable_areas_map.clone(); // copy to draw positions that get later drawn into free space of the original map
	const int coverage_radius_pixel = coverage_radius*map_resolution_inverse;
	// iterate trough all poses and draw them into the given map
	for(std::vector<cv::Point3d>::const_iterator pose=robot_poses.begin(); pose!=robot_poses.end(); ++pose)
	{
		// draw the transformed robot footprint
		cv::Point current_point((pose->x-map_origin.x)*map_resolution_inverse, (pose->y-map_origin.y)*map_resolution_inverse);
		cv::circle(map_copy, current_point, coverage_radius_pixel, cv::Scalar(127), -1);

		// update the number of visits at this location, if wanted
		if(number_of_coverages_image!=NULL)
		{
			cv::Mat coverage_area = cv::Mat::zeros(map_copy.rows, map_copy.cols, CV_32SC1);
			cv::circle(coverage_area, current_point, coverage_radius_pixel, cv::Scalar(1), -1);
			*number_of_coverages_image = *number_of_coverages_image + coverage_area;
		}
	}

	// draw visited areas into free space of the original map
	for(size_t y=0; y<map_copy.rows; ++y)
		for(size_t x=0; x<map_copy.cols; ++x)
			if(reachable_areas_map.at<uchar>(y, x) == 255)
				reachable_areas_map.at<uchar>(y, x) = map_copy.at<uchar>(y, x);
}


inline cv::Point CoverageCheckServer::clampImageCoordinates(const cv::Point& p, const int rows, const int cols)
{
	cv::Point c;
	c.x = std::min(std::max(p.x, 0), cols-1);
	c.y = std::min(std::max(p.y, 0), rows-1);
	return c;
}

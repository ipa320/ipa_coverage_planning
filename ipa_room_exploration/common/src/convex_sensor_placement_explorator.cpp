#include <ipa_room_exploration/convex_sensor_placement_explorator.h>

// Constructor
convexSPPExplorator::convexSPPExplorator()
{

}

// Function that creates a Qsopt optimization problem and solves it, using the given matrices and vectors.
void convexSPPExplorator::solveOptimizationProblem(std::vector<double>& C, const cv::Mat& V, const std::vector<double>* W)
{
	// initialize the problem
	QSprob problem;
	problem = QScreate_prob("conv-SPP", QS_MIN);

	ROS_INFO("Creating and solving linear program.");

	// add the optimization variables to the problem
	if(W != NULL) // if a weight-vector is provided, use it to set the weights for the variables
	{
		std::cout << "creating problem with weights" << std::endl;

		int rval;
		// variables, which are constrained to 0<=v<=1
		for(size_t variable=0; variable<C.size(); ++variable)
		{
			rval = QSnew_col(problem, W->operator[](variable), 0.0, 1.0, (const char *) NULL);

			if(rval)
				std::cout << "!!!!! failed to add variable !!!!!" << std::endl;
		}

		// equality constraints to ensure that every position has been seen at least once
		for(size_t row=0; row<V.rows; ++row)
		{
			// gather the indices of the variables that are used in this constraint (row), i.e. where V[row][column] == 1
			std::vector<int> variable_indices;
			for(size_t col=0; col<V.cols; ++col)
				if(V.at<uchar>(row, col) == 1)
					variable_indices.push_back((int) col);

			// all indices are 1 in this constraint
			std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

			// add the constraint
			rval = QSadd_row(problem, (int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0, 'L', (const char *) NULL);

			if(rval)
				std::cout << "!!!!! failed to add constraint !!!!!" << std::endl;
		}

		// solve the optimization problem
		int status=0;
		rval = QSopt_dual(problem, &status);

		if (rval)
		{
		    fprintf (stderr, "QSopt_dual failed with return code %d\n", rval);
		}
		else
		{
		    switch (status)
		    {
		    case QS_LP_OPTIMAL:
		        printf ("Found optimal solution to LP\n");
		        break;
		    case QS_LP_INFEASIBLE:
		        printf ("No feasible solution exists for the LP\n");
		        break;
		    case QS_LP_UNBOUNDED:
		        printf ("The LP objective is unbounded\n");
		        break;
		    default:
		        printf ("LP could not be solved, status = %d\n", status);
		        break;
		    }
		}

		// retrieve solution
		int ncols = QSget_colcount(problem);
		double* result;
		result  = (double *) malloc(ncols * sizeof (double));
		QSget_solution(problem, NULL, result, NULL, NULL, NULL);
		for(size_t i=0; i<ncols; ++i)
			std::cout << result[i] << std::endl;
	}
}

// Function that is used to get a coverage path that covers the free space of the given map. It is programmed after
//
// Arain, M. A., Cirillo, M., Bennetts, V. H., Schaffernicht, E., Trincavelli, M., & Lilienthal, A. J. (2015, May). Efficient measurement planning for remote gas sensing with mobile robots. In 2015 IEEE International Conference on Robotics and Automation (ICRA) (pp. 3428-3434). IEEE.
//
// In this paper a linear program is used to get the minimal set of sensing poses to check the whole area for gas, but it
// can also be used to to do coverage path planning by defining the visibility function in a certain way. To do so the
// following steps are done
//	I.	Discretize the given map into cells by using the given cell size. The free cells are the ones that have a center
//		that is a white pixel in the map (value=255), i.e. belongs to the free space. After this compute the candidate
//		sensing poses, that determine the possible poses the robot later is.
//	II.	Construct the matrices that are used in the linear program. These are:
//			W: weight matrix for the re-weighted convex relaxation method
//			V: visibility matrix that stores 1 if cell i can be observed from candidate pose j (V[i,j] = 1) or 0 else
void convexSPPExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const int cell_size, const double delta_theta, const geometry_msgs::Polygon& room_min_max_coordinates,
		const std::vector<geometry_msgs::Point32>& footprint, const Eigen::Matrix<float, 2, 1>& robot_to_fow_middlepoint_vector,
		const double max_fow_angle, const double smallest_robot_to_fow_distance, const double largest_robot_to_fow_distance)
{
	// ************* I. Go trough the map and discretize it. *************
	// get cells
	std::vector<cv::Point> cell_centers;
	for(size_t y=room_min_max_coordinates.points[0].y+0.5*cell_size; y<=room_min_max_coordinates.points[1].y; y+=cell_size)
		for(size_t x=room_min_max_coordinates.points[0].x+0.5*cell_size; x<=room_min_max_coordinates.points[1].x; x+=cell_size)
			if(room_map.at<uchar>(y,x)==255)
				cell_centers.push_back(cv::Point(x,y));

	// get candidate sensing poses
	std::vector<geometry_msgs::Pose2D> candidate_sensing_poses;
	for(std::vector<cv::Point>::iterator center=cell_centers.begin(); center!=cell_centers.end(); ++center)
	{
		for(double angle=0; angle<2.0*PI; angle+=delta_theta)
		{
			// create and save pose
			geometry_msgs::Pose2D candidate_pose;
			candidate_pose.x = center->x;
			candidate_pose.y = center->y;
			candidate_pose.theta = angle;
			candidate_sensing_poses.push_back(candidate_pose);
		}
	}

	// ************* II. Construct the matrices needed in the linear program. *************
	// construct W
	int number_of_candidates=candidate_sensing_poses.size();
	std::vector<double> W(number_of_candidates, 1.0); // initial weights

	// construct V
	cv::Mat V = cv::Mat(cell_centers.size(), number_of_candidates, CV_8U); // binary variables

	// transpose vector here s.t. it doesn't has to be done in every step new
	for(std::vector<geometry_msgs::Pose2D>::iterator pose=candidate_sensing_poses.begin(); pose!=candidate_sensing_poses.end(); ++pose)
	{
		// get the transformed field of view
		// get the rotation matrix
		float sin_theta = std::sin(pose->theta);
		float cos_theta = std::cos(pose->theta);
		Eigen::Matrix<float, 2, 2> R;
		R << cos_theta, -sin_theta, sin_theta, cos_theta;

		// transform field of view points
		std::vector<cv::Point> transformed_fow_points;
		Eigen::Matrix<float, 2, 1> pose_as_matrix;
		pose_as_matrix << (pose->x*map_resolution)+map_origin.x, (pose->y*map_resolution)+map_origin.y; // convert to [meter]
		for(size_t point = 0; point < footprint.size(); ++point)
		{
			// transform fow-point from geometry_msgs::Point32 to Eigen::Matrix
			Eigen::Matrix<float, 2, 1> fow_point;
			fow_point << footprint[point].x, footprint[point].y;

			// linear transformation
			Eigen::Matrix<float, 2, 1> transformed_vector = pose_as_matrix + R * fow_point;

			// save the transformed point as cv::Point, also check if map borders are satisfied and transform it into pixel
			// values
			cv::Point current_point = cv::Point((transformed_vector(0, 0) - map_origin.x)/map_resolution, (transformed_vector(1, 0) - map_origin.y)/map_resolution);
			current_point.x = std::max(current_point.x, 0);
			current_point.y = std::max(current_point.y, 0);
			current_point.x = std::min(current_point.x, room_map.cols);
			current_point.y = std::min(current_point.y, room_map.rows);
			transformed_fow_points.push_back(current_point);
		}

		// rotate the vector from the robot to the field of view middle point
		Eigen::Matrix<float, 2, 1> transformed_robot_to_middlepoint_vector = R * robot_to_fow_middlepoint_vector;

		// for each pose check the cells that are closer than the max distance from robot to fow-corner and more far away
		// than the min distance, also only check points that span an angle to the robot-to-fow vector smaller than the
		// max found angle to the corners
//		cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, room_map.type(), cv::Scalar(0));
//		cv::circle(black_map, cv::Point(pose->x, pose->y), 3, cv::Scalar(200), CV_FILLED);
//		cv::fillConvexPoly(black_map, transformed_fow_points, cv::Scalar(150));
		for(std::vector<cv::Point>::iterator neighbor=cell_centers.begin(); neighbor!=cell_centers.end(); ++neighbor)
		{
			// compute pose to neighbor vector
			Eigen::Matrix<float, 2, 1> pose_to_neighbor;
			pose_to_neighbor << neighbor->x-pose->x, neighbor->y-pose->y;
			double distance = pose_to_neighbor.norm();

			// if neighbor is in the possible distance range check it further, distances given in [meter]
			if(distance >= smallest_robot_to_fow_distance/map_resolution && distance <= largest_robot_to_fow_distance/map_resolution)
			{
//				cv::circle(black_map, *neighbor, 2, cv::Scalar(50), CV_FILLED);

				// compute angle between the rotated robot-to-fow vector and the robot-to-neighbor vector
				float dot = transformed_robot_to_middlepoint_vector.transpose()*pose_to_neighbor;
				float abs = transformed_robot_to_middlepoint_vector.norm()*pose_to_neighbor.norm();
				float quotient = dot/abs;
				if(quotient > 1) // prevent errors resulting from round errors
					quotient = 1;
				else if(quotient < -1)
					quotient = -1;
				float angle = std::acos(quotient);

				// if this angle is smaller than the given max angle then check if this point is inside the transformed
				// field of view
				if(angle <= max_fow_angle)
				{
					if(cv::pointPolygonTest(transformed_fow_points, *neighbor, false) >= 0) // point inside
					{
						// check if the line from the robot pose to the neighbor crosses an obstacle, if so it is not
						// observable from the pose
						cv::LineIterator border_line(room_map, cv::Point(pose->x, pose->y), *neighbor, 8); // opencv implementation of bresenham algorithm, 8: color, irrelevant
						bool hit_obstacle = false;
						for(size_t i = 0; i < border_line.count; i++, ++border_line)
							if(room_map.at<uchar>(border_line.pos()) == 0)
								hit_obstacle = true;

						if(hit_obstacle == false)
						{
							V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 1;
//							cv::circle(black_map, *neighbor, 2, cv::Scalar(100), CV_FILLED);
						}
						else
							V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
					}
					else // point outside the field of view
						V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
				}
				else // point spans too big angle with middle point vector to be inside the fow
					V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
			}
			else // point not in the right range to be inside the fow
				V.at<uchar>(neighbor-cell_centers.begin(), pose-candidate_sensing_poses.begin()) = 0;
		}
//		cv::imshow("observable cells", black_map);
//		cv::waitKey();
	}

//	testing
//	for(size_t i=0; i<cell_centers.size(); ++i)
//	{
//		cv::Mat black_map = cv::Mat(room_map.rows, room_map.cols, room_map.type(), cv::Scalar(0));
//		cv::circle(black_map, cell_centers[i], 2, cv::Scalar(127), CV_FILLED);
//		for(size_t j=0; j<V.cols; ++j)
//		{
//			if(V.at<uchar>(i, j) == 1)
//			{
//				cv::circle(black_map, cv::Point(candidate_sensing_poses[j].x, candidate_sensing_poses[j].y), 2, cv::Scalar(100), CV_FILLED);
//				cv::imshow("candidates", black_map);
//				cv::waitKey();
//			}
//		}
//	}

	// TODO: create Qsopt problem and solve it iteratively, adapting W
	std::vector<double> C(W.size());
	solveOptimizationProblem(C, V, &W);

//	testing
//	std::cout << "number of free cells: " << cell_centers.size() << ", number of candidates: " << number_of_candidates << std::endl;
	// TODO: check for cells that can't be seen from any position and discard them
}

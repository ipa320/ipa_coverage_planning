#include <ipa_room_exploration/flow_network_explorator.h>

// Constructor
flowNetworkExplorator::flowNetworkExplorator()
{

}

// Function that creates a Qsopt optimization problem and solves it, using the given matrices and vectors.
template<typename T>
void flowNetworkExplorator::solveOptimizationProblem(std::vector<T>& C, const cv::Mat& V,
		const std::vector<std::vector<uint> >& flows_into_nodes, const std::vector<std::vector<uint> >& flows_out_of_nodes,
		const int stages, const uint start_index, const std::vector<uint>& start_arcs,  const std::vector<double>* W)
{
	// initialize the problem
	QSprob problem;
	problem = QScreate_prob("conv-SPP", QS_MIN);

	std::cout << "Creating and solving linear program." << std::endl;

	// add the optimization variables to the problem at each stage
	int rval;
	for(size_t r=0; r<stages; ++r)
	{
		// at initial stage only add arcs going out from the start node
		if(r==0)
		{
			for(size_t arc=0; arc<start_arcs.size(); ++arc)
			{
				if(W != NULL) // if a weight-vector is provided, use it to set the weights for the variables
					rval = QSnew_col(problem, W->operator[](start_arcs[arc]), 0.0, 1.0, (const char *) NULL);
				else
					rval = QSnew_col(problem, 1.0, 0.0, 1.0, (const char *) NULL);
			}
		}
		else
		{
			for(size_t variable=0; variable<C.size(); ++variable)
			{
				if(W != NULL) // if a weight-vector is provided, use it to set the weights for the variables
					rval = QSnew_col(problem, W->operator[](variable), 0.0, 1.0, (const char *) NULL);
				else
					rval = QSnew_col(problem, 1.0, 0.0, 1.0, (const char *) NULL);

				if(rval)
					std::cout << "!!!!! failed to add variable !!!!!" << std::endl;
			}
		}
	}

	// equality constraints to ensure that every position has been seen at least once
	for(size_t r=0; r<stages; ++r)
	{
		if(r==0)
		{
			for(size_t row=0; row<start_arcs.size(); ++row)
			{
				// gather the indices of the start arcs that are used in the initial constraint, i.e. where V[row][column] == 1
				std::vector<int> variable_indices;
				for(size_t col=0; col<start_arcs.size(); ++col)
					if(V.at<uchar>(row, start_arcs[col]) == 1)
						variable_indices.push_back((int) col);

				// all indices are 1 in this constraint
				std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

				// add the constraint
				rval = QSadd_row(problem, (int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0, 'G', (const char *) NULL);

				if(rval)
					std::cout << "!!!!! failed to add constraint !!!!!" << std::endl;
			}
		}
		else
		{
			for(size_t row=0; row<V.rows; ++row)
			{
				// gather the indices of the arcs that are used in this constraint (row), i.e. where V[row][column] == 1
				std::vector<int> variable_indices;
				for(size_t col=0; col<V.cols; ++col)
					if(V.at<uchar>(row, col) == 1)
						variable_indices.push_back((int) col);

				// all indices are 1 in this constraint
				std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

				// add the constraint
				rval = QSadd_row(problem, (int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0, 'G', (const char *) NULL);

				if(rval)
					std::cout << "!!!!! failed to add constraint !!!!!" << std::endl;
			}
		}
	}

	// if no weights are given an integer linear program should be solved, so the problem needs to be changed to this
	// by saving it to a file and reloading it (no better way available from Qsopt)
	if(W == NULL)
	{
		// save problem
		QSwrite_prob(problem, "lin_prog.lp", "LP");

		// read in the original problem, before "End" include the definition of the variables as integers
		std::ifstream original_problem;
		original_problem.open("lin_prog.lp", std::ifstream::in);
		std::ofstream new_problem;
		new_problem.open("int_lin_prog.lp", std::ofstream::out);
		std::string interception_line = "End";
		std::string line;
		while (getline(original_problem,line))
		{
			if (line != interception_line)
			{
				new_problem << line << std::endl;
			}
			else
			{
				// include Integer section
				new_problem << "Integer" << std::endl;
				for(size_t variable=1; variable<=C.size(); ++variable)
				{
					new_problem << " x" << variable;

					// new line for reading convenience after 5 variables
					if(variable%5 == 0 && variable != C.size()-1)
					{
						new_problem << std::endl;
					}
				}

				// add "End" to the file to show end of it
				new_problem << std::endl << std::left << line << std::endl;
			}
		}
		original_problem.close();
		new_problem.close();

		// reload the problem
		problem = QSread_prob("int_lin_prog.lp", "LP");
		if(problem == (QSprob) NULL)
		{
		    fprintf(stderr, "Unable to read and load the LP\n");
		}
	}

	// solve the optimization problem
	int status=0;
	QSget_intcount(problem, &status);
	std::cout << "number of integer variables in the problem: " << status << std::endl;
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
	for(size_t variable=0; variable<ncols; ++variable)
	{
		C[variable] = result[variable];
//		std::cout << result[variable] << std::endl;
	}

//	testing
	QSwrite_prob(problem, "lin_prog.lp", "LP");

	// free space used by the optimization problem
	QSfree(problem);
}

// This Function checks if the given cv::Point is close enough to one cv::Point in the given vector. If one point gets found
// that this Point is nearer than the defined min_distance the function returns false to stop it immediately.
bool flowNetworkExplorator::pointClose(const std::vector<cv::Point>& points, const cv::Point& point, const double min_distance)
{
	double square_distance = min_distance * min_distance;
	for(std::vector<cv::Point>::const_iterator current_point = points.begin(); current_point != points.end(); ++current_point)
	{
		double dx = current_point->x - point.x;
		double dy = current_point->y - point.y;
		if( ((dx*dx + dy*dy)) <= square_distance)
			return true;
	}
	return false;
}

// Function that uses the flow network based method to determine a coverage path. To do so the following steps are done
//	I.	Discretize the free space into cells that have to be visited a least once by using the sampling distance given to
//		the function. Also create a flow network by sweeping a line along the y-/x-axis and creating an edge, whenever it
//		hits an obstacle. From this hit point go back along the sweep line until the distance is equal to the coverage
//		radius, because the free space should represent the area that should be totally covered. If in both directions
//		along the sweep line no point in the free space can be found, ignore it.
//	II.	Create the matrices and vectors for the optimization problem:
//		1. The weight vector w, storing the distances between edges.
//		2. The coverage matrix V, storing which cell can be covered when going along the arcs.
//			remark: A cell counts as covered, when its center is in the coverage radius around an arc.
//		3. The sets of arcs for each node, that store the incoming and outgoing arcs
// III.	Create and solve the optimization problems in the following order:
//		1. 	Iteratively solve the weighted optimization problem to approximate the problem by a convex optimization. This
//			speeds up the solution and is done until the sparsity of the optimization variables doesn't change anymore,
//			i.e. converged, or a specific number of iterations is reached. To measure the sparsity a l^0_eps measure is
//			used, that checks |{i: c[i] <= eps}|. In each step the weights are adapted with respect to the previous solution.
//		2.	Solve the final optimization problem by discarding the arcs that correspond to zero elements in the previous
//			determined solution. This reduces the dimensionality of the problem and allows the algorithm to faster find
//			a solution.
void flowNetworkExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const int cell_size, const geometry_msgs::Polygon& room_min_max_coordinates,
		const Eigen::Matrix<float, 2, 1>& robot_to_fow_middlepoint_vector, const float coverage_radius,
		const bool plan_for_footprint)
{
	// *********** I. Discretize the free space and create the flow network ***********
	// find cell centers that need to be covered
	std::vector<cv::Point> cell_centers;
	for(size_t y=room_min_max_coordinates.points[0].y+0.5*cell_size; y<=room_min_max_coordinates.points[1].y; y+=cell_size)
		for(size_t x=room_min_max_coordinates.points[0].x+0.5*cell_size; x<=room_min_max_coordinates.points[1].x; x+=cell_size)
			if(room_map.at<uchar>(y,x)==255)
				cell_centers.push_back(cv::Point(x,y));

	// find edges for the flow network, sweeping along the y-axis
	std::vector<cv::Point> edges;
	int coverage_int = (int) std::floor(coverage_radius);
	std::cout << "y sweeping, radius: " << coverage_int << std::endl;
	for(size_t y=room_min_max_coordinates.points[0].y+coverage_int; y<=room_min_max_coordinates.points[1].y; ++y)
	{
//		cv::Mat test_map = room_map.clone();
		for(size_t x=0; x<room_map.cols; ++x)
		{
			// check if an obstacle has been found, only check outer parts of the occupied space
			if(room_map.at<uchar>(y,x)==0 && (room_map.at<uchar>(y-1,x)==255 || room_map.at<uchar>(y+1,x)==255))
			{
//				cv::circle(test_map, cv::Point(x,y), 2, cv::Scalar(127), CV_FILLED);
				// check on both sides along the sweep line if a free point is available, don't exceed matrix dimensions
				if(room_map.at<uchar>(y-coverage_int, x)==255 && y-coverage_int>=0)
					edges.push_back(cv::Point(x, y-coverage_int));
				else if(room_map.at<uchar>(y+coverage_int, x)==255 && y+coverage_int<room_map.rows)
					edges.push_back(cv::Point(x, y+coverage_int));

				// increase x according to the coverage radius, -1 because it gets increased after this for step
				x += 2.0*coverage_int-1;
			}
		}
//		cv::imshow("test", test_map);
//		cv::waitKey();
	}

	// sweep along x-axis
//	std::cout << "x sweeping" << std::endl;
//	for(size_t x=room_min_max_coordinates.points[0].x+coverage_int; x<=room_min_max_coordinates.points[1].x; ++x)
//	{
////		cv::Mat test_map = room_map.clone();
//		for(size_t y=0; y<room_map.rows; ++y)
//		{
//			// check if an obstacle has been found, only check outer parts of the occupied space
//			if(room_map.at<uchar>(y,x)==0 && (room_map.at<uchar>(y,x-1)==255 || room_map.at<uchar>(y,x+1)==255))
//			{
////				cv::circle(test_map, cv::Point(x,y), 2, cv::Scalar(127), CV_FILLED);
//				// check on both sides along the sweep line if a free point is available, don't exceed matrix dimensions
//				if(room_map.at<uchar>(y, x-coverage_int)==255 && x-coverage_int>=0)
//					edges.push_back(cv::Point(x-coverage_int, y));
//				else if(room_map.at<uchar>(y, x+coverage_int)==255 && x+coverage_int<room_map.cols)
//					edges.push_back(cv::Point(x+coverage_int, y));
//
//				// increase y according to the coverage radius, -1 because it gets increased after this for step
//				y += 2.0*coverage_int-1;
//			}
//		}
////		cv::imshow("test", test_map);
////		cv::waitKey();
//	}
	std::cout << "found " << edges.size() << " edges" << std::endl;

	// create the arcs for the flow network
	// TODO: reduce dimensionality, maybe only arcs that are straight (close enough to straight line)?
	std::cout << "Constructing distance matrix" << std::endl;
	cv::Mat distance_matrix; // determine weights
	// 3D vector storing the calculated paths from each node to each node
	DistanceMatrix::constructDistanceMatrix(distance_matrix, room_map, edges, 0.25, 0.0, map_resolution, path_planner_);
	std::cout << "Constructed distance matrix, defining arcs" << std::endl;
	std::vector<arcStruct> arcs;
	double max_distance = room_min_max_coordinates.points[1].y - room_min_max_coordinates.points[0].y; // arcs should at least go the maximal room distance to allow straight arcs
	for(size_t start=0; start<distance_matrix.rows; ++start)
	{
		for(size_t end=0; end<distance_matrix.cols; ++end)
		{
			// don't add arc from node to itself, only consider upper triangle of the distance matrix, one path from edge
			// to edge provides both arcs
			if(start!=end && end>start)
			{
				arcStruct current_forward_arc;
				current_forward_arc.start_point = edges[start];
				current_forward_arc.end_point = edges[end];
				current_forward_arc.weight = distance_matrix.at<double>(start, end);
				arcStruct current_backward_arc;
				current_backward_arc.start_point = edges[end];
				current_backward_arc.end_point = edges[start];
				current_forward_arc.weight = distance_matrix.at<double>(end, start);
				cv::Point vector = current_forward_arc.start_point - current_forward_arc.end_point;
				// don't add too long arcs to reduce dimensionality, because they certainly won't get chosen anyway
				// also don't add arcs that are too far away from the straight line (start-end) because they are likely
				// to go completely around obstacles and are not good
				if(current_forward_arc.weight <= max_distance && current_forward_arc.weight <= 1.1*cv::norm(vector)) // TODO: param
				{
					std::vector<cv::Point> astar_path;
					path_planner_.planPath(room_map, current_forward_arc.start_point, current_forward_arc.end_point, 1.0, 0.0, map_resolution, 0, &astar_path);
					current_forward_arc.edge_points = astar_path;
					// reverse path for backward arc
					std::reverse(astar_path.begin(), astar_path.end());
					current_backward_arc.edge_points = astar_path;
					arcs.push_back(current_forward_arc);
					arcs.push_back(current_backward_arc);
				}
			}
		}
	}
	std::cout << "arcs: " << arcs.size() << std::endl;

	// *********** II. Construct the matrices for the optimization problem ***********
	std::cout << "Starting to construct the matrices for the optimization problem." << std::endl;
	// 1. weight vector
	int number_of_candidates = arcs.size();
	std::vector<double> w(number_of_candidates);
	for(std::vector<arcStruct>::iterator arc=arcs.begin(); arc!=arcs.end(); ++arc)
		w[arc-arcs.begin()] = arc->weight;

	// 2. visibility matrix, storing which call can be covered when going along the arc
	//		remark: a cell counts as covered, when the center of each cell is in the coverage radius around the arc
	cv::Mat V = cv::Mat(cell_centers.size(), number_of_candidates, CV_8U); // binary variables
	for(std::vector<arcStruct>::iterator arc=arcs.begin(); arc!=arcs.end(); ++arc)
	{
		// use the pointClose function to check if a cell can be covered along the path
		for(std::vector<cv::Point>::iterator cell=cell_centers.begin(); cell!=cell_centers.end(); ++cell)
		{
			if(pointClose(arc->edge_points, *cell, 1.1*coverage_radius) == true)
				V.at<uchar>(cell-cell_centers.begin(), arc-arcs.begin()) = 1;
			else
				V.at<uchar>(cell-cell_centers.begin(), arc-arcs.begin()) = 0;
		}
	}

	// 3. set of arcs (indices) that are going into and out of one node
	std::vector<std::vector<uint> > flows_into_nodes(edges.size());
	std::vector<std::vector<uint> > flows_out_of_nodes(edges.size());
	for(std::vector<cv::Point>::iterator edge=edges.begin(); edge!=edges.end(); ++edge)
	{
		for(std::vector<arcStruct>::iterator arc=arcs.begin(); arc!=arcs.end(); ++arc)
		{
			// if the start point of the arc is the edge save it as incoming flow
			if(arc->start_point == *edge)
				flows_into_nodes[edge-edges.begin()].push_back(arc-arcs.begin());
			// if the end point of the arc is the edge save it as outgoing flow
			else if(arc->end_point == *edge)
				flows_out_of_nodes[edge-edges.begin()].push_back(arc-arcs.begin());
		}
	}

	std::cout << "Constructed all matrices for the optimization problem." << std::endl;

//	testing
//	cv::Mat test_map = room_map.clone();
////	for(size_t i=0; i<cell_centers.size(); ++i)
////		cv::circle(test_map, cell_centers[i], 2, cv::Scalar(75), CV_FILLED);
//	for(size_t i=0; i<edges.size(); ++i)
//		cv::circle(test_map, edges[i], 2, cv::Scalar(150), CV_FILLED);
//	cv::imshow("discretized", test_map);
//	cv::waitKey();
}

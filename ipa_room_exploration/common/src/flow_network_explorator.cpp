#include <ipa_room_exploration/flow_network_explorator.h>

// Constructor
FlowNetworkExplorator::FlowNetworkExplorator()
{

}

// Function that creates a Cbc optimization problem and solves it, using the given matrices and vectors and the 3-stage
// ansatz, that takes an initial step going from the start node and then a coverage stage assuming that the number of
// flows into and out of a node must be the same. At last a final stage is gone, that terminates the path in one of the
// possible nodes.
void FlowNetworkExplorator::solveThreeStageOptimizationProblem(std::vector<double>& C, const cv::Mat& V, const std::vector<double>& weights,
			const std::vector<std::vector<uint> >& flows_into_nodes, const std::vector<std::vector<uint> >& flows_out_of_nodes,
			const std::vector<uint>& start_arcs)
{
	// initialize the problem
	CoinModel problem_builder;

	std::cout << "Creating and solving linear program." << std::endl;

	// add the optimization variables to the problem
	int number_of_variables = 0;
	for(size_t arc=0; arc<start_arcs.size(); ++arc) // initial stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[start_arcs[arc]]);
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
//		}
	}
	for(size_t variable=0; variable<V.cols; ++variable) // coverage stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[variable]);
//		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
	}
	for(size_t variable=0; variable<V.cols; ++variable) // final stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[variable]);
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
	}
	for(size_t aux_flow=0; aux_flow<V.cols+start_arcs.size(); ++aux_flow) // auxiliary flow variables for initial and coverage stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, COIN_DBL_MAX); // auxiliary flow at least 0
		problem_builder.setObjective(number_of_variables, 0.0); // no additional part in the objective
		++number_of_variables;
	}
	for(size_t indicator=0; indicator<flows_into_nodes.size(); ++indicator) // indicator variables showing if a node is in the path
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, 0.0); // no additional part in the objective
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
	}

	std::cout << "number of variables in the problem: " << number_of_variables << std::endl;

	// inequality constraints to ensure that every position has been seen at least once:
	//		for each center that should be covered, find the arcs of the three stages that cover it
	for(size_t row=0; row<V.rows; ++row)
	{
		std::vector<int> variable_indices;

		for(size_t col=0; col<start_arcs.size(); ++col)
			if(V.at<uchar>(row, start_arcs[col])==1)
				variable_indices.push_back((int) col);

		// coverage and final stage
		for(size_t col=0; col<V.cols; ++col)
		{
			if(V.at<uchar>(row, col)==1)
			{
				variable_indices.push_back((int) col + start_arcs.size()); // coverage stage
				variable_indices.push_back((int) col + start_arcs.size() + V.cols); // final stage
			}
		}

		// all indices are 1 in this constraint
		std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

		// add the constraint, if the current cell can be covered by the given arcs
		if(variable_indices.size()>0)
			problem_builder.addRow((int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0);
	}


	// equality constraint to ensure that the number of flows out of one node is the same as the number of flows into the
	// node during the coverage stage
	//	Remark: for initial stage ensure that exactly one arc is gone, because there only the outgoing flows are taken
	//			into account
	// initial stage
	std::vector<int> start_indices(start_arcs.size());
	std::vector<double> start_coefficients(start_arcs.size());
	for(size_t start=0; start<start_arcs.size(); ++start)
	{
		start_indices[start] = start;
		start_coefficients[start] = 1.0;
	}
	problem_builder.addRow((int) start_indices.size(), &start_indices[0], &start_coefficients[0], 1.0, 1.0);

	// coverage stage, also add the flow decreasing and node indicator constraints
	for(size_t node=0; node<flows_into_nodes.size(); ++node)
	{
		// vectors for the conservativity constraint
		std::vector<int> variable_indices;
		std::vector<double> variable_coefficients;

		// vectors for the decreasing equality constraint that ensures that the flow gets reduced by 1, every time it passes a node,
		// cycles are prevented by this, because a start of a cycle is also an end of it, producing a constraint that the flow
		// trough this node needs to be larger than from any other arc in this cycle but also it needs to be smaller than
		// any other flow, which is not possible
		std::vector<int> flow_decrease_indices;
		std::vector<double> flow_decrease_coefficients;

		// vectors for the node indicator equality constraints that sets the indicator for the node to 1, if an arc flows
		// into this node during the initial or coverage stage
		std::vector<int> indicator_indices;
		std::vector<double> indicator_coefficients;

		// gather flows into node
		for(size_t inflow=0; inflow<flows_into_nodes[node].size(); ++inflow)
		{
			// if a start arcs flows into the node, additionally take the index of the arc in the start_arc vector
			if(contains(start_arcs, flows_into_nodes[node][inflow])==true)
			{
				// conservativity
				variable_indices.push_back(std::find(start_arcs.begin(), start_arcs.end(), flows_into_nodes[node][inflow])-start_arcs.begin());
				variable_coefficients.push_back(1.0);
				// decreasing flow
				flow_decrease_indices.push_back(variable_indices.back() + start_arcs.size() + 2.0*V.cols);
				flow_decrease_coefficients.push_back(1.0);
				// node indicator
				indicator_indices.push_back(variable_indices.back());
				indicator_coefficients.push_back(1.0);
			}
			// get the index of the arc in the optimization vector
			// conservativity
			variable_indices.push_back(flows_into_nodes[node][inflow] + start_arcs.size());
			variable_coefficients.push_back(1.0);
			// decreasing flow
			flow_decrease_indices.push_back(variable_indices.back() + start_arcs.size() + 2.0*V.cols);
			flow_decrease_coefficients.push_back(1.0);
			// node indicator
			indicator_indices.push_back(flows_into_nodes[node][inflow] + start_arcs.size());
			indicator_coefficients.push_back(1.0);
		}

		// gather flows out of node, also include flows into final nodes (for conservativity)
		for(size_t outflow=0; outflow<flows_out_of_nodes[node].size(); ++outflow)
		{
			// coverage stage variable
			// conservativity
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size());
			variable_coefficients.push_back(-1.0);
			// flow decreasing
			flow_decrease_indices.push_back(flows_out_of_nodes[node][outflow] + 2.0*(start_arcs.size()+V.cols));
			flow_decrease_coefficients.push_back(-1.0);
			// final stage variable
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size() + V.cols);
			variable_coefficients.push_back(-1.0);
		}

//		testing
//		std::cout << "number of flows: " << variable_indices.size() << std::endl;
//		for(size_t i=0; i<variable_indices.size(); ++i)
//			std::cout << variable_indices[i] << std::endl;

		// add conservativity constraint
		problem_builder.addRow((int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 0.0, 0.0);

		// add node indicator variable to flow decreasing constraint
		flow_decrease_indices.push_back(node + 2.0*start_arcs.size() + 3.0*V.cols);
		flow_decrease_coefficients.push_back(-1.0);

		// add flow decreasing constraint
//		std::cout << "decreasing constraint" << std::endl;
		problem_builder.addRow((int) flow_decrease_indices.size(), &flow_decrease_indices[0], &flow_decrease_coefficients[0], 0.0, 0.0);

		// get node indicator variable for the indicator constraint
//		std::cout << "indicator constraint" << std::endl;
		indicator_indices.push_back(node + 2.0*start_arcs.size() + 3.0*V.cols);
		indicator_coefficients.push_back(-1.0);

		// add node indicator constraint
		problem_builder.addRow((int) indicator_indices.size(), &indicator_indices[0], &indicator_coefficients[0], 0.0, 0.0);
	}

	// equality constraint to ensure that the path only once goes to the final stage
	std::vector<int> final_indices(V.cols);
	std::vector<double> final_coefficients(final_indices.size());
	// gather indices
	for(size_t node=0; node<final_indices.size(); ++node)
	{
		final_indices[node] = node + start_arcs.size() + V.cols;
		final_coefficients[node] = 1.0;
	}
	// add constraint
	problem_builder.addRow((int) final_indices.size(), &final_indices[0], &final_coefficients[0], 1.0, 1.0);

	// inequality constraints changing the maximal flow along an arc, if this arc is gone in the path
	std::cout << "max flow constraints" << std::endl;
	for(size_t node=0; node<V.cols+start_arcs.size(); ++node)
	{
		// size of two, because each auxiliary flow corresponds to exactly one arc indication variable
		std::vector<int> aux_flow_indices(2);
		std::vector<double> aux_flow_coefficients(2);

		// first entry shows indication variable
		aux_flow_indices[0] = node;
		aux_flow_coefficients[0] = flows_into_nodes.size()-1; // allow a high flow if the arc is chosen in the path

		// second entry shows the flow variable
		aux_flow_indices[1] = node+start_arcs.size()+2.0*V.cols;
		aux_flow_coefficients[1] = -1.0;

		// add constraint
		problem_builder.addRow((int) aux_flow_indices.size(), &aux_flow_indices[0], &aux_flow_coefficients[0], 0.0);
	}

	// equality constraints to set the flow out of the start to the number of gone nodes
	std::cout << "init flow constraints" << std::endl;
	std::vector<int> start_flow_indices(start_arcs.size()+flows_into_nodes.size());
	std::vector<double> start_flow_coefficients(start_flow_indices.size());
	for(size_t node=0; node<start_arcs.size(); ++node) // start arcs
	{
		start_flow_indices[node] = node+start_arcs.size()+2.0*V.cols;
		start_flow_coefficients[node] = 1.0;
	}
	for(size_t indicator=0; indicator<flows_into_nodes.size(); ++indicator) // node indicator variables
	{
		start_flow_indices[indicator+start_arcs.size()] = indicator+2.0*start_arcs.size()+3.0*V.cols;
		start_flow_coefficients[indicator+start_arcs.size()] = -1.0;
	}
	problem_builder.addRow((int) start_flow_indices.size(), &start_flow_indices[0], &start_flow_coefficients[0], 0.0, 0.0);

	// load the created LP problem to the solver
	OsiClpSolverInterface LP_solver;
	OsiClpSolverInterface* solver_pointer = &LP_solver;

	solver_pointer->loadFromCoinModel(problem_builder);

	// testing
	solver_pointer->writeLp("lin_flow_prog", "lp");

	// solve the created integer optimization problem
	CbcModel model(*solver_pointer);
	model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

//	CbcHeuristicLocal heuristic2(model);
	CbcHeuristicFPump heuristic(model);
	model.addHeuristic(&heuristic);

	model.initialSolve();
	model.branchAndBound();

	// retrieve solution
	const double* solution = model.solver()->getColSolution();

	for(size_t res=0; res<number_of_variables; ++res)
	{
//		std::cout << solution[res] << std::endl;
		C[res] = solution[res];
	}
}

// Function that creates a Gurobi optimization problem and solves it, using the given matrices and vectors and the 3-stage
// ansatz, that takes an initial step going from the start node and then a coverage stage assuming that the number of
// flows into and out of a node must be the same. At last a final stage is gone, that terminates the path in one of the
// possible nodes. This function uses lazy generalized cutset inequalities (GCI) to prevent cycles. For that a solution
// without cycle prevention constraints is determined and then cycles are detected in this solution. For these cycles
// then additional constraints are added and a new solution is determined. This procedure gets repeated until no cycle
// is detected in the solution or the only cycle contains all visited nodes, because such a solution is a traveling
// salesman like solution, which is a valid solution.
void FlowNetworkExplorator::solveGurobiOptimizationProblem(std::vector<double>& C, const cv::Mat& V, const std::vector<double>& weights,
		const std::vector<std::vector<uint> >& flows_into_nodes, const std::vector<std::vector<uint> >& flows_out_of_nodes,
		const std::vector<uint>& start_arcs)
{
#ifdef GUROBI_FOUND
	std::cout << "Creating and solving linear program with Gurobi." << std::endl;
	// initialize the problem
	GRBEnv *env = new GRBEnv();
    GRBModel model = GRBModel(*env);
//    model.set("Threads", "1");

    // one must set LazyConstraints parameter when using lazy constraints
    model.set(GRB_IntParam_LazyConstraints, 1);

    // vector that stores the variables of the problem
    std::vector<GRBVar> optimization_variables;

	// add the optimization variables to the problem
	int number_of_variables = 0;
	for(size_t arc=0; arc<start_arcs.size(); ++arc) // initial stage
	{
		GRBVar current_variable = model.addVar(0.0, 1.0, weights[start_arcs[arc]], GRB_BINARY);
		optimization_variables.push_back(current_variable);
		++number_of_variables;
	}
	for(size_t variable=0; variable<V.cols; ++variable) // coverage stage
	{
		GRBVar current_variable = model.addVar(0.0, 1.0, weights[variable], GRB_BINARY);
		optimization_variables.push_back(current_variable);
		++number_of_variables;
	}
	for(size_t variable=0; variable<V.cols; ++variable) // final stage
	{
		GRBVar current_variable = model.addVar(0.0, 1.0, weights[variable], GRB_BINARY);
		optimization_variables.push_back(current_variable);
		++number_of_variables;
	}
	std::cout << "number of variables in the problem: " << number_of_variables << std::endl;

	// inequality constraints to ensure that every position has been seen at least once:
	//		for each center that should be covered, find the arcs of the three stages that cover it
	for(size_t row=0; row<V.rows; ++row)
	{
		std::vector<int> variable_indices;

		for(size_t col=0; col<start_arcs.size(); ++col)
			if(V.at<uchar>(row, start_arcs[col])==1)
				variable_indices.push_back((int) col);

		// coverage and final stage
		for(size_t col=0; col<V.cols; ++col)
		{
			if(V.at<uchar>(row, col)==1)
			{
				variable_indices.push_back((int) col + start_arcs.size()); // coverage stage
				variable_indices.push_back((int) col + start_arcs.size() + V.cols); // final stage
			}
		}

		// add the constraint, if the current cell can be covered by the given arcs, indices=1 in this constraint
		if(variable_indices.size()>0)
		{
			GRBLinExpr current_coverage_constraint;
			for(size_t var=0; var<variable_indices.size(); ++var)
				current_coverage_constraint += optimization_variables[variable_indices[var]];
			model.addConstr(current_coverage_constraint>=1);
		}
	}


	// equality constraint to ensure that the number of flows out of one node is the same as the number of flows into the
	// node during the coverage stage
	//	Remark: for initial stage ensure that exactly one arc is gone, because there only the outgoing flows are taken
	//			into account
	// initial stage
	GRBLinExpr initial_stage_constraint;
	for(size_t start=0; start<start_arcs.size(); ++start)
		initial_stage_constraint += optimization_variables[start];
	model.addConstr(initial_stage_constraint==1);

	// coverage stage
	for(size_t node=0; node<flows_into_nodes.size(); ++node)
	{
		std::vector<int> variable_indices;
		std::vector<double> variable_coefficients;

		// gather flows into node
		for(size_t inflow=0; inflow<flows_into_nodes[node].size(); ++inflow)
		{
			// if a start arcs flows into the node, additionally take the index of the arc in the start_arc vector
			if(contains(start_arcs, flows_into_nodes[node][inflow])==true)
			{
				// conservativity
				variable_indices.push_back(std::find(start_arcs.begin(), start_arcs.end(), flows_into_nodes[node][inflow])-start_arcs.begin());
				variable_coefficients.push_back(1.0);
			}
			// get the index of the arc in the optimization vector
			// conservativity
			variable_indices.push_back(flows_into_nodes[node][inflow] + start_arcs.size());
			variable_coefficients.push_back(1.0);
		}

		// gather flows out of node, also include flows into final nodes (for conservativity)
		for(size_t outflow=0; outflow<flows_out_of_nodes[node].size(); ++outflow)
		{
			// coverage stage variable
			// conservativity
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size());
			variable_coefficients.push_back(-1.0);
			// final stage variable
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size() + V.cols);
			variable_coefficients.push_back(-1.0);
		}

//		testing
//		std::cout << "number of flows: " << variable_indices.size() << std::endl;
//		for(size_t i=0; i<variable_indices.size(); ++i)
//			std::cout << variable_indices[i] << std::endl;

		// add conservativity constraint
		GRBLinExpr current_conservativity_constraint;
		for(size_t var=0; var<variable_indices.size(); ++var)
			current_conservativity_constraint += variable_coefficients[var]*optimization_variables[variable_indices[var]];
		model.addConstr(current_conservativity_constraint==0);
	}

	// equality constraint to ensure that the path only once goes to the final stage
	GRBLinExpr final_stage_constraint;
	for(size_t node=0; node<V.cols; ++node)
		final_stage_constraint += optimization_variables[node + start_arcs.size() + V.cols];
	model.addConstr(final_stage_constraint==1);

	// add the lazy constraint callback object that adds a lazy constraint if it gets violated after solving the problem
	CyclePreventionCallbackClass callback_object = CyclePreventionCallbackClass(&optimization_variables, V.cols, flows_out_of_nodes, flows_into_nodes, start_arcs);
	model.setCallback(&callback_object);

	// solve the optimization
	model.optimize();

	// testing
	model.write("lin_flow_prog_gurobi.lp");
	std::string filename_out = "lazy_constraints.txt";
	std::ofstream file_out(filename_out.c_str(), std::ofstream::out);
	if (file_out.is_open())
	{
//		file_out << cumulative_statistics.str();
		for(size_t l=0; l<callback_object.lhs.size(); ++l)
		{
			for(size_t i=0; i<callback_object.lhs[l].size(); ++i)
			{
				file_out << callback_object.lhs[l][i] << " + ";
			}
			file_out << " <= " << callback_object.rhs[l] << std::endl;
		}
	}
	else
		ROS_ERROR("Could not open file '%s' for writing cumulative data.", filename_out.c_str());
	file_out.close();

	// retrieve solution
	std::cout << "retrieving solution" << std::endl;
	for(size_t var=0; var<number_of_variables; ++var)
	{
		C[var]= optimization_variables[var].get(GRB_DoubleAttr_X);
//		if(C[var]>0.01)
//			std::cout << var << std::endl;
	}

	// garbage collection
	delete env;

	return;
#endif
}

// Function that creates a Cbc optimization problem and solves it, using the given matrices and vectors and the 3-stage
// ansatz, that takes an initial step going from the start node and then a coverage stage assuming that the number of
// flows into and out of a node must be the same. At last a final stage is gone, that terminates the path in one of the
// possible nodes. This function uses lazy generalized cutset inequalities (GCI) to prevent cycles. For that a solution
// without cycle prevention constraints is determined and then cycles are detected in this solution. For these cycles
// then additional constraints are added and a new solution is determined. This procedure gets repeated until no cycle
// is detected in the solution or the only cycle contains all visited nodes, because such a solution is a traveling
// salesman like solution, which is a valid solution.
void FlowNetworkExplorator::solveLazyConstraintOptimizationProblem(std::vector<double>& C, const cv::Mat& V, const std::vector<double>& weights,
		const std::vector<std::vector<uint> >& flows_into_nodes, const std::vector<std::vector<uint> >& flows_out_of_nodes,
		const std::vector<uint>& start_arcs)
{
	// initialize the problem
	CoinModel problem_builder;

	std::cout << "Creating and solving linear program." << std::endl;

	// add the optimization variables to the problem
	int number_of_variables = 0;
	for(size_t arc=0; arc<start_arcs.size(); ++arc) // initial stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[start_arcs[arc]]);
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
//		}
	}
	for(size_t variable=0; variable<V.cols; ++variable) // coverage stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[variable]);
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
	}
	for(size_t variable=0; variable<V.cols; ++variable) // final stage
	{
		problem_builder.setColBounds(number_of_variables, 0.0, 1.0);
		problem_builder.setObjective(number_of_variables, weights[variable]);
		problem_builder.setInteger(number_of_variables);
		++number_of_variables;
	}
	std::cout << "number of variables in the problem: " << number_of_variables << std::endl;

	// inequality constraints to ensure that every position has been seen at least once:
	//		for each center that should be covered, find the arcs of the three stages that cover it
	for(size_t row=0; row<V.rows; ++row)
	{
		std::vector<int> variable_indices;

		for(size_t col=0; col<start_arcs.size(); ++col)
			if(V.at<uchar>(row, start_arcs[col])==1)
				variable_indices.push_back((int) col);

		// coverage and final stage
		for(size_t col=0; col<V.cols; ++col)
		{
			if(V.at<uchar>(row, col)==1)
			{
				variable_indices.push_back((int) col + start_arcs.size()); // coverage stage
				variable_indices.push_back((int) col + start_arcs.size() + V.cols); // final stage
			}
		}

		// all indices are 1 in this constraint
		std::vector<double> variable_coefficients(variable_indices.size(), 1.0);

		// add the constraint, if the current cell can be covered by the given arcs
		if(variable_indices.size()>0)
			problem_builder.addRow((int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 1.0);
	}


	// equality constraint to ensure that the number of flows out of one node is the same as the number of flows into the
	// node during the coverage stage
	//	Remark: for initial stage ensure that exactly one arc is gone, because there only the outgoing flows are taken
	//			into account
	// initial stage
	std::vector<int> start_indices(start_arcs.size());
	std::vector<double> start_coefficients(start_arcs.size());
	for(size_t start=0; start<start_arcs.size(); ++start)
	{
		start_indices[start] = start;
		start_coefficients[start] = 1.0;
	}
	problem_builder.addRow((int) start_indices.size(), &start_indices[0], &start_coefficients[0], 1.0, 1.0);

	// coverage stage
	for(size_t node=0; node<flows_into_nodes.size(); ++node)
	{
		std::vector<int> variable_indices;
		std::vector<double> variable_coefficients;

		// gather flows into node
		for(size_t inflow=0; inflow<flows_into_nodes[node].size(); ++inflow)
		{
			// if a start arcs flows into the node, additionally take the index of the arc in the start_arc vector
			if(contains(start_arcs, flows_into_nodes[node][inflow])==true)
			{
				// conservativity
				variable_indices.push_back(std::find(start_arcs.begin(), start_arcs.end(), flows_into_nodes[node][inflow])-start_arcs.begin());
				variable_coefficients.push_back(1.0);
			}
			// get the index of the arc in the optimization vector
			// conservativity
			variable_indices.push_back(flows_into_nodes[node][inflow] + start_arcs.size());
			variable_coefficients.push_back(1.0);
		}

		// gather flows out of node, also include flows into final nodes (for conservativity)
		for(size_t outflow=0; outflow<flows_out_of_nodes[node].size(); ++outflow)
		{
			// coverage stage variable
			// conservativity
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size());
			variable_coefficients.push_back(-1.0);
			// final stage variable
			variable_indices.push_back(flows_out_of_nodes[node][outflow] + start_arcs.size() + V.cols);
			variable_coefficients.push_back(-1.0);
		}

//		testing
//		std::cout << "number of flows: " << variable_indices.size() << std::endl;
//		for(size_t i=0; i<variable_indices.size(); ++i)
//			std::cout << variable_indices[i] << std::endl;

		// add conservativity constraint
		problem_builder.addRow((int) variable_indices.size(), &variable_indices[0], &variable_coefficients[0], 0.0, 0.0);
	}

	// equality constraint to ensure that the path only once goes to the final stage
	std::vector<int> final_indices(V.cols);
	std::vector<double> final_coefficients(final_indices.size());
	// gather indices
	for(size_t node=0; node<final_indices.size(); ++node)
	{
		final_indices[node] = node + start_arcs.size() + V.cols;
		final_coefficients[node] = 1.0;
	}
	// add constraint
	problem_builder.addRow((int) final_indices.size(), &final_indices[0], &final_coefficients[0], 1.0, 1.0);

	// load the created LP problem to the solver
	OsiClpSolverInterface LP_solver;
	OsiClpSolverInterface* solver_pointer = &LP_solver;

	solver_pointer->loadFromCoinModel(problem_builder);

	// testing
	solver_pointer->writeLp("lin_flow_prog", "lp");

	// solve the created integer optimization problem
	CbcModel model(*solver_pointer);
	model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

	CbcHeuristicFPump heuristic(model);
	model.addHeuristic(&heuristic);
	model.initialSolve();
	model.branchAndBound();

//	testing
//	std::vector<int> test_row(2);
//	std::vector<double> test_coeff(2);
//
//	test_row[0] = 0;
//	test_row[1] = 1;
//
//	test_coeff[0] = 1.0;
//	test_coeff[1] = 1.0;
//	solver_pointer->addRow(2, &test_row[0], &test_coeff[0], 0.0, 0.0);
//	solver_pointer->writeLp("lin_flow_prog", "lp");
//	solver_pointer->resolve();

	// retrieve solution
	const double* solution = model.solver()->getColSolution();

	// search for cycles in the retrieved solution, if one is found add a constraint to prevent this cycle
	bool cycle_free = false;

	do
	{
		// get the arcs that are used in the previously calculated solution
		std::set<uint> used_arcs; // set that stores the indices of the arcs corresponding to non-zero elements in the solution

		// go trough the start arcs
		for(size_t start_arc=0; start_arc<start_arcs.size(); ++start_arc)
		{
			if(solution[start_arc]!=0)
			{
				// insert start index
				used_arcs.insert(start_arcs[start_arc]);
			}
		}

		// go trough the coverage stage
		for(size_t arc=start_arcs.size(); arc<start_arcs.size()+V.cols; ++arc)
		{
			if(solution[arc]!=0)
			{
				// insert index, relative to the first coverage variable
				used_arcs.insert(arc-start_arcs.size());
			}
		}

		 // go trough the final stage and find the remaining used arcs
		 for(uint flow=start_arcs.size()+V.cols; flow<start_arcs.size()+2*V.cols; ++flow)
		 {
			 if(solution[flow]>0)
			 {
				 // insert saved outgoing flow index
				 used_arcs.insert(flow-start_arcs.size()-V.cols);
			 }
		}
//		 go trough the final stage and find the remaining used arcs
//		for(size_t node=0; node<flows_out_of_nodes.size(); ++node)
//		{
//			for(size_t flow=0; flow<flows_out_of_nodes[node].size(); ++flow)
//			{
//				if(solution[flows_out_of_nodes[node][flow]+start_arcs.size()+V.cols]!=0)
//				{
//					// insert saved outgoing flow index
//					used_arcs.insert(flows_out_of_nodes[node][flow]);
//				}
//			}
//		}

		std::cout << "got " << used_arcs.size() << " used arcs" << std::endl;
//		std::cout << "got the used arcs:" << std::endl;
//		for(std::set<uint>::iterator sol=used_arcs.begin(); sol!=used_arcs.end(); ++sol)
//			std::cout << *sol << std::endl;
//		std::cout << std::endl;

		// construct the directed edges out of the used arcs
		std::vector<std::vector<int> > directed_edges; // vector that stores the directed edges for each node
		for(uint start_node=0; start_node<flows_out_of_nodes.size(); ++start_node)
		{
			// check if a used arc is flowing out of the current start node
			std::vector<uint> originating_flows;
			bool originating = false;
			for(std::set<uint>::iterator arc=used_arcs.begin(); arc!=used_arcs.end(); ++arc)
			{
				if(contains(flows_out_of_nodes[start_node], *arc)==true)
				{
					originating = true;
					originating_flows.push_back(*arc);
				}
			}

			// if the arc is originating from this node, find its destination
			std::vector<int> current_directed_edges;
			if(originating==true)
			{
				for(uint end_node=0; end_node<flows_into_nodes.size(); ++end_node)
				{
					if(end_node==start_node)
						continue;

					for(std::vector<uint>::iterator arc=originating_flows.begin(); arc!=originating_flows.end(); ++arc)
						if(contains(flows_into_nodes[end_node], *arc)==true)
							current_directed_edges.push_back(end_node);
				}
			}

			// if the current node doesn't flow into another node insert a vector storing -1
			if(current_directed_edges.size()==0)
				current_directed_edges.push_back(-1);

			// save the found used directed edges
			directed_edges.push_back(current_directed_edges);
		}

//		// testing
//		std::cout << "used destinations: " << std::endl;
////		directed_edges[1].push_back(0);
//		for(size_t i=0; i<directed_edges.size(); ++i)
//		{
//			for(size_t j=0; j<directed_edges[i].size(); ++j)
//			{
//				std::cout << directed_edges[i][j] << " ";
//			}
//			std::cout << std::endl;
//		}
//		std::cout << std::endl;

		// construct the support graph out of the directed edges
		directedGraph support_graph(flows_out_of_nodes.size()); // initialize with the right number of edges

		int number_of_not_used_nodes = 0;
		for(size_t start=0; start<directed_edges.size(); ++start)
		{
			for(size_t end=0; end<directed_edges[start].size(); ++end)
			{
				// if no destination starting from this node could be found ignore this node
				if(directed_edges[start][end]==-1)
				{
					break;
					++number_of_not_used_nodes;
				}

				// add the directed edge
				boost::add_edge(start, directed_edges[start][end], support_graph);
			}
		}

		// search for the strongly connected components
		std::vector<int> c(flows_into_nodes.size());
		int number_of_strong_components = boost::strong_components(support_graph, boost::make_iterator_property_map(c.begin(), boost::get(boost::vertex_index, support_graph), c[0]));
		std::cout << "got " << number_of_strong_components << " strongly connected components" << std::endl;
//		for (std::vector <int>::iterator i = c.begin(); i != c.end(); ++i)
//		  std::cout << "Vertex " << i - c.begin() << " is in component " << *i << std::endl;

		// check how many cycles there are in the solution (components with a size >= 2)
		int number_of_cycles = 0;
		std::set<int> done_components; // set that stores the component indices for that the nodes already were found
		for(std::vector<int>::iterator comp=c.begin(); comp!=c.end(); ++comp)
		{
			// don't check a component more than one time
			if(done_components.find(*comp)!=done_components.end())
				continue;

			int elements = std::count(c.begin(), c.end(), *comp);
			if(elements>=2)
				++number_of_cycles;

			// check if a tsp path is computed (number of arcs is same as number of nodes), each arc is a strongly
			// connected component itself or all the nodes belong to one strongly connected component
			if(elements==used_arcs.size() || elements==flows_out_of_nodes.size())
				number_of_cycles = 0;

			// add it to done components
			done_components.insert(*comp);
		}

		// check if no cycle appears in the solution, i.e. if not each node is a component of its own or a traveling
		// salesman path has been computed (not_used_nodes+1) or each arc flows to another node
		if(number_of_cycles==0)
			cycle_free = true;

		// if a cycle appears find it and add the prevention constraint to the problem and resolve it
		if(cycle_free==false)
		{
			// go trough the components and find components with more than 1 element in it
			std::vector<std::vector<uint> > cycle_nodes;
			done_components.clear();
			for (int component=0; component<c.size(); ++component)
			{
				// check if component hasn't been done yet
				if(done_components.find(c[component])==done_components.end())
				{
					std::vector<uint> current_component_nodes;
					int elements = std::count(c.begin(), c.end(), c[component]);
//					std::cout << c[component] << std::endl;
					if(elements>=2 && elements!=used_arcs.size())
					{
						for(std::vector<int>::iterator element=c.begin(); element!=c.end(); ++element)
						{
							if(*element==c[component])
							{
								current_component_nodes.push_back(element-c.begin());
							}
						}

						// save the found cycle
						if(current_component_nodes.size()>0)
							cycle_nodes.push_back(current_component_nodes);

						// add it to done components
						done_components.insert(c[component]);
					}
				}
			}

//			std::cout << "found nodes that are part of a cycle: " << std::endl;
//			for(size_t i=0; i<cycle_nodes.size(); ++i)
//			{
//				for(size_t j=0; j<cycle_nodes[i].size(); ++j)
//				{
//					std::cout << cycle_nodes[i][j] << " ";
//				}
//				std::cout << std::endl;
//			}

			// add the cycle prevention constraints for each found cycle to the optimization problem and resolve it
			for(size_t cycle=0; cycle<cycle_nodes.size(); ++cycle)
			{
				// for each node in this cycle only use the outgoing arcs that are part of the cycle and the outflows
				// out of the cycle that don't start at the current node
//				std::cout << "searching for outflows" << std::endl;
//				for(size_t node=0; node<cycle_nodes[cycle].size(); ++node)
//				{
//					std::vector<int> cpc_indices;
//					std::vector<double> cpc_coefficients;
//
//					// gather the arcs in outgoing from all neighbors
//					for(size_t neighbor=0; neighbor<cycle_nodes[cycle].size(); ++neighbor)
//					{
//						// for the node itself gather the outflows that belong to the cycle
//						if(neighbor==node)
//						{
////							std::cout << "node itself: " << cycle_nodes[cycle][neighbor] << std::endl;
//							int flow_index = -1;
//							for(std::vector<uint>::const_iterator outflow=flows_out_of_nodes[cycle_nodes[cycle][neighbor]].begin(); outflow!=flows_out_of_nodes[cycle_nodes[cycle][neighbor]].end(); ++outflow)
//							{
//								// if the current arc is used in the solution, search for it in the incoming flows of
//								// the other nodes in the cycle
//								if(used_arcs.find(*outflow)!=used_arcs.end())
//									for(size_t new_node=0; new_node<cycle_nodes[cycle].size(); ++new_node)
//										if(contains(flows_into_nodes[cycle_nodes[cycle][new_node]], *outflow)==true)
//											flow_index=*outflow;
//							}
//
//							// if the outflow is an inflow of another cycle node, add its index to the constraint
//							if(flow_index!=-1)
//							{
//								cpc_indices.push_back(flow_index+start_arcs.size());
//								cpc_coefficients.push_back(-1.0);
//							}
//
//						}
//						// for other nodes gather outflows that are not part of the cycle
//						else
//						{
////							std::cout << "neighbor" << std::endl;
//							bool in_cycle = false;
//							for(std::vector<uint>::const_iterator outflow=flows_out_of_nodes[cycle_nodes[cycle][neighbor]].begin(); outflow!=flows_out_of_nodes[cycle_nodes[cycle][neighbor]].end(); ++outflow)
//							{
//								// search for the current flow in the incoming flows of the other nodes in the cycle
//								for(size_t new_node=0; new_node<cycle_nodes[cycle].size(); ++new_node)
//									if(contains(flows_into_nodes[cycle_nodes[cycle][new_node]], *outflow)==true)
//										in_cycle=true;
//
//								// if the arc is not in the cycle add its index
//								if(in_cycle==false)
//								{
//									cpc_indices.push_back(*outflow+start_arcs.size());
//									cpc_coefficients.push_back(1.0);
//								}
//
//								// reset the indication boolean
//								in_cycle = false;
//							}
//						}
//					}
//
////					std::cout << "adding constraint" << std::endl;
//					// add the constraint
//					solver_pointer->addRow((int) cpc_indices.size(), &cpc_indices[0], &cpc_coefficients[0], 0.0, COIN_DBL_MAX);
//				}
				// for each cycle find the arcs that lie in it

//				std::cout << "size: " << cycle_nodes[cycle].size() << std::endl;
//				std::cout << "constraint: " << std::endl;
				std::vector<int> cpc_indices;
				std::vector<double> cpc_coefficients;
				for(size_t node=0; node<cycle_nodes[cycle].size(); ++node)
				{
					for(std::vector<uint>::const_iterator outflow=flows_out_of_nodes[cycle_nodes[cycle][node]].begin(); outflow!=flows_out_of_nodes[cycle_nodes[cycle][node]].end(); ++outflow)
					{
						for(size_t neighbor=0; neighbor<cycle_nodes[cycle].size(); ++neighbor)
						{
							if(neighbor==node)
								continue;

							// if a cycle-node contains this arc is incoming node and was used in the solution, it belongs to the subset
							if(contains(flows_into_nodes[cycle_nodes[cycle][neighbor]], *outflow)==true && used_arcs.find(*outflow)!=used_arcs.end())
							{
								cpc_indices.push_back(*outflow+start_arcs.size());
								cpc_coefficients.push_back(1.0);
							}
						}
					}
				}
				solver_pointer->addRow((int) cpc_indices.size(), &cpc_indices[0], &cpc_coefficients[0], COIN_DBL_MIN , cycle_nodes[cycle].size()-1);
			}

//			testing
			solver_pointer->writeLp("lin_flow_prog", "lp");

			// resolve the problem with the new constraints
			solver_pointer->resolve();

			// create a new model with the updated optimization problem and solve it
			CbcModel new_model(*solver_pointer);
			new_model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

			CbcHeuristicFPump heuristic_new(new_model);
			new_model.addHeuristic(&heuristic_new);

//			new_model.initialSolve();
			new_model.branchAndBound();

			// retrieve new solution
			solution = new_model.solver()->getColSolution();

		}
	}while(cycle_free == false);

	for(size_t res=0; res<number_of_variables; ++res)
	{
//		std::cout << solution[res] << std::endl;
		C[res] = solution[res];
	}
}

// This Function checks if the given cv::Point is close enough to one cv::Point in the given vector. If one point gets found
// that this Point is nearer than the defined min_distance the function returns false to stop it immediately.
bool FlowNetworkExplorator::pointClose(const std::vector<cv::Point>& points, const cv::Point& point, const double min_distance)
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
// I.	Using the Sobel operator the direction of the gradient at each pixel is computed. Using this information, the direction is
//		found that suits best for calculating the edges, i.e. such that longer edges occur, and the map is rotated in this manner.
//		This allows to use the algorithm as it was and in the last step, the found path points simply will be transformed back to the
//		original orientation.
//	II.	Discretize the free space into cells that have to be visited a least once by using the sampling distance given to
//		the function. Also create a flow network by sweeping a line along the y-/x-axis and creating an edge, whenever it
//		hits an obstacle. From this hit point go back along the sweep line until the distance is equal to the coverage
//		radius, because the free space should represent the area that should be totally covered. If in both directions
//		along the sweep line no point in the free space can be found, ignore it.
//	III.	Create the matrices and vectors for the optimization problem:
//				1. The weight vector w, storing the distances between edges.
//				2. The coverage matrix V, storing which cell can be covered when going along the arcs.
//					remark: A cell counts as covered, when its center is in the coverage radius around an arc.
//				3. The sets of arcs for each node, that store the incoming and outgoing arcs
// IV.	Create and solve the optimization problems in the following order:
//			1.	Find the start node that is closest to the given starting position. This start node is used as initial step
//				in the optimization problem.
//			2.	Solve the optimization problem, using lazy constraints to prevent cycles in the solution. See the upper
//				function for further details.
//			3.	Retrieve the solution provided by the optimizer and create a path trough the environment.
//			4.	Construct a pose path out of the calculated point path.
// V.	The previous step produces a path for the field of view. If wanted this path gets mapped to the robot path s.t.
//		the field of view follows the wanted path. To do so simply a vector transformation is applied. If the computed robot
//		pose is not in the free space, another accessible point is generated by finding it on the radius around the fov
//		middlepoint s.t. the distance to the last robot position is minimized. If this is not wanted one has to set the
//		corresponding Boolean to false (shows that the path planning should be done for the robot footprint).
void FlowNetworkExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const int cell_size, const Eigen::Matrix<float, 2, 1>& robot_to_fov_middlepoint_vector, const float coverage_radius,
		const bool plan_for_footprint, const double path_eps, const double curvature_factor, const double max_distance_factor)
{
	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

//	// generate matrices for gradient in x/y direction
//	cv::Mat gradient_x, gradient_y;
//
//	// compute gradient in x direction
//	cv::Sobel(room_map, gradient_x, CV_64F, 1, 0, 5, 1.0, 0.0, cv::BORDER_DEFAULT);
//
//	// compute gradient in y direction
//	cv::Sobel(room_map, gradient_y, CV_64F, 0, 1, 5, 1.0, 0.0, cv::BORDER_DEFAULT);
//
//	// compute the direction of the gradient for each pixel and save the occurring gradients
//	std::vector<double> gradient_directions;
//	for(size_t y=0; y<room_map.rows; ++y)
//	{
//		for(size_t x=0; x<room_map.cols; ++x)
//		{
//			// check if the gradient has a value larger than zero, to only take the edge-gradients into account
//			int dx= gradient_x.at<double>(y,x);
//			int dy= gradient_y.at<double>(y,x);
//			if(dy*dy+dx*dx > 0.0)
//			{
//				double current_gradient = std::atan2(dy, dx);
//				gradient_directions.push_back(0.1*(double)((int)((current_gradient*10)+0.5)));	// round to one digit
//			}
//		}
//	}
//
//	// find the gradient that occurs most often, this direction is used to rotate the map
//	int max_number = 0;
//	double most_occurring_gradient = 0.0;
//	std::set<double> done_gradients;
//	for(std::vector<double>::iterator grad=gradient_directions.begin(); grad!=gradient_directions.end(); ++grad)
//	{
//		// if gradient has been done, don't check it again
//		if(done_gradients.find(*grad)==done_gradients.end())
//		{
//			int current_count = std::count(gradient_directions.begin(), gradient_directions.end(), *grad);
////			std::cout << "current gradient: " << *grad << ", occurs " << current_count << " times." << std::endl;
//			if(current_count > max_number)
//			{
//				max_number = current_count;
//				most_occurring_gradient = *grad;
//			}
//			done_gradients.insert(*grad);
//		}
//	}
//	std::cout << "most occurring gradient angle: " << most_occurring_gradient << std::endl;
//
//	// rotation angle of the map s.t. the most occurring gradient is in 0 degree to the x-axis
//	double rotation_angle = std::abs(most_occurring_gradient);
//	std::cout << "rotation angle: " << rotation_angle << std::endl;
//
//	// get rotation matrix R for rotating the image around the center of the room contour
//	//	Remark: rotation angle in degrees for opencv
//	std::vector < std::vector<cv::Point> > contour;
//	cv::Mat contour_map = room_map.clone();
//	cv::findContours(contour_map, contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//
//	// get the moment--> for a given map, there should only be one contour
//	cv::Moments moment = cv::moments(contour[0], false);
//
//	// calculate rotation center
//	cv::Point2f center = cv::Point(moment.m10/moment.m00 , moment.m01/moment.m00 );
//	cv::Mat R = cv::getRotationMatrix2D(center, (rotation_angle*180)/PI, 1.0);
//
//	// determine bounding rectangle to find the size of the new image
//	cv::Rect bbox = cv::RotatedRect(center, room_map.size(), (rotation_angle*180)/PI).boundingRect();
//
//	// adjust transformation matrix
//	R.at<double>(0,2) += bbox.width/2.0 - center.x;
//	R.at<double>(1,2) += bbox.height/2.0 - center.y;
//
//	// rotate the image
//	cv::Mat rotated_room_map;
//	cv::warpAffine(room_map, rotated_room_map, R, bbox.size(), cv::INTER_AREA);
//
//	// apply a binary filter to create a binary image, also use a closing operator to smooth the output (the rotation might produce
//	// black pixels reaching into the white area that were not there before, causing new, wrong cells to open)
//	cv::threshold(rotated_room_map, rotated_room_map, 200, 255, CV_THRESH_BINARY);
//	cv::dilate(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
//	cv::erode(rotated_room_map, rotated_room_map, cv::Mat(), cv::Point(-1,-1), 1);
//	cv::imshow("rotated_room_map", rotated_room_map);
//	cv::imshow("room_map", room_map);
//	cv::waitKey();

	// *********** II. Discretize the free space and create the flow network ***********
	// find the min/max coordinates
	int min_y = 1000000, max_y = 0, min_x = 1000000, max_x = 0;
	for (int y=0; y<rotated_room_map.rows; y++)
	{
		for (int x=0; x<rotated_room_map.cols; x++)
		{
			//find not reachable regions and make them black
			if (rotated_room_map.at<unsigned char>(y,x)==255)
			{
				if(y<min_y)
					min_y = y;
				if(y>max_y)
					max_y = y;
				if(x<min_x)
					min_x = x;
				if(x>max_x)
					max_x = x;
			}
		}
	}
//	min_y -= 1;
//	min_x -= 1;
//	max_y += 1;
//	max_x += 1;

//	testing
//	cv::circle(rotated_room_map, cv::Point(min_x, min_y), 3, cv::Scalar(127), CV_FILLED);
//	cv::circle(rotated_room_map, cv::Point(max_x, max_y), 3, cv::Scalar(127), CV_FILLED);
//    cv::imshow("rotated", rotated_room_map);
//    cv::waitKey();

	// todo: create grid in external class - it is the same in all approaches
	// todo: if first/last row or column in grid has accessible areas but center is inaccessible, create a node in the accessible area
	// find cell centers that need to be covered
	std::vector<cv::Point> cell_centers;
	for(size_t y=min_y; y<=max_y; y+=cell_size)
		for(size_t x=min_x; x<=max_x; x+=cell_size)
			if(rotated_room_map.at<uchar>(y,x)==255)
				cell_centers.push_back(cv::Point(x,y));

	// find edges for the flow network, sweeping along the y-axis
	std::vector<cv::Point> edges;
	int coverage_int = (int) std::floor(coverage_radius);
	std::cout << "y sweeping, radius: " << coverage_int << std::endl;
	for(size_t y=min_y-1; y<=max_y+1; ++y)
	{
//		cv::Mat test_map = rotated_room_map.clone();
//		for(std::vector<cv::Point>::iterator center=cell_centers.begin(); center!=cell_centers.end(); ++center)
//			cv::circle(test_map, *center, 2, cv::Scalar(50), CV_FILLED);
//		cv::line(test_map, cv::Point(0, y), cv::Point(rotated_room_map.cols, y), cv::Scalar(127), 1);
		for(size_t x=min_x; x<=max_x+1; x+=2.0*coverage_int)
		{
			// check if an obstacle has been found, only check outer parts of the occupied space
			if(rotated_room_map.at<uchar>(y,x)==0 && (rotated_room_map.at<uchar>(y-1,x)==255 || rotated_room_map.at<uchar>(y+1,x)==255))
			{
//				cv::circle(test_map, cv::Point(x,y), 2, cv::Scalar(127), CV_FILLED);
				// check on both sides along the sweep line if a free point is available, don't exceed matrix dimensions
				if(rotated_room_map.at<uchar>(y-coverage_int, x)==255 && y-coverage_int>=0)
					edges.push_back(cv::Point(x, y-coverage_int));
				else if(rotated_room_map.at<uchar>(y+coverage_int, x)==255 && y+coverage_int<rotated_room_map.rows)
					edges.push_back(cv::Point(x, y+coverage_int));

				// increase x according to the coverage radius, -1 because it gets increased after this step
//				x += 2.0*coverage_int-1;
			}
		}
//		cv::imshow("test", test_map);
//		cv::waitKey();
	}

	// sweep along x-axis
//	std::cout << "x sweeping" << std::endl;
//	for(size_t x=min_x-1; x<=max_x+1; ++x)
//	{
////		cv::Mat test_map = rotated_room_map.clone();
//		for(size_t y=min_y; y<=max_y+1; y+=2.0*coverage_int)
//		{
//			// check if an obstacle has been found, only check outer parts of the occupied space
//			if(rotated_room_map.at<uchar>(y,x)==0 && (rotated_room_map.at<uchar>(y,x-1)==255 || rotated_room_map.at<uchar>(y,x+1)==255))
//			{
////				cv::circle(test_map, cv::Point(x,y), 2, cv::Scalar(127), CV_FILLED);
//				// check on both sides along the sweep line if a free point is available, don't exceed matrix dimensions
//				if(rotated_room_map.at<uchar>(y, x-coverage_int)==255 && x-coverage_int>=0)
//					edges.push_back(cv::Point(x-coverage_int, y));
//				else if(rotated_room_map.at<uchar>(y, x+coverage_int)==255 && x+coverage_int<rotated_room_map.cols)
//					edges.push_back(cv::Point(x+coverage_int, y));
//
//				// increase y according to the coverage radius, -1 because it gets increased after this for step
////				y += 2.0*coverage_int-1;
//			}
//		}
////		cv::imshow("test", test_map);
////		cv::waitKey();
//	}
	std::cout << "found " << edges.size() << " edges" << std::endl;

//	cv::Mat edges_map = rotated_room_map.clone();
//	for(std::vector<cv::Point>::iterator p=edges.begin(); p!=edges.end(); ++p)
//		cv::circle(edges_map, *p, 2, cv::Scalar(100), CV_FILLED);
//	cv::imshow("edges", edges_map);
//	cv::waitKey();

	// create the arcs for the flow network
	std::cout << "Constructing distance matrix" << std::endl;
	cv::Mat distance_matrix; // determine weights
	DistanceMatrix distance_matrix_computation;
	distance_matrix_computation.constructDistanceMatrix(distance_matrix, rotated_room_map, edges, 0.25, 0.0, map_resolution, path_planner_);
	std::cout << "Constructed distance matrix, defining arcs" << std::endl;
	std::vector<arcStruct> arcs;
	double max_distance = max_y - min_y; // arcs should at least go the maximal room distance to allow straight arcs
	if(max_x-min_x>max_distance)
		max_distance=max_x-min_x;
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
				current_backward_arc.weight = distance_matrix.at<double>(end, start);
				cv::Point vector = current_forward_arc.start_point - current_forward_arc.end_point;
				// don't add too long arcs to reduce dimensionality, because they certainly won't get chosen anyway
				// also don't add arcs that are too far away from the straight line (start-end) because they are likely
				// to go completely around obstacles and are not good
				if(current_forward_arc.weight <= max_distance_factor*max_distance && current_forward_arc.weight <= curvature_factor*cv::norm(vector))
				{
					std::vector<cv::Point> astar_path;
					path_planner_.planPath(rotated_room_map, current_forward_arc.start_point, current_forward_arc.end_point, 1.0, 0.0, map_resolution, 0, &astar_path);
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
	// TODO: exclude nodes that aren't connected to the rest of the edges
	std::cout << "arcs: " << arcs.size() << std::endl;

//	testing
//	cv::Mat arc_map = rotated_room_map.clone();
//	for(size_t i=0; i<arcs.size(); ++i)
//	{
//		arcStruct current_arc = arcs[i];
//		for(size_t j=0; j<current_arc.edge_points.size(); ++j)
//			arc_map.at<uchar>(current_arc.edge_points[j]) = 127;
//	}
//	cv::imshow("test", arc_map);
//	cv::waitKey();

	// *********** III. Construct the matrices for the optimization problem ***********
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
	int number_of_outflows = 0;
	for(std::vector<cv::Point>::iterator edge=edges.begin(); edge!=edges.end(); ++edge)
	{
		for(std::vector<arcStruct>::iterator arc=arcs.begin(); arc!=arcs.end(); ++arc)
		{
			// if the start point of the arc is the edge save it as incoming flow
			if(arc->end_point == *edge)
			{
				flows_into_nodes[edge-edges.begin()].push_back(arc-arcs.begin());
			}
			// if the end point of the arc is the edge save it as outgoing flow
			else if(arc->start_point == *edge)
			{
				flows_out_of_nodes[edge-edges.begin()].push_back(arc-arcs.begin());
				++number_of_outflows;
			}
		}
	}

//	testing
//	for(size_t i=0; i<flows_into_nodes.size(); ++i)
//	{
//		std::cout << "in: " << std::endl;
//		for(size_t j=0; j<flows_into_nodes[i].size(); ++j)
//			std::cout << flows_into_nodes[i][j] << std::endl;
//		std::cout << "out: " << std::endl;
//		for(size_t j=0; j<flows_out_of_nodes[i].size(); ++j)
//			std::cout << flows_out_of_nodes[i][j] << std::endl;
//		std::cout << std::endl;
//	}
//	for(size_t node=0; node<flows_out_of_nodes.size(); ++node)
//	{
//		cv::Mat paths = rotated_room_map.clone();
//		for(size_t flow=0; flow<flows_out_of_nodes[node].size(); ++flow)
//		{
//			std::vector<cv::Point> path = arcs[flows_out_of_nodes[node][flow]].edge_points;
//			for(size_t p=0; p<path.size(); ++p)
//				paths.at<uchar>(path[p]) = 127;
//		}
//		cv::imshow("paths", paths);
//		cv::waitKey();
//	}

	std::cout << "Constructed all matrices for the optimization problem. Checking if all cells can be covered." << std::endl;

	// print out warning if a defined cell is not coverable with the chosen arcs
	bool all_cells_covered = true;
	for(size_t row=0; row<V.rows; ++row)
	{
		int number_of_paths = 0;
		for(size_t col=0; col<V.cols; ++col)
			if(V.at<uchar>(row, col)==1)
				++number_of_paths;
		if(number_of_paths==0)
		{
			std::cout << "!!!!!!!! EMPTY ROW OF VISIBILITY MATRIX !!!!!!!!!!!!!" << std::endl << "cell " << row << " not coverable" << std::endl;
			all_cells_covered = false;
		}
	}
	if(all_cells_covered == false)
		std::cout << "!!!!! WARNING: Not all cells could be covered with the given parameters, try changing them or ignore it to not cover the whole free space." << std::endl;

	// *********** IV. Solve the optimization problem ***********
	// 1. Find the start node closest to the starting position.
	double min_distance = 1e5;
	uint start_index = 0;
	for(std::vector<cv::Point>::iterator edge=edges.begin(); edge!=edges.end(); ++edge)
	{
		cv::Point difference_vector = *edge - starting_position;
		double current_distance = cv::norm(difference_vector);
		if(current_distance<min_distance)
		{
			min_distance = current_distance;
			start_index = edge-edges.begin();
		}
	}

	// 2. solve the optimization problem, using the available optimization library
	std::vector<double> C(2.0*(flows_out_of_nodes[start_index].size()+number_of_candidates) + number_of_outflows + edges.size());
	std::cout << "number of outgoing arcs: " << number_of_outflows << std::endl;
#ifdef GUROBI_FOUND
	solveGurobiOptimizationProblem(C, V, w, flows_into_nodes, flows_out_of_nodes, flows_out_of_nodes[start_index]);
#else
	solveLazyConstraintOptimizationProblem(C, V, w, flows_into_nodes, flows_out_of_nodes, flows_out_of_nodes[start_index]);
#endif

//	testing
//	for(size_t i=0; i<C.size(); ++i)
//		if(C[i]!=0)
//			std::cout << "var" << i << ": " << C[i] << std::endl;

	// 3. retrieve the solution and create a path
//	cv::Mat test_map = rotated_room_map.clone();
//	for(std::vector<cv::Point>::iterator p=edges.begin(); p!=edges.end(); ++p)
//		cv::circle(test_map, *p, 2, cv::Scalar(100), CV_FILLED);

	std::set<uint> used_arcs; // set that stores the indices of the arcs corresponding to non-zero elements in the solution
	// go trough the start arcs and determine the new start arcs
	uint path_start = 0;
//	cv::Mat test_map = rotated_room_map.clone();
//	for(std::vector<cv::Point>::iterator p=edges.begin(); p!=edges.end(); ++p)
//		cv::circle(test_map, *p, 2, cv::Scalar(100), CV_FILLED);
	for(size_t start_arc=0; start_arc<flows_out_of_nodes[start_index].size(); ++start_arc)
	{
		if(C[start_arc]>0.01) // taking integer precision in solver into account
		{
			// insert start index
//			used_arcs.insert(flows_out_of_nodes[start_index][start_arc]);
			path_start = flows_out_of_nodes[start_index][start_arc];

//			std::vector<cv::Point> path=arcs[flows_out_of_nodes[start_index][start_arc]].edge_points;
//			for(size_t j=0; j<path.size(); ++j)
//				test_map.at<uchar>(path[j])=50;
//
//			cv::imshow("discretized", test_map);
//			cv::waitKey();
		}
	}

	// go trough the coverage stage
	for(size_t cover_arc=flows_out_of_nodes[start_index].size(); cover_arc<flows_out_of_nodes[start_index].size()+arcs.size(); ++cover_arc)
	{
//		cv::Mat test_map = rotated_room_map.clone();
//		for(std::vector<cv::Point>::iterator p=edges.begin(); p!=edges.end(); ++p)
//			cv::circle(test_map, *p, 2, cv::Scalar(100), CV_FILLED);
		if(C[cover_arc]>0.01) // taking integer precision in solver into account
		{
//			std::cout << cover_arc-flows_out_of_nodes[start_index].size() << std::endl;

			// insert index, relative to the first coverage variable
			used_arcs.insert(cover_arc-flows_out_of_nodes[start_index].size());

//			std::vector<cv::Point> path=arcs[cover_arc-flows_out_of_nodes[start_index].size()].edge_points;
//			for(size_t j=0; j<path.size(); ++j)
//				test_map.at<uchar>(path[j])=100;
//
//			cv::imshow("discretized", test_map);
//			cv::waitKey();
		}
	}

	// go trough the final stage and find the remaining used arcs
	std::cout << "final: " << std::endl;
	uint path_end = 0;
	for(uint final_arc=flows_out_of_nodes[start_index].size()+arcs.size(); final_arc<flows_out_of_nodes[start_index].size()+2*arcs.size(); ++final_arc)
	{
//		cv::Mat test_map = rotated_room_map.clone();
//		for(std::vector<cv::Point>::iterator p=edges.begin(); p!=edges.end(); ++p)
//			cv::circle(test_map, *p, 2, cv::Scalar(100), CV_FILLED);
		if(C[final_arc]>0.01)
		{
			// insert saved outgoing flow index
//			used_arcs.insert(final_arc-flows_out_of_nodes[start_index].size()-V.cols);
			path_end = final_arc-flows_out_of_nodes[start_index].size()-V.cols;

//			std::vector<cv::Point> path=arcs[final_arc-flows_out_of_nodes[start_index].size()-arcs.size()].edge_points;
//			for(size_t j=0; j<path.size(); ++j)
//				test_map.at<uchar>(path[j])=150;
//
//			cv::imshow("discretized", test_map);
//			cv::waitKey();
		}
	}
//	for(size_t node=0; node<flows_out_of_nodes.size(); ++node)
//	{
//		for(size_t flow=0; flow<flows_out_of_nodes[node].size(); ++flow)
//		{
//			if(C[flows_out_of_nodes[node][flow]+flows_out_of_nodes[start_index].size()+V.cols]>0.01) // taking integer precision in solver into account
//			{
//				// insert saved outgoing flow index
//				used_arcs.insert(flows_out_of_nodes[node][flow]);
//				std::vector<cv::Point> path=arcs[flows_out_of_nodes[node][flow]].edge_points;
//				for(size_t j=0; j<path.size(); ++j)
//					test_map.at<uchar>(path[j])=150;
//
//				cv::imshow("discretized", test_map);
//				cv::waitKey();
//			}
//		}
//	}
//	cv::imshow("discretized", test_map);
//	cv::waitKey();
	std::cout << "got " << used_arcs.size() << " used arcs" << std::endl;

//	// testing --> check how often a node is a start/end-node of the arcs
//	std::cout << "appereances of the nodes: " << std::endl;
//	cv::Mat node_map = rotated_room_map.clone();
//	for(size_t i=0; i<edges.size(); ++i)
//	{
//		int number=0;
//		for(std::set<uint>::iterator used=used_arcs.begin(); used!=used_arcs.end(); ++used)
//			if(contains(flows_out_of_nodes[i], *used)==true || contains(flows_into_nodes[i], *used)==true)
//				++number;
//		std::cout << "n" << i << ": " << number << std::endl;
//
//		if(i==5)
//		{
//			cv::circle(node_map, edges[i], 2, cv::Scalar(127), CV_FILLED);
//			for(std::set<uint>::iterator used=used_arcs.begin(); used!=used_arcs.end(); ++used)
//			{
//				if(contains(flows_out_of_nodes[i], *used)==true || contains(flows_into_nodes[i], *used)==true)
//				{
//					std::vector<cv::Point> points=arcs[*used].edge_points;
//					for(size_t j=0; j<points.size(); ++j)
//						node_map.at<uchar>(points[j])=150;
//				}
//			}
//			cv::imshow("node", node_map);
//			cv::waitKey();
//		}
//	}


	// starting from the start node, go trough the arcs and create a coverage path
	std::vector<cv::Point> path_positions;
	path_positions.push_back(edges[start_index]);
	cv::Point last_point = edges[start_index];
	int last_index = start_index;
	std::set<uint> gone_arcs;
	std::cout << "getting path using arcs" << std::endl;
	// start path at start node
	std::vector<cv::Point> start_edge = arcs[path_start].edge_points;
	for(std::vector<cv::Point>::iterator pos=start_edge.begin(); pos!=start_edge.end(); ++pos)
	{
		cv::Point difference = last_point - *pos;
		// if the next point is far enough away from the last point insert it into the coverage path
		if(difference.x*difference.x+difference.y*difference.y<=path_eps*path_eps)
		{
			path_positions.push_back(*pos);
			last_point = *pos;
		}
	}
	// get index of the start arcs end-node
	cv::Point end_start_node = arcs[path_start].end_point;
	last_index = std::find(edges.begin(), edges.end(), end_start_node)-edges.begin();
	// TODO: find path in directed graph, covering all edges --> allow cycles connected to the rest
	int number_of_gone_arcs = 0, loopcounter = 0;
	do
	{
		++loopcounter;
//		std::cout << "n: " << number_of_gone_arcs << std::endl;
		// go trough the arcs and find the one that has the last point as begin and go along it
		for(std::set<uint>::iterator arc_index=used_arcs.begin(); arc_index!=used_arcs.end(); ++arc_index)
		{
//			std::cout << arcs[*arc_index].start_point << " " << last_point << std::endl;
//			if(arcs[*arc_index].start_point==last_point)
//			{
//				std::vector<cv::Point> edge = arcs[*arc_index].edge_points;
//				for(std::vector<cv::Point>::iterator pos=edge.begin(); pos!=edge.end(); ++pos)
//				{
//					cv::Point difference = last_point - *pos;
//					// if the next point is far enough away from the last point insert it into the coverage path
//					if(difference.x*difference.x+difference.y*difference.y<=path_eps*path_eps)
//					{
//						path_positions.push_back(*pos);
//						last_point = *pos;
//					}
//				}
//
//				// increase number of gone arcs
//				++number_of_gone_arcs;
//			}

			// check if current arc has been gone already (e.g. when a cycle is appended to the rest of the path --> still a valid solution)
			if(gone_arcs.find(*arc_index)!=gone_arcs.end())
				continue;

			if(contains(flows_out_of_nodes[last_index], *arc_index)==true)
			{
				std::vector<cv::Point> edge = arcs[*arc_index].edge_points;
				for(std::vector<cv::Point>::iterator pos=edge.begin(); pos!=edge.end(); ++pos)
				{
					cv::Point difference = last_point - *pos;
					// if the next point is far enough away from the last point insert it into the coverage path
					if(difference.x*difference.x+difference.y*difference.y<=path_eps*path_eps)
					{
						path_positions.push_back(*pos);
						last_point = *pos;
					}
				}

				// get index of last edge
				cv::Point node = arcs[*arc_index].end_point;
				last_index = std::find(edges.begin(), edges.end(), node)-edges.begin();
				gone_arcs.insert(*arc_index);

				// increase number of gone arcs
				++number_of_gone_arcs;

				// reset the loop-counter
				loopcounter = 0;
			}
//			std::cout << number_of_gone_arcs << std::endl;
		}
	}while(number_of_gone_arcs<used_arcs.size() && loopcounter<=100);
	// end the path at the final stage
	std::vector<cv::Point> final_edge = arcs[path_end].edge_points;
	for(std::vector<cv::Point>::iterator pos=final_edge.begin(); pos!=final_edge.end(); ++pos)
	{
		cv::Point difference = last_point - *pos;
		// if the next point is far enough away from the last point insert it into the coverage path
		if(difference.x*difference.x+difference.y*difference.y<=path_eps*path_eps)
		{
			path_positions.push_back(*pos);
			last_point = *pos;
		}
	}
	std::cout << "got path" << std::endl;

	// transform the calculated path back to the originally rotated map and create poses with an angle
	std::vector<geometry_msgs::Pose2D> fov_poses;
	std::vector<cv::Point2f> path_positions_2f(path_positions.size());
	for (size_t i=0; i<path_positions.size(); ++i)
		path_positions_2f[i] = cv::Point2f(path_positions[i].x, path_positions[i].y);
	room_rotation.transformPathBackToOriginalRotation(path_positions_2f, fov_poses, R);

//	// 4. calculate a pose path out of the point path
//	std::vector<geometry_msgs::Pose2D> fov_poses;
//	for(unsigned int point_index=0; point_index<path_positions.size(); ++point_index)
//	{
//		// get the vector from the current point to the next point
//		cv::Point current_point = path_positions[point_index];
//		cv::Point next_point = path_positions[(point_index+1)%(path_positions.size())];
//		cv::Point vector = next_point - current_point;
//
//		float angle = std::atan2(vector.y, vector.x);
//
//		// add the next navigation goal to the path
//		geometry_msgs::Pose2D current_pose;
//		current_pose.x = current_point.x;
//		current_pose.y = current_point.y;
//		current_pose.theta = angle;
//
//		fov_poses.push_back(current_pose);
//	}


	// if the path should be planned for the robot footprint create the path and return here
	if(plan_for_footprint == true)
	{
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=fov_poses.begin(); pose != fov_poses.end(); ++pose)
		{
			geometry_msgs::Pose2D current_pose;
			current_pose.x = (pose->x * map_resolution) + map_origin.x;
			current_pose.y = (pose->y * map_resolution) + map_origin.y;
			current_pose.theta = pose->theta;
			path.push_back(current_pose);
		}
		return;
	}

	// *********************** V. Get the robot path out of the fov path. ***********************
	// clean path from double occurrences of the same pose in a row
	std::vector<geometry_msgs::Pose2D> fov_path;
	fov_path.push_back(fov_poses[0]);
	cv::Point last_added_point(fov_poses[0].x, fov_poses[0].y);
	const double min_dist_squared = 5 * 5;	// [pixel]
	for (size_t i=1; i<fov_poses.size(); ++i)
	{
		const cv::Point current_point(fov_poses[i].x, fov_poses[i].y);
		cv::Point vector = current_point - last_added_point;
		if (vector.x*vector.x+vector.y*vector.y > min_dist_squared || i==fov_poses.size()-1)
		{
			fov_path.push_back(fov_poses[i]);
			last_added_point = current_point;
		}
	}

	// go trough all computed fov poses and compute the corresponding robot pose
	std::cout << "mapping path" << std::endl;
	mapPath(room_map, path, fov_path, robot_to_fov_middlepoint_vector, map_resolution, map_origin, starting_position);

////	testing
//	// transform the found path back to the original map
//	cv::invertAffineTransform(R, R);
//	cv::transform(path_positions, path_positions, R);
//	cv::Mat path_map = room_map.clone();
////	cv::imshow("solution", test_map);
////	cv::imwrite("/home/rmbce/Pictures/room_exploration/coverage_path.png", test_map);
//	for(std::vector<cv::Point>::iterator point=path_positions.begin(); point!=path_positions.end(); ++point)
//	{
//		cv::circle(path_map, *point, 2, cv::Scalar(127), CV_FILLED);
////		cv::imshow("path", path_map);
////		cv::waitKey();
//	}
//	cv::imshow("path", path_map);
//	cv::waitKey();
}

// test function for an easy case to check correctness
void FlowNetworkExplorator::testFunc()
{
//	std::vector<double> w(6, 1.0);
//	std::vector<int> C(2+6+6);
//	cv::Mat V = cv::Mat(8, 6, CV_8U, cv::Scalar(0));
//	std::vector<std::vector<uint> > flows_out_of_nodes(3);
//	std::vector<std::vector<uint> > flows_in_nodes(3);
//
//	// cell 1
//	V.at<uchar>(0,0) = 1;
//	V.at<uchar>(0,1) = 1;
//	//cell 2
//	V.at<uchar>(1,0) = 1;
//	V.at<uchar>(1,1) = 1;
//	// cell 3
//	V.at<uchar>(2,4) = 1;
//	V.at<uchar>(2,5) = 1;
//	// cell 4
//	V.at<uchar>(3,0) = 1;
//	V.at<uchar>(3,1) = 1;
//	V.at<uchar>(3,4) = 1;
//	V.at<uchar>(3,5) = 1;
//	// cell 5
//	V.at<uchar>(4,0) = 1;
//	V.at<uchar>(4,1) = 1;
//	V.at<uchar>(4,2) = 1;
//	V.at<uchar>(4,3) = 1;
//	// cell 6
//	V.at<uchar>(5,2) = 1;
//	V.at<uchar>(5,3) = 1;
//	// cell 7
//	V.at<uchar>(6,4) = 1;
//	V.at<uchar>(6,5) = 1;
//	// cell 8
//	V.at<uchar>(7,2) = 1;
//	V.at<uchar>(7,3) = 1;
//
//	flows_out_of_nodes[0].push_back(0);
//	flows_out_of_nodes[0].push_back(4);
//	flows_out_of_nodes[1].push_back(1);
//	flows_out_of_nodes[1].push_back(2);
//	flows_out_of_nodes[2].push_back(3);
//	flows_out_of_nodes[2].push_back(5);
//
//	flows_in_nodes[0].push_back(1);
//	flows_in_nodes[0].push_back(5);
//	flows_in_nodes[1].push_back(0);
//	flows_in_nodes[1].push_back(3);
//	flows_in_nodes[2].push_back(2);
//	flows_in_nodes[2].push_back(4);
//
//	for(size_t row=0; row<V.rows; ++row)
//	{
//		for(size_t col=0; col<V.cols; ++col)
//		{
//			std::cout << (int) V.at<uchar>(row, col) << " ";
//		}
//		std::cout << std::endl;
//	}
	std::vector<double> w(14, 1.0);
	std::vector<double> C(2+14+14+2+14+6);
	cv::Mat V = cv::Mat(12, 14, CV_8U, cv::Scalar(0));
	std::vector<std::vector<uint> > flows_out_of_nodes(6);
	std::vector<std::vector<uint> > flows_in_nodes(6);

	// cell 1
	V.at<uchar>(0,0) = 1;
	V.at<uchar>(0,1) = 1;
	//cell 2
	V.at<uchar>(1,0) = 1;
	V.at<uchar>(1,1) = 1;
	V.at<uchar>(1,4) = 1;
	V.at<uchar>(1,5) = 1;
	// cell 3
	V.at<uchar>(2,4) = 1;
	V.at<uchar>(2,5) = 1;
	V.at<uchar>(2,10) = 1;
	V.at<uchar>(2,11) = 1;
	// cell 4
	V.at<uchar>(3,0) = 1;
	V.at<uchar>(3,1) = 1;
	// cell 5
	V.at<uchar>(4,0) = 1;
	V.at<uchar>(4,1) = 1;
	V.at<uchar>(4,6) = 1;
	V.at<uchar>(4,7) = 1;
	// cell 6
	V.at<uchar>(5,6) = 1;
	V.at<uchar>(5,7) = 1;
	V.at<uchar>(5,10) = 1;
	V.at<uchar>(5,11) = 1;
	// cell 7
	V.at<uchar>(6,2) = 1;
	V.at<uchar>(6,3) = 1;
	// cell 8
	V.at<uchar>(7,2) = 1;
	V.at<uchar>(7,3) = 1;
	V.at<uchar>(7,6) = 1;
	V.at<uchar>(7,7) = 1;
	// cell 9
	V.at<uchar>(8,6) = 1;
	V.at<uchar>(8,7) = 1;
	V.at<uchar>(8,12) = 1;
	V.at<uchar>(8,13) = 1;
	// cell 10
	V.at<uchar>(9,2) = 1;
	V.at<uchar>(9,3) = 1;
	// cell 11
	V.at<uchar>(10,2) = 1;
	V.at<uchar>(10,3) = 1;
	V.at<uchar>(10,8) = 1;
	V.at<uchar>(10,9) = 1;
	// cell 12
	V.at<uchar>(11,8) = 1;
	V.at<uchar>(11,9) = 1;
	V.at<uchar>(11,12) = 1;
	V.at<uchar>(11,13) = 1;

	flows_out_of_nodes[0].push_back(0);
	flows_out_of_nodes[0].push_back(4);

	flows_out_of_nodes[1].push_back(1);
	flows_out_of_nodes[1].push_back(2);
	flows_out_of_nodes[1].push_back(6);

	flows_out_of_nodes[2].push_back(3);
	flows_out_of_nodes[2].push_back(8);

	flows_out_of_nodes[3].push_back(5);
	flows_out_of_nodes[3].push_back(10);

	flows_out_of_nodes[4].push_back(7);
	flows_out_of_nodes[4].push_back(11);
	flows_out_of_nodes[4].push_back(12);

	flows_out_of_nodes[5].push_back(9);
	flows_out_of_nodes[5].push_back(13);


	flows_in_nodes[0].push_back(1);
	flows_in_nodes[0].push_back(5);

	flows_in_nodes[1].push_back(0);
	flows_in_nodes[1].push_back(3);
	flows_in_nodes[1].push_back(7);

	flows_in_nodes[2].push_back(2);
	flows_in_nodes[2].push_back(9);

	flows_in_nodes[3].push_back(4);
	flows_in_nodes[3].push_back(11);

	flows_in_nodes[4].push_back(6);
	flows_in_nodes[4].push_back(10);
	flows_in_nodes[4].push_back(13);

	flows_in_nodes[5].push_back(8);
	flows_in_nodes[5].push_back(12);

	for(size_t row=0; row<V.rows; ++row)
	{
		for(size_t col=0; col<V.cols; ++col)
		{
			std::cout << (int) V.at<uchar>(row, col) << " ";
		}
		std::cout << std::endl;
	}

	std::vector<double> W(C.size(), 1.0);
	w[10] = 0.25;
	w[11] = 0.25;
	w[12] = 0.25;
	w[13] = 0.25;
	double weight_epsilon = 0.0;
	double euler_constant = std::exp(1.0);
	for(size_t i=1; i<=1; ++i)
	{

//		solveThreeStageOptimizationProblem(C, V, w, flows_in_nodes, flows_out_of_nodes, flows_out_of_nodes[0]);//, &W);
		solveGurobiOptimizationProblem(C, V, w, flows_in_nodes, flows_out_of_nodes, flows_out_of_nodes[0]);
		for(size_t c=0; c<C.size(); ++c)
			std::cout << C[c] << std::endl;
		std::cout << std::endl;

		int exponent = 1 + (i - 1)*0.1;
		weight_epsilon = std::pow(1/(euler_constant-1), exponent);
		for(size_t weight=0; weight<W.size(); ++weight)
		{
			W[weight] = weight_epsilon/(weight_epsilon + C[weight]);
			std::cout << W[weight] << std::endl;
		}
		std::cout << std::endl;

		cv::imshow("V", V);
		cv::waitKey();
	}

	std::set<uint> used_arcs; // set that stores the indices of the arcs corresponding to non-zero elements in the solution
	// go trough the start arcs and determine the new start arcs
	std::cout << "initial: " << std::endl;
	uint n = (uint) V.cols;
	for(size_t start_arc=0; start_arc<flows_out_of_nodes[0].size(); ++start_arc)
	{
		if(C[start_arc]!=0)
		{
			// insert start index
			used_arcs.insert(flows_out_of_nodes[0][start_arc]);
			std::cout << flows_out_of_nodes[0][start_arc] << std::endl;
		}
	}

	// go trough the coverage stage
	std::cout << "coverage: " << std::endl;
	for(size_t arc=flows_out_of_nodes[0].size(); arc<flows_out_of_nodes[0].size()+n; ++arc)
	{
		if(C[arc]!=0)
		{
			// insert index, relative to the first coverage variable
			used_arcs.insert(arc-flows_out_of_nodes[0].size());

			std::cout << arc-flows_out_of_nodes[0].size() << std::endl;
		}
	}

	// go trough the final stage and find the remaining used arcs
	std::cout << "final: " << std::endl;
	for(uint flow=flows_out_of_nodes[0].size()+V.cols; flow<flows_out_of_nodes[0].size()+2*n; ++flow)
	{
		if(C[flow]>0)
		{
			// insert saved outgoing flow index
			used_arcs.insert(flow-flows_out_of_nodes[0].size()-n);
		}
	}
//	for(size_t node=0; node<flows_out_of_nodes.size(); ++node)
//	{
//		for(size_t flow=0; flow<flows_out_of_nodes[node].size(); ++flow)
//		{
//			if(C[flows_out_of_nodes[node][flow]+flows_out_of_nodes[0].size()+V.cols]!=0)
//			{
//				// insert saved outgoing flow index
//				used_arcs.insert(flows_out_of_nodes[node][flow]);
//
//				std::cout << flows_out_of_nodes[node][flow] << std::endl;
//			}
//		}
//	}

	std::cout << "got " << used_arcs.size() << " used arcs" << std::endl;

	// remove the first initial column
	uint new_number_of_variables = 0;
	cv::Mat V_reduced = cv::Mat(V.rows, 1, CV_8U); // initialize one column because opencv wants it this way, add other columns later
	for(std::set<uint>::iterator var=used_arcs.begin(); var!=used_arcs.end(); ++var)
	{
		// gather column corresponding to this candidate pose and add it to the new observability matrix
		cv::Mat column = V.col(*var);
		cv::hconcat(V_reduced, column, V_reduced);
	}
	V_reduced = V_reduced.colRange(1, V_reduced.cols);

	for(size_t row=0; row<V_reduced.rows; ++row)
	{
		int one_count = 0;
		for(size_t col=0; col<V_reduced.cols; ++col)
		{
			std::cout << (int) V_reduced.at<uchar>(row, col) << " ";
			if(V_reduced.at<uchar>(row, col)!=0)
				++one_count;
		}
		std::cout << std::endl;
		if(one_count == 0)
			std::cout << "!!!!!!!!!!!!! empty row !!!!!!!!!!!!!!!!!!" << std::endl;
	}

	V_reduced = V_reduced.colRange(1, V_reduced.cols);

	std::cout << "read out: " << std::endl;
	for(size_t c=0; c<C.size(); ++c)
		std::cout << C[c] << std::endl;

//	QSprob problem;
//	problem = QSread_prob("int_lin_flow_prog.lp", "LP");
//	int status=0;
//	QSget_intcount(problem, &status);
//	std::cout << "number of integer variables in the problem: " << status << std::endl;
//	int* intflags = (int *) malloc (14 * sizeof (int));
//	QSget_intflags (problem, intflags);
//    printf ("Integer Variables\n");
//    for (int j = 0; j < 14; j++)
//    {
//        if (intflags[j] == 1)
//        {
//            printf ("%d ", j);
//        }
//    }
//    printf ("\n");
//	QSopt_dual(problem, NULL);
//	double* result;
//	result  = (double *) malloc(14 * sizeof (double));
//	QSget_solution(problem, NULL, result, NULL, NULL, NULL);
//	for(size_t variable=0; variable<14; ++variable)
//	{
//		std::cout << result[variable] << std::endl;
//	}
//	QSwrite_prob(problem, "lin_flow_prog.lp", "LP");

	OsiClpSolverInterface LP_solver;
	OsiClpSolverInterface* solver_pointer = &LP_solver;

	double obj[] = {1, 1, 1, 1, 1, 1, 1, 1};
	double lower[] = {0, 0, 0, 0, 0, 0, 0, 0};
	double upper[] = {1, 1, 1, 1, 1, 1, 1, 1};
	int which_int[] = {0, 1, 2, 3, 4, 5, 6, 7};
	int initial_constr[] = {0, 1};
	int cover_constr1[] = {0, 1, 2, 3, 5, 6};
	int cover_constr2[] = {4, 7};
	int cover_constr3[] = {0, 2, 4, 5, 7};
	int cover_constr4[] = {1, 3, 4, 6, 7};
	int con_constr[] = {1, 3, 4, 7};
	int final_constr[] = {5, 6, 7};
	double init_constr_obj[] = {1, 1};
	double cover_obj1[] = {1, 1, 1, 1, 1, 1};
	double cover_obj2[] = {1, 1};
	double cover_obj3[] = {1, 1, 1, 1, 1};
	double cover_obj4[] = {1, 1, 1, 1, 1};
	double con_obj[] = {1, 1, -1, -1};
	double final_constr_obj[] = {1, 1, 1};
	int numberColumns=(int) (sizeof(lower)/sizeof(double));

	CoinModel problem_builder;

	for(size_t i=0; i<numberColumns; ++i)
	{
		problem_builder.setColBounds(i, lower[i], upper[i]);
		problem_builder.setObjective(i, obj[i]);

		problem_builder.setInteger(i);
	}

	problem_builder.addRow(2, initial_constr, init_constr_obj, 1, 1);
	problem_builder.addRow(6, cover_constr1, cover_obj1, 1);
	problem_builder.addRow(2, cover_constr2, cover_obj2, 1);
	problem_builder.addRow(5, cover_constr3, cover_obj3, 1);
	problem_builder.addRow(5, cover_constr4, cover_obj4, 1);
	problem_builder.addRow(4, con_constr, con_obj, 0, 0);
	problem_builder.addRow(3, final_constr, final_constr_obj, 1, 1);

	solver_pointer->loadFromCoinModel(problem_builder);

	solver_pointer->writeLp("test_lp", "lp");

	CbcModel model(*solver_pointer);
	model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

//	CbcHeuristicLocal heuristic2(model);
//	model.addHeuristic(&heuristic2);

	model.initialSolve();
	model.branchAndBound();

	const double * solution = model.solver()->getColSolution();

	for(size_t i=0; i<numberColumns; ++i)
		std::cout << solution[i] << std::endl;
	std::cout << std::endl;

//  CglProbing generator1;
//  generator1.setUsingObjective(true);
//  generator1.setMaxPass(3);
//  generator1.setMaxProbe(100);
//  generator1.setMaxLook(50);
//  generator1.setRowCuts(3);
}

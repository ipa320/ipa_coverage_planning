# ipa_building_navigation
Algorithms for systematic coverage of different spaces in an optimal traveling order.

If you find this software useful in your work, please cite our corresponding paper:
- R. Bormann, F. Jordan, J. Hampp, and M. Hägele. Indoor coverage path planning: Survey, implementation, analysis. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1718–1725, May 2018. https://ieeexplore.ieee.org/abstract/document/8460566 , https://publica.fraunhofer.de/entities/publication/f537c15d-4cbe-4672-9d86-6e756a9ce71b/details

# General Procedure

1. Change the algorithm parameters in ros/launch/room_sequence_planning_action_server_params.yaml in ros/launch to the wanted algorithms and settings.
	* tsp_solver: Choose which traveling salesman problem (TSP) solver you want to use. 
	* planning_method: Choose which planning method for the trolley you want to use. 
		  * for both planning methods the parameter **max_clique_path_length** determines how far two rooms can be away from each other until they get separated into two different cliques.
	* map_downsampling_factor: The algorithm plans an Astar-path trough the environment to determine distances between the roomcenters that are used as edge weights for the TSP solver. For this you can set this parameter to reduce the size of the map, which speeds up the pathfinding. The originally sized map is checked, if no path for the downsampled map could be found. **Range**: 0<factor<=1
	* check_accessibility_of_rooms: Choose if you want the action server to check which given roomcenters are accessible from the starting position. 
	* return_sequence_map: If enabled, the server returns an image containing the drawn in cliques, that contain the corresponding roomcenters and trolley positions, and also shows the visiting order of each clique and in each clique.
	       * display_map: If true, the server displays the sequence map. **REMARK**: Only possible if the sequence map should be returned, since this enables the computation of it.
	* maximum_clique_size: This parameter determines how many roomcenters one clique is allowed to have. this can be useful e.g. when you have a limited number of rooms the robot can clean with one battery loading. If you want to allow all cliques, simply set this parameter very high. E.g. for the cases that result from the room-segmentations in http://wiki.ros.org/ipa_room_segmentation a value of **9001** was more than enough. Of course if in your case more than 9001 centers occur, this is not enough, so when setting this parameter you unfortunately have to think of the way you use this package.
	
2. Start the action server using the file /ros/launch/room_sequence_planning_action_server.launch, which executes the /ros/src/room_sequence_planning_action_server.cpp file. If the server crashes for some reason (e.g. no valid map given by the client) it respawns with a little delay.

3. Start an action client, which sends a goal to the action server, corresponding to the FindRoomSequenceWithCheckpoints.action message, which lies in ipa_building_msgs/action. The goal consists of the following parts

    * input_map: The map for which the visiting sequence should be planned, as sensor_msgs/Image. **Has to be a 8-Bit single channel image, with 0 as occupied space and 255 as free space**.
    * map_resolution: The resolution the map has been sampled with [meter/cell].
    * map_origin: The origin of the map in [meter] to bring the provided map coordinates in absolute coordinates, if wanted.
    * room_information_in_pixel: Gives for each room in the map the min/max coordinates of the pixels belonging to this room and the computed center of it. See ipa_building_msgs/msg/RoomInformation.msg for further details.
    * robot_radius: The radius of your robot in [meter]. The given map will be eroded corresponding to this radius, s.t. each white pixel in the image can actually be reached by the robot.
    * robot_start_coordinate: Determines the position of your robot before executing the computed sequence. This is used to find the room that is closest to this position and starts the sequence there. 
    
4. The action server returns as result the sequence of cliques. This is based on ipa_building_msgs/RoomSequence.msg and gives for each clique
    * room_indices: The room indices, corresponding to the positions in room_information_in_pixel, belonging to this clique.
    * checkpoint_position_in_pixel/.._in_meter: The trolley location for this clique.
    
A Client also can change the parameters of the server by using dynamic_reconfigure. This can be done using the dynamic_reconfigure_client.h file that provides a client for this purpose. With client.setConfig("param_name", value) one specific parameter can be changed from the client, see ros/src/room_sequence_planning_action_client.cpp for an example. Of course rqt_reconfigure can also be used to change specific parameters.

The algorithms are implemented in common/src, using the headers in common/include/ipa_building_navigation.

The first planning method is faster than the second one, but may give worse results because of the underlying algorithm. The choice of the TSP solver depends heavily on the scale of your problem. The nearest neighbor solver is significantly faster than the concorde solver, but of course gives bad results in large scale problems. An advantage of our second planning procedure is, that the server divides the problem into smaller subproblems, meaning a TSP over the trolley positions and a TSP for each clique over the rooms belonging to this clique. This reduces the dimensionality for each problem and allows in most cases to get good results with the genetic solver that approximates the best solution, so in most cases this this solver should do fine. If you have very large problems with hundreds of rooms or you want exactly the optimal tour and not just an approximation of it, the concorde solver is the best choice.

# Available TSP solvers

1. Nearest Neighbor solver: An approximate TSP solver that starts at the given start-point and iteratively goes to the node that is nearest to the last node of the path. At the end, the path returns to the start-point again to close the tour. 

2. Genetic solver: This solver is based on the work of Chatterjee et. al. [1]. The proposed method takes the nearest neighbor path and uses a genetic optimization algorithm to iteratively improve the computed path.

3. Concorde solver: This solver is based on the Concorde TSP solver package, obtained from Applegate et. al. [3], which is free for academic research. It provides an exact TSP solver that has proven to obtain the optimal solution for several large TSPs in a rather short time. Anyway this solver of course is a little bit slower than the other solvers, but gives the optimal solution.

# Available planning algorithms

1. Trolley drag method: This method is very intuitive, but produces not the best results. The trolley starts at the given robot starting position and stays there for the first clique. Then a TSP over all rooms is solved to get an optimal visiting order. Following the tour, the algorithm checks which room is still in the range you defined from the current trolley location and adds these rooms to one clique. When the next room is too far away from the trolley, a new clique is opened and the trolley is dragged to the room opening it. When the last clique will become larger than the specified max. size of one clique, a new one is also opened. This is done until all rooms have been assigned to cliques.

2. Room-group method: For this method the whole rooms are spanned as a graph, with edges between two different rooms if the path-distance between them is not larger than the specified parameter. In this graph then a set-cover problem is solved, finding the biggest cliques in the graph, i.e. subgraphs in which all nodes are connected to all other nodes in it, to cover the whole graph. This produces several groups that are reachable with the given max. travel distance from each other. For each group then a central trolley position is computed s.t. the travel distance from the trolley to all rooms is minimized. At last, two TSPs are solved, one for the computed trolley positions and one for each room to determine the optimal room visiting order.  
    
# References

[1] Chatterjee, S., Carrera, C., and Lynch, L. A. Genetic algorithms and
traveling salesman problems. European journal of operational research 93, 3 (1996),
490–510.

[2] Applegate, D., Bixby, R., Chvatal, V., and Cook, W. Concorde tsp solver.
http://www.math.uwaterloo.ca/tsp/concorde.html, 2006.

In this pakage an Astar pathplanning algorithm is implemented. It was provided and slightly changed from:

	http://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/
It was released under the MIT-license (https://en.wikipedia.org/wiki/MIT_License), so it is freely usable for everyone.

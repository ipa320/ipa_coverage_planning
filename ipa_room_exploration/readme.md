# General Procedure

1. Change the algorithm parameters in ros/launch/room_room_exploration_action_server_params.yaml in ros/launch to the wanted algorithms and settings.
	* room_exploration_algorithm: Choose which exploration algorithm you want to use. 
	* plan_for_footprint: Choose if you want to plan the coverage path for the field of view (fov) or the robot footprint. When planning for the fov, the generated path is transformed to robot poses, s.t. the given fov follows the generated path. 
	* revisit_areas: Choose if you want to revisit areas, that haven't been covered bythe robot or the fov during the execution of the coverage path.
	       * left_sections_min_area: When the revisiting of areas is enabled, this parameter defines how small not covered ares can be, before they will not be taken into account any further. This is useful, when you rather want a fast execution, than a very detailed coverage. This parameter is in [meter^2].
	* goal_eps: This parameter allows the action server to publish a new navigation goal, when the robot is in a specific range around the current goal. This allows the path to be more smoother, but of course not so exactly. If you don't want this feature, then simpply set this parameter to 0. This parameter is in [pixel].
	* Each exploration algorithm has it's own specific parameters, see the ros/launch/room_room_exploration_action_server_params.yaml for further descriptions of these. 
	
2. Start the action server using the file /ros/launch/room_exploration_action_server.launch, which executes the /ros/src/room_exploration_action_server.cpp file. If the server crashes for some reason (e.g. no valid map given by the client) it respawns with a little delay.

3. Start an action client, which sends a goal to the action server, corresponding to the RoomExploration.action message, which lies in ipa_building_msgs/action. The goal consists of the following parts

    * input_map: The map of the whole area the robot moves in, as sensor_msgs/Image. **Has to be a 8-Bit single channel image, with 0 as occupied space and 255 as free space**.
    * map_resolution: 
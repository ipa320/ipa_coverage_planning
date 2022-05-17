# ipa_room_exploration
Algorithms for systematic coverage driving patterns.

If you find this software useful in your work, please cite our corresponding paper:
- R. Bormann, F. Jordan, J. Hampp, and M. Hägele. Indoor coverage path planning: Survey, implementation, analysis. In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA), pages 1718–1725, May 2018. https://ieeexplore.ieee.org/abstract/document/8460566 , https://publica.fraunhofer.de/entities/publication/f537c15d-4cbe-4672-9d86-6e756a9ce71b/details

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
    * map_resolution: The resolution of the map as float32 in [meter/cell]
    * map_origin: The origin of the map, will be used to transform the computed map-coordinates into real-world-coordinates.
    * robot_radius: The radius of the robot as float32, used to find the areas that the robot can access, in [meter].
    * coverage_radius: The radius of the device that should cover the defined area as float32. This can be a cleaning device or an inspection tool, the value should be given in [meter].
    * field_of_view: Array of 4 geometry_msgs/Points32 that define the field of view or footprint (points around robot origin) of the robot. Used to check what areas have been left uncovered during the execution (only used when the action server shall execute the path).
    * field_of_view_origin: The mounting position of the camera spanning the field of view, relative to the robot center (x-axis points to robot's front side, y-axis points to robot's left side, z-axis upwards), in [meter].
    * starting_position: The position at which the robot shall start the execution of the path in the room coordinate system [meter, meter, rad].
    * planning_mode: Int32 that controls if the robot footprint or the defined field-of-view shall cover the whole area (1: footprint, 2: fov).

sensor_msgs/Image input_map				# the action server need a map as a input image to segment it, IMPORTANT: The algorithm needs a black and white 8bit single-channel image, which is 0 (black) for obstacles and 255 (white) for free space
float32 map_resolution					# resolution of the given map, in [meter/cell]
geometry_msgs/Pose map_origin			# the origin of the map, in [meter]
geometry_msgs/Pose2D[] path				# check the coverage along this path of the robot center, in the world frame in [meter]
geometry_msgs/Point32[] field_of_view	# the points that define the field of view of the robot, relative to the robot center (x-axis points to robot's front side, y-axis points to robot's left side, z-axis upwards), in [meter]
geometry_msgs/Point32 field_of_view_origin	# the mounting position of the camera spanning the field of view, relative to the robot center (x-axis points to robot's front side, y-axis points to robot's left side, z-axis upwards), in [meter]
float32 coverage_radius					# radius that is used to plan the coverage planning for the robot and not the field of view, assuming that the part that needs to cover everything (e.g. the cleaning part) can be represented by a fitting circle (e.g. smaller than the actual part to ensure coverage), in [meter]
bool check_for_footprint				# determine, if the coverage check should be done for the footprint or the field of view
bool check_number_of_coverages			# if set, the server returns a map that shows how often one pixel has been covered during the path, return format: 32bit single-channel image
---
sensor_msgs/Image coverage_map			# the map that has the covered areas drawn in, with a value of 255, an 8bit single-channel image
sensor_msgs/Image number_of_coverage_image	# the image that carries for each pixel the number of coverages when executing the path, 32bit single-channel image

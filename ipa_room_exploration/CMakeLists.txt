cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_exploration)

set(catkin_RUN_PACKAGES
	actionlib
	actionlib_msgs
	cv_bridge
	geometry_msgs
	ipa_building_msgs
	libconcorde_tsp_solver
	roscpp
	roslib
	sensor_msgs
	std_msgs
	tf
	move_base_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	dynamic_reconfigure
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

generate_dynamic_reconfigure_options(
  cfg/RoomExploration.cfg
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)


# Add the Files for actionlib
#add_action_files(
#	DIRECTORY action
#	FILES RoomExploration.action
#)

# Generate messages in the 'msg' folder
#add_message_files(
#	FILES RoomSequence.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#	FILES
#)


# Generate added messages and services with any dependencies listed here
#generate_messages(
#	DEPENDENCIES
#		actionlib_msgs
#		geometry_msgs
#		move_base_msgs
#		sensor_msgs
#		std_msgs
#)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# important: common/include needs to be here if you have such a directory
catkin_package(
INCLUDE_DIRS
	common/include
	ros/include
LIBRARIES
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
	message_runtime
	actionlib_msgs
DEPENDS
	OpenCV
	Boost
)


###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# important: common/include also needs to be here if you have it
include_directories(
	common/include
	ros/include
	${include_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# turtlebot mover --> simulation, not relevant
#add_executable(turtlebot_mover ros/src/turtlebot_mover.cpp) 
#target_link_libraries(turtlebot_mover
#	${catkin_LIBRARIES} 
#	${OpenCV_LIBRARIES}
#)
#add_dependencies(turtlebot_mover ${catkin_EXPORTED_TARGETS} ${ipa_room_exploration_EXPORTED_TARGETS} ${Boost_LIBRARIES})

# room exploration action server
add_executable(room_exploration_server ros/src/room_exploration_action_server.cpp
					common/src/grid_point_explorator.cpp
					common/src/A_star_pathplanner.cpp common/src/node.cpp
					common/src/concorde_TSP.cpp)
add_dependencies(
	room_exploration_server ${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg)
target_link_libraries(room_exploration_server
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen_LIBRARIES}
)

# room exploration client
add_executable(room_exploration_client ros/src/room_exploration_action_client.cpp)
target_link_libraries(room_exploration_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(room_exploration_client ${catkin_EXPORTED_TARGETS})

 
#############
## Install ##
#############
# Mark executables and/or libraries for installation
install(TARGETS room_exploration_server room_exploration_client
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#uncomment this if you have a common-directory with header-files
install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

#uncomment this if you have header-files in your project
install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	PATTERN ".svn" EXCLUDE
#)

# uncomment this if you have launch files
#install(DIRECTORY ros/launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

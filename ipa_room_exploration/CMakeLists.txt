cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_exploration)

set(catkin_RUN_PACKAGES
	cob_map_accessibility_analysis
	cv_bridge
	geometry_msgs
	ipa_building_msgs
	ipa_building_navigation
	libconcorde_tsp_solver
	libqsopt
	move_base_msgs
	nav_msgs
	roscpp
	roslib
	sensor_msgs
	std_msgs
	tf
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
find_package(libqsopt REQUIRED)

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
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

# room exploration action server, note: qsopt is a static library --> need an absolut path to it
add_executable(room_exploration_server ros/src/room_exploration_action_server.cpp
					common/src/grid_point_explorator.cpp
					common/src/boustrophedon_explorator.cpp
					common/src/neural_network_explorator.cpp
					common/src/convex_sensor_placement_explorator.cpp
					common/src/meanshift2d.cpp
					common/src/fow_to_robot_mapper.cpp
					common/src/A_star_pathplanner.cpp common/src/node.cpp
					common/src/concorde_TSP.cpp)
add_dependencies(room_exploration_server
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg)
target_link_libraries(room_exploration_server
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	${Eigen_LIBRARIES}
	${libqsopt_INCLUDE_DIRS}/libqsopt/qsopt.a
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

cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_exploration)

set(catkin_RUN_PACKAGES
	cob_map_accessibility_analysis
	cv_bridge
	geometry_msgs
	ipa_building_msgs
	ipa_building_navigation
	libconcorde_tsp_solver
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

# Coin-Or and Cbc, included as ubuntu system package, without Cmake-module so include pkg-config file
#	!!! Important note: the order of the libraries when linking your executable is important, if it is wrong you get undefined references !!!
find_package(PkgConfig REQUIRED)
pkg_check_modules(CoinUtils REQUIRED coinutils)# coinutils osi-clp clp osi cbc cgl
pkg_check_modules(OsiClp REQUIRED osi-clp)
pkg_check_modules(Clp REQUIRED clp)
pkg_check_modules(Osi REQUIRED osi)
pkg_check_modules(Cbc-lib REQUIRED cbc)
pkg_check_modules(Cgl REQUIRED cgl)
Message(STATUS "coinutils include dirs: " ${CoinUtils_INCLUDE_DIRS})
Message(STATUS "osi-clp include dirs: " ${OsiClp_INCLUDE_DIRS})
Message(STATUS "clp include dirs: " ${Clp_INCLUDE_DIRS})
Message(STATUS "osi include dirs: " ${Osi_INCLUDE_DIRS})
Message(STATUS "cbc include dirs: " ${Cbc-lib_INCLUDE_DIRS})
Message(STATUS "cgl include dirs: " ${Cgl_INCLUDE_DIRS})

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
	CoinUtils
	OsiClp
	Clp
	Osi
	Cgl
	Cbc-lib
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
	${CoinUtils_INCLUDE_DIRS}
	${OsiClp_INCLUDE_DIRS}
	${Clp_INCLUDE_DIRS}
	${Osi_INCLUDE_DIRS}
	${Cgl_INCLUDE_DIRS}
	${Cbc-lib_INCLUDE_DIRS}
)

# room exploration action server, note: order of linking the Coin-Or packages important
add_executable(room_exploration_server ros/src/room_exploration_action_server.cpp
					common/src/grid_point_explorator.cpp
					common/src/boustrophedon_explorator.cpp
					common/src/neural_network_explorator.cpp
					common/src/convex_sensor_placement_explorator.cpp
					common/src/flow_network_explorator.cpp
					common/src/energy_functional_explorator.cpp
					common/src/meanshift2d.cpp
					ros/src/fow_to_robot_mapper.cpp
					common/src/A_star_pathplanner.cpp common/src/node.cpp
					common/src/concorde_TSP.cpp
					common/src/nearest_neighbor_TSP.cpp)
add_dependencies(room_exploration_server
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_gencfg)
target_link_libraries(room_exploration_server
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${CoinUtils_LIBRARIES}
	${OsiClp_LIBRARIES}
	${Clp_LIBRARIES}
	${Osi_LIBRARIES}
	${Cgl_LIBRARIES}
	${Cbc-lib_LIBRARIES}
)

# server for coverage checking
add_executable(coverage_check_server ros/src/coverage_check_server.cpp)
target_link_libraries(coverage_check_server
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)
add_dependencies(coverage_check_server ${catkin_EXPORTED_TARGETS})

# room exploration client
add_executable(room_exploration_client ros/src/room_exploration_action_client.cpp)
target_link_libraries(room_exploration_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)
add_dependencies(room_exploration_client ${catkin_EXPORTED_TARGETS})

# evaluation of room exploration algorithms
add_executable(room_exploration_evaluation ros/src/room_exploration_evaluation.cpp
						common/src/A_star_pathplanner.cpp common/src/node.cpp)
target_link_libraries(room_exploration_evaluation
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
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
install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

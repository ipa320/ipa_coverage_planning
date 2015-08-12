cmake_minimum_required(VERSION 2.8.3)
project(ipa_building_navigation)

set(catkin_RUN_PACKAGES
	roscpp
	rospy
	roslib
	actionlib
	actionlib_msgs
	std_msgs
	sensor_msgs
	cv_bridge
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
	genmsg
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## Add the Files for actionlib
#add_action_files(
#	DIRECTORY action
#	FILES MapSegmentation.action
#)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
#add_service_files(
#	FILES
#)


## Generate added messages and services with any dependencies listed here
#generate_messages(
#	DEPENDENCIES
#		std_msgs
#		actionlib_msgs
#		sensor_msgs
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
	#ros/include
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
	#ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

#tester for different functions
add_executable(a_star_tester ros/src/tester.cpp common/src/A_star_pathplanner.cpp common/src/node.cpp common/src/nearest_neighbor_TSP.cpp common/src/genetic_TSP.cpp common/src/concorde_TSP.cpp common/src/maximal_clique_finder.cpp common/src/contains.cpp common/src/set_cover_solver.cpp common/src/trolley_position_finder.cpp)
target_link_libraries(a_star_tester ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(a_star_tester ${catkin_EXPORTED_TARGETS})

add_executable(tester ros/src/boosttest.cpp common/src/A_star_pathplanner.cpp common/src/node.cpp)
target_link_libraries(tester ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(tester ${catkin_EXPORTED_TARGETS})

 
#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS a_star_tester tester
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##uncomment this if you have a common-directory with header-files
install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

##uncomment this if you have header-files in your project
#install(DIRECTORY ros/include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	FILES_MATCHING PATTERN "*.h"
#	PATTERN ".svn" EXCLUDE
#)

install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY ros/launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

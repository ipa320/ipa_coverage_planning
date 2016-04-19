cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_segmentation)

set(catkin_RUN_PACKAGES
	roscpp
	roslib
	actionlib
	actionlib_msgs
	std_msgs
	sensor_msgs
	geometry_msgs
	cv_bridge
	dlib_lib
	opengm_lib
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
	actionlib_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## Add the Files for actionlib
add_action_files(
	DIRECTORY action
	FILES MapSegmentation.action
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES RoomInformation.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#	FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		actionlib_msgs
		std_msgs
		sensor_msgs
		geometry_msgs
)


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
)

### segmentation action server: see room_segmentation_action_server_params.yaml to change the used method
add_executable(room_segmentation_server
	ros/src/room_segmentation_server.cpp
	common/src/distance_segmentation.cpp
	common/src/morphological_segmentation.cpp
	common/src/voronoi_segmentation.cpp
	common/src/adaboost_classifier.cpp
	common/src/wavefront_region_growing.cpp
	common/src/contains.cpp common/src/features.cpp
	common/src/raycasting.cpp
	common/src/meanshift2d.cpp
	common/src/room_class.cpp
	common/src/voronoi_random_field_segmentation.cpp
	common/src/clique_class.cpp
	common/src/voronoi_random_field_features.cpp)
target_link_libraries(room_segmentation_server 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})
add_dependencies(room_segmentation_server
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})

### client for testing purpose
add_executable(room_segmentation_client ros/src/room_segmentation_client.cpp)
target_link_libraries(room_segmentation_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})
add_dependencies(room_segmentation_client
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})

### evaluation function for numeric properties and comparison to a given ground truth segmentation
add_executable(evaluation
	ros/src/evaluation_numeric_properties.cpp
	common/src/evaluation_segmentation.cpp)
target_link_libraries(evaluation
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_dependencies(evaluation
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})

### test algorithm for implementing new segmentation algorithms
add_executable(segmentation_tester common/src/segmentation_tester.cpp common/src/voronoi_random_field_segmentation.cpp common/src/contains.cpp common/src/voronoi_random_field_features.cpp common/src/raycasting.cpp common/src/wavefront_region_growing.cpp common/src/clique_class.cpp common/src/room_class.cpp)
target_link_libraries(segmentation_tester ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(segmentation_tester ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

 
#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS   room_segmentation_server   room_segmentation_client   evaluation
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
install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY ros/launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_segmentation)

# build with c++11
add_compile_options(-std=c++11)

set(catkin_RUN_PACKAGES
	actionlib
	cv_bridge
	ipa_building_msgs
	libdlib
	nav_msgs
	opengm
	roscpp
	roslib
	sensor_msgs
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	dynamic_reconfigure
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

generate_dynamic_reconfigure_options(
	cfg/RoomSegmentation.cfg
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread filesystem)

find_package(OpenMP)
if(OPENMP_FOUND)
	message(STATUS "OPENMP FOUND")
	set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  #${OpenMP_C_FLAGS}
	set(OpenMP_LIBS gomp)
endif()

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
	${PROJECT_NAME}
CATKIN_DEPENDS
	${catkin_RUN_PACKAGES}
DEPENDS
	OpenCV
	Boost
	OpenMP
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

# library of the created algorithms
add_library(${PROJECT_NAME} STATIC
	common/src/distance_segmentation.cpp
	common/src/morphological_segmentation.cpp
	common/src/abstract_voronoi_segmentation.cpp
	common/src/voronoi_segmentation.cpp
	common/src/adaboost_classifier.cpp
	common/src/wavefront_region_growing.cpp
	common/src/contains.cpp common/src/features.cpp
	common/src/raycasting.cpp
	common/src/meanshift2d.cpp
	common/src/room_class.cpp
	common/src/voronoi_random_field_segmentation.cpp
	common/src/clique_class.cpp
	common/src/cv_boost_loader.cpp
	common/src/voronoi_random_field_features.cpp
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

### segmentation action server: see room_segmentation_action_server_params.yaml to change the used method
add_executable(room_segmentation_server
	ros/src/room_segmentation_server.cpp
	)
target_compile_options(room_segmentation_server PRIVATE ${OpenMP_FLAGS})
add_dependencies(room_segmentation_server
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(room_segmentation_server
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenMP_LIBS}
	${OpenCV_LIBRARIES}
	${PROJECT_NAME})

### client for testing purpose
add_executable(room_segmentation_client ros/src/room_segmentation_client.cpp)
target_link_libraries(room_segmentation_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})
add_dependencies(room_segmentation_client
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)

### evaluation function for numeric properties and comparison to a given ground truth segmentation
add_executable(evaluation
	ros/src/evaluation_numeric_properties.cpp
	common/src/evaluation_segmentation.cpp)
target_link_libraries(evaluation
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_dependencies(evaluation
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}  room_segmentation_server   room_segmentation_client   evaluation
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

##uncomment this if you use launch and yaml files
install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

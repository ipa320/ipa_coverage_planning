cmake_minimum_required(VERSION 2.8.3)
project(ipa_room_segmentation)

set(catkin_RUN_PACKAGES
	roscpp
	rospy
	std_msgs
)
set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
#add_service_files(
#	FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		std_msgs
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
)


add_executable(morpholigical_segmentation_server ros/src/morphological_segmentation_server.cpp common/src/morphological_segmentation.cpp common/src/watershed_region_spreading.cpp common/src/contains.cpp)
target_link_libraries(morpholigical_segmentation_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(morpholigical_segmentation_server ${catkin_EXPORTED_TARGETS})

add_executable(distance_segmentation_server ros/src/distance_segmentation_server.cpp common/src/distance_segmentation.cpp common/src/watershed_region_spreading.cpp common/src/contains.cpp)
target_link_libraries(distance_segmentation_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(distance_segmentation_server ${catkin_EXPORTED_TARGETS})

add_executable(voronoi_segmentation_server ros/src/voronoi_segmentation_server.cpp common/src/voronoi_segmentation.cpp common/src/watershed_region_spreading.cpp common/src/contains.cpp)
target_link_libraries(voronoi_segmentation_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(voronoi_segmentation_server ${catkin_EXPORTED_TARGETS})

add_executable(semantic ros/src/semantic_labeling.cpp ros/src/weak_classifier.cpp ros/src/features.cpp)
target_link_libraries(semantic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(semantic ${catkin_EXPORTED_TARGETS})

add_executable(opencv_semantic ros/src/opencvboost.cpp ros/src/features.cpp)
target_link_libraries(opencv_semantic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(opencv_semantic ${catkin_EXPORTED_TARGETS})

add_executable(evaluation ros/src/evaluation.cpp)
target_link_libraries(evaluation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(evaluation ${catkin_EXPORTED_TARGETS})

 
#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS morpholigical_segmentation_server distance_segmentation_server voronoi_segmentation_server semantic opencv_semantic evaluation
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

#install(DIRECTORY ros/launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

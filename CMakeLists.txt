cmake_minimum_required(VERSION 2.8.3)
project(free_space_detection)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
        rospy
        std_msgs
	lidar_segmentation
	colormap
        pcl_ros
        velodyne_pointcloud
        tf_conversions)
find_package(cmake_modules)
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES free_space_detection
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime colormap pcl_ros velodyne_pointcloud
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/free_space_detection.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(device_frame_publisher_node src/device_frame_publisher.cpp)
target_link_libraries(device_frame_publisher_node
  ${catkin_LIBRARIES}
)


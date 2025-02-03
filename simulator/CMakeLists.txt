cmake_minimum_required(VERSION 3.0.2)
project(simulator)
 
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
 
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  std_msgs
  gazebo_ros
)
 
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
 
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
# For tc_simple_world_plugin plugin
add_library(buoyancy_force src/buoyancy_force.cpp)
target_link_libraries(buoyancy_force ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
 
catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

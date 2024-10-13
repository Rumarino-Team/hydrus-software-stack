cmake_minimum_required(VERSION 3.0.2)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  NavigateToWaypoint.srv
  SetParameters.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs std_srvs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

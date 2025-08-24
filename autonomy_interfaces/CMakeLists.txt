cmake_minimum_required(VERSION 3.0.2)
project(autonomy_interfaces)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Detection.msg
  Detections.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FireTorpedo.srv
  SetColorFilter.srv
  SetYoloModel.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  NavigateToWaypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime
)

cmake_minimum_required(VERSION 3.0.2)
project(computer_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  Detection.msg
  Detections.msg  # Add the new message file
)

add_service_files(
  FILES
  EnableDetector.srv
  SetCustomClasses.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

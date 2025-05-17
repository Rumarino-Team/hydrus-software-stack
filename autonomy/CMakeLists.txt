cmake_minimum_required(VERSION 3.0.2)
project(autonomy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  stereo_msgs
  geometry_msgs
  visualization_msgs
  message_runtime
  rospy
  actionlib
  actionlib_msgs
  message_generation
  rosbag
  std_msgs
  topic_tools
)

# Add message files
add_message_files(
  FILES
  Detection.msg
  Detections.msg
)


add_action_files(
  FILES
  NavigateToWaypoint.action
)

# Add service files
add_service_files(
  FILES
  SetColorFilter.srv
  SetYoloModel.srv
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
    sensor_msgs
    stereo_msgs
    dynamic_reconfigure
    diagnostic_updater
    geometry_msgs
    visualization_msgs
    message_runtime
    rospy
    actionlib
    actionlib_msgs
    message_generation
    rosbag
    std_msgs
    topic_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

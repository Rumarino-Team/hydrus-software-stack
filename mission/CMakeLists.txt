cmake_minimum_required(VERSION 3.0.2)
project(mission)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  stereo_msgs
  geometry_msgs
  visualization_msgs
  message_runtime
  rospy
  smach
  smach_ros
  actionlib
  actionlib_msgs
  message_generation
  rosbag
  std_msgs
  topic_tools
  detector_node
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  CATKIN_DEPENDS
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    sensor_msgs
    stereo_msgs
    dynamic_reconfigure
    diagnostic_updater
    geometry_msgs
    visualization_msgs
    message_runtime
    rospy
    smach
    smach_ros
    actionlib
    actionlib_msgs
    message_generation
    rosbag
    std_msgs
    topic_tools
    detector_node
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

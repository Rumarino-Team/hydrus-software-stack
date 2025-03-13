cmake_minimum_required(VERSION 3.0.2)
project(hydrus_simulator)
 
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
 
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  std_msgs
  ros_ign_gazebo
)
# Depend on system install of Gazebo
find_package(ros_ign_gazebo REQUIRED)
 
link_directories(${ROS_IGN_GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${ROS_IGN_GAZEBO_INCLUDE_DIRS})
 
catkin_package(
  DEPENDS
    roscpp
    ros_ign_gazebo
)

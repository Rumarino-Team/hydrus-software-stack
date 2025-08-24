cmake_minimum_required(VERSION 3.0.2)
project(autonomy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Add Python setup to handle the mission_planner module properly
catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  autonomy_interfaces
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
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()

# --- Begin: Copy shared interface definitions into this package ---
# Location of shared interface source files (relative to this package)
set(SHARED_INTERFACES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../shared_interfaces")

# Ensure local interface directories exist
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/action")

# Gather shared files
file(GLOB SHARED_MSGS "${SHARED_INTERFACES_DIR}/msg/*.msg")
file(GLOB SHARED_SRVS "${SHARED_INTERFACES_DIR}/srv/*.srv")
file(GLOB SHARED_ACTIONS "${SHARED_INTERFACES_DIR}/action/*.action")

# Copy shared files into this package so catkin can find them
foreach(f ${SHARED_MSGS})
  get_filename_component(_name "${f}" NAME)
  configure_file("${f}" "${CMAKE_CURRENT_SOURCE_DIR}/msg/${_name}" COPYONLY)
endforeach()
foreach(f ${SHARED_SRVS})
  get_filename_component(_name "${f}" NAME)
  configure_file("${f}" "${CMAKE_CURRENT_SOURCE_DIR}/srv/${_name}" COPYONLY)
endforeach()
foreach(f ${SHARED_ACTIONS})
  get_filename_component(_name "${f}" NAME)
  configure_file("${f}" "${CMAKE_CURRENT_SOURCE_DIR}/action/${_name}" COPYONLY)
endforeach()

# Build lists of filenames relative to package dirs
set(MSG_FILES)
foreach(f ${SHARED_MSGS})
  get_filename_component(_name "${f}" NAME)
  list(APPEND MSG_FILES ${_name})
endforeach()

set(SRV_FILES)
foreach(f ${SHARED_SRVS})
  get_filename_component(_name "${f}" NAME)
  list(APPEND SRV_FILES ${_name})
endforeach()

set(ACTION_FILES)
foreach(f ${SHARED_ACTIONS})
  get_filename_component(_name "${f}" NAME)
  list(APPEND ACTION_FILES ${_name})
endforeach()
# --- End: Copy shared interface definitions ---

# Add message files
add_message_files(
  FILES
  ${MSG_FILES}
)


add_action_files(
  FILES
  ${ACTION_FILES}
)

# Add service files
add_service_files(
  FILES
  ${SRV_FILES}
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
    autonomy_interfaces
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

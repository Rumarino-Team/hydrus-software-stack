cmake_minimum_required(VERSION 3.8)
project(autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# Copy shared files into this package so rosidl can find them
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
  list(APPEND MSG_FILES "msg/${_name}")
endforeach()

set(SRV_FILES)
foreach(f ${SHARED_SRVS})
  get_filename_component(_name "${f}" NAME)
  list(APPEND SRV_FILES "srv/${_name}")
endforeach()

set(ACTION_FILES)
foreach(f ${SHARED_ACTIONS})
  get_filename_component(_name "${f}" NAME)
  list(APPEND ACTION_FILES "action/${_name}")
endforeach()
# --- End: Copy shared interface definitions ---

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

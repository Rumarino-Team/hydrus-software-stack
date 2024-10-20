#!/bin/bash
set -e

# Detect ROS distribution (Noetic or Melodic)
if [ -d "/opt/ros/noetic" ]; then
    ROS_DISTRO="noetic"
    echo "Detected ROS Noetic"
elif [ -d "/opt/ros/melodic" ]; then
    ROS_DISTRO="melodic"
    echo "Detected ROS Melodic"
else
    echo "No compatible ROS distribution found!"
    exit 1
fi

# Source the corresponding ROS setup.bash
source /opt/ros/$ROS_DISTRO/setup.bash


if [ "$VOLUME" == "true" ]; then
    echo "Use the Volume directory for building the packages."
    ROS_DIR='/home/catkin_ws'    
else
    echo "Use the Copied Packages from Docker."
    ROS_DIR='/catkin_ws'   
fi


# Start roscore in the background
cd "$ROS_DIR"
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 \
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so
source devel/setup.bash
roscore &
sleep 2

# Run the rosserial node
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &

# Conditionally run the ROS launch file based on the DEPLOY environment variable
if [ "$DEPLOY" == "true" ]; then
    echo "Deploy is set to true. Launching hydrus_start.launch..."
    # Compile the Arduino project
    cd /root/Arduino/libraries/sensor_actuator_pkg/examples/Hydrus
    arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
    arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino
    cd /catkin_ws

    roslaunch hydrus hydrus_start.launch _bags:=false &
else
    echo "Deploy is not set or is set to false. Skipping roslaunch."
fi

# Keep the container running by tailing the log
tail -f /dev/null

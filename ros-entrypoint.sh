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

# Detect system architecture
ARCH=$(uname -m)

# If running on ARM (Jetson), delete the simulator folder
if [[ "$ARCH" == "aarch64" ]]; then
    echo "Detected ARM architecture (Jetson or similar), deleting simulator folder..."
    rm -rf /catkin_ws/src/hydrus-software-stack/simulator
else
    echo "Non-ARM architecture detected, keeping simulator folder."
fi

# Determine ROS directory based on volume usage
if [ "$VOLUME" == "true" ]; then
    echo "Using the Volume directory for building the packages."
    ROS_DIR='/home/catkin_ws'    
else
    echo "Using the Copied Packages from Docker."
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
# roscore &
sleep 2

# Run the rosserial node

# Conditionally run the ROS launch file based on the DEPLOY environment variable
if [ "$DEPLOY" == "true" ]; then
    echo "Deploy is set to true. Launching hydrus_start.launch..."
    
    # Compile the Arduino project
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &

    cd /root/Arduino/libraries/embedded_arduino/Hydrus
    arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
    arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino
    
    cd /catkin_ws
    roslaunch autonomy autonomy.launch
else
    echo "Deploy is not set or is set to false. Skipping roslaunch."
fi

# Keep the container running by tailing the log
tail -f /dev/null

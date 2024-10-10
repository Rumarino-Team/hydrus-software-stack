#!/bin/bash
set -e

# Start roscore in the background
source /opt/ros/melodic/setup.bash
./src/catkin.sh
source devel/setup.bash
roscore &
sleep 2


# Compile the Arduino project
cd /root/Arduino/libraries/sensor_actuator_pkg/examples/Hydrus
arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino

# Run the rosserial node
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &

# Add another ROS node
roslaunch hydrus hydrus_start.launch _bags:=false  &


# Keep the container running
exec "$@"

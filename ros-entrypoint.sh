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

# Conditionally run the ROS launch file based on the DEPLOY environment variable
if [ "$DEPLOY" == "true" ]; then
    echo "Starting rosserial_python node..."
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &
    sleep 3  # Give rosserial some time to initialize
    
    # Compile the Arduino project
    cd /root/Arduino/libraries/embedded_arduino/Hydrus
    echo "Compiling Arduino sketch..."
    arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
    echo "Uploading Arduino sketch..."
    arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino
    
    # Make serial port accessible for both rosserial and our monitoring script
    chmod +x /catkin_ws/src/hydrus-software-stack/setup_serial_monitor.sh
    chmod +x /catkin_ws/src/hydrus-software-stack/monitor_arduino_logs.sh
    
    # Start rosserial node
    
    cd /catkin_ws
    
    # Check if tmux is installed and run our custom tmux session script
    echo "Setting up tmux sessions with Arduino monitoring..."
    chmod +x /catkin_ws/src/hydrus-software-stack/start_tmux_sessions.sh
    /catkin_ws/src/hydrus-software-stack/start_tmux_sessions.sh
else
    echo "Deploy is not set or is set to false. Skipping roslaunch."
fi




# Check if ROSBAG_PLAYBACK is enabled
if [ "$ROSBAG_PLAYBACK" == "true" ]; then
    echo "ROSBAG playback is enabled. Looking for rosbag files in /rosbags directory..."
    
    # Check if the rosbags directory exists and has .bag files
    if [ -d "/rosbags" ] && [ "$(find /rosbags -name '*.bag' | wc -l)" -gt 0 ]; then
        echo "Found the following rosbag files:"
        find /rosbags -name "*.bag" -exec echo "  - {}" \;
        
        # Get the most recent rosbag file (sorted by modification time)
        BAGFILE=$(find /rosbags -name "*.bag" | sort -r | head -1)
        echo "Automatically selecting most recent rosbag: $BAGFILE"
                
        # Start rosbag play in the background with loop mode
        echo "Playing rosbag in loop mode..."
        rosbag play $BAGFILE --loop &
        
        echo "Rosbag playback started in background."
    else
        echo "No rosbag files found in /rosbags directory. Skipping rosbag playback."
    fi
fi

# Keep the container running by tailing the log
tail -f /dev/null

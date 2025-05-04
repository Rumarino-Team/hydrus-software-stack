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

# Check if ROSBAG_PLAYBACK is enabled
if [ "$ROSBAG_PLAYBACK" == "true" ]; then
    echo "ROSBAG playback is enabled. Looking for rosbag files in /rosbags directory..."
    
    # Check if the rosbags directory exists and has .bag files
    if [ -d "/rosbags" ] && [ "$(find /rosbags -name '*.bag' | wc -l)" -gt 0 ]; then
        echo "Found the following rosbag files:"
        find /rosbags -name "*.bag" -exec echo "  - {}" \;
        
        # Display menu to select a rosbag file
        echo "Select a rosbag file to play:"
        select BAGFILE in $(find /rosbags -name "*.bag"); do
            if [ -n "$BAGFILE" ]; then
                echo "Selected rosbag: $BAGFILE"
                
                # Ask if the user wants to loop the rosbag
                echo "Do you want to loop the rosbag playback? (y/n)"
                read -p "> " LOOP_CHOICE
                
                # Create a tmux session for rosbag playback
                tmux new-session -d -s rosbag
                
                if [[ "$LOOP_CHOICE" =~ ^[Yy] ]]; then
                    echo "Playing rosbag with loop option..."
                    tmux send-keys -t rosbag "rosbag play $BAGFILE --loop" C-m
                else
                    echo "Playing rosbag once..."
                    tmux send-keys -t rosbag "rosbag play $BAGFILE" C-m
                fi
                
                echo "Rosbag playback started in background. Use 'tmux attach -t rosbag' to view playback status."
                break
            else
                echo "Invalid selection. Please try again."
            fi
        done
    else
        echo "No rosbag files found in /rosbags directory. Skipping rosbag playback."
    fi
fi

# Conditionally run the ROS launch file based on the DEPLOY environment variable
if [ "$DEPLOY" == "true" ]; then
    echo "Deploy is set to true. Launching hydrus_start.launch..."
    
    # Compile the Arduino project
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600 &

    cd /root/Arduino/libraries/embedded_arduino/Hydrus
    arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino
    arduino-cli upload -p /dev/ttyACM0 --fqbn $ARDUINO_BOARD Hydrus.ino
    
    cd /catkin_ws
    
    # Check if tmux is installed
    if ! command -v tmux &> /dev/null; then
        echo "tmux could not be found. Installing tmux..."
        apt-get update && apt-get install -y tmux
    fi
    
    # Create a new tmux session named 'hydrus'
    tmux new-session -d -s hydrus
    
    # Split the window horizontally for controllers.py
    tmux send-keys -t hydrus "python3 /catkin_ws/src/hydrus-software-stack/src/controllers.py" C-m
    # Split horizontally again for cv_publisher
    tmux split-window -v -t hydrus
    tmux send-keys -t hydrus:0.2 "python3 /catkin_ws/src/hydrus-software-stack/src/cv_publisher.py" C-m
    # Attach to the tmux session
    tmux attach-session -t hydrus
else
    echo "Deploy is not set or is set to false. Skipping roslaunch."
fi

# Keep the container running by tailing the log
tail -f /dev/null

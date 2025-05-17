#!/bin/bash
# Removed "set -e" to prevent the script from exiting on errors

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
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so || true
source devel/setup.bash
# roscore &
sleep 2

# Conditionally run the ROS launch file based on the DEPLOY environment variable
if [ "$DEPLOY" == "true" ]; then
    echo "Starting rosserial_python node..."
    sleep 1  # Give rosserial some time to initialize
    
    # Compile the Arduino project
    cd /root/Arduino/libraries/embedded_arduino/Hydrus
    echo "Compiling Arduino sketch..."
    arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino || true
    
    # Improved Arduino upload process with better error handling
    echo "Uploading Arduino sketch..."
    
    # Color definitions for better visibility
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    NC='\033[0m' # No Color
    
    # Simple function to reset Arduino by toggling DTR
    reset_arduino() {
        local port=$1
        echo "Resetting Arduino on $port..."
        stty -F $port hupcl 2>/dev/null || true
        sleep 0.1
        stty -F $port -hupcl 2>/dev/null || true
        sleep 0.5
    }
    
    # Try to upload with a proper retry mechanism
    MAX_ATTEMPTS=3
    upload_success=false
    
    # Define ports to try in order
    PORTS_TO_TRY=("/dev/ttyACM0" "/dev/ttyACM1" "/dev/ttyUSB0")
    
    # Try each port in sequence
    for ((attempt=1; attempt<=MAX_ATTEMPTS; attempt++)); do
        # Get port for this attempt
        ARDUINO_PORT="${PORTS_TO_TRY[$((attempt-1))]}"
        
        echo -e "${YELLOW}Upload attempt $attempt of $MAX_ATTEMPTS using $ARDUINO_PORT${NC}"
        
        # Check if port exists
        if [ ! -e "$ARDUINO_PORT" ]; then
            echo -e "${YELLOW}Port $ARDUINO_PORT does not exist, skipping this attempt${NC}"
            continue
        fi
        
        # Reset Arduino before upload
        reset_arduino "$ARDUINO_PORT" || true
        
        # Try to upload - IMPORTANT: capture output to prevent early termination
        echo -e "${YELLOW}Uploading to $ARDUINO_PORT...${NC}"
        upload_output=$(arduino-cli upload -p "$ARDUINO_PORT" --fqbn $ARDUINO_BOARD Hydrus.ino 2>&1) || true
        upload_status=$?
        
        # Print the output
        echo "$upload_output"
        
        if [ $upload_status -eq 0 ]; then
            echo -e "${GREEN}Upload successful to $ARDUINO_PORT!${NC}"
            upload_success=true
            # Save the successful port for future reference
            echo "Used port: $ARDUINO_PORT" > /tmp/arduino_port.txt
            break
        else
            echo -e "${RED}Upload to $ARDUINO_PORT failed (exit code: $upload_status)${NC}"
            
            # Check if we have more ports to try
            if [ $attempt -lt $MAX_ATTEMPTS ]; then
                echo -e "${YELLOW}Trying next port in 2 seconds...${NC}"
                sleep 2
            else
                echo -e "${RED}All upload attempts failed.${NC}"
            fi
        fi
    done
    
    # Continue with the rest of the setup process regardless of upload success
    echo -e "${YELLOW}Continuing with setup regardless of upload status...${NC}"
    
    # Make serial port accessible for both rosserial and our monitoring script
    chmod +x /catkin_ws/src/hydrus-software-stack/setup_serial_monitor.sh || true
    chmod +x /catkin_ws/src/hydrus-software-stack/monitor_arduino_logs.sh || true
    
    # Start rosserial node
    cd /catkin_ws
    
    # Check if tmux is installed and run our custom tmux session script
    echo "Setting up tmux sessions with Arduino monitoring..."
    chmod +x /catkin_ws/src/hydrus-software-stack/start_tmux_sessions.sh || true
    /catkin_ws/src/hydrus-software-stack/start_tmux_sessions.sh || true
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

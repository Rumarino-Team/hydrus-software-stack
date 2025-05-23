#!/bin/bash
# File to handle tmux session creation and management for Hydrus
set -e

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Installing tmux..."
    apt-get update && apt-get install -y tmux
fi

# Determine ROS directory based on volume usage
if [ "$VOLUME" == "true" ]; then
    echo "Using Volume directory for tmux scripts: /home/catkin_ws"
    CATKIN_WS='/home/catkin_ws'
else
    echo "Using Docker container directory: /catkin_ws"
    CATKIN_WS='/catkin_ws'
fi

# Setup serial monitor for Arduino if it's not already running
set +e # Ignore errors for the next command, allows tmux to stil run even if the arduino is not connected
if [ ! -f "/tmp/hydrus_serial/catpid.txt" ]; then
    echo "Setting up Arduino serial monitoring..."
    bash ${CATKIN_WS}/src/hydrus-software-stack/setup_serial_monitor.sh /dev/ttyACM0
    sleep 2  # Give some time for setup
fi
set -e
# ----------------------------------------------------------------------
# Create a new tmux session named 'hydrus' for thrusters visualization
# ----------------------------------------------------------------------
tmux new-session -d -s hydrus -n "Controls"

# First pane: Run serial_ros_bridge.py
tmux send-keys -t hydrus "echo 'Starting Serial ROS Bridge'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/controller/serial_ros_bridge.py _port:=/dev/ttyACM0 _baud_rate:=115200" C-m

# Second pane: Run controllers.py
tmux split-window -v -t hydrus
tmux send-keys -t hydrus:0.1 "echo 'Starting Controller Node'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/src/controllers.py" C-m

# Third pane: Run thruster_visualizer.py
tmux split-window -h -t hydrus:0.1
tmux send-keys -t hydrus:0.2 "echo 'Starting Thruster Visualizer'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/controller/thruster_visualizer.py" C-m

# Create a new window for Arduino monitoring with two panes
tmux new-window -t hydrus:1 -n "Arduino"

# First pane in Arduino window: Setup serial monitor
tmux send-keys -t hydrus:1.0 "echo 'Setting up Arduino monitoring'; bash ${CATKIN_WS}/src/hydrus-software-stack/setup_serial_monitor.sh " C-m

# Second pane in Arduino window: Monitor Arduino logs
tmux split-window -h -t hydrus:1
tmux send-keys -t hydrus:1.1 "echo 'Starting Arduino log monitor'; sleep 2; bash ${CATKIN_WS}/src/hydrus-software-stack/monitor_arduino_logs.sh" C-m

# Adjust the pane layout for the Arduino window to make it more readable
# Set a tiled layout for the Arduino window
tmux select-layout -t hydrus:1 even-horizontal

# Create a new window for Computer Vision with four panes
tmux new-window -t hydrus:2 -n "Computer Vision"

# First pane in CV window: Run the color filter controller (taking half the screen)
tmux send-keys -t hydrus:2.0 "echo 'Starting Color Filter Controller'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/cv/color_filter_controller.py ui" C-m

# Second pane in CV window: Run cv_publishers.py (right side, top half)
tmux split-window -h -t hydrus:2.0
tmux send-keys -t hydrus:2.1 "echo 'Starting Computer Vision Publishers'; source ${CATKIN_WS}/devel/setup.bash && roslaunch autonomy cv_publishers.launch" C-m

# Third pane in CV window: Run the web detection viewer (right side, bottom left)
tmux split-window -v -t hydrus:2.1
tmux send-keys -t hydrus:2.2 "echo 'Starting Web Detection Viewer'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/web/detection_viewer.py" C-m

# Fourth pane in CV window: Run the API (right side, bottom right)
tmux split-window -h -t hydrus:2.2
tmux send-keys -t hydrus:2.3 "echo 'Starting API Server'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/src/API.py" C-m

# Resize panes to achieve the desired layout (controller takes 50% of width)
tmux resize-pane -t hydrus:2.0 -x "$(( $(tput cols) / 2 ))"

# Resize the bottom panes to be equal width
tmux select-pane -t hydrus:2.2
tmux resize-pane -Z  # unzoom if zoomed
tmux resize-pane -x "$(( $(tput cols) / 4 ))"


# Create a new window for Mission Planning
tmux new-window -t hydrus:3 -n "Mission Planner"
tmux send-keys -t hydrus:3.0 "echo 'Starting Mission Manager'; source ${CATKIN_WS}/devel/setup.bash && roslaunch autonomy mission_planner.launch" C-m

# Split window for mission visualization/status
tmux split-window -h -t hydrus:3.0
tmux send-keys -t hydrus:3.1 "echo 'Starting Mission Controller'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/mission/mission_controller.py" C-m

# Add a new pane for controller monitoring
tmux split-window -v -t hydrus:3.1
tmux send-keys -t hydrus:3.2 "echo 'Starting Controller Monitor'; source ${CATKIN_WS}/devel/setup.bash && python3 ${CATKIN_WS}/src/hydrus-software-stack/autonomy/scripts/controller/controller_monitor.py" C-m

# Set a tiled layout for the Mission Planning window
tmux select-layout -t hydrus:3 tiled

# Return to the main control window and select teleop pane for keyboard control
tmux select-window -t hydrus:0
# Set the layout to maintain the custom sizing
tmux select-pane -t hydrus:0.1

echo "Tmux session 'hydrus' created with catkin workspace at ${CATKIN_WS}. You can attach to it using 'tmux attach -t hydrus'."
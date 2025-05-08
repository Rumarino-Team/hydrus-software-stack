#!/bin/bash
# File to handle tmux session creation and management for Hydrus
set -e

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Installing tmux..."
    apt-get update && apt-get install -y tmux
fi

# Setup serial monitor for Arduino if it's not already running
if [ ! -f "/tmp/hydrus_serial/catpid.txt" ]; then
    echo "Setting up Arduino serial monitoring..."
    bash /catkin_ws/src/hydrus-software-stack/setup_serial_monitor.sh /dev/ttyACM0
    sleep 2  # Give some time for setup
fi

# # Create a new tmux session named 'hydrus'
# tmux new-session -d -s hydrus

# # Source the catkin workspace in the first pane and run controllers.py
# tmux send-keys -t hydrus "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/controllers.py" C-m

# # Split horizontally for cv_publisher
# tmux split-window -v -t hydrus
# # Source the catkin workspace in the second pane and run cv_publisher.py
# tmux send-keys -t hydrus:0.1 "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/cv_publisher.py" C-m

# # Split horizontally again for Arduino serial monitoring
# tmux split-window -v -t hydrus
# # Run the Arduino log monitor in the third pane
# tmux send-keys -t hydrus:0.2 "echo 'Arduino Serial Monitor'; bash /catkin_ws/src/hydrus-software-stack/monitor_arduino_logs.sh" C-m

# # Make the Arduino monitor pane smaller
# tmux resize-pane -t hydrus:0.2 -y 10

# ----------------------------------------------------------------------
# Create a new tmux session named 'serial_connection' for thrusters visualization
# ----------------------------------------------------------------------
tmux new-session -d -s serial_connection -n "Controls"

# First pane: Run serial_ros_bridge.py
tmux send-keys -t serial_connection "echo 'Starting Serial ROS Bridge'; source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/scripts/controller/serial_ros_bridge.py _port:=/dev/ttyACM0 _baud_rate:=115200" C-m

# Second pane: Run submarine_teleop.py
tmux split-window -v -t serial_connection
tmux send-keys -t serial_connection:0.1 "echo 'Starting Submarine Teleop'; source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/controllers.py" C-m

# Third pane: Run thruster_visualizer.py
tmux split-window -h -t serial_connection:0.1
tmux send-keys -t serial_connection:0.2 "echo 'Starting Thruster Visualizer'; source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/scripts/controller/thruster_visualizer.py" C-m

# Create a new window for Arduino monitoring with two panes
tmux new-window -t serial_connection:1 -n "Arduino"

# First pane in Arduino window: Setup serial monitor
tmux send-keys -t serial_connection:1.0 "echo 'Setting up Arduino monitoring'; bash /catkin_ws/src/hydrus-software-stack/setup_serial_monitor.sh " C-m

# Second pane in Arduino window: Monitor Arduino logs
tmux split-window -h -t serial_connection:1
tmux send-keys -t serial_connection:1.1 "echo 'Starting Arduino log monitor'; sleep 2; bash /catkin_ws/src/hydrus-software-stack/monitor_arduino_logs.sh" C-m

# Adjust the pane layout for the Arduino window to make it more readable
# Set a tiled layout for the Arduino window
tmux select-layout -t serial_connection:1 even-horizontal

# Return to the main control window and select teleop pane for keyboard control
tmux select-window -t serial_connection:0
tmux select-pane -t serial_connection:0.1

# Attach to the hydrus tmux session (we'll let the user switch to the serial_connection session manually)
# tmux attach-session -t hydrus
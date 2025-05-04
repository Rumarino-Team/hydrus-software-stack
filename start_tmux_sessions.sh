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

# Create a new tmux session named 'hydrus'
tmux new-session -d -s hydrus

# Source the catkin workspace in the first pane and run controllers.py
tmux send-keys -t hydrus "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/controllers.py" C-m

# Split horizontally for cv_publisher
tmux split-window -v -t hydrus
# Source the catkin workspace in the second pane and run cv_publisher.py
tmux send-keys -t hydrus:0.1 "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/cv_publisher.py" C-m

# Split horizontally again for Arduino serial monitoring
tmux split-window -v -t hydrus
# Run the Arduino log monitor in the third pane
tmux send-keys -t hydrus:0.2 "echo 'Arduino Serial Monitor'; bash /catkin_ws/src/hydrus-software-stack/monitor_arduino_logs.sh" C-m

# Make the Arduino monitor pane smaller
tmux resize-pane -t hydrus:0.2 -y 10

# Attach to the tmux session
tmux attach-session -t hydrus
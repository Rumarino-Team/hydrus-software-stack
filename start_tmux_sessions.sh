#!/bin/bash
# File to handle tmux session creation and management for Hydrus
set -e

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Installing tmux..."
    apt-get update && apt-get install -y tmux
fi

# Create a new tmux session named 'hydrus'
tmux new-session -d -s hydrus

# Source the catkin workspace in the first pane and run controllers.py
tmux send-keys -t hydrus "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/controllers.py" C-m

# Split horizontally again for cv_publisher
tmux split-window -v -t hydrus
# Source the catkin workspace in the second pane and run cv_publisher.py
tmux send-keys -t hydrus:0.2 "source /catkin_ws/devel/setup.bash && python3 /catkin_ws/src/hydrus-software-stack/autonomy/src/cv_publisher.py" C-m

# Attach to the tmux session
tmux attach-session -t hydrus
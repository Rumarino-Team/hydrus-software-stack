#!/usr/bin/env bash
colcon build --symlink-install
source install/setup.bash
ros2 launch hydrus_sim_ros2 hydrussim.launch.py
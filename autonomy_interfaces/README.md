# Shared Interfaces (ROS 1 and ROS 2)

This folder contains the single source of truth for Hydrus message, service, and action interface definitions.

Structure:
- `msg/` — `.msg` message files
- `srv/` — `.srv` service files
- `action/` — `.action` action files

Both ROS 1 (`autonomy_ros`) and ROS 2 (`autonomy_ros2`) packages copy these files at CMake configure-time into their local `msg/`, `srv/`, and `action/` folders and generate code from them. Edit files here only, then rebuild the ROS 1/ROS 2 workspaces.

To add a new interface:
1. Add the `.msg` / `.srv` / `.action` file in this folder hierarchy.
2. Rebuild your ROS 1 and/or ROS 2 workspaces as usual.

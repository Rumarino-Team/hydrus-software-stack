# cv_node_py

ROS 2 Python node wrapping existing `src/computer_vision/detection_core.py`.

## Build

```bash
# In workspace root (ros2_ws)
colcon build --symlink-install
. install/setup.bash
ros2 launch cv_node_py cv_node.launch.py
```

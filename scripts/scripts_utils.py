from pathlib import Path


def get_building_path(volume: bool = False) -> Path:
    """Get the path where packages are built. Inside the copy ROS workspace or the volume."""
    # Determine ROS directory based on volume usage
    if volume:
        print("Using Volume directory for tmux scripts: /home/catkin_ws")
        return Path("/home/catkin_ws")
    else:
        print("Using Docker container directory: /catkin_ws")
        return Path("/catkin_ws")

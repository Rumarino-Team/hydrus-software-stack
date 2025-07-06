#  The following  are the thing that this ros.py command should have available:

#  - build the catkin workspace whenever it is.
# - have the camera-entrypoint available. Run the camera.
#  - download ros bags.
#  - video to rosbags
import os
import subprocess
import sys
from pathlib import Path

import typer

ros_app = typer.Typer()


class HydrusRosManager:
    def __init__(self):
        pass

    def build_workspace(self):
        """Build the catkin workspace."""
        print("Building catkin workspace...")

    def run_camera_entrypoint(self):
        """Run the camera entrypoint."""
        print("Running camera entrypoint...")

    def download_ros_bags(self):
        """Download ROS bags."""
        print("Downloading ROS bags...")

    def video_to_ros_bag(self):
        """Convert video files to ROS bags."""
        print("Converting video to ROS bag...")

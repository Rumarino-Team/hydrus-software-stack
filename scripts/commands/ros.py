#  The following  are the thing that this ros.py command should have available:

#  - build the catkin workspace whenever it is.
# - have the camera-entrypoint available. Run the camera.
#  - download ros bags.
#  - video to rosbags
import os
import subprocess
import sys
from pathlib import Path


class Video2Rosbag:
    def __init__(self, video_path, rosbag_path):
        self.video_path = video_path
        self.rosbag_path = rosbag_path

    def convert(self):
        """Convert video to ROS bag format."""
        print(f"🎥 Converting video {self.video_path} to ROS bag {self.rosbag_path}...")
        # Conversion logic goes here
        return True


class RosBagDownloader:
    def __init__(self, download_url, save_path):
        self.download_url = download_url
        self.save_path = save_path

    def download(self):
        """Download ROS bag from a URL."""
        print(f"⬇️ Downloading ROS bag from {self.download_url} to {self.save_path}...")
        # Download logic goes here
        return True


class CameraEntrypoint:
    def __init__(self):
        pass


class CatkinWorkspaceBuilder:
    def __init__(self):
        pass

    def register_subcommand(self, subparsers):
        """Register the build subcommand with the main parser"""
        build_parser = subparsers.add_parser(
            "build",
            help="Build the ROS catkin workspace",
            description="Build the ROS catkin workspace with optional cleaning.",
        )

        build_parser.add_argument(
            "--clean", action="store_true", help="Clean workspace before building"
        )
        build_parser.add_argument(
            "--no-build", action="store_true", help="Skip the actual build step"
        )

        # Set the function to call
        build_parser.set_defaults(func=self.run_build)

    def run_build(self, args):
        """Build the catkin workspace"""
        print("🔨 Building catkin workspace..")

        # Find workspace
        script_dir = Path(__file__).parent.parent.absolute()
        hydrus_root = script_dir.parent

        # Determine workspace type
        volume_path = Path("/home/catkin_ws")
        container_path = Path("/catkin_ws")

        if volume_path.exists() and (volume_path / "src").exists():
            workspace_dir = volume_path
        elif container_path.exists() and (container_path / "src").exists():
            workspace_dir = container_path
        else:
            print("❌ Could not find catkin workspace")
            return False

        # Setup environment
        env = os.environ.copy()
        if (workspace_dir / "devel/setup.bash").exists():
            env[
                "ROS_PACKAGE_PATH"
            ] = f"{workspace_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"
            env[
                "CMAKE_PREFIX_PATH"
            ] = f"{workspace_dir}/devel:{env.get('CMAKE_PREFIX_PATH', '')}"
            env[
                "LD_LIBRARY_PATH"
            ] = f"{workspace_dir}/devel/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env[
                "PYTHONPATH"
            ] = f"{workspace_dir}/devel/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"

        # Clean if requested
        if args.clean:
            print("🧹 Cleaning workspace...")
            clean_cmd = f"cd {workspace_dir} && catkin_make clean"
            subprocess.run(["bash", "-c", clean_cmd], check=False)

        # Build
        if not args.no_build:
            source_cmd = f"source /opt/ros/{env.get('ROS_DISTRO', 'noetic')}/setup.bash"
            build_cmd = f"{source_cmd} && cd {workspace_dir} && catkin_make"

            result = subprocess.run(["bash", "-c", build_cmd], env=env, check=False)

            if result.returncode == 0:
                print("✅ Workspace build completed successfully")
                return True
            else:
                print("❌ Workspace build failed")
                return False

        return True


def register_subcommand(subparsers):
    """Register the build subcommand with the main parser"""
    build_parser = subparsers.add_parser(
        "build",
        help="Build the ROS catkin workspace",
        description="Build the ROS catkin workspace with optional cleaning.",
    )

    build_parser.add_argument(
        "--clean", action="store_true", help="Clean workspace before building"
    )
    build_parser.add_argument(
        "--no-build", action="store_true", help="Skip the actual build step"
    )

    # Set the function to call
    build_parser.set_defaults(func=run_build)


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

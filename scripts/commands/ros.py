"""
ROS command module for Hydrus CLI
Handles ROS workspace operations, camera management, and rosbag utilities
"""

import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional

import typer

from .utils import get_building_path

ros_app = typer.Typer()


class HydrusRosManager:
    def __init__(self, volume: bool = False):
        self.volume = volume
        self.workspace_dir = get_building_path(self.volume)
        self.rosbags_dir = self.workspace_dir / "src/hydrus-software-stack/rosbags"

        # Create rosbags directory if it doesn't exist
        self.rosbags_dir.mkdir(parents=True, exist_ok=True)

        # Constants for download
        self.DEFAULT_ROSBAG_URL = "https://drive.google.com/file/d/16Lr-CbW1rW6rKh8_mWClTQMIjm2u0y8X/view?usp=drive_link"

    def _run_command(
        self,
        cmd: List[str],
        check: bool = True,
        capture_output: bool = False,
        env: Optional[Dict] = None,
        cwd: Optional[Path] = None,
    ) -> subprocess.CompletedProcess:
        """Run a command with proper error handling"""
        if env is None:
            env = os.environ.copy()

        cmd_str = " ".join(cmd)
        print(f"🔧 Executing: {cmd_str}")

        try:
            return subprocess.run(
                cmd,
                check=check,
                capture_output=capture_output,
                env=env,
                cwd=cwd,
                text=True,
            )
        except subprocess.CalledProcessError as e:
            if check:
                print(f"❌ Command failed: {cmd_str}")
                print(f"   Exit code: {e.returncode}")
                if capture_output and e.stderr:
                    print(f"   Error: {e.stderr}")
            raise
        except FileNotFoundError:
            print(f"❌ Command not found: {cmd_str}")
            raise

    def _setup_environment(self) -> Dict[str, str]:
        """Setup ROS environment"""
        env = os.environ.copy()

        # Source devel/setup.bash equivalent
        setup_file = self.workspace_dir / "devel/setup.bash"
        if setup_file.exists():
            # Add workspace paths to environment
            env[
                "ROS_PACKAGE_PATH"
            ] = f"{self.workspace_dir}/src:{env.get('ROS_PACKAGE_PATH', '')}"
            env[
                "CMAKE_PREFIX_PATH"
            ] = f"{self.workspace_dir}/devel:{env.get('CMAKE_PREFIX_PATH', '')}"
            env[
                "LD_LIBRARY_PATH"
            ] = f"{self.workspace_dir}/devel/lib:{env.get('LD_LIBRARY_PATH', '')}"
            env[
                "PYTHONPATH"
            ] = f"{self.workspace_dir}/devel/lib/python3/dist-packages:{env.get('PYTHONPATH', '')}"

        return env

    def build_workspace(self, clean: bool = False):
        """Build the catkin workspace."""
        print("🔨 Building catkin workspace...")

        if not self.workspace_dir.exists():
            print(f"❌ Workspace directory not found: {self.workspace_dir}")
            return False

        env = self._setup_environment()

        # Clean if requested
        if clean:
            print("🧹 Cleaning workspace...")
            clean_cmd = ["catkin_make", "clean"]
            try:
                self._run_command(
                    clean_cmd, cwd=self.workspace_dir, env=env, check=False
                )
            except subprocess.CalledProcessError:
                print("Warning: Clean failed, but continuing...")

        # Build command that sources ROS environment first
        ros_distro = env.get("ROS_DISTRO", "noetic")
        cmake_command = f"source /opt/ros/{ros_distro}/setup.bash && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.9.so"

        cmake_args = ["bash", "-c", cmake_command]

        try:
            self._run_command(cmake_args, cwd=self.workspace_dir, env=env)
            print("✅ Workspace built successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Build failed: {e}")
            return False

    def run_camera_entrypoint(self, zed_option: bool = False):
        """Run the camera entrypoint."""
        print("📷 Starting ZED camera entrypoint...")
        print(f"ZED_OPTION: {zed_option}")

        env = self._setup_environment()
        env["ZED_OPTION"] = "true" if zed_option else "false"

        try:
            # Build workspace first
            if not self.build_workspace():
                print("❌ Failed to build workspace")
                return False

            # Launch ZED camera
            if zed_option:
                print("Launching ZED2i camera with RViz display...")
                launch_cmd = ["roslaunch", "zed_display_rviz", "display_zed2i.launch"]
            else:
                print("Launching ZED2i camera...")
                launch_cmd = ["roslaunch", "--wait", "zed_wrapper", "zed2i.launch"]

            self._run_command(launch_cmd, env=env, cwd=self.workspace_dir)
            return True

        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to launch ZED camera: {e}")
            return False
        except KeyboardInterrupt:
            print("\n🛑 Camera entrypoint stopped by user")
            return True
        except Exception as e:
            print(f"❌ Unexpected error: {e}")
            return False

    def _extract_file_id(self, url: str) -> Optional[str]:
        """Extract the file ID from a Google Drive URL"""
        pattern = r"drive\.google\.com/file/d/([a-zA-Z0-9_-]+)"
        match = re.search(pattern, url)
        if match:
            return match.group(1)
        return None

    def _install_download_dependencies(self) -> bool:
        """Install required dependencies for downloading"""
        try:
            print("Installing gdown and requests...")
            self._run_command(
                [sys.executable, "-m", "pip", "install", "gdown", "requests"]
            )
            print("✅ Dependencies installed successfully!")
            return True
        except subprocess.CalledProcessError:
            print("❌ Failed to install dependencies. Please install them manually:")
            print("pip install gdown requests")
            print("\nOr if you have permission issues, try:")
            print("pip install --user gdown requests")
            return False

    def _check_rosbags_exist(self) -> bool:
        """Check if any .bag files exist in the rosbags directory"""
        bag_files = list(self.rosbags_dir.glob("*.bag"))
        return len(bag_files) > 0

    def download_ros_bags(self, url: Optional[str] = None, force: bool = False):
        """Download ROS bags."""
        print("📥 Checking for ROS bag files...")

        if not force and self._check_rosbags_exist():
            print("✅ ROS bag files already exist in the rosbags directory.")
            existing_bags = list(self.rosbags_dir.glob("*.bag"))
            print("Existing bag files:")
            for bag in existing_bags:
                print(f"  • {bag.name}")
            return True

        if not force:
            print("No ROS bag files found.")

        download_url = url or self.DEFAULT_ROSBAG_URL

        # Install dependencies
        if not self._install_download_dependencies():
            return False

        print(f"📥 Downloading rosbag from {download_url}")
        print("This may take a few minutes depending on your internet connection...")

        # Import gdown here to avoid import error if not installed
        try:
            import gdown
        except ImportError:
            print("❌ Error: gdown is not available. Please run the installation first.")
            return False

        file_id = self._extract_file_id(download_url)
        if not file_id:
            print("❌ Error: Could not extract file ID from the URL.")
            return False

        output_path = self.rosbags_dir / "zed2i_camera.bag"

        try:
            # Use gdown to download from Google Drive
            gdown.download(id=file_id, output=str(output_path), quiet=False)

            if output_path.exists() and output_path.stat().st_size > 0:
                print(f"✅ Successfully downloaded rosbag to {output_path}")
                return True
            else:
                print("❌ Error: Download failed or file is empty.")
                return False
        except Exception as e:
            print(f"❌ Error downloading file: {e}")
            return False

    def video_to_ros_bag(
        self,
        video_path: str,
        output_bag_path: Optional[str] = None,
        target_fps: float = 10.0,
        include_camera_info: bool = False,
        rgb_topic: str = "/zed2i/zed_node/rgb/image_rect_color",
        camera_info_topic: str = "/zed2i/zed_node/rgb/camera_info",
        frame_id: str = "zed2i_left_camera_optical_frame",
    ):
        """Convert video files to ROS bags."""
        print("🎬 Converting video to ROS bag...")
        print(f"Input video: {video_path}")

        video_path_obj = Path(video_path)
        if not video_path_obj.exists():
            print(f"❌ Error: Video file not found: {video_path}")
            return False

        # Set default output path if not provided
        if output_bag_path is None:
            output_bag_path = str(self.rosbags_dir / f"{video_path_obj.stem}.bag")

        print(f"Output bag: {output_bag_path}")
        print(f"Target FPS: {target_fps}")
        print(f"Include camera info: {include_camera_info}")

        try:
            import cv2
            import rosbag
            import rospy
        except ImportError as e:
            print(f"❌ Error: Required dependencies not found: {e}")
            print("Please install: pip install opencv-python")
            print("And ensure ROS packages are available in the environment")
            return False

        # Open the video file
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"❌ Error: Could not open video file: {video_path}")
            return False

        # Get video properties
        original_fps = cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("Video properties:")
        print(f"  • Resolution: {width}x{height}")
        print(f"  • Original FPS: {original_fps}")
        print(f"  • Total frames: {total_frames}")

        # Calculate frame skip for target FPS
        frame_skip = max(1, int(original_fps / target_fps))
        duration_seconds = total_frames / original_fps
        expected_output_frames = int(duration_seconds * target_fps)

        print("Conversion settings:")
        print(f"  • Frame skip: {frame_skip}")
        print(f"  • Expected output frames: {expected_output_frames}")

        try:
            with rosbag.Bag(output_bag_path, "w") as bag:
                frame_count = 0
                written_frames = 0

                # Create camera info message if requested
                camera_info = None
                if include_camera_info:
                    camera_info = self._create_camera_info(width, height, frame_id)

                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break

                    # Skip frames to achieve target FPS
                    if frame_count % frame_skip != 0:
                        frame_count += 1
                        continue

                    # Calculate timestamp
                    timestamp = rospy.Time.from_sec(written_frames / target_fps)

                    # Convert OpenCV image to ROS Image message
                    ros_image = self._cv2_to_ros_image(frame, "bgr8", frame_id)
                    ros_image.header.stamp = timestamp

                    # Write image message to bag
                    bag.write(rgb_topic, ros_image, timestamp)

                    # Write camera info if requested
                    if camera_info:
                        camera_info.header.stamp = timestamp
                        bag.write(camera_info_topic, camera_info, timestamp)

                    written_frames += 1
                    frame_count += 1

                    # Progress update
                    if written_frames % 50 == 0:
                        progress = (frame_count / total_frames) * 100
                        print(
                            f"Progress: {progress:.1f}% ({written_frames} frames written)"
                        )

            cap.release()

            print("✅ Conversion completed successfully!")
            print(f"  • Output file: {output_bag_path}")
            print(f"  • Frames written: {written_frames}")
            print(f"  • Duration: {written_frames / target_fps:.2f} seconds")
            return True

        except Exception as e:
            print(f"❌ Error during conversion: {e}")
            cap.release()
            return False

    def _cv2_to_ros_image(
        self, cv_image, encoding: str = "bgr8", frame_id: str = "camera_link"
    ):
        """Convert an OpenCV image to a ROS sensor_msgs/Image message."""
        from sensor_msgs.msg import Image

        ros_image = Image()
        ros_image.header.frame_id = frame_id
        ros_image.height, ros_image.width = cv_image.shape[:2]
        ros_image.encoding = encoding
        ros_image.is_bigendian = False
        ros_image.step = (
            cv_image.strides[0]
            if cv_image.strides
            else cv_image.shape[1]
            * cv_image.itemsize
            * (cv_image.shape[2] if len(cv_image.shape) > 2 else 1)
        )
        ros_image.data = cv_image.tobytes()
        return ros_image

    def _create_camera_info(
        self, width: int, height: int, frame_id: str = "camera_link"
    ):
        """Create a basic CameraInfo message with reasonable defaults."""
        from sensor_msgs.msg import CameraInfo

        camera_info = CameraInfo()
        camera_info.header.frame_id = frame_id
        camera_info.width = width
        camera_info.height = height

        # Basic camera parameters (these are rough estimates)
        fx = fy = width * 0.8  # Rough focal length estimate
        cx = width / 2.0  # Principal point x
        cy = height / 2.0  # Principal point y

        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]  # No distortion
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        camera_info.distortion_model = "plumb_bob"

        return camera_info


# === Typer Commands ===


@ros_app.command("build")
def build_workspace_cmd(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory"),
    clean: bool = typer.Option(
        False, "--clean", "-c", help="Clean workspace before building"
    ),
):
    """Build the catkin workspace."""
    typer.echo("🔨 Building ROS workspace...")

    manager = HydrusRosManager(volume=volume)
    success = manager.build_workspace(clean=clean)

    if success:
        typer.echo("✅ Workspace built successfully!")
        raise typer.Exit(0)
    else:
        typer.echo("❌ Workspace build failed!")
        raise typer.Exit(1)


@ros_app.command("camera")
def run_camera_cmd(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory"),
    rviz: bool = typer.Option(
        False, "--rviz", "-r", help="Launch camera with RViz display"
    ),
):
    """Run the ZED camera entrypoint."""
    typer.echo("📷 Starting ZED camera...")

    manager = HydrusRosManager(volume=volume)
    success = manager.run_camera_entrypoint(zed_option=rviz)

    if success:
        typer.echo("✅ Camera stopped successfully!")
        raise typer.Exit(0)
    else:
        typer.echo("❌ Camera failed!")
        raise typer.Exit(1)


@ros_app.command("download-bags")
def download_bags_cmd(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory"),
    url: Optional[str] = typer.Option(None, "--url", "-u", help="Custom download URL"),
    force: bool = typer.Option(
        False, "--force", "-f", help="Force download even if bags exist"
    ),
):
    """Download ROS bag files."""
    typer.echo("📥 Downloading ROS bags...")

    manager = HydrusRosManager(volume=volume)
    success = manager.download_ros_bags(url=url, force=force)

    if success:
        typer.echo("✅ ROS bags downloaded successfully!")
        raise typer.Exit(0)
    else:
        typer.echo("❌ ROS bags download failed!")
        raise typer.Exit(1)


@ros_app.command("video-to-bag")
def video_to_bag_cmd(
    video_path: str = typer.Argument(..., help="Path to input video file"),
    output_path: Optional[str] = typer.Option(
        None, "--output", "-o", help="Output bag file path"
    ),
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory"),
    fps: float = typer.Option(10.0, "--fps", "-f", help="Target FPS for output bag"),
    camera_info: bool = typer.Option(
        False, "--camera-info", "-c", help="Include camera info messages"
    ),
    rgb_topic: str = typer.Option(
        "/zed2i/zed_node/rgb/image_rect_color", "--rgb-topic", help="RGB image topic"
    ),
    info_topic: str = typer.Option(
        "/zed2i/zed_node/rgb/camera_info", "--info-topic", help="Camera info topic"
    ),
    frame_id: str = typer.Option(
        "zed2i_left_camera_optical_frame", "--frame-id", help="Frame ID for messages"
    ),
):
    """Convert video file to ROS bag."""
    typer.echo(f"🎬 Converting video {video_path} to ROS bag...")

    manager = HydrusRosManager(volume=volume)
    success = manager.video_to_ros_bag(
        video_path=video_path,
        output_bag_path=output_path,
        target_fps=fps,
        include_camera_info=camera_info,
        rgb_topic=rgb_topic,
        camera_info_topic=info_topic,
        frame_id=frame_id,
    )

    if success:
        typer.echo("✅ Video converted to ROS bag successfully!")
        raise typer.Exit(0)
    else:
        typer.echo("❌ Video conversion failed!")
        raise typer.Exit(1)


@ros_app.command("list-bags")
def list_bags_cmd(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory")
):
    """List available ROS bag files."""
    manager = HydrusRosManager(volume=volume)

    bag_files = list(manager.rosbags_dir.glob("*.bag"))

    if bag_files:
        typer.echo("📋 Available ROS bag files:")
        for bag in bag_files:
            size_mb = bag.stat().st_size / (1024 * 1024)
            typer.echo(f"  • {bag.name} ({size_mb:.1f} MB)")
    else:
        typer.echo("📋 No ROS bag files found.")
        typer.echo(f"Directory: {manager.rosbags_dir}")


@ros_app.command("info")
def ros_info_cmd(
    volume: bool = typer.Option(False, "--volume", "-v", help="Use volume directory")
):
    """Show ROS workspace information."""
    manager = HydrusRosManager(volume=volume)

    typer.echo("📊 ROS Workspace Information:")
    typer.echo(f"  • Workspace path: {manager.workspace_dir}")
    typer.echo(f"  • ROSbags directory: {manager.rosbags_dir}")

    # Check if workspace exists
    if manager.workspace_dir.exists():
        typer.echo("  • Workspace: ✅ Found")

        # Check if built
        devel_dir = manager.workspace_dir / "devel"
        if devel_dir.exists():
            typer.echo("  • Built: ✅ Yes")
        else:
            typer.echo("  • Built: ❌ No")
    else:
        typer.echo("  • Workspace: ❌ Not found")

    # Check bag files
    bag_count = len(list(manager.rosbags_dir.glob("*.bag")))
    typer.echo(f"  • ROS bag files: {bag_count}")


if __name__ == "__main__":
    ros_app()

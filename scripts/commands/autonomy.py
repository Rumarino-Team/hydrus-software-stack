#!/usr/bin/env python3
"""
Autonomy command module for managing CV pipeline and related systems.
"""

import subprocess
import sys
import time
from pathlib import Path

try:
    import typer
    from rich.console import Console
    from rich.panel import Panel
    from rich.progress import Progress, SpinnerColumn, TextColumn
except ImportError as e:
    print(f"Error: Missing required dependency: {e}")
    print("Please install with: pip install typer rich")
    sys.exit(1)

console = Console()
autonomy_app = typer.Typer(help="Autonomy system management commands")

# Project root directory
PROJECT_ROOT = Path(__file__).parent.parent.parent


@autonomy_app.command()
def start_camera_publisher(
    device: int = typer.Option(0, help="Camera device index (e.g., 0 for /dev/video0)"),
    width: int = typer.Option(640, help="Camera image width"),
    height: int = typer.Option(480, help="Camera image height"),
    fps: int = typer.Option(30, help="Camera frame rate"),
    rgb_topic: str = typer.Option(
        "/camera/rgb/image_rect_color", help="RGB image topic name"
    ),
    depth_topic: str = typer.Option(
        "/camera/depth/depth_registered", help="Depth image topic name"
    ),
    camera_info_topic: str = typer.Option(
        "/camera/rgb/camera_info", help="Camera info topic name"
    ),
):
    """Start the webcam publisher that publishes ROS image topics for the CV pipeline."""
    console.print(
        Panel.fit(
            f"[bold green]Starting Camera Publisher[/bold green]\n"
            f"Device: /dev/video{device}\n"
            f"Resolution: {width}x{height}\n"
            f"FPS: {fps}\n"
            f"RGB Topic: {rgb_topic}\n"
            f"Depth Topic: {depth_topic}\n"
            f"Camera Info Topic: {camera_info_topic}",
            title="Camera Publisher Configuration",
        )
    )

    # Path to the camera publisher script
    script_path = PROJECT_ROOT / "autonomy" / "scripts" / "cv" / "camera_publisher.py"

    try:
        # Run the camera publisher with the specified parameters
        cmd = [
            sys.executable,
            str(script_path),
            "--device",
            str(device),
            "--width",
            str(width),
            "--height",
            str(height),
            "--fps",
            str(fps),
            "--rgb-topic",
            rgb_topic,
            "--depth-topic",
            depth_topic,
            "--camera-info-topic",
            camera_info_topic,
        ]

        console.print(f"[yellow]Executing: {' '.join(cmd)}[/yellow]")
        subprocess.run(cmd, check=True)

    except subprocess.CalledProcessError as e:
        console.print(f"[red]Error running camera publisher: {e}[/red]")
        raise typer.Exit(1)
    except KeyboardInterrupt:
        console.print("\n[yellow]Camera publisher stopped by user[/yellow]")
    except FileNotFoundError:
        console.print(f"[red]Camera publisher script not found at: {script_path}[/red]")
        console.print("[yellow]Creating the camera publisher script...[/yellow]")
        _create_camera_publisher_script()
        console.print(
            "[green]Camera publisher script created! Please run the command again.[/green]"
        )


@autonomy_app.command()
def start_cv_pipeline(
    model: str = typer.Option("yolov8n.pt", help="YOLO model to use"),
    rgb_topic: str = typer.Option(
        "/camera/rgb/image_rect_color", help="RGB image topic to subscribe to"
    ),
    depth_topic: str = typer.Option(
        "/camera/depth/depth_registered", help="Depth image topic to subscribe to"
    ),
    camera_info_topic: str = typer.Option(
        "/camera/rgb/camera_info", help="Camera info topic to subscribe to"
    ),
    camera_pose_topic: str = typer.Option(
        "/camera/pose", help="Camera pose topic to subscribe to"
    ),
):
    """Start the complete CV pipeline (cv_publishers) with specified parameters."""
    console.print(
        Panel.fit(
            f"[bold green]Starting CV Pipeline[/bold green]\n"
            f"YOLO Model: {model}\n"
            f"RGB Topic: {rgb_topic}\n"
            f"Depth Topic: {depth_topic}\n"
            f"Camera Info Topic: {camera_info_topic}\n"
            f"Camera Pose Topic: {camera_pose_topic}",
            title="CV Pipeline Configuration",
        )
    )

    # Path to cv_publishers.py
    cv_publishers_path = PROJECT_ROOT / "autonomy" / "src" / "cv_publishers.py"

    if not cv_publishers_path.exists():
        console.print(
            f"[red]CV publishers script not found at: {cv_publishers_path}[/red]"
        )
        raise typer.Exit(1)

    try:
        # Set ROS parameters and run cv_publishers
        cmd = [
            sys.executable,
            str(cv_publishers_path),
            f"_rgb_image_topic:={rgb_topic}",
            f"_depth_image_topic:={depth_topic}",
            f"_camera_info_topic:={camera_info_topic}",
            f"_camera_pose_topic:={camera_pose_topic}",
        ]

        console.print(f"[yellow]Executing: {' '.join(cmd)}[/yellow]")

        # Run with progress indicator
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            console=console,
        ) as progress:
            progress.add_task("Starting CV Pipeline...", total=None)

            # Start the process
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )

            # Monitor the process
            while process.poll() is None:
                time.sleep(0.1)

            if process.returncode != 0:
                output = process.stdout.read() if process.stdout else ""
                console.print(
                    f"[red]CV Pipeline failed with return code {process.returncode}[/red]"
                )
                if output:
                    console.print(f"[red]Output: {output}[/red]")
                raise typer.Exit(1)

    except subprocess.CalledProcessError as e:
        console.print(f"[red]Error running CV pipeline: {e}[/red]")
        raise typer.Exit(1)
    except KeyboardInterrupt:
        console.print("\n[yellow]CV Pipeline stopped by user[/yellow]")
    except Exception as e:
        console.print(f"[red]Unexpected error: {e}[/red]")
        raise typer.Exit(1)


@autonomy_app.command()
def start_full_pipeline(
    camera_device: int = typer.Option(0, help="Camera device index"),
    camera_width: int = typer.Option(640, help="Camera image width"),
    camera_height: int = typer.Option(480, help="Camera image height"),
    camera_fps: int = typer.Option(30, help="Camera frame rate"),
    model: str = typer.Option("yolov8n.pt", help="YOLO model to use"),
    delay: int = typer.Option(
        3, help="Delay between starting camera and CV pipeline (seconds)"
    ),
):
    """Start both the camera publisher and CV pipeline in sequence."""
    console.print(
        Panel.fit(
            "[bold green]Starting Full CV Pipeline[/bold green]\n"
            "This will start:\n"
            "1. Camera publisher (webcam → ROS topics)\n"
            "2. CV pipeline (object detection & processing)",
            title="Full Pipeline Startup",
        )
    )

    # Define common topic names
    rgb_topic = "/camera/rgb/image_rect_color"
    depth_topic = "/camera/depth/depth_registered"
    camera_info_topic = "/camera/rgb/camera_info"
    camera_pose_topic = "/camera/pose"

    try:
        # Step 1: Start camera publisher in background
        console.print("[yellow]Step 1: Starting camera publisher...[/yellow]")
        camera_script = (
            PROJECT_ROOT / "autonomy" / "scripts" / "cv" / "camera_publisher.py"
        )

        if not camera_script.exists():
            console.print("[yellow]Creating camera publisher script...[/yellow]")
            _create_camera_publisher_script()

        camera_cmd = [
            sys.executable,
            str(camera_script),
            "--device",
            str(camera_device),
            "--width",
            str(camera_width),
            "--height",
            str(camera_height),
            "--fps",
            str(camera_fps),
            "--rgb-topic",
            rgb_topic,
            "--depth-topic",
            depth_topic,
            "--camera-info-topic",
            camera_info_topic,
        ]

        camera_process = subprocess.Popen(camera_cmd)
        console.print("[green]Camera publisher started![/green]")

        # Step 2: Wait for camera to initialize
        console.print(
            f"[yellow]Waiting {delay} seconds for camera to initialize...[/yellow]"
        )
        time.sleep(delay)

        # Step 3: Start CV pipeline
        console.print("[yellow]Step 2: Starting CV pipeline...[/yellow]")
        cv_cmd = [
            sys.executable,
            str(PROJECT_ROOT / "autonomy" / "src" / "cv_publishers.py"),
            f"_rgb_image_topic:={rgb_topic}",
            f"_depth_image_topic:={depth_topic}",
            f"_camera_info_topic:={camera_info_topic}",
            f"_camera_pose_topic:={camera_pose_topic}",
        ]

        cv_process = subprocess.Popen(cv_cmd)
        console.print("[green]CV pipeline started![/green]")

        console.print(
            Panel.fit(
                "[bold green]Full Pipeline Running![/bold green]\n"
                "Press Ctrl+C to stop both processes",
                title="Pipeline Status",
            )
        )

        # Wait for user interruption
        try:
            camera_process.wait()
        except KeyboardInterrupt:
            console.print("\n[yellow]Stopping pipeline...[/yellow]")
            camera_process.terminate()
            cv_process.terminate()

            # Wait for processes to terminate
            camera_process.wait(timeout=5)
            cv_process.wait(timeout=5)

            console.print("[green]Pipeline stopped successfully![/green]")

    except Exception as e:
        console.print(f"[red]Error in full pipeline: {e}[/red]")
        raise typer.Exit(1)


def _create_camera_publisher_script():
    """Create the camera publisher script if it doesn't exist."""
    script_dir = PROJECT_ROOT / "autonomy" / "scripts" / "cv"
    script_dir.mkdir(parents=True, exist_ok=True)

    script_path = script_dir / "camera_publisher.py"

    script_content = '''#!/usr/bin/env python3
"""
Camera Publisher - Publishes webcam images to ROS topics
This script captures images from a webcam and publishes them as ROS Image messages.
"""

import argparse
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class CameraPublisher:
    def __init__(self, device=0, width=640, height=480, fps=30,
                 rgb_topic="/camera/rgb/image_rect_color",
                 depth_topic="/camera/depth/depth_registered",
                 camera_info_topic="/camera/rgb/camera_info"):

        rospy.init_node('camera_publisher', anonymous=True)

        self.device = device
        self.width = width
        self.height = height
        self.fps = fps

        # Initialize camera
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera device {device}")
            return

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Publishers
        self.rgb_pub = rospy.Publisher(rgb_topic, Image, queue_size=1)
        self.depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=1)

        # Camera info message (basic calibration)
        self.camera_info = self._create_camera_info()

        rospy.loginfo(f"Camera publisher initialized:")
        rospy.loginfo(f"  Device: {device}")
        rospy.loginfo(f"  Resolution: {width}x{height}")
        rospy.loginfo(f"  FPS: {fps}")
        rospy.loginfo(f"  RGB Topic: {rgb_topic}")
        rospy.loginfo(f"  Depth Topic: {depth_topic}")
        rospy.loginfo(f"  Camera Info Topic: {camera_info_topic}")

    def _create_camera_info(self):
        """Create a basic camera info message."""
        cam_info = CameraInfo()
        cam_info.width = self.width
        cam_info.height = self.height

        # Basic camera matrix (estimated for webcam)
        fx = fy = self.width * 0.8  # Rough estimate
        cx = self.width / 2.0
        cy = self.height / 2.0

        cam_info.K = [fx, 0, cx,
                      0, fy, cy,
                      0, 0, 1]

        cam_info.D = [0, 0, 0, 0, 0]  # No distortion

        cam_info.R = [1, 0, 0,
                      0, 1, 0,
                      0, 0, 1]

        cam_info.P = [fx, 0, cx, 0,
                      0, fy, cy, 0,
                      0, 0, 1, 0]

        return cam_info

    def cv2_to_ros_image(self, cv_image, encoding="bgr8"):
        """Convert OpenCV image to ROS Image message."""
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = "camera_link"
        ros_image.height = cv_image.shape[0]
        ros_image.width = cv_image.shape[1]
        ros_image.encoding = encoding
        ros_image.is_bigendian = False
        ros_image.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) == 3 else cv_image.shape[1]
        ros_image.data = cv_image.tobytes()
        return ros_image

    def create_mock_depth(self, rgb_image):
        """Create a mock depth image from RGB (for demonstration)."""
        # Convert to grayscale and use as depth
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        # Scale to simulate depth (closer objects are brighter)
        depth = 255 - gray
        return depth.astype(np.uint8)

    def run(self):
        """Main publishing loop."""
        rate = rospy.Rate(self.fps)

        rospy.loginfo("Starting camera publishing loop...")

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue

            # Create timestamp
            timestamp = rospy.Time.now()

            # Publish RGB image
            rgb_msg = self.cv2_to_ros_image(frame, "bgr8")
            rgb_msg.header.stamp = timestamp
            self.rgb_pub.publish(rgb_msg)

            # Create and publish mock depth image
            depth_frame = self.create_mock_depth(frame)
            depth_msg = self.cv2_to_ros_image(depth_frame, "mono8")
            depth_msg.header.stamp = timestamp
            self.depth_pub.publish(depth_msg)

            # Publish camera info
            self.camera_info.header.stamp = timestamp
            self.camera_info.header.frame_id = "camera_link"
            self.camera_info_pub.publish(self.camera_info)

            rate.sleep()

    def __del__(self):
        """Cleanup resources."""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Camera Publisher for ROS")
    parser.add_argument("--device", type=int, default=0, help="Camera device index")
    parser.add_argument("--width", type=int, default=640, help="Image width")
    parser.add_argument("--height", type=int, default=480, help="Image height")
    parser.add_argument("--fps", type=int, default=30, help="Frame rate")
    parser.add_argument("--rgb-topic", default="/camera/rgb/image_rect_color", help="RGB topic")
    parser.add_argument("--depth-topic", default="/camera/depth/depth_registered", help="Depth topic")
    parser.add_argument("--camera-info-topic", default="/camera/rgb/camera_info", help="Camera info topic")

    args = parser.parse_args()

    try:
        publisher = CameraPublisher(
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            rgb_topic=args.rgb_topic,
            depth_topic=args.depth_topic,
            camera_info_topic=args.camera_info_topic
        )
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera publisher interrupted")
    except KeyboardInterrupt:
        rospy.loginfo("Camera publisher stopped by user")
    except Exception as e:
        rospy.logerr(f"Camera publisher error: {e}")


if __name__ == "__main__":
    main()
'''

    with open(script_path, "w") as f:
        f.write(script_content)

    # Make script executable
    script_path.chmod(0o755)
    console.print(f"[green]Created camera publisher script at: {script_path}[/green]")


if __name__ == "__main__":
    autonomy_app()

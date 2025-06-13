#!/usr/bin/env python3
"""
Video to ROS Bag Converter

This script converts a video file to a ROS bag containing sensor_msgs/Image messages
that can be used to simulate camera input for the cv_publishers node.

The generated rosbag will contain:
- RGB images on topic: /zed2i/zed_node/rgb/image_rect_color
- Camera info on topic: /zed2i/zed_node/rgb/camera_info (optional)

Usage:
    python3 video_to_rosbag.py input_video.mp4 output.bag [--fps 10] [--camera-info]

Author: GitHub Copilot
Date: June 13, 2025
"""

import argparse
import os
import sys
from typing import Optional

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def cv2_to_ros_image(
    cv_image: np.ndarray, encoding: str = "bgr8", frame_id: str = "camera_link"
) -> Image:
    """
    Convert an OpenCV image to a ROS sensor_msgs/Image message.

    Args:
        cv_image: OpenCV image (numpy array)
        encoding: Image encoding (default: "bgr8")
        frame_id: Frame ID for the image header

    Returns:
        ROS Image message
    """
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


def create_camera_info(
    width: int, height: int, frame_id: str = "camera_link"
) -> CameraInfo:
    """
    Create a basic CameraInfo message with reasonable defaults.

    Args:
        width: Image width
        height: Image height
        frame_id: Frame ID for the camera info header

    Returns:
        CameraInfo message
    """
    camera_info = CameraInfo()
    camera_info.header.frame_id = frame_id
    camera_info.width = width
    camera_info.height = height

    # Basic camera parameters (these are rough estimates)
    # In a real scenario, you would calibrate your camera
    fx = fy = width * 0.8  # Rough focal length estimate
    cx = width / 2.0  # Principal point x
    cy = height / 2.0  # Principal point y

    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    camera_info.D = [0, 0, 0, 0, 0]  # No distortion

    camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    camera_info.distortion_model = "plumb_bob"

    return camera_info


def convert_video_to_rosbag(
    video_path: str,
    output_bag_path: str,
    target_fps: float = 10.0,
    include_camera_info: bool = False,
    rgb_topic: str = "/zed2i/zed_node/rgb/image_rect_color",
    camera_info_topic: str = "/zed2i/zed_node/rgb/camera_info",
    frame_id: str = "zed2i_left_camera_optical_frame",
) -> bool:
    """
    Convert a video file to a ROS bag with image messages.

    Args:
        video_path: Path to input video file
        output_bag_path: Path for output bag file
        target_fps: Target FPS for the output bag (will subsample if needed)
        include_camera_info: Whether to include camera info messages
        rgb_topic: Topic name for RGB images
        camera_info_topic: Topic name for camera info
        frame_id: Frame ID for messages

    Returns:
        True if successful, False otherwise
    """
    if not os.path.exists(video_path):
        print(f"Error: Video file '{video_path}' not found")
        return False

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file '{video_path}'")
        return False

    # Get video properties
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Video properties:")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {video_fps}")
    print(f"  Total frames: {total_frames}")
    print(f"  Duration: {total_frames/video_fps:.2f} seconds")
    print(f"  Target FPS: {target_fps}")

    # Calculate frame skip to achieve target FPS
    frame_skip = max(1, int(video_fps / target_fps))
    effective_fps = video_fps / frame_skip

    print(f"  Frame skip: {frame_skip}")
    print(f"  Effective FPS: {effective_fps:.2f}")

    # Create camera info message if needed
    camera_info_msg = None
    if include_camera_info:
        camera_info_msg = create_camera_info(width, height, frame_id)

    # Open the output bag
    try:
        bag = rosbag.Bag(output_bag_path, "w")
    except Exception as e:
        print(f"Error: Could not create bag file '{output_bag_path}': {e}")
        cap.release()
        return False

    frame_count = 0
    written_frames = 0

    print(f"\nConverting video to rosbag...")
    print(f"Output: {output_bag_path}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Skip frames to achieve target FPS
            if frame_count % frame_skip != 0:
                frame_count += 1
                continue

            # Calculate timestamp
            timestamp = rospy.Time.from_sec(written_frames / effective_fps)

            # Convert frame to ROS Image message
            ros_image = cv2_to_ros_image(frame, "bgr8", frame_id)
            ros_image.header.stamp = timestamp

            # Write image to bag
            bag.write(rgb_topic, ros_image, timestamp)

            # Write camera info if requested
            if camera_info_msg is not None:
                camera_info_msg.header.stamp = timestamp
                bag.write(camera_info_topic, camera_info_msg, timestamp)

            written_frames += 1
            frame_count += 1

            # Progress indicator
            if written_frames % 10 == 0:
                progress = (frame_count / total_frames) * 100
                print(f"  Progress: {progress:.1f}% ({written_frames} frames written)")

        print(f"\nConversion complete!")
        print(f"  Written {written_frames} frames")
        print(f"  Duration: {written_frames / effective_fps:.2f} seconds")

    except Exception as e:
        print(f"Error during conversion: {e}")
        return False

    finally:
        cap.release()
        bag.close()

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Convert a video file to a ROS bag for cv_publishers simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 video_to_rosbag.py input.mp4 output.bag
  python3 video_to_rosbag.py input.mp4 output.bag --fps 15 --camera-info
  python3 video_to_rosbag.py input.mp4 output.bag --rgb-topic /camera/image_raw
        """,
    )

    parser.add_argument("video_path", help="Path to input video file")
    parser.add_argument("output_bag", help="Path for output ROS bag file")
    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
        help="Target FPS for output bag (default: 10.0)",
    )
    parser.add_argument(
        "--camera-info",
        action="store_true",
        help="Include camera info messages in the bag",
    )
    parser.add_argument(
        "--rgb-topic",
        default="/zed2i/zed_node/rgb/image_rect_color",
        help="RGB image topic name (default: /zed2i/zed_node/rgb/image_rect_color)",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/zed2i/zed_node/rgb/camera_info",
        help="Camera info topic name (default: /zed2i/zed_node/rgb/camera_info)",
    )
    parser.add_argument(
        "--frame-id",
        default="zed2i_left_camera_optical_frame",
        help="Frame ID for messages (default: zed2i_left_camera_optical_frame)",
    )

    args = parser.parse_args()

    # Validate FPS
    if args.fps <= 0:
        print("Error: FPS must be positive")
        return 1

    # Validate output path
    output_dir = os.path.dirname(args.output_bag)
    if output_dir and not os.path.exists(output_dir):
        print(f"Error: Output directory '{output_dir}' does not exist")
        return 1

    # Convert the video
    success = convert_video_to_rosbag(
        video_path=args.video_path,
        output_bag_path=args.output_bag,
        target_fps=args.fps,
        include_camera_info=args.camera_info,
        rgb_topic=args.rgb_topic,
        camera_info_topic=args.camera_info_topic,
        frame_id=args.frame_id,
    )

    if success:
        print(f"\n✅ Successfully converted '{args.video_path}' to '{args.output_bag}'")
        print(f"\nTo play the bag with cv_publishers:")
        print(f"  1. Start roscore: roscore")
        print(f"  2. Play the bag: rosbag play {args.output_bag}")
        print(f"  3. Launch cv_publishers: roslaunch autonomy cv_publishers.launch")
        return 0
    else:
        print(f"\n❌ Failed to convert '{args.video_path}'")
        return 1


if __name__ == "__main__":
    sys.exit(main())

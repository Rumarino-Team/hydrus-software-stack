#!/usr/bin/env python3
"""
Example usage of the 3D bounding box system.
This script demonstrates how to use the new 3D bounding box functionality.
"""

import sys
import os

# Add the autonomy src to the path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import numpy as np
import custom_types
from computer_vision.detection_core import calculate_3d_bounding_boxes


def example_3d_bounding_box_usage():
    """Example showing how to use the 3D bounding box system."""
    print("=== 3D Bounding Box Example ===")

    # Create a mock detection (normally from YOLO)
    detection = custom_types.Detection(
        x1=200, y1=150, x2=400, y2=350, cls=0, conf=0.9  # 2D bounding box
    )

    # Create a mock depth image (normally from depth camera)
    depth_image = np.full((480, 640), 5.0, dtype=np.float32)
    depth_image[150:350, 200:400] = 3.0  # Object at 3 meters

    # Camera intrinsics (normally from camera calibration)
    camera_intrinsics = (525.0, 525.0, 320.0, 240.0)  # fx, fy, cx, cy

    # Calculate 3D bounding box
    calculate_3d_bounding_boxes([detection], depth_image, camera_intrinsics)

    if detection.bbox_3d is not None:
        print("✅ 3D Bounding Box calculated!")
        print(
            f"Center: ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f}) meters"
        )
        print(
            f"Size: {detection.bbox_3d.width:.2f} x {detection.bbox_3d.height:.2f} x {detection.bbox_3d.depth:.2f} meters"
        )

        # Get the 8 corner points
        corners = detection.bbox_3d.get_corners()
        print(f"Corner points: {len(corners)} corners calculated")
        for i, corner in enumerate(corners[:4]):  # Show first 4 corners
            print(f"  Corner {i+1}: ({corner.x:.2f}, {corner.y:.2f}, {corner.z:.2f})")
        print("  ...")

        # Compare with legacy centroid point
        print(
            f"Legacy centroid: ({detection.point.x:.2f}, {detection.point.y:.2f}, {detection.point.z:.2f}) meters"
        )
    else:
        print("❌ Failed to calculate 3D bounding box")


if __name__ == "__main__":
    example_3d_bounding_box_usage()

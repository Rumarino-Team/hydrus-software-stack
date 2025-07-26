#!/usr/bin/env python3
"""
Test script for 3D bounding box functionality.
"""

import os
import sys

# Add the parent directory to the path to import custom_types and detection_core
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

import custom_types
import numpy as np

from computer_vision.detection_core import (
    DetectionPipelineManager,
    calculate_3d_bounding_boxes,
)


def test_bounding_box_3d_creation():
    """Test basic 3D bounding box creation and corner calculation."""
    print("Testing BoundingBox3D creation...")

    center = custom_types.Point3D(x=1.0, y=2.0, z=3.0)
    bbox_3d = custom_types.BoundingBox3D(
        center=center, width=2.0, height=4.0, depth=6.0
    )

    corners = bbox_3d.get_corners()
    print(f"Created 3D bounding box with center: {center}")
    print(
        f"Dimensions: width={bbox_3d.width}, height={bbox_3d.height}, depth={bbox_3d.depth}"
    )
    print(f"Number of corners: {len(corners)}")

    # Verify we have 8 corners
    assert len(corners) == 8, f"Expected 8 corners, got {len(corners)}"

    # Verify corner positions (basic sanity check)
    expected_x_values = [0.0, 2.0]  # center(1.0) +/- width/2(1.0)
    expected_y_values = [0.0, 4.0]  # center(2.0) +/- height/2(2.0)
    expected_z_values = [0.0, 6.0]  # center(3.0) +/- depth/2(3.0)

    for corner in corners:
        assert corner.x in expected_x_values, f"Unexpected X value: {corner.x}"
        assert corner.y in expected_y_values, f"Unexpected Y value: {corner.y}"
        assert corner.z in expected_z_values, f"Unexpected Z value: {corner.z}"

    print("✓ BoundingBox3D creation test passed!")


def test_3d_bounding_box_calculation():
    """Test 3D bounding box calculation from 2D detections and depth."""
    print("\nTesting 3D bounding box calculation...")

    # Create test detection (2D bounding box)
    detection = custom_types.Detection(
        x1=100, y1=50, x2=200, y2=150, cls=0, conf=0.9  # 100x100 pixel box
    )

    # Create synthetic depth image with constant depth of 5 meters
    depth_image = np.full((480, 640), 5.0, dtype=np.float32)

    # Add some depth variation in the detection region to test depth estimation
    depth_image[50:150, 100:200] = 4.8  # Main object at 4.8m
    depth_image[60:80, 120:140] = 5.2  # Some variation

    # Camera intrinsics (typical values)
    camera_intrinsics = (500.0, 500.0, 320.0, 240.0)  # fx, fy, cx, cy

    # Calculate 3D bounding box
    calculate_3d_bounding_boxes([detection], depth_image, camera_intrinsics)

    # Verify results
    assert detection.point is not None, "3D point should be calculated"
    assert detection.bbox_3d is not None, "3D bounding box should be calculated"

    print(
        f"3D point: x={detection.point.x:.3f}, y={detection.point.y:.3f}, z={detection.point.z:.3f}"
    )
    print(
        f"3D bbox center: x={detection.bbox_3d.center.x:.3f}, y={detection.bbox_3d.center.y:.3f}, z={detection.bbox_3d.center.z:.3f}"
    )
    print(
        f"3D bbox dimensions: w={detection.bbox_3d.width:.3f}, h={detection.bbox_3d.height:.3f}, d={detection.bbox_3d.depth:.3f}"
    )

    # Basic sanity checks
    assert detection.depth > 0, "Depth should be positive"
    assert detection.bbox_3d.width > 0, "Width should be positive"
    assert detection.bbox_3d.height > 0, "Height should be positive"
    assert detection.bbox_3d.depth > 0, "Depth should be positive"

    print("✓ 3D bounding box calculation test passed!")


def test_detection_pipeline_with_3d_boxes():
    """Test the complete detection pipeline with 3D bounding boxes."""
    print("\nTesting detection pipeline with 3D bounding boxes...")

    # Create test image and depth image
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)

    # Add a simple pattern to potentially trigger detection
    # Create a white rectangle that might be detected
    test_image[200:280, 250:350] = 255

    depth_image = np.full((480, 640), 3.0, dtype=np.float32)
    camera_intrinsics = (500.0, 500.0, 320.0, 240.0)

    # Test pipeline manager
    manager = DetectionPipelineManager()

    # Run detections with 3D calculation
    results = manager.run_detections(
        image=test_image, depth_image=depth_image, camera_intrinsics=camera_intrinsics
    )

    print(f"Pipeline returned {len(results)} detector results")

    for detector_name, detections in results:
        print(f"  {detector_name}: {len(detections)} detections")
        for i, detection in enumerate(detections):
            if detection.bbox_3d is not None:
                print(
                    f"    Detection {i}: 3D bbox center=({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f})"
                )
                print(
                    f"    Detection {i}: 3D bbox dims=({detection.bbox_3d.width:.2f}, {detection.bbox_3d.height:.2f}, {detection.bbox_3d.depth:.2f})"
                )

    print("✓ Detection pipeline with 3D bounding boxes test passed!")


def test_corner_calculation():
    """Test the corner calculation functionality."""
    print("\nTesting corner calculation...")

    # Simple test case: unit cube at origin
    center = custom_types.Point3D(x=0.0, y=0.0, z=0.0)
    bbox_3d = custom_types.BoundingBox3D(
        center=center, width=2.0, height=2.0, depth=2.0
    )

    corners = bbox_3d.get_corners()

    # Expected corners for a 2x2x2 cube centered at origin
    expected_corners = [
        (-1, -1, -1),  # front-bottom-left
        (1, -1, -1),  # front-bottom-right
        (-1, 1, -1),  # front-top-left
        (1, 1, -1),  # front-top-right
        (-1, -1, 1),  # back-bottom-left
        (1, -1, 1),  # back-bottom-right
        (-1, 1, 1),  # back-top-left
        (1, 1, 1),  # back-top-right
    ]

    for i, (corner, expected) in enumerate(zip(corners, expected_corners)):
        assert (
            abs(corner.x - expected[0]) < 1e-6
        ), f"Corner {i} X mismatch: {corner.x} vs {expected[0]}"
        assert (
            abs(corner.y - expected[1]) < 1e-6
        ), f"Corner {i} Y mismatch: {corner.y} vs {expected[1]}"
        assert (
            abs(corner.z - expected[2]) < 1e-6
        ), f"Corner {i} Z mismatch: {corner.z} vs {expected[2]}"

    print("✓ Corner calculation test passed!")


def main():
    """Run all tests."""
    print("Running 3D bounding box tests...\n")

    try:
        test_bounding_box_3d_creation()
        test_corner_calculation()
        test_3d_bounding_box_calculation()
        test_detection_pipeline_with_3d_boxes()

        print("\n" + "=" * 50)
        print("✅ All 3D bounding box tests passed!")
        print("=" * 50)

    except AssertionError as e:
        print(f"\n❌ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())

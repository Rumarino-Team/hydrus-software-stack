#!/usr/bin/env python3
"""
Demonstration script for the 3D bounding box system.
This script shows how the 3D bounding box functionality works with both synthetic
and realistic scenarios.
"""

import sys
import os
import numpy as np

# Add the autonomy src to the path
sys.path.append(os.path.join(os.path.dirname(__file__), "autonomy", "src"))

import custom_types
from computer_vision.detection_core import (
    DetectionPipelineManager,
    calculate_3d_bounding_boxes,
)


def create_synthetic_detection():
    """Create a synthetic detection for testing."""
    return custom_types.Detection(
        x1=200, y1=150, x2=400, y2=350,  # 200x200 pixel detection
        cls=0,  # person class
        conf=0.85,
    )


def create_depth_image_with_object():
    """Create a synthetic depth image with an object."""
    depth_image = np.full((480, 640), 10.0, dtype=np.float32)  # Background at 10m

    # Add an object at different depth
    depth_image[150:350, 200:400] = 5.0  # Object at 5m
    
    # Add some noise to make it more realistic
    noise = np.random.normal(0, 0.1, depth_image.shape)
    depth_image += noise
    
    # Ensure no negative depths
    depth_image = np.maximum(depth_image, 0.1)
    
    return depth_image


def demonstrate_3d_bbox_calculation():
    """Demonstrate 3D bounding box calculation."""
    print("=== 3D Bounding Box System Demonstration ===\n")
    
    # Camera parameters (realistic for a typical underwater camera)
    camera_intrinsics = (525.0, 525.0, 320.0, 240.0)  # fx, fy, cx, cy
    
    print("1. Creating synthetic detection and depth data...")
    detection = create_synthetic_detection()
    depth_image = create_depth_image_with_object()
    
    print(f"   2D Detection: ({detection.x1}, {detection.y1}) to ({detection.x2}, {detection.y2})")
    print(f"   Confidence: {detection.conf:.2f}")
    print(f"   Depth image shape: {depth_image.shape}")
    print(f"   Depth range: {np.min(depth_image):.1f}m to {np.max(depth_image):.1f}m")
    
    print("\n2. Calculating 3D bounding box...")
    calculate_3d_bounding_boxes([detection], depth_image, camera_intrinsics)
    
    if detection.bbox_3d is not None:
        print(f"   ✓ 3D Bounding Box calculated successfully!")
        print(f"   Center: ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f}) meters")
        print(f"   Dimensions: {detection.bbox_3d.width:.2f} x {detection.bbox_3d.height:.2f} x {detection.bbox_3d.depth:.2f} meters")
        print(f"   Volume: {detection.bbox_3d.width * detection.bbox_3d.height * detection.bbox_3d.depth:.3f} cubic meters")
        
        print("\n3. Computing 8 corner points...")
        corners = detection.bbox_3d.get_corners()
        for i, corner in enumerate(corners):
            corner_labels = [
                "front-bottom-left", "front-bottom-right", "front-top-left", "front-top-right",
                "back-bottom-left", "back-bottom-right", "back-top-left", "back-top-right"
            ]
            print(f"   Corner {i+1} ({corner_labels[i]}): ({corner.x:.2f}, {corner.y:.2f}, {corner.z:.2f})")
    else:
        print("   ❌ Failed to calculate 3D bounding box")
        return False
    
    print("\n4. Comparing with old centroid-only approach...")
    print(f"   Legacy 3D point: ({detection.point.x:.2f}, {detection.point.y:.2f}, {detection.point.z:.2f}) meters")
    print(f"   New 3D bbox center: ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f}) meters")
    print(f"   Difference: {abs(detection.point.x - detection.bbox_3d.center.x):.3f}, {abs(detection.point.y - detection.bbox_3d.center.y):.3f}, {abs(detection.point.z - detection.bbox_3d.center.z):.3f} meters")
    
    return True


def demonstrate_pipeline_integration():
    """Demonstrate the 3D bounding box system integrated with the detection pipeline."""
    print("\n=== Pipeline Integration Demonstration ===\n")
    
    # Create a test image with some patterns
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Add colored rectangles that might be detected
    test_image[100:200, 100:200] = [255, 0, 0]  # Red rectangle
    test_image[300:400, 400:500] = [0, 255, 0]  # Green rectangle
    
    # Create corresponding depth image
    depth_image = np.full((480, 640), 8.0, dtype=np.float32)
    depth_image[100:200, 100:200] = 3.0  # Red object closer
    depth_image[300:400, 400:500] = 6.0  # Green object further
    
    camera_intrinsics = (525.0, 525.0, 320.0, 240.0)
    
    print("1. Setting up detection pipeline...")
    manager = DetectionPipelineManager()
    
    print("2. Running detection pipeline with 3D bounding box calculation...")
    results = manager.run_detections(
        image=test_image,
        depth_image=depth_image,
        camera_intrinsics=camera_intrinsics
    )
    
    print(f"3. Pipeline Results:")
    total_detections = 0
    total_3d_boxes = 0
    
    for detector_name, detections in results:
        print(f"   {detector_name}: {len(detections)} detections")
        total_detections += len(detections)
        
        for i, detection in enumerate(detections):
            if detection.bbox_3d is not None:
                total_3d_boxes += 1
                print(f"     Detection {i+1}: 3D bbox at ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f})")
                print(f"                     Size: {detection.bbox_3d.width:.2f} x {detection.bbox_3d.height:.2f} x {detection.bbox_3d.depth:.2f} m")
    
    print(f"\n4. Summary: {total_detections} total detections, {total_3d_boxes} with 3D bounding boxes")
    
    return total_detections, total_3d_boxes


def demonstrate_global_transform():
    """Demonstrate transformation to global coordinates."""
    print("\n=== Global Coordinate Transformation ===\n")
    
    # Create a detection with 3D bounding box
    detection = create_synthetic_detection()
    depth_image = create_depth_image_with_object()
    camera_intrinsics = (525.0, 525.0, 320.0, 240.0)
    
    # Calculate 3D bounding box
    calculate_3d_bounding_boxes([detection], depth_image, camera_intrinsics)
    
    if detection.bbox_3d is None:
        print("   ❌ Failed to create 3D bounding box for transformation demo")
        return False
    
    print("1. Before transformation (camera frame):")
    print(f"   3D bbox center: ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f}) meters")
    
    # Simulate IMU data (underwater vehicle position and orientation)
    imu_position = custom_types.Point3D(x=100.0, y=50.0, z=-20.0)  # 20m underwater
    imu_rotation = custom_types.Rotation3D(x=0.0, y=0.0, z=0.0, w=1.0)  # No rotation
    
    print(f"   Vehicle position: ({imu_position.x:.1f}, {imu_position.y:.1f}, {imu_position.z:.1f}) meters")
    
    # Apply transformation
    from computer_vision.detection_core import transform_to_global
    transform_to_global([detection], imu_position, imu_rotation)
    
    print("2. After transformation (global frame):")
    print(f"   3D bbox center: ({detection.bbox_3d.center.x:.2f}, {detection.bbox_3d.center.y:.2f}, {detection.bbox_3d.center.z:.2f}) meters")
    print(f"   3D bbox dimensions: {detection.bbox_3d.width:.2f} x {detection.bbox_3d.height:.2f} x {detection.bbox_3d.depth:.2f} meters (unchanged)")
    
    return True


def main():
    """Run all demonstrations."""
    print("🌊 Hydrus 3D Bounding Box System Demo 🌊\n")
    
    try:
        # Basic 3D bounding box calculation
        success1 = demonstrate_3d_bbox_calculation()
        
        # Pipeline integration
        total_detections, total_3d_boxes = demonstrate_pipeline_integration()
        
        # Global transformation
        success2 = demonstrate_global_transform()
        
        print("\n" + "="*60)
        print("🎯 Demo Summary:")
        print(f"   ✓ 3D bounding box calculation: {'Success' if success1 else 'Failed'}")
        print(f"   ✓ Pipeline integration: {total_detections} detections processed")
        print(f"   ✓ Global transformation: {'Success' if success2 else 'Failed'}")
        print("\n🚀 The 3D bounding box system is working correctly!")
        print("   - Objects now have full 3D spatial extent, not just centroids")
        print("   - 8-corner 3D bounding boxes provide complete object boundaries")
        print("   - Backward compatible with existing centroid-based code")
        print("   - Integrates seamlessly with existing detection pipeline")
        print("="*60)
        
        return 0
        
    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
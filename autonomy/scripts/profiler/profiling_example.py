#!/usr/bin/env python3
"""
Example: Instrumented CV Publishers with Function Profiling

This is an example of how to add function-level profiling to your cv_publishers.py
to measure execution times of individual functions.
"""

import os
import sys

# Add these imports at the top of cv_publishers.py
import cv2
import rospy

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
from profiler.profiling_decorators import (
    FunctionProfiler,
    ProfileManager,
    profile_function,
    profile_method,
    profile_ros_callback,
)

# Initialize profiling for this node
ProfileManager.get_instance().set_node_name("cv_publisher")

# Example global variables (replace with your actual variables)
rgb_image = None
depth_image = None
camera_intrinsics = None
imu_point = None
imu_rotation = None
color_filter_config = None
model = None  # Your YOLO model (set to None for mock detection)

# Example of how to instrument the existing functions:


@profile_ros_callback("rgb_image")
def rgb_image_callback(msg):
    """Process incoming RGB images from ROS with profiling"""
    global rgb_image
    rospy.loginfo("Received RGB image message")

    with FunctionProfiler("cv.image_conversion"):
        try:
            from cv_bridge import CvBridge

            bridge = CvBridge()
            height = msg.height
            width = msg.width
            encoding = msg.encoding

            # Convert ROS image to OpenCV format
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"Converted image: {width}x{height}, encoding: {encoding}")

        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
            rgb_image = None


@profile_function("color_filter_detection", "cv")
def color_filter(image, config):
    """Color filter detection with profiling"""
    if image is None or config is None:
        return []

    with FunctionProfiler("cv.hsv_conversion"):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    with FunctionProfiler("cv.color_threshold"):
        # Example HSV ranges for detecting orange objects
        lower_orange = (10, 100, 100)
        upper_orange = (25, 255, 255)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

    with FunctionProfiler("cv.contour_detection"):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detection_results = []
        for contour in contours:
            # Filter by area to remove noise
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                detection_results.append(
                    {
                        "bbox": (x, y, w, h),
                        "center": (center_x, center_y),
                        "area": area,
                        "confidence": min(
                            area / 10000.0, 1.0
                        ),  # Simple confidence based on area
                    }
                )

    return detection_results


@profile_function("yolo_detection", "cv")
def yolo_object_detection(image):
    """YOLO detection with profiling"""
    if image is None:
        return []

    result_list = []

    with FunctionProfiler("cv.yolo_inference"):
        # Simulate YOLO model inference
        if model is not None:
            # Actual YOLO inference would happen here
            # For this example, we'll create mock results
            height, width = image.shape[:2]

            # Mock detection results (in practice, this would come from the model)
            mock_detections = [
                {
                    "bbox": [100, 100, 200, 150],
                    "confidence": 0.85,
                    "class_id": 0,
                    "class_name": "gate",
                },
                {
                    "bbox": [300, 200, 100, 120],
                    "confidence": 0.72,
                    "class_id": 1,
                    "class_name": "buoy",
                },
            ]
        else:
            mock_detections = []

    with FunctionProfiler("cv.yolo_postprocess"):
        for detection in mock_detections:
            confidence = detection["confidence"]
            if confidence > 0.5:  # Confidence threshold
                bbox = detection["bbox"]
                x, y, w, h = bbox[0], bbox[1], bbox[2], bbox[3]
                center_x = x + w // 2
                center_y = y + h // 2

                result_list.append(
                    {
                        "bbox": bbox,
                        "center": (center_x, center_y),
                        "confidence": confidence,
                        "class_id": detection["class_id"],
                        "class_name": detection["class_name"],
                    }
                )

    return result_list


@profile_function("depth_calculation", "cv")
def calculate_point_3d(detections, depth_image, camera_intrinsic):
    """3D point calculation with profiling"""
    if not detections or depth_image is None or camera_intrinsic is None:
        return

    # Mock camera intrinsics
    fx, fy = 525.0, 525.0  # focal lengths
    cx, cy = 320.0, 240.0  # principal point

    for detection in detections:
        center_x, center_y = detection["center"]

        # Get depth value at the center point
        if (
            0 <= center_y < depth_image.shape[0]
            and 0 <= center_x < depth_image.shape[1]
        ):
            depth_value = depth_image[center_y, center_x]

            if depth_value > 0:  # Valid depth
                # Convert to 3D coordinates
                z = depth_value / 1000.0  # Convert mm to meters
                x = (center_x - cx) * z / fx
                y = (center_y - cy) * z / fy

                detection["point_3d"] = (x, y, z)
                detection["depth"] = depth_value
            else:
                detection["point_3d"] = None
                detection["depth"] = 0


@profile_function("global_transform", "cv")
def transform_to_global(detections, imu_point, imu_rotation):
    """Global transformation with profiling"""
    if not detections or imu_point is None or imu_rotation is None:
        return

    import math

    # Extract IMU data (mock values if not available)
    pos_x = getattr(imu_point, "x", 0.0)
    pos_y = getattr(imu_point, "y", 0.0)
    pos_z = getattr(imu_point, "z", 0.0)

    # Extract rotation (assuming quaternion or euler angles)
    yaw = getattr(imu_rotation, "z", 0.0)  # Rotation around Z axis
    pitch = getattr(imu_rotation, "y", 0.0)  # Rotation around Y axis
    roll = getattr(imu_rotation, "x", 0.0)  # Rotation around X axis

    # Simple rotation matrix for yaw (assuming mostly level movement)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for detection in detections:
        if "point_3d" in detection and detection["point_3d"] is not None:
            local_x, local_y, local_z = detection["point_3d"]

            # Apply rotation (simplified to yaw only for this example)
            global_x = local_x * cos_yaw - local_y * sin_yaw + pos_x
            global_y = local_x * sin_yaw + local_y * cos_yaw + pos_y
            global_z = local_z + pos_z

            detection["global_position"] = (global_x, global_y, global_z)
        else:
            detection["global_position"] = None


def create_detector_message(detection_results):
    """Create detector message from detection results"""
    from geometry_msgs.msg import Point

    from autonomy.msg import Detection, Detections

    detector_msg = Detections()
    detector_msg.header.stamp = rospy.Time.now()
    detector_msg.header.frame_id = "camera_frame"

    for detection in detection_results:
        det_msg = Detection()

        # Bounding box
        if "bbox" in detection:
            bbox = detection["bbox"]
            det_msg.x = bbox[0] if len(bbox) > 0 else 0
            det_msg.y = bbox[1] if len(bbox) > 1 else 0
            det_msg.width = bbox[2] if len(bbox) > 2 else 0
            det_msg.height = bbox[3] if len(bbox) > 3 else 0

        # 3D point
        if "point_3d" in detection and detection["point_3d"] is not None:
            point_3d = detection["point_3d"]
            det_msg.point.x = point_3d[0]
            det_msg.point.y = point_3d[1]
            det_msg.point.z = point_3d[2]
        else:
            det_msg.point = Point(0, 0, 0)

        # Global position
        if "global_position" in detection and detection["global_position"] is not None:
            global_pos = detection["global_position"]
            det_msg.global_x = global_pos[0]
            det_msg.global_y = global_pos[1]
            det_msg.global_z = global_pos[2]

        # Confidence and class
        det_msg.confidence = detection.get("confidence", 0.0)
        det_msg.cls = detection.get("class_id", 0)

        detector_msg.detections.append(det_msg)

    return detector_msg


# Example profiled detection pipeline
@profile_function("detection_pipeline", "cv")
def run_detection_pipelines():
    """Main detection pipeline with comprehensive profiling"""
    global rgb_image, depth_image, camera_intrinsics, imu_point, imu_rotation, color_filter_config

    detectors_output = []
    pipelines = [color_filter, yolo_object_detection]
    detectors_names = ["color_detector", "yolo_detector"]

    for i, detector_name in enumerate(detectors_names):
        if rgb_image is None:
            rospy.logwarn("RGB image is None. Skipping detection for this iteration.")
            return []

        # Profile each detector separately
        with FunctionProfiler(f"cv.{detector_name}"):
            if detector_name == "color_detector":
                detection_results = pipelines[i](rgb_image, color_filter_config)
            else:
                detection_results = pipelines[i](rgb_image)

        # Profile 3D calculations
        with FunctionProfiler("cv.3d_calculations"):
            if camera_intrinsics is not None and depth_image is not None:
                calculate_point_3d(detection_results, depth_image, camera_intrinsics)
            else:
                rospy.logwarn(
                    "Camera intrinsics or depth_image is None. Skipping 3D calculations."
                )

        # Profile global transformation
        with FunctionProfiler("cv.global_transform"):
            if imu_point is None or imu_rotation is None:
                rospy.logwarn("IMU data is missing. Skipping global transformation.")
            else:
                transform_to_global(
                    detections=detection_results,
                    imu_point=imu_point,
                    imu_rotation=imu_rotation,
                )

        # Profile message creation
        with FunctionProfiler("cv.message_creation"):
            detector_msg = create_detector_message(detection_results)
            detectors_output.append((detector_name, detector_msg))

    return detectors_output


if __name__ == "__main__":
    rospy.init_node("cv_publisher_profiling_example")

    # Set up profiling export (optional)
    import atexit

    def export_on_exit():
        from profiler.profiling_decorators import export_profiling_data

        export_profiling_data("cv_publisher_profile.json")
        print("Profiling data exported to cv_publisher_profile.json")

    atexit.register(export_on_exit)

    # Initialize global variables with mock data
    import numpy as np

    print("🔧 Starting CV Publisher Profiling Example")
    print("📊 Profiling data will be collected and exported on exit")

    # Create mock data for testing
    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)  # Mock RGB image
    depth_image = np.random.randint(
        500, 5000, (480, 640), dtype=np.uint16
    )  # Mock depth

    # Mock camera intrinsics
    camera_intrinsics = {"fx": 525.0, "fy": 525.0, "cx": 320.0, "cy": 240.0}

    # Mock IMU data
    class MockPoint:
        def __init__(self, x=0, y=0, z=0):
            self.x, self.y, self.z = x, y, z

    class MockRotation:
        def __init__(self, x=0, y=0, z=0):
            self.x, self.y, self.z = x, y, z

    imu_point = MockPoint(1.0, 2.0, -0.5)
    imu_rotation = MockRotation(0.1, 0.05, 0.3)

    # Mock color filter config
    color_filter_config = {"enabled": True}

    # Run detection pipeline multiple times to collect profiling data
    for i in range(10):
        print(f"Running detection pipeline iteration {i+1}/10...")

        # Run the profiled detection pipeline
        results = run_detection_pipelines()

        rospy.sleep(0.1)  # Small delay between iterations

    # Print current profiling stats
    stats = ProfileManager.get_instance().get_stats()
    print("\n📈 Profiling Results Preview:")
    print(f"Node: {stats['node_name']}")
    print(f"Functions profiled: {len(stats['functions'])}")

    for func_name, func_stats in stats["functions"].items():
        avg_time = func_stats["avg_time"] * 1000  # Convert to milliseconds
        call_count = func_stats["call_count"]
        print(f"  {func_name}: {avg_time:.2f}ms avg ({call_count} calls)")

    print(
        "\n✅ Example completed. Check cv_publisher_profile.json for detailed results."
    )

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

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from services.profiling_decorators import (
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
model = None  # Your YOLO model

# Example of how to instrument the existing functions:


@profile_ros_callback("rgb_image")
def rgb_image_callback(msg):
    """Process incoming RGB images from ROS with profiling"""
    global rgb_image
    rospy.loginfo("Received RGB image message")

    with FunctionProfiler("cv.image_conversion"):
        try:
            height = msg.height
            width = msg.width
            # ...existing code...
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")


@profile_function("color_filter_detection", "cv")
def color_filter(image, config):
    """Color filter detection with profiling"""
    mask = None  # Placeholder

    with FunctionProfiler("cv.hsv_conversion"):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    with FunctionProfiler("cv.contour_detection"):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # ...rest of existing code...
    return []  # Return empty list as placeholder


@profile_function("yolo_detection", "cv")
def yolo_object_detection(image):
    """YOLO detection with profiling"""
    result_list = []

    with FunctionProfiler("cv.yolo_inference"):
        results = model(image) if model else []

    with FunctionProfiler("cv.yolo_postprocess"):
        for result in results:
            # ...existing processing code...
            pass

    return result_list


@profile_function("depth_calculation", "cv")
def calculate_point_3d(detections, depth_image, camera_intrinsic):
    """3D point calculation with profiling"""
    # ...existing code...
    pass


@profile_function("global_transform", "cv")
def transform_to_global(detections, imu_point, imu_rotation):
    """Global transformation with profiling"""
    # ...existing code...
    pass


def create_detector_message(detection_results):
    """Create detector message - placeholder function"""
    # ...existing code...
    return []


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
            detector_msg_list = create_detector_message(detection_results)
            detectors_output.append((detector_name, detector_msg_list))

    return detectors_output


if __name__ == "__main__":
    rospy.init_node("cv_publisher")

    # Set up profiling export (optional)
    import atexit

    def export_on_exit():
        from services.profiling_decorators import export_profiling_data

        export_profiling_data("cv_publisher_profile.json")

    atexit.register(export_on_exit)

    # ...rest of your existing main code...

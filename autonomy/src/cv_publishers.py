#!/usr/bin/env python3
"""
cv_publishers.py

ROS node for computer vision detection publishing.
Refactored to separate ROS-specific code from core detection functionality.
"""

from typing import List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
from visualization_msgs.msg import Marker, MarkerArray

from autonomy_interfaces.msg import Detection, Detections
from autonomy_interfaces.srv import (
    SetColorFilter,
    SetColorFilterResponse,
    SetYoloModel,
    SetYoloModelResponse,
)

# Import our custom detection core module
from computer_vision.detection_core import ColorFilterConfig, DetectionPipelineManager
from computer_vision import custom_types

############################
# Global variables
############################
rgb_image: Optional[np.ndarray] = None
depth_image: Optional[np.ndarray] = None
imu_point: Optional[custom_types.Point3D] = None
imu_rotation: Optional[custom_types.Rotation3D] = None
camera_intrinsics: Optional[tuple] = None
camera_info = None

# Detection pipeline manager
pipeline_manager = DetectionPipelineManager()

# Camera topic parameters with defaults (will be overridden by ROS params)
rgb_image_topic = "/zed2i/zed_node/rgb/image_rect_color"
depth_image_topic = "/zed2i/zed_node/depth/depth_registered"
camera_info_topic = "/zed2i/zed_node/rgb/camera_info"
camera_pose_topic = "/zed2i/zed_node/pose"


############################
# ROS callbacks
############################
def depth_image_callback(msg):
    global depth_image
    try:
        height = msg.height
        width = msg.width

        # Para imágenes de profundidad en formato 32FC1
        img_array = np.frombuffer(msg.data, dtype=np.float32)
        expected_size = height * width

        if img_array.size != expected_size:
            rospy.logwarn(
                f"Unexpected depth image size: got {img_array.size}, expected {expected_size}"
            )
            return

        depth_image = img_array.reshape((height, width))

    except Exception as e:
        rospy.logerr(f"Error converting depth image: {e}")


def rgb_image_callback(msg):
    global rgb_image
    rospy.loginfo("Received RGB image message")
    try:
        height = msg.height
        width = msg.width

        img_array = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding in ["rgb8", "bgr8"]:
            channels = 3
        elif msg.encoding in ["rgba8", "bgra8"]:
            channels = 4
        elif msg.encoding == "mono8":
            channels = 1
        else:
            rospy.logwarn(f"Unsupported RGB encoding: {msg.encoding}")
            return

        expected_size = height * width * channels
        if img_array.size != expected_size:
            rospy.logwarn(
                f"Unexpected RGB image size: got {img_array.size}, expected {expected_size}"
            )
            return

        img_array = img_array.reshape((height, width, channels))

        # Si hay canal alpha, lo eliminamos
        if channels == 4:
            img_array = img_array[:, :, :3]

        rgb_image = img_array

    except Exception as e:
        rospy.logerr(f"Error converting RGB image: {e}")


def camera_info_callback(msg: CameraInfo):
    global camera_intrinsics
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]
    camera_intrinsics = (fx, fy, cx, cy)


def imu_pose_callback(msg: PoseStamped):
    global imu_point, imu_rotation
    try:
        imu_point = custom_types.Point3D(
            x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z
        )
        imu_rotation = custom_types.Rotation3D(
            x=msg.pose.orientation.x,
            y=msg.pose.orientation.y,
            z=msg.pose.orientation.z,
            w=msg.pose.orientation.w,
        )
        rospy.loginfo("Successfully parsed IMU pose data.")
    except AttributeError as e:
        rospy.logerr(f"AttributeError: {e}. Failed to parse IMU data.")
    except Exception as e:
        rospy.logerr(f"Unexpected error in imu_pose_callback: {e}")


############################
# Detection + Publishing
############################
def create_detector_message(
    detected_object: List[custom_types.Detection],
) -> List[Detection]:
    detections_conversion = []
    for detection in detected_object:
        detector_msg = Detection()
        bbox = RegionOfInterest(
            x_offset=int(detection.x1),
            y_offset=int(detection.y1),
            height=int(detection.y2 - detection.y1),
            width=int(detection.x2 - detection.x1),
        )
        # Handle case where point might be None
        if detection.point is not None:
            point = Point(x=detection.point.x, y=detection.point.y, z=detection.point.z)
        else:
            point = Point(x=0.0, y=0.0, z=0.0)

        detector_msg.cls = detection.cls
        detector_msg.confidence = detection.conf
        detector_msg.point = point
        detector_msg.bounding_box = bbox
        detections_conversion.append(detector_msg)
    return detections_conversion


def run_detection_pipelines() -> List[Tuple[str, List[Detection]]]:
    global rgb_image, depth_image, camera_intrinsics, imu_point, imu_rotation, pipeline_manager

    if rgb_image is None:
        rospy.logwarn("RGB image is None. Skipping detection for this iteration.")
        return []

    # Use the pipeline manager to run detections
    pipelines_results = pipeline_manager.run_detections(
        image=rgb_image,
        depth_image=depth_image,
        camera_intrinsics=camera_intrinsics,
        imu_point=imu_point,
        imu_rotation=imu_rotation,
    )

    # Convert to ROS messages
    detectors_output = []
    for detector_name, detection_results in pipelines_results:
        detector_msg_list = create_detector_message(detection_results)
        detectors_output.append((detector_name, detector_msg_list))

    return detectors_output


def publish_vision_detections():
    detection_pub = rospy.Publisher(
        "/detector/box_detection", Detections, queue_size=10
    )
    marker_pub = rospy.Publisher("/detector/markers", MarkerArray, queue_size=10)
    rate = rospy.Rate(10)

    # Keep a list of previously published markers to manage deletion
    previous_marker_count = {"color_detector": 0, "yolo_detector": 0}

    while not rospy.is_shutdown():
        pipelines_results = run_detection_pipelines()
        marker_array = MarkerArray()

        # Track current marker counts to manage stale markers
        current_marker_count = {"color_detector": 0, "yolo_detector": 0}

        for detector_name, detections in pipelines_results:
            detection_msg = Detections()
            detection_msg.detections = detections
            detection_msg.detector_name = detector_name
            detection_pub.publish(detection_msg)

            # Add markers for each detection
            for i, detection in enumerate(detections):
                # Create point marker
                marker = Marker()
                marker.header.frame_id = "map"  # Use appropriate frame_id
                marker.header.stamp = rospy.Time.now()
                marker.ns = detector_name
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Set position from detection point
                marker.pose.position.x = detection.point.x
                marker.pose.position.y = detection.point.y
                marker.pose.position.z = detection.point.z
                marker.pose.orientation.w = 1.0

                # Scale marker based on confidence (bigger = more confident)
                confidence_scale = 0.1 + (detection.confidence * 0.1)
                marker.scale.x = confidence_scale
                marker.scale.y = confidence_scale
                marker.scale.z = confidence_scale

                # Set color based on detector type
                if detector_name == "color_detector":
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif detector_name == "yolo_detector":
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                marker.color.a = 1.0  # Fully opaque

                # Add marker to array
                marker_array.markers.append(marker)

                # Add text label marker
                text_marker = Marker()
                text_marker.header.frame_id = "map"  # Use appropriate frame_id
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = f"{detector_name}_text"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                # Position text slightly above the sphere
                text_marker.pose.position.x = detection.point.x
                text_marker.pose.position.y = detection.point.y
                text_marker.pose.position.z = (
                    detection.point.z + confidence_scale + 0.05
                )
                text_marker.pose.orientation.w = 1.0

                # Set text size
                text_marker.scale.z = 0.1  # Text height

                # Same color as the sphere
                if detector_name == "color_detector":
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0
                elif detector_name == "yolo_detector":
                    text_marker.color.r = 0.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 0.0
                text_marker.color.a = 1.0

                # Set text content (class ID and confidence)
                text_marker.text = (
                    f"{detector_name}_{i}: {detection.cls}, {detection.confidence:.2f}"
                )
                marker_array.markers.append(text_marker)

                # Update marker count for this detector
                current_marker_count[detector_name] = i + 1

            # Add DELETE markers for any stale markers
            for j in range(
                current_marker_count[detector_name],
                previous_marker_count[detector_name],
            ):
                # Remove stale point markers
                delete_marker = Marker()
                delete_marker.header.frame_id = "map"
                delete_marker.header.stamp = rospy.Time.now()
                delete_marker.ns = detector_name
                delete_marker.id = j
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)

                # Remove stale text markers
                delete_text_marker = Marker()
                delete_text_marker.header.frame_id = "map"
                delete_text_marker.header.stamp = rospy.Time.now()
                delete_text_marker.ns = f"{detector_name}_text"
                delete_text_marker.id = j
                delete_text_marker.action = Marker.DELETE
                marker_array.markers.append(delete_text_marker)

        # Update previous marker counts
        previous_marker_count = current_marker_count.copy()

        # Publish marker array
        marker_pub.publish(marker_array)
        rate.sleep()


############################
# Service handlers
############################
def handle_set_color_filter(req):
    global pipeline_manager

    try:
        # Parse RGB values from the integer array
        if len(req.rgb_range) != 3:
            return SetColorFilterResponse(
                success=False, message="RGB range must have exactly 3 values (R,G,B)"
            )

        rgb_tuple = tuple(req.rgb_range)

        # Validate the RGB values
        for val in rgb_tuple:
            if not 0 <= val <= 255:
                return SetColorFilterResponse(
                    success=False, message="RGB values must be between 0 and 255"
                )

        # Update the color filter configuration
        color_filter_config = ColorFilterConfig(
            tolerance=req.tolerance,
            min_confidence=req.min_confidence,
            min_area=req.min_area,
            rgb_range=rgb_tuple,
        )

        pipeline_manager.update_color_filter_config(color_filter_config)

        rospy.loginfo(
            f"Color filter updated: tolerance={req.tolerance}, min_confidence={req.min_confidence}, "
            f"min_area={req.min_area}, rgb_range={rgb_tuple}"
        )

        return SetColorFilterResponse(
            success=True, message="Color filter config updated successfully"
        )

    except Exception as e:
        error_msg = f"Failed to update color filter config: {str(e)}"
        rospy.logerr(error_msg)
        return SetColorFilterResponse(success=False, message=error_msg)


def handle_set_yolo_model(req):
    global pipeline_manager

    try:
        # Validate the model name
        if not req.model_name.endswith(".pt"):
            return SetYoloModelResponse(
                success=False, message="Model name must end with .pt"
            )

        # Use pipeline manager to load the model
        success = pipeline_manager.load_yolo_model(req.model_name)

        if success:
            rospy.loginfo(f"YOLO model switched to: {req.model_name}")
            return SetYoloModelResponse(
                success=True, message=f"YOLO model switched to: {req.model_name}"
            )
        else:
            return SetYoloModelResponse(
                success=False, message=f"Failed to load model: {req.model_name}"
            )

    except Exception as e:
        error_msg = f"Failed to switch YOLO model: {str(e)}"
        rospy.logerr(error_msg)
        return SetYoloModelResponse(success=False, message=error_msg)


############################
# Initialization functions
############################
def initialize_service_servers():
    rospy.loginfo("Initializing service servers...")
    rospy.Service("/detector/set_color_filter", SetColorFilter, handle_set_color_filter)
    rospy.Service("/detector/set_yolo_model", SetYoloModel, handle_set_yolo_model)
    rospy.loginfo("Service servers are ready")


def initialize_subscribers():
    global rgb_image_topic, depth_image_topic, camera_info_topic, camera_pose_topic

    # Get topic names from ROS parameters (with defaults)
    rgb_image_topic = rospy.get_param("~rgb_image_topic", rgb_image_topic)
    depth_image_topic = rospy.get_param("~depth_image_topic", depth_image_topic)
    camera_info_topic = rospy.get_param("~camera_info_topic", camera_info_topic)
    camera_pose_topic = rospy.get_param("~camera_pose_topic", camera_pose_topic)

    # Log the topic names being used
    rospy.loginfo(f"Using RGB image topic: {rgb_image_topic}")
    rospy.loginfo(f"Using depth image topic: {depth_image_topic}")
    rospy.loginfo(f"Using camera info topic: {camera_info_topic}")
    rospy.loginfo(f"Using camera pose topic: {camera_pose_topic}")

    # Subscribe to the topics
    rospy.Subscriber(rgb_image_topic, Image, rgb_image_callback)
    rospy.Subscriber(depth_image_topic, Image, depth_image_callback)
    rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)
    rospy.Subscriber(camera_pose_topic, PoseStamped, imu_pose_callback)


if __name__ == "__main__":
    rospy.init_node("cv_publisher")
    initialize_subscribers()
    initialize_service_servers()
    rospy.sleep(3)  # Give some time to gather messages
    publish_vision_detections()
    rospy.spin()

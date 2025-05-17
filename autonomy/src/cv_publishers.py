#!/usr/bin/env python3
"""
cv_publishers_custom_bridge.py

An example script replacing cv_bridge with custom functions for
converting ROS sensor_msgs/Image to/from OpenCV numpy arrays.
Works for Python 3 with ROS Melodic.
"""

import cv2
import numpy as np
import os
from typing import List, Tuple
from ultralytics import YOLO
from dataclasses import dataclass

# Custom user modules
import custom_types

# ROS dependencies
import rospy
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from autonomy.msg import Detection, Detections
from autonomy.srv import SetColorFilter, SetColorFilterResponse
from autonomy.srv import SetYoloModel, SetYoloModelResponse

# Add constants for YOLO model directory
YOLO_MODEL_DIR = "/yolo_models"
DEFAULT_YOLO_MODEL = "yolov8n.pt"

############################
# Custom cv_bridge replacementss
############################

# Convert ROS Image to OpenCV (numpy array)
def ros_img_to_cv2(msg: Image, encoding="bgr8") -> np.ndarray:
    """
    Convert a ROS sensor_msgs/Image message to an OpenCV numpy array.
    :param msg: ROS Image message
    :param encoding: Desired encoding ("bgr8", "mono8", "mono16", "32FC1")
    :return: OpenCV numpy array
    """
    if encoding not in ["bgr8", "mono8", "mono16", "32FC1"]:
        raise ValueError(f"Unsupported encoding: {encoding}")

    dtype_map = {
        "bgr8": np.uint8,
        "mono8": np.uint8,
        "mono16": np.uint16,
        "32FC1": np.float32
    }

    dtype = dtype_map[encoding]

    # Calculate expected array size based on encoding and dimensions
    channels = 3 if encoding == "bgr8" else 1
    expected_size = msg.height * msg.width * channels
    actual_size = len(msg.data)
    
    # Debug information about the image dimensions
    rospy.logdebug(f"Image dimensions: {msg.height}x{msg.width}, channels: {channels}")
    rospy.logdebug(f"Expected data size: {expected_size}, actual data size: {actual_size}")
    
    if actual_size != expected_size:
        rospy.logwarn(f"Data size mismatch! Expected {expected_size}, got {actual_size}. Attempting to fix...")
        # Try to infer correct dimensions based on actual data size
        if encoding == "bgr8" and actual_size % 3 == 0:
            total_pixels = actual_size // 3
            # Try to determine if width is correct but height is wrong
            if total_pixels % msg.width == 0:
                corrected_height = total_pixels // msg.width
                rospy.logwarn(f"Correcting height from {msg.height} to {corrected_height}")
                img_array = np.frombuffer(msg.data, dtype=dtype).reshape((corrected_height, msg.width, 3))
                return img_array
    
    # Convert the byte data to a NumPy array
    img_array = np.frombuffer(msg.data, dtype=dtype)

    try:
        # Reshape based on image dimensions and encoding
        if encoding == "bgr8":
            # Use step value if available for proper alignment
            if msg.step > 0 and msg.step >= msg.width * 3:
                img_array = img_array.reshape((msg.height, msg.width, 3))
            else:
                img_array = np.reshape(img_array, (msg.height, msg.width, 3))
        else:
            # Single-channel
            img_array = np.reshape(img_array, (msg.height, msg.width))
        
        return img_array
    
    except Exception as e:
        # More detailed error information
        rospy.logerr(f"Reshape failed: {e}")
        rospy.logerr(f"Image info: height={msg.height}, width={msg.width}, step={msg.step}, encoding={msg.encoding}")
        rospy.logerr(f"Data length: {len(msg.data)}, array shape before reshape: {img_array.shape}")
        raise


# Optionally, if you need to convert back to ROS Images:
def cv2_to_ros_img(cv_image: np.ndarray, encoding="bgr8") -> Image:
    """
    Convert an OpenCV numpy array to a ROS sensor_msgs/Image message.
    :param cv_image: OpenCV image (numpy array)
    :param encoding: Desired encoding ("bgr8", "mono8", "mono16", "32FC1")
    :return: ROS Image message
    """
    ros_msg = Image()
    ros_msg.height, ros_msg.width = cv_image.shape[:2]
    ros_msg.encoding = encoding
    ros_msg.step = cv_image.strides[0]
    ros_msg.data = cv_image.tobytes()
    return ros_msg

############################
# YOLO model
############################
model = YOLO("yolo11n.pt")

############################
# Data classes
############################
@dataclass
class ColorFilterConfig:
    tolerance: float
    min_confidence: float
    min_area: float
    rgb_range: Tuple[int, int, int]

############################
# Global variables
############################
rgb_image: np.ndarray = None
depth_image: np.ndarray = None
imu_point: custom_types.Point3D = None
imu_rotation: custom_types.Rotation3D = None
camera_intrinsics: tuple = None
camera_info = None
color_filter_config: ColorFilterConfig = ColorFilterConfig(
    tolerance=0.4,
    min_confidence=0.3,
    min_area=0.2,
    rgb_range=(255, 0, 0)
)

# Camera topic parameters with defaults (will be overridden by ROS params)
rgb_image_topic = "/zed2i/zed_node/rgb/image_rect_color"
depth_image_topic = "/zed2i/zed_node/depth/depth_registered"
camera_info_topic = "/zed2i/zed_node/rgb/camera_info"
camera_pose_topic = "/zed2i/zed_node/pose"

############################
# Color filter detection
############################
def color_filter(
    image: np.ndarray,
    config: ColorFilterConfig = ColorFilterConfig(
        tolerance=0.4, 
        min_confidence=0.3, 
        min_area=0.2, 
        rgb_range=(255,0,0)
    )
) -> List[custom_types.Detection]:
    assert 0 < config.min_confidence < 1.0, "min_confidence must be between 0 and 1"

    result = []
    rospy.loginfo(f"Color Filter: image shape = {image.shape}")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Convert the RGB color to HSV for filtering
    target_hsv = cv2.cvtColor(
        np.uint8([[config.rgb_range]]), cv2.COLOR_RGB2HSV
    )[0][0]

    lower_bound = np.array([
        max(0, target_hsv[0] - config.tolerance * 180),
        100,
        100
    ], dtype=np.uint8)
    upper_bound = np.array([
        min(180, target_hsv[0] + config.tolerance * 180),
        255,
        255
    ], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > config.min_area:
            x, y, w, h = cv2.boundingRect(contour)
            contour_mask = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

            # Calculate the confidence based on color presence within the contour
            red_pixels_in_contour = cv2.bitwise_and(mask, mask, mask=contour_mask)
            red_pixel_count = np.sum(red_pixels_in_contour == 255)
            total_pixels_in_contour = w * h

            if total_pixels_in_contour > 0:
                confidence = red_pixel_count / total_pixels_in_contour
                if confidence > config.min_confidence:
                    # (x1, y1, x2, y2, cls, conf)
                    result.append(
                        custom_types.Detection(
                            x1=x,
                            y1=y,
                            x2=x + w,
                            y2=y + h,
                            cls=0,
                            conf=confidence
                        )
                    )

    return result

############################
# YOLO detection
############################
def yolo_object_detection(image: np.ndarray) -> List[custom_types.Detection]:
    result_list = []
    results = model(image)

    for result in results:
        if hasattr(result, 'boxes'):
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                conf = float(box.conf.cpu().numpy()[0])
                cls_w = int(box.cls.cpu().numpy()[0])
                # (x1, y1, x2, y2, cls, conf)
                result_list.append(
                    custom_types.Detection(x1, y1, x2, y2, cls_w, conf, 0, None)
                )

    return result_list

############################
# Depth + 3D calculations
############################
def calculate_point_3d(
    detections: List[custom_types.Detection],
    depth_image: np.ndarray,
    camera_intrinsic: tuple
):
    # camera_intrinsic = (fx, fy, cx, cy)
    for detection in detections:
        x_min, y_min, x_max, y_max = detection.x1, detection.y1, detection.x2, detection.y2
        if depth_image is not None:
            x_min_int = int(x_min)
            x_max_int = int(x_max)
            y_min_int = int(y_min)
            y_max_int = int(y_max)

            # Extract the depth values within the bounding box
            bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
            if bbox_depth.size > 0:
                mean_depth = np.nanmean(bbox_depth)
                if not np.isnan(mean_depth):
                    fx, fy, cx, cy = camera_intrinsic
                    z = mean_depth
                    detection.depth = z

                    x_center = (x_min + x_max) / 2.0
                    y_center = (y_min + y_max) / 2.0
                    x = (x_center - cx) * z / fx
                    y = (y_center - cy) * z / fy

                    detection.point = custom_types.Point3D(x=x, y=y, z=z)
                    rospy.loginfo(f"Detection 3D point added: ({x}, {y}, {z})")
                else:
                    detection.point = custom_types.Point3D(x=0, y=0, z=0)
                    detection.depth = 0
            else:
                detection.point = custom_types.Point3D(x=0, y=0, z=0)
                detection.depth = 0

def transform_to_global(
    detections: List[custom_types.Detection],
    imu_point: custom_types.Point3D,
    imu_rotation: custom_types.Rotation3D
):
    def quaternion_to_matrix(rotation: custom_types.Rotation3D):
        # w, x, y, z
        w, x, y, z = rotation.w, rotation.x, rotation.y, rotation.z
        rotation_matrix = np.array([
            [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
        ])

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        return transform_matrix

    transform_matrix = quaternion_to_matrix(imu_rotation)
    transform_matrix[0:3, 3] = [imu_point.x, imu_point.y, imu_point.z]

    for detection in detections:
        point_homogeneous = np.array([
            detection.point.x,
            detection.point.y,
            detection.point.z,
            1.0
        ])
        point_global = np.dot(transform_matrix, point_homogeneous)
        detection.point = custom_types.Point3D(
            x=point_global[0],
            y=point_global[1],
            z=point_global[2]
        )

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
            rospy.logwarn(f"Unexpected depth image size: got {img_array.size}, expected {expected_size}")
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
            rospy.logwarn(f"Unexpected RGB image size: got {img_array.size}, expected {expected_size}")
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
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
        imu_rotation = custom_types.Rotation3D(
            x=msg.pose.orientation.x,
            y=msg.pose.orientation.y,
            z=msg.pose.orientation.z,
            w=msg.pose.orientation.w
        )
        rospy.loginfo("Successfully parsed IMU pose data.")
    except AttributeError as e:
        rospy.logerr(f"AttributeError: {e}. Failed to parse IMU data.")
    except Exception as e:
        rospy.logerr(f"Unexpected error in imu_pose_callback: {e}")

############################
# Detection + Publishing
############################
def create_detector_message(detected_object: List[custom_types.Detection]) -> List[Detection]:
    detections_conversion = []
    for detection in detected_object:
        detector_msg = Detection()
        bbox = RegionOfInterest(
            x_offset=int(detection.x1),
            y_offset=int(detection.y1),
            height=int(detection.y2 - detection.y1),
            width=int(detection.x2 - detection.x1)
        )
        point = Point(
            x=detection.point.x,
            y=detection.point.y,
            z=detection.point.z
        )

        detector_msg.cls = detection.cls
        detector_msg.confidence = detection.conf
        detector_msg.point = point
        detector_msg.bounding_box = bbox
        detections_conversion.append(detector_msg)
    return detections_conversion

def run_detection_pipelines() -> List[Tuple[str, List[Detection]]]:
    global rgb_image, depth_image, camera_intrinsics, imu_point, imu_rotation, color_filter_config
    detectors_output = []
    pipelines = [color_filter, yolo_object_detection]
    detectors_names = ['color_detector', 'yolo_detector']

    for i, detector_name in enumerate(detectors_names):
        if rgb_image is None:
            rospy.logwarn("RGB image is None. Skipping detection for this iteration.")
            return []

        if detector_name == 'color_detector':
            detection_results = pipelines[i](rgb_image, color_filter_config)
        else:
            detection_results = pipelines[i](rgb_image)

        # Calculate 3D points
        if camera_intrinsics is not None and depth_image is not None:
            calculate_point_3d(detection_results, depth_image, camera_intrinsics)
        else:
            rospy.logwarn("Camera intrinsics or depth_image is None. Skipping 3D calculations.")

        # Transform to global if IMU data is available
        if imu_point is None or imu_rotation is None:
            rospy.logwarn("IMU data is missing. Skipping global transformation.")
        else:
            transform_to_global(detections=detection_results, imu_point=imu_point, imu_rotation=imu_rotation)

        # Create the message list
        detector_msg_list = create_detector_message(detection_results)
        detectors_output.append((detector_name, detector_msg_list))

    return detectors_output

def publish_vision_detections():
    detection_pub = rospy.Publisher('/detector/box_detection', Detections, queue_size=10)
    marker_pub = rospy.Publisher('/detector/markers', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)
    
    # Keep a list of previously published markers to manage deletion
    previous_marker_count = {'color_detector': 0, 'yolo_detector': 0}
    
    while not rospy.is_shutdown():
        pipelines_results = run_detection_pipelines()
        marker_array = MarkerArray()
        
        # Track current marker counts to manage stale markers
        current_marker_count = {'color_detector': 0, 'yolo_detector': 0}
        
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
                if detector_name == 'color_detector':
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif detector_name == 'yolo_detector':
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
                text_marker.pose.position.z = detection.point.z + confidence_scale + 0.05
                text_marker.pose.orientation.w = 1.0
                
                # Set text size
                text_marker.scale.z = 0.1  # Text height
                
                # Same color as the sphere
                if detector_name == 'color_detector':
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0
                elif detector_name == 'yolo_detector':
                    text_marker.color.r = 0.0
                    text_marker.color.g = 1.0
                    text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                
                # Set text content (class ID and confidence)
                text_marker.text = f"{detector_name}_{i}: {detection.cls}, {detection.confidence:.2f}"
                marker_array.markers.append(text_marker)
                
                # Update marker count for this detector
                current_marker_count[detector_name] = i + 1
            
            # Add DELETE markers for any stale markers
            for j in range(current_marker_count[detector_name], previous_marker_count[detector_name]):
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

def handle_set_color_filter(req):
    global color_filter_config
    
    try:
        # Parse RGB values from the integer array
        if len(req.rgb_range) != 3:
            return SetColorFilterResponse(success=False, message="RGB range must have exactly 3 values (R,G,B)")
        
        rgb_tuple = tuple(req.rgb_range)
        
        # Validate the RGB values
        for val in rgb_tuple:
            if not 0 <= val <= 255:
                return SetColorFilterResponse(success=False, message="RGB values must be between 0 and 255")
        
        # Update the color filter configuration
        color_filter_config = ColorFilterConfig(
            tolerance=req.tolerance,
            min_confidence=req.min_confidence,
            min_area=req.min_area,
            rgb_range=rgb_tuple
        )
        
        rospy.loginfo(f"Color filter updated: tolerance={req.tolerance}, min_confidence={req.min_confidence}, " 
                      f"min_area={req.min_area}, rgb_range={rgb_tuple}")
        
        return SetColorFilterResponse(success=True, message="Color filter config updated successfully")
    
    except Exception as e:
        error_msg = f"Failed to update color filter config: {str(e)}"
        rospy.logerr(error_msg)
        return SetColorFilterResponse(success=False, message=error_msg)

def handle_set_yolo_model(req):
    global model
    
    try:
        # Validate the model name
        if not req.model_name.endswith('.pt'):
            return SetYoloModelResponse(success=False, message="Model name must end with .pt")
        
        model_path = os.path.join(YOLO_MODEL_DIR, req.model_name)
        
        # Check if the model file exists
        if not os.path.isfile(model_path):
            return SetYoloModelResponse(success=False, message=f"Model file not found: {model_path}")
        
        # Load the new model
        model = YOLO(model_path)
        
        rospy.loginfo(f"YOLO model switched to: {req.model_name}")
        return SetYoloModelResponse(success=True, message=f"YOLO model switched to: {req.model_name}")
    
    except Exception as e:
        error_msg = f"Failed to switch YOLO model: {str(e)}"
        rospy.logerr(error_msg)
        return SetYoloModelResponse(success=False, message=error_msg)

def initialize_service_servers():
    rospy.loginfo("Initializing service servers...")
    rospy.Service('/detector/set_color_filter', SetColorFilter, handle_set_color_filter)
    rospy.Service('/detector/set_yolo_model', SetYoloModel, handle_set_yolo_model)
    rospy.loginfo("Service servers are ready")

def initialize_subscribers():
    global rgb_image_topic, depth_image_topic, camera_info_topic, camera_pose_topic
    
    # Get topic names from ROS parameters (with defaults)
    rgb_image_topic = rospy.get_param('~rgb_image_topic', rgb_image_topic)
    depth_image_topic = rospy.get_param('~depth_image_topic', depth_image_topic)
    camera_info_topic = rospy.get_param('~camera_info_topic', camera_info_topic)
    camera_pose_topic = rospy.get_param('~camera_pose_topic', camera_pose_topic)
    
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
    rospy.init_node('cv_publisher')
    initialize_subscribers()
    initialize_service_servers()
    rospy.sleep(3)  # Give some time to gather messages
    publish_vision_detections()
    rospy.spin()

#!/usr/bin/env python3
"""
cv_publishers_custom_bridge.py

An example script replacing cv_bridge with custom functions for
converting ROS sensor_msgs/Image to/from OpenCV numpy arrays.
Works for Python 3 with ROS Melodic.
"""

import cv2
import numpy as np
from typing import List, Tuple
from ultralytics import YOLO
from dataclasses import dataclass

# Custom user modules
import custom_types

# ROS dependencies
import rospy
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import Point, PoseStamped
from autonomy.msg import Detection, Detections
from autonomy.srv import SetColorFilterResponse

############################
# Custom cv_bridge replacement
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

    # Convert the byte data to a NumPy array
    img_array = np.frombuffer(msg.data, dtype=dtype)

    # Reshape based on image dimensions and encoding
    if encoding == "bgr8":
        # 3-channel BGR
        img_array = img_array.reshape((msg.height, msg.width, 3))
    else:
        # Single-channel
        img_array = img_array.reshape((msg.height, msg.width))

    return img_array


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
def depth_image_callback(msg: Image):
    global depth_image
    try:
        depth_image = ros_img_to_cv2(msg, "32FC1")
    except Exception as e:
        rospy.logerr(f"Image Conversion Error (Depth): {e}")

def rgb_image_callback(msg: Image):
    global rgb_image
    rospy.loginfo("Received RGB image message")
    try:
        rgb_image = ros_img_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"Image Conversion Error (RGB): {e}")

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
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pipelines_results = run_detection_pipelines()
        for detector_name, detections in pipelines_results:
            detection_msg = Detections()
            detection_msg.detections = detections
            detection_msg.detector_name = detector_name
            detection_pub.publish(detection_msg)

        rate.sleep()

def handle_set_color_filter(req):
    global color_filter_config
    color_filter_config = ColorFilterConfig(
        tolerance=req.tolerance,
        min_confidence=req.min_confidence,
        min_area=req.min_area,
        rgb_range=tuple(req.rgb_range)
    )
    return SetColorFilterResponse(success=True, message="Color filter config updated.")

def initialize_subscribers():
    rospy.loginfo("Initializing subscribers...")
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, rgb_image_callback)
    rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, depth_image_callback)
    rospy.Subscriber("/zed2i/zed_node/rgb/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/zed2i/zed_node/pose", PoseStamped, imu_pose_callback)

if __name__ == "__main__":
    rospy.init_node('cv_publihser')
    initialize_subscribers()
    rospy.sleep(3)  # Give some time to gather messages
    publish_vision_detections()
    rospy.spin()

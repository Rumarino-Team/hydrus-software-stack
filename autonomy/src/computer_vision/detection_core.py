#!/usr/bin/env python3
"""
detection_core.py

Core computer vision detection functionality separated from ROS-specific code.
Contains YOLO detection, color filtering, 3D calculations, and image conversion utilities.
"""

import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

import custom_types
import cv2
import numpy as np
from ultralytics import YOLO

############################
# Constants
############################
# YOLO model directory constant
YOLO_MODEL_DIR = "/yolo_models"


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
# Custom cv_bridge replacements
############################
def ros_img_to_cv2(msg, encoding="bgr8") -> np.ndarray:
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
        "32FC1": np.float32,
    }

    dtype = dtype_map[encoding]

    # Calculate expected array size based on encoding and dimensions
    channels = 3 if encoding == "bgr8" else 1
    expected_size = msg.height * msg.width * channels
    actual_size = len(msg.data)

    if actual_size != expected_size:
        # Try to infer correct dimensions based on actual data size
        if encoding == "bgr8" and actual_size % 3 == 0:
            total_pixels = actual_size // 3
            # Try to determine if width is correct but height is wrong
            if total_pixels % msg.width == 0:
                corrected_height = total_pixels // msg.width
                img_array = np.frombuffer(msg.data, dtype=dtype).reshape(
                    (corrected_height, msg.width, 3)
                )
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
        raise ValueError(
            f"Reshape failed: {e}. Image info: height={msg.height}, width={msg.width}, step={msg.step}, encoding={msg.encoding}"
        )


def cv2_to_ros_img(cv_image: np.ndarray, encoding="bgr8"):
    """
    Convert an OpenCV numpy array to a ROS sensor_msgs/Image message.
    :param cv_image: OpenCV image (numpy array)
    :param encoding: Desired encoding ("bgr8", "mono8", "mono16", "32FC1")
    :return: ROS Image message
    """
    from sensor_msgs.msg import Image

    ros_msg = Image()
    ros_msg.height, ros_msg.width = cv_image.shape[:2]
    ros_msg.encoding = encoding
    ros_msg.step = cv_image.strides[0]
    ros_msg.data = cv_image.tobytes()
    return ros_msg


############################
# YOLO Model Manager
############################
class YOLOModelManager:
    def __init__(self, default_model="yolo11n.pt"):
        self.model = YOLO(default_model)

    def load_model(self, model_name: str) -> bool:
        """
        Load a new YOLO model.
        :param model_name: Name of the model file
        :return: True if successful, False otherwise
        """
        try:
            if not model_name.endswith(".pt"):
                raise ValueError("Model name must end with .pt")

            model_path = os.path.join(YOLO_MODEL_DIR, model_name)

            if not os.path.isfile(model_path):
                raise FileNotFoundError(f"Model file not found: {model_path}")

            self.model = YOLO(model_path)
            return True

        except Exception as e:
            raise RuntimeError(f"Failed to load YOLO model: {str(e)}")

    def detect(self, image: np.ndarray) -> List[custom_types.Detection]:
        """
        Run YOLO object detection on an image.
        :param image: Input image as numpy array
        :return: List of Detection objects
        """
        result_list = []
        results = self.model(image)

        for result in results:
            if hasattr(result, "boxes"):
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy.cpu().numpy()[0]
                    conf = float(box.conf.cpu().numpy()[0])
                    cls_w = int(box.cls.cpu().numpy()[0])
                    result_list.append(
                        custom_types.Detection(x1, y1, x2, y2, cls_w, conf, 0, None)
                    )

        return result_list


############################
# Color filter detection
############################
def color_filter_detection(
    image: np.ndarray,
    config: ColorFilterConfig = ColorFilterConfig(
        tolerance=0.4, min_confidence=0.3, min_area=0.2, rgb_range=(255, 0, 0)
    ),
) -> List[custom_types.Detection]:
    """
    Perform color-based object detection.
    :param image: Input image as numpy array
    :param config: ColorFilterConfig with detection parameters
    :return: List of Detection objects
    """
    assert 0 < config.min_confidence < 1.0, "min_confidence must be between 0 and 1"

    result = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # Convert the RGB color to HSV for filtering
    target_rgb = np.array([[config.rgb_range]], dtype=np.uint8)
    target_hsv = cv2.cvtColor(target_rgb, cv2.COLOR_RGB2HSV)[0][0]

    lower_bound = np.array(
        [max(0, target_hsv[0] - config.tolerance * 180), 100, 100], dtype=np.uint8
    )
    upper_bound = np.array(
        [min(180, target_hsv[0] + config.tolerance * 180), 255, 255], dtype=np.uint8
    )

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
                    result.append(
                        custom_types.Detection(
                            x1=x, y1=y, x2=x + w, y2=y + h, cls=0, conf=confidence
                        )
                    )

    return result


############################
# 3D calculations and transformations
############################
def calculate_3d_bounding_boxes(
    detections: List[custom_types.Detection],
    depth_image: np.ndarray,
    camera_intrinsic: tuple,
):
    """
    Calculate 3D bounding boxes for detections using depth information.
    This extends the existing point calculation to provide full 3D bounding boxes.

    :param detections: List of Detection objects to update
    :param depth_image: Depth image as numpy array
    :param camera_intrinsic: Camera intrinsic parameters (fx, fy, cx, cy)
    """
    for detection in detections:
        x_min, y_min, x_max, y_max = (
            detection.x1,
            detection.y1,
            detection.x2,
            detection.y2,
        )

        if depth_image is not None:
            x_min_int = int(x_min)
            x_max_int = int(x_max)
            y_min_int = int(y_min)
            y_max_int = int(y_max)

            # Extract the depth values within the bounding box
            bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]

            if bbox_depth.size > 0:
                # Filter out invalid depth values (NaN and zeros)
                valid_depths = bbox_depth[~np.isnan(bbox_depth) & (bbox_depth > 0)]

                if len(valid_depths) > 0:
                    fx, fy, cx, cy = camera_intrinsic

                    # Calculate center point in 3D (same as before for backward compatibility)
                    mean_depth = float(np.mean(valid_depths))
                    detection.depth = mean_depth

                    x_center = (x_min + x_max) / 2.0
                    y_center = (y_min + y_max) / 2.0
                    x_3d = (x_center - cx) * mean_depth / fx
                    y_3d = (y_center - cy) * mean_depth / fy
                    z_3d = mean_depth

                    detection.point = custom_types.Point3D(x=x_3d, y=y_3d, z=z_3d)

                    # Calculate 3D bounding box dimensions
                    # Project 2D bounding box corners to 3D space using depth information

                    # Sample depth at different points in the bounding box for better estimation
                    depth_samples = []
                    sample_points = [
                        (x_min_int, y_min_int),  # top-left
                        (x_max_int, y_min_int),  # top-right
                        (x_min_int, y_max_int),  # bottom-left
                        (x_max_int, y_max_int),  # bottom-right
                        (int(x_center), int(y_center)),  # center
                    ]

                    for px, py in sample_points:
                        # Ensure we don't go out of bounds
                        px = max(0, min(px, depth_image.shape[1] - 1))
                        py = max(0, min(py, depth_image.shape[0] - 1))
                        depth_val = depth_image[py, px]
                        if not np.isnan(depth_val) and depth_val > 0:
                            depth_samples.append(depth_val)

                    # Use range of depths to estimate 3D box depth, or use mean if not enough samples
                    if len(depth_samples) >= 2:
                        min_depth = float(np.min(depth_samples))
                        max_depth = float(np.max(depth_samples))
                        depth_3d = max_depth - min_depth
                        # Use a minimum depth to avoid unrealistically thin objects
                        depth_3d = max(depth_3d, 0.1)
                    else:
                        # Fallback: estimate depth as a fraction of the mean depth
                        depth_3d = (
                            mean_depth * 0.1
                        )  # Assume object is 10% of its distance in depth

                    # Calculate 3D width and height by projecting 2D box corners
                    # Project corners to 3D space
                    x1_3d = (x_min - cx) * mean_depth / fx
                    y1_3d = (y_min - cy) * mean_depth / fy
                    x2_3d = (x_max - cx) * mean_depth / fx
                    y2_3d = (y_max - cy) * mean_depth / fy

                    width_3d = abs(x2_3d - x1_3d)
                    height_3d = abs(y2_3d - y1_3d)

                    # Create the 3D bounding box
                    center_3d = custom_types.Point3D(x=x_3d, y=y_3d, z=z_3d)
                    detection.bbox_3d = custom_types.BoundingBox3D(
                        center=center_3d,
                        width=width_3d,
                        height=height_3d,
                        depth=depth_3d,
                    )
                else:
                    # No valid depth data
                    detection.point = custom_types.Point3D(x=0, y=0, z=0)
                    detection.depth = 0
                    detection.bbox_3d = None
            else:
                # Empty bounding box region
                detection.point = custom_types.Point3D(x=0, y=0, z=0)
                detection.depth = 0
                detection.bbox_3d = None


def calculate_point_3d(
    detections: List[custom_types.Detection],
    depth_image: np.ndarray,
    camera_intrinsic: tuple,
):
    """
    Calculate 3D points for detections using depth information.
    This function is kept for backward compatibility and now also calculates 3D bounding boxes.
    :param detections: List of Detection objects to update
    :param depth_image: Depth image as numpy array
    :param camera_intrinsic: Camera intrinsic parameters (fx, fy, cx, cy)
    """
    # Call the new 3D bounding box function which also calculates the point
    calculate_3d_bounding_boxes(detections, depth_image, camera_intrinsic)


def quaternion_to_transform_matrix(rotation: custom_types.Rotation3D) -> np.ndarray:
    """
    Convert quaternion rotation to 4x4 transformation matrix.
    :param rotation: Rotation3D object with quaternion components
    :return: 4x4 transformation matrix
    """
    w, x, y, z = rotation.w, rotation.x, rotation.y, rotation.z
    rotation_matrix = np.array(
        [
            [
                1 - 2 * y**2 - 2 * z**2,
                2 * x * y - 2 * z * w,
                2 * x * z + 2 * y * w,
            ],
            [
                2 * x * y + 2 * z * w,
                1 - 2 * x**2 - 2 * z**2,
                2 * y * z - 2 * x * w,
            ],
            [
                2 * x * z - 2 * y * w,
                2 * y * z + 2 * x * w,
                1 - 2 * x**2 - 2 * y**2,
            ],
        ]
    )

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    return transform_matrix


def transform_to_global(
    detections: List[custom_types.Detection],
    imu_point: custom_types.Point3D,
    imu_rotation: custom_types.Rotation3D,
):
    """
    Transform detection points and 3D bounding boxes from camera frame to global frame.
    :param detections: List of Detection objects to transform
    :param imu_point: IMU position in global frame
    :param imu_rotation: IMU orientation as quaternion
    """
    transform_matrix = quaternion_to_transform_matrix(imu_rotation)
    transform_matrix[0:3, 3] = [imu_point.x, imu_point.y, imu_point.z]

    for detection in detections:
        # Transform the centroid point
        if detection.point is not None:
            point_homogeneous = np.array(
                [detection.point.x, detection.point.y, detection.point.z, 1.0]
            )
            point_global = np.dot(transform_matrix, point_homogeneous)
            detection.point = custom_types.Point3D(
                x=point_global[0], y=point_global[1], z=point_global[2]
            )

        # Transform the 3D bounding box center
        if detection.bbox_3d is not None:
            center_homogeneous = np.array(
                [
                    detection.bbox_3d.center.x,
                    detection.bbox_3d.center.y,
                    detection.bbox_3d.center.z,
                    1.0,
                ]
            )
            center_global = np.dot(transform_matrix, center_homogeneous)

            # Create new 3D bounding box with transformed center
            # Note: dimensions remain the same, only center is transformed
            # In a more advanced implementation, we might also transform the orientation
            detection.bbox_3d = custom_types.BoundingBox3D(
                center=custom_types.Point3D(
                    x=center_global[0], y=center_global[1], z=center_global[2]
                ),
                width=detection.bbox_3d.width,
                height=detection.bbox_3d.height,
                depth=detection.bbox_3d.depth,
            )


############################
# Detection Pipeline Manager
############################
class DetectionPipelineManager:
    """
    Manages multiple detection pipelines and their execution.
    """

    def __init__(self):
        self.yolo_manager = YOLOModelManager()
        self.color_filter_config = ColorFilterConfig(
            tolerance=0.4, min_confidence=0.3, min_area=0.2, rgb_range=(255, 0, 0)
        )

    def update_color_filter_config(self, config: ColorFilterConfig):
        """Update the color filter configuration."""
        self.color_filter_config = config

    def load_yolo_model(self, model_name: str) -> bool:
        """Load a new YOLO model."""
        return self.yolo_manager.load_model(model_name)

    def run_detections(
        self,
        image: np.ndarray,
        depth_image: Optional[np.ndarray] = None,
        camera_intrinsics: Optional[tuple] = None,
        imu_point: Optional[custom_types.Point3D] = None,
        imu_rotation: Optional[custom_types.Rotation3D] = None,
    ) -> List[Tuple[str, List[custom_types.Detection]]]:
        """
        Run all detection pipelines on the given image.
        :param image: Input RGB image
        :param depth_image: Optional depth image for 3D calculations
        :param camera_intrinsics: Camera intrinsic parameters (fx, fy, cx, cy)
        :param imu_point: IMU position for global transformation
        :param imu_rotation: IMU orientation for global transformation
        :return: List of tuples (detector_name, detections)
        """
        if image is None:
            return []

        pipelines_results = []

        # Run color filter detection
        color_detections = color_filter_detection(image, self.color_filter_config)
        pipelines_results.append(("color_detector", color_detections))

        # Run YOLO detection
        yolo_detections = self.yolo_manager.detect(image)
        pipelines_results.append(("yolo_detector", yolo_detections))

        # Process all detections for 3D calculations and transformations
        for detector_name, detections in pipelines_results:
            # Calculate 3D points and bounding boxes if depth and camera info available
            if camera_intrinsics is not None and depth_image is not None:
                calculate_3d_bounding_boxes(detections, depth_image, camera_intrinsics)

            # Transform to global frame if IMU data available
            if imu_point is not None and imu_rotation is not None:
                transform_to_global(detections, imu_point, imu_rotation)

        return pipelines_results

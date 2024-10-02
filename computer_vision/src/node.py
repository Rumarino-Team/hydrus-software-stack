#!/usr/bin/env python3

import rospy
from computer_vision.msg import Detection, Detections  # Import the new message type
from computer_vision.srv import EnableDetector, EnableDetectorResponse, SetCustomClasses, SetCustomClassesResponse
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO, YOLOWorld
import numpy as np
import tf.transformations as tft
import yaml
import os

# Global variables to enable/disable detectors
detectors = {
    "yolo": YOLO("/yolov8n.pt"),
    "yolo_world": YOLOWorld("/yolov8s-world.pt")
}

pub = rospy.Publisher('/detector/box_detection', Detections, queue_size=10)
annotated_pub = rospy.Publisher('annotated_image', Image, queue_size=10)

current_detector = None
bridge = CvBridge()
depth_image = None
camera_info = None
imu_pose = None

def read_yaml_file(file_path):
    with open(file_path, 'r') as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr(exc)
            return None

# Function to handle the EnableDetector service
def handle_enable_detector(req):
    global current_detector
    detector_name = req.detector_name

    if detector_name in detectors:
        current_detector = detectors[detector_name]
        current_detector.to("cuda")
        rospy.loginfo(f"{detector_name} detector enabled.")
        return EnableDetectorResponse(success=True)
    else:
        rospy.logerr(f"Detector '{detector_name}' not found.")
        return EnableDetectorResponse(success=False)


# Function to handle the SetCustomClasses service
def handle_set_custom_classes(req):
    global current_detector
    classes = req.classes
    save_path = req.save_path

    if current_detector is not None:
        current_detector.set_classes(classes)
        # current_detector.save(save_path)
        rospy.loginfo(f"Custom classes set and model saved to {save_path}")
        return SetCustomClassesResponse(success=True)
    else:
        rospy.logerr("No detector is currently enabled.")
        return SetCustomClassesResponse(success=False)

# Callback function for depth image data
def depth_image_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Callback function for camera info
def camera_info_callback(msg):
    global camera_info
    camera_info = msg

# Callback function for IMU PoseStamped data
def imu_pose_callback(msg):
    global imu_pose
    imu_pose = msg

# Transform a point from camera frame to global frame
def transform_to_global(point, pose):
    # Create transformation matrix from IMU pose
    translation = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    rotation = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    transform_matrix = tft.quaternion_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    # Transform the point
    point_homogeneous = np.array([point.x, point.y, point.z, 1.0])
    point_global = np.dot(transform_matrix, point_homogeneous)

    return Point(point_global[0], point_global[1], point_global[2])

# Callback function for image data
def zed_image_callback(msg):
    global depth_image, camera_info, imu_pose, current_detector, pub, annotated_pub
    if current_detector is None:
        return

    try:
        # Convert the ROS Image message to a numpy array
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        annotated_image = cv_image.copy()

        # Run detection using the current detector
        results = current_detector.track(cv_image, persist=True)[0]

        # List to store all detections
        all_detections = Detections()
        class_names = []

        for box in results.boxes:
            x_min, y_min, x_max, y_max = box.xyxy.cpu().numpy()[0]

            track_id = box.id
            conf = box.conf
            cls = box.cls

            detection_msg = Detection()
            detection_msg.cls = int(cls)  # The class label of the detected object
            detection_msg.confidence = float(conf)  # The confidence score of the detection

            detection_msg.bounding_box = RegionOfInterest(
                x_offset=int(x_min),
                y_offset=int(y_min),
                height=int(y_max - y_min),
                width=int(x_max - x_min)
            )

            # Draw bounding box and label on the image
            cv2.rectangle(annotated_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
            label = f"{int(cls)}: {int(conf):.2f}"
            cv2.putText(annotated_image, label, (int(x_min), int(y_min) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Calculate the average depth within the bounding box
            if depth_image is not None:
                x_min_int = int(x_min)
                x_max_int = int(x_max)
                y_min_int = int(y_min)
                y_max_int = int(y_max)

                bbox_depth = depth_image[y_min_int:y_max_int, x_min_int:x_max_int]
                if bbox_depth.size > 0:
                    mean_depth = np.nanmean(bbox_depth)
                    if not np.isnan(mean_depth) and camera_info is not None:
                        # Convert depth to 3D point using camera intrinsic parameters
                        fx = camera_info.K[0]
                        fy = camera_info.K[4]
                        cx = camera_info.K[2]
                        cy = camera_info.K[5]

                        z = mean_depth
                        x_center = (x_min + x_max) / 2
                        y_center = (y_min + y_max) / 2
                        x = (x_center - cx) * z / fx
                        y = (y_center - cy) * z / fy

                        point_camera = Point(x=x, y=y, z=z)

                        # Transform the point to global coordinates if IMU pose is available
                        if imu_pose is not None:
                            point_global = transform_to_global(point_camera, imu_pose)
                            detection_msg.point = point_global
                        else:
                            detection_msg.point = point_camera

            # Append the detection to the list
            all_detections.detections.append(detection_msg)
            class_names.append(current_detector.names[int(cls)])

        # Add class names to the detections message
        all_detections.class_names = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "TV", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"
]

        # Publish all detections at once
        pub.publish(all_detections)

        # Publish the annotated image
        annotated_msg = bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        annotated_pub.publish(annotated_msg)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

# Function to initialize subscribers
def initialize_subscribers(topics_file):
    topics_info = read_yaml_file(topics_file)
    if topics_info is None:
        rospy.logerr("Failed to read YAML file or file is empty.")
        return

    rospy.loginfo("YAML file read successfully.")
    rospy.Subscriber(topics_info['zed_camera']['image'], Image, zed_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['depth_image'], Image, depth_image_callback)
    rospy.Subscriber(topics_info['zed_camera']['camera_info'], CameraInfo, camera_info_callback)
    rospy.Subscriber(topics_info['zed_camera']['imu_pose'], PoseStamped, imu_pose_callback)
    rospy.sleep(5)

if __name__ == "__main__":
    rospy.init_node('yolo_detector')

    service_enable_detector = rospy.Service('enable_detector', EnableDetector, handle_enable_detector)
    service_set_custom_classes = rospy.Service('set_custom_classes', SetCustomClasses, handle_set_custom_classes)
    config_path = os.path.join(rospy.get_param('detector_node'), '../configs/topics.yml')
    initialize_subscribers(config_path)

    rospy.spin()

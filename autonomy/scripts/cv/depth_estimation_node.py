#!/usr/bin/env python3
"""
ROS node for depth estimation using Depth-Anything-ONNX
Publishes depth maps and provides depth estimation services
"""

import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32

# Add the computer vision module to path
sys.path.append(os.path.dirname(__file__))

try:
    from computer_vision.depth_estimation import DepthEstimator

    DEPTH_ESTIMATION_AVAILABLE = True
except ImportError as e:
    DEPTH_ESTIMATION_AVAILABLE = False
    rospy.logwarn(f"Depth estimation dependencies not available: {e}")
    rospy.logwarn("Install with: pip install -e .[depth-estimation]")


class DepthEstimationNode:
    """ROS node for depth estimation"""

    def __init__(self):
        rospy.init_node("depth_estimation_node")

        # Check if depth estimation is available
        if not DEPTH_ESTIMATION_AVAILABLE:
            rospy.logerr("Depth estimation dependencies not installed!")
            rospy.logerr("Install with: pip install -e .[depth-estimation]")
            rospy.signal_shutdown("Missing dependencies")
            return

        # Parameters
        self.encoder = rospy.get_param("~encoder", "vitb")
        self.model_path = rospy.get_param("~model_path", None)
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.use_compressed = rospy.get_param("~use_compressed", False)
        self.publish_colored = rospy.get_param("~publish_colored", True)

        # Initialize components
        self.bridge = CvBridge()

        try:
            self.depth_estimator = DepthEstimator(
                model_path=self.model_path, encoder=self.encoder
            )
        except ImportError as e:
            rospy.logerr(f"Failed to initialize depth estimator: {e}")
            rospy.signal_shutdown("Depth estimator initialization failed")
            return

        # Publishers
        self.depth_pub = rospy.Publisher("~depth_map", Image, queue_size=1)
        self.depth_colored_pub = rospy.Publisher("~depth_colored", Image, queue_size=1)
        self.center_depth_pub = rospy.Publisher("~center_depth", Float32, queue_size=1)

        # Subscribers
        if self.use_compressed:
            self.image_sub = rospy.Subscriber(
                self.input_topic, CompressedImage, self.compressed_image_callback
            )
        else:
            self.image_sub = rospy.Subscriber(
                self.input_topic, Image, self.image_callback
            )

        rospy.loginfo(f"Depth estimation node started with encoder: {self.encoder}")
        rospy.loginfo(f"Subscribing to: {self.input_topic}")

    def image_callback(self, msg):
        """Process uncompressed image messages"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image, msg.header)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def compressed_image_callback(self, msg):
        """Process compressed image messages"""
        try:
            # Convert compressed image to OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(cv_image, msg.header)
        except Exception as e:
            rospy.logerr(f"Error processing compressed image: {e}")

    def process_image(self, cv_image, header):
        """Process image and publish depth information"""
        try:
            # Estimate depth
            raw_depth, viz_depth = self.depth_estimator.estimate_depth(cv_image)

            # Publish raw depth map
            depth_msg = self.bridge.cv2_to_imgmsg(raw_depth.astype(np.float32), "32FC1")
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)

            # Publish colored depth map if enabled
            if self.publish_colored:
                colored_depth = self.depth_estimator.create_colored_depth_map(viz_depth)
                colored_msg = self.bridge.cv2_to_imgmsg(colored_depth, "bgr8")
                colored_msg.header = header
                self.depth_colored_pub.publish(colored_msg)

            # Publish center depth value
            h, w = cv_image.shape[:2]
            center_depth = self.depth_estimator.get_depth_at_point(
                raw_depth, w // 2, h // 2
            )
            center_msg = Float32()
            center_msg.data = center_depth
            self.center_depth_pub.publish(center_msg)

        except Exception as e:
            rospy.logerr(f"Error in depth processing: {e}")


if __name__ == "__main__":
    try:
        node = DepthEstimationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

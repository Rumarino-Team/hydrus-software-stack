#!/usr/bin/env python3
import os

# Reuse existing detection core utilities
import sys
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, RegionOfInterest

sys.path.append(
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../src/computer_vision")
    )
)
from detection_core import ColorFilterConfig, DetectionPipelineManager, ros_img_to_cv2

from autonomy.msg import Detection as DetectionMsg
from autonomy.msg import Detections as DetectionsMsg


class CvNode(Node):
    def __init__(self):
        super().__init__("cv_node")
        # Parameters
        self.declare_parameter("yolo_model", "yolo11n.pt")
        self.declare_parameter("color_target_rgb", [255, 0, 0])
        self.declare_parameter("color_tolerance", 0.4)
        self.declare_parameter("color_min_confidence", 0.3)
        self.declare_parameter("color_min_area", 0.2)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/detections")

        # Build pipeline
        self.pipeline = DetectionPipelineManager()
        model = self.get_parameter("yolo_model").get_parameter_value().string_value
        try:
            self.pipeline.load_yolo_model(model)
        except Exception as e:
            self.get_logger().warn(f"YOLO model load failed: {e}")

        rgb = (
            self.get_parameter("color_target_rgb")
            .get_parameter_value()
            .integer_array_value
        )
        tol = float(self.get_parameter("color_tolerance").value)
        min_conf = float(self.get_parameter("color_min_confidence").value)
        min_area = float(self.get_parameter("color_min_area").value)
        self.pipeline.update_color_filter_config(
            ColorFilterConfig(
                tolerance=tol,
                min_confidence=min_conf,
                min_area=min_area,
                rgb_range=tuple(int(x) for x in rgb),
            )
        )

        # Pub/Sub
        image_topic = self.get_parameter("image_topic").value
        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )
        det_topic = self.get_parameter("detections_topic").value
        self.pub = self.create_publisher(DetectionsMsg, det_topic, 10)

        self.get_logger().info(
            f"cv_node started. Subscribing to {image_topic}, publishing {det_topic}"
        )

    def on_image(self, msg: Image):
        try:
            img = ros_img_to_cv2(msg, encoding=msg.encoding if msg.encoding else "bgr8")
        except Exception:
            # fallback assume bgr8
            img = ros_img_to_cv2(msg, encoding="bgr8")

        results: List[Tuple[str, list]] = self.pipeline.run_detections(img)
        for name, detections in results:
            out = DetectionsMsg()
            out.detector_name = name
            out.detections = []
            for d in detections:
                dm = DetectionMsg()
                dm.cls = int(d.cls)
                dm.confidence = float(d.conf)
                # point may be None
                if getattr(d, "point", None) is not None:
                    dm.point = Point(
                        x=float(d.point.x), y=float(d.point.y), z=float(d.point.z)
                    )
                else:
                    dm.point = Point()
                roi = RegionOfInterest()
                roi.x_offset = int(d.x1)
                roi.y_offset = int(d.y1)
                roi.width = int(d.x2 - d.x1)
                roi.height = int(d.y2 - d.y1)
                dm.bounding_box = roi
                out.detections.append(dm)
            self.pub.publish(out)


def main():
    rclpy.init()
    node = CvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

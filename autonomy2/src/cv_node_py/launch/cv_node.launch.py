from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cv_node_py",
                executable="cv_node",
                name="cv_node",
                output="screen",
                parameters=[
                    {"yolo_model": "yolo11n.pt"},
                    {"color_target_rgb": [255, 0, 0]},
                    {"color_tolerance": 0.4},
                    {"color_min_confidence": 0.3},
                    {"color_min_area": 0.2},
                    {"image_topic": "/camera/image_raw"},
                    {"detections_topic": "/detections"},
                ],
            )
        ]
    )

# Vision Node Project

This project involves a ROS node that uses the YOLO-World model for object detection. It includes functionality to dynamically switch between different YOLO models, set custom classes for detection, and stream annotated video feeds.

## Features

- **Dynamic Model Switching**: Switch between different YOLO models using a ROS service.
- **Custom Classes**: Set custom classes for YOLO detection and save the customized model.
- **Video Streaming**: Stream annotated video feeds via a FastAPI server.
- **Depth and Global Positioning**: Calculate 3D points and transform them to global coordinates using depth and IMU data.

## Directory Structure
```markdown
catkin_ws/
├── src/
│ ├── vision_node/
│ │ ├── CMakeLists.txt
│ │ ├── package.xml
│ │ ├── msg/
│ │ │ ├── Detection.msg
│ │ ├── srv/
│ │ │ ├── EnableDetector.srv
│ │ │ ├── SetCustomClasses.srv
│ │ ├── scripts/
│ │ │ ├── detector_node.py
│ │ │ ├── enable_detector_cli.py
│ │ │ ├── set_custom_classes_cli.py
│ │ │ ├── video_stream_with_yolo_fastapi.py
│ │ └── params/
│ │ ├── topics.yml
├── docker-compose.yml
└── Dockerfile
```

## Prerequisites

- **ROS Noetic**
- **Python 3**
- **Docker**
- **Docker Compose**

## Setup Instructions

### Step 1: Build the Docker Image

Make sure you have Docker and Docker Compose installed. Then, build the Docker image:

```bash
docker-compose build
```
Step 2: Run the Docker Container
Start the Docker container with the following command:
```bash
docker-compose up
```
Step 3: Source the ROS Setup Script
In a new terminal, source the ROS setup script:

```bash
source devel/setup.bash
```
Step 4: Build the Workspace
Build the ROS workspace:

```bash
catkin_make
source devel/setup.bash
```
Running the Nodes
Run the Detector Node
```bash

rosrun vision_node detector_node.py
```
Run the FastAPI Video Stream Server
```bash
rosrun vision_node video_stream_with_yolo_fastapi.py
```
Services
Enable Detector
To switch between different YOLO models, use the enable_detector service.

Service Definition: EnableDetector.srv
```
string detector_name
---
bool success
```
Service Client:

```bash
rosrun vision_node enable_detector_cli.py --detector yolo
```
Set Custom Classes
To set custom classes for detection and save the customized model, use the set_custom_classes service.

Service Definition: SetCustomClasses.srv

```plaintext
string[] classes
string save_path
---
bool success
Service Client:
```
```bash
rosrun vision_node set_custom_classes_cli.py --classes person bus --save_path /path/to/save/custom_model.pt
```

Topics
Image Topics:

 - /zed_camera/image (type: sensor_msgs/Image)
 - /zed_camera/depth_image (type: sensor_msgs/Image)
 - /zed_camera/camera_info (type: sensor_msgs/CameraInfo)
 - /zed_camera/imu_pose (type: geometry_msgs/PoseStamped)

Published Topics:

 - /detections (type: detector_node/Detection)
 - /annotated_image (type: sensor_msgs/Image)


API Endpoints
Access the video stream at:

```plaintext
http://<your-ros-master-ip>:8000/video_feed
```

License
This project is licensed under the MIT License.
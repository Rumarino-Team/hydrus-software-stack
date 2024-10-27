#!/bin/bash

# Set environment variables
export ROS_MASTER_URI=http://localhost:11311

# Step 1: Run the ROS master container
docker run -d \
  --name ros-master \
  -p 11311:11311 \
  ros:noetic-ros-core \
  stdbuf -o L roscore

# Wait for ROS master to be up (adjust sleep time if necessary)
echo "Waiting for ROS master to start..."
sleep 8

# Step 2: Build and run the ZED camera container
docker build -t zed-camera -f docker/jetson/camera.Dockerfile .

docker run -d \
  --name zed-camera \
  --privileged \
  --gpus all \
  --env ROS_MASTER_URI=http://ros-master:11311 \
  zed-camera

# Wait for ZED camera to be ready (adjust sleep time if necessary)
echo "Waiting for ZED camera to start..."
sleep 10

# Step 3: Build and run the Hydrus container
docker build -t hydrus -f docker/jetson/hydrus.Dockerfile .

docker run -d \
  --name hydrus \
  --privileged \
  --gpus all \
  -p 8000:8000 \
  -v "$(pwd)/:/catkin_ws/src" \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  --env ROS_MASTER_URI=http://ros-master:11311 \
  --env ARDUINO_BOARD=arduino:avr:mega \
  -it hydrus

echo "Containers are up and running!"

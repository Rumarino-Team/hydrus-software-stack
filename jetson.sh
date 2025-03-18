#!/bin/bash
set -e

# Set environment variables
export ROS_MASTER_URI=http://localhost:11311

# Function to check if a container exists
container_exists() {
  docker ps -a --format '{{.Names}}' | grep -q "^$1$"
}

# Function to check if a container is running
container_running() {
  docker ps --format '{{.Names}}' | grep -q "^$1$"
}

# Step 1: Run the ROS master container
if container_exists "ros-master"; then
  if container_running "ros-master"; then
    echo "ROS master is already running."
  else
    echo "Starting existing ROS master container..."
    docker start ros-master
  fi
else
  echo "Creating and starting ROS master container..."
  docker run -d \
    --name ros-master \
    -p 11311:11311 \
    ros:noetic-ros-core \
    stdbuf -o L roscore
fi

# Wait for ROS master to be up (adjust sleep time if necessary)
echo "Waiting for ROS master to start..."
sleep 8

# Step 2: Build the ZED camera container (Only if it doesn't exist)

docker build -t zed-camera -f docker/jetson/camera.Dockerfile .


# Run the ZED camera container
if container_exists "zed-camera"; then
  if container_running "zed-camera"; then
    echo "ZED camera container is already running."
  else
    echo "Starting existing ZED camera container..."
    docker start zed-camera
  fi
else
  echo "Creating and starting ZED camera container..."
  docker run -d \
    --name zed-camera \
    --privileged \
    --gpus all \
    --env ROS_MASTER_URI=http://ros-master:11311 \
    zed-camera
fi

# Wait for ZED camera to be ready (adjust sleep time if necessary)
echo "Waiting for ZED camera to start..."
sleep 10

# Step 3: Build the Hydrus container (Only if it doesn't exist)
if ! docker images | grep -q "hydrus"; then
  echo "Building Hydrus image..."
  docker build -t hydrus -f docker/jetson/hydrus.Dockerfile .
else
  echo "Hydrus image already exists, skipping build."
fi

# Run the Hydrus container
if container_exists "hydrus"; then
  if container_running "hydrus"; then
    echo "Hydrus container is already running."
  else
    echo "Starting existing Hydrus container..."
    docker start hydrus
  fi
else
  echo "Creating and starting Hydrus container..."
  docker run -d \
    --name hydrus \
    --privileged \
    --gpus all \
    -p 8000:8000 \
    -v "$(pwd)/:/home/catkin_ws/src" \
    --device /dev/ttyACM0:/dev/ttyACM0 \
    --env ROS_MASTER_URI=http://ros-master:11311 \
    --env ARDUINO_BOARD=arduino:avr:mega \
    -it hydrus
fi

docker cp ./ hydrus:/catkin_ws/src/hydrus-software-stack/

echo "Containers are up and running!"

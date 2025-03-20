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

# Detect system architecture and set up QEMU if needed
ARCH=$(uname -m)
USE_QEMU=false
QEMU_ARGS=""

if [[ "$ARCH" == "x86_64" ]]; then
  echo "Detected x86_64 architecture. Will use QEMU for ARM emulation."
  
  # Check if QEMU is installed
  if ! command -v qemu-system-aarch64 &> /dev/null; then
    echo "ERROR: QEMU is not installed. Please install it with:"
    echo "  sudo apt-get install qemu-system-arm qemu-efi qemu-user-static"
    exit 1
  fi
  
  # Register QEMU binary formats if not already done
  if [ ! -f /proc/sys/fs/binfmt_misc/qemu-aarch64 ]; then
    echo "Setting up QEMU binary formats for ARM emulation..."
    sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
  fi
  
  USE_QEMU=true
  QEMU_ARGS="--platform linux/arm64"
  echo "QEMU emulation enabled for ARM containers."
fi

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
    ros:melodic-ros-core \
    stdbuf -o L roscore
fi

# Wait for ROS master to be up (adjust sleep time if necessary)
echo "Waiting for ROS master to start..."
sleep 3

# Step 2: Build the ZED camera container (Only if it doesn't exist)
if [[ "$USE_QEMU" == "true" ]]; then
  echo "Building ZED camera container with QEMU emulation..."
  docker build $QEMU_ARGS -t zed-camera -f docker/jetson/camera.Dockerfile .
else
  docker build -t zed-camera -f docker/jetson/camera.Dockerfile .
fi

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
  if [[ "$USE_QEMU" == "true" ]]; then
    docker run -d \
      --name zed-camera \
      --privileged \
      --gpus all \
      --env ROS_MASTER_URI=http://ros-master:11311 \
      $QEMU_ARGS \
      zed-camera
  else
    docker run -d \
      --name zed-camera \
      --privileged \
      --gpus all \
      --env ROS_MASTER_URI=http://ros-master:11311 \
      zed-camera
  fi
fi

# Wait for ZED camera to be ready (adjust sleep time if necessary)
echo "Waiting for ZED camera to start..."
sleep 3

# Step 3: Build the Hydrus container (Only if it doesn't exist)
if ! docker images hydrus:latest --format '{{.Repository}}:{{.Tag}}' | grep -q "hydrus:latest"; then
  echo "Building Hydrus image..."
  if [[ "$USE_QEMU" == "true" ]]; then
    docker build $QEMU_ARGS -t hydrus:latest -f docker/jetson/hydrus.Dockerfile .
  else
    docker build -t hydrus:latest -f docker/jetson/hydrus.Dockerfile .
  fi
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
  if [[ "$USE_QEMU" == "true" ]]; then
    docker run -d \
      --name hydrus \
      --privileged \
      --gpus all \
      -p 8000:8000 \
      -v "$(pwd)/:/home/catkin_ws/src" \
      --device /dev/ttyACM0:/dev/ttyACM0 \
      --env ROS_MASTER_URI=http://ros-master:11311 \
      --env ARDUINO_BOARD=arduino:avr:mega \
      $QEMU_ARGS \
      -it hydrus:latest
  else
    docker run -d \
      --name hydrus \
      --privileged \
      --gpus all \
      -p 8000:8000 \
      -v "$(pwd)/:/home/catkin_ws/src" \
      --device /dev/ttyACM0:/dev/ttyACM0 \
      --env ROS_MASTER_URI=http://ros-master:11311 \
      --env ARDUINO_BOARD=arduino:avr:mega \
      -it hydrus:latest
  fi
fi

docker cp ./ hydrus:/home/catkin_ws/src/hydrus-software-stack/

echo "Containers are up and running!"

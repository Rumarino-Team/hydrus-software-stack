# Use an official ROS image as a parent image
FROM ros:noetic-ros-base

# Print Ubuntu version
RUN apt-get update && apt-get install -y lsb-release gnupg curl
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

# Camera and Computer Vision Dependencies Python-3
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy\
    python3-opencv \
    libgl1-mesa-glx \
    ros-noetic-cv-bridge \
    ros-noetic-vision-opencv\
    libbullet-dev \
    python3-empy

# Mission Node Dependencies
RUN apt-get install -y \
    ros-noetic-smach-ros \
    ros-noetic-executive-smach \
    ros-noetic-smach-viewer\
    ros-noetic-tf2-geometry-msgs\
    libeigen3-dev\
    python3-tf2-kdl

# Embedded Node Dependencies
RUN apt-get install -y --no-install-recommends \
       gcc \
       curl \
       git

# ROS setup
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    mkdir -p /home/catkin_ws/src && \
    cd /home/catkin_ws/ && \
    catkin_make'

# Install Arduino CLI and libraries
WORKDIR /usr/local/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh && \
    arduino-cli core update-index && \
    arduino-cli core install arduino:avr
RUN arduino-cli lib install "Rosserial Arduino Library@0.7.9" && \
    sed -i '/#include "ros\/node_handle.h"/a #include "geometry_msgs/Vector3.h"' /root/Arduino/libraries/Rosserial_Arduino_Library/src/ros.h && \
    arduino-cli lib install "Servo@1.2.1" && \
    arduino-cli lib install "BlueRobotics MS5837 Library@1.1.1"
RUN apt-get install -y ros-noetic-rosserial-arduino


# Copy embedded Arduino code in the Arduino libraries folder
COPY ./embedded_arduino /root/Arduino/libraries/embedded_arduino


# Copy the Python Dependencies and Install them
COPY ./requirements.txt /requirements.txt
RUN apt-get install -y python3
# Ultralytics with NO GPU
RUN python3 -m pip install --extra-index-url https://download.pytorch.org/whl/cpu ultralytics
RUN python3 -m pip install -r /requirements.txt

# Install Default models for YOLO
RUN curl -Lo /yolov8n.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt
RUN curl -Lo /yolov8s-world.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8s-world.pt

COPY ./ /catkin_ws/src/hydrus-software-stack
WORKDIR /catkin_ws/src/hydrus-software-stack
RUN chmod +x ros-entrypoint.sh
CMD ["./ros-entrypoint.sh"]

# Use an official ROS image as a parent image
FROM ros:noetic-ros-base

# Print Ubuntu version
RUN apt-get update && apt-get install -y lsb-release gnupg curl
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

# Install Python and detector node Dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgl1-mesa-glx \
    ros-noetic-cv-bridge \
#    ros-noetic-python-orocos-kdl \
    libbullet-dev \
    python3-empy

# Install Gazebo and ROS Noetic integration packages
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Mission Node Dependencies
RUN apt-get install -y \
    ros-noetic-smach-ros \
    ros-noetic-executive-smach \
    ros-noetic-smach-viewer

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


# Copy the rest of your application code
COPY ./requirements.txt /requirements.txt

# Install additional Python packages using pip
RUN apt-get install -y python3.9

# Install the default ultralytics with Pytorch Cuda dependencies.
RUN python3.9 -m pip install  ultralytics
RUN python3.9 -m pip install -r /requirements.txt

RUN curl -Lo /yolov8n.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt
RUN curl -Lo /yolov8s-world.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8s-world.pt

RUN apt-get install -y libeigen3-dev python3-tf2-kdl
RUN apt-get update && apt-get install -y ros-noetic-tf2-geometry-msgs

COPY ./ /catkin_ws/src/hydrus-software-stack
WORKDIR /catkin_ws/src/hydrus-software-stack
RUN chmod +x ros-entrypoint.sh
CMD ["./ros-entrypoint.sh"]

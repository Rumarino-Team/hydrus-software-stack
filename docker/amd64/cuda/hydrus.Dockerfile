# Use an official ROS image as a parent image
FROM ros:noetic-ros-base

# Update package list
RUN apt-get update && apt-get install -y lsb-release gnupg curl software-properties-common

# Add the deadsnakes PPA and install Python 3.8
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8 python3.8-distutils python3.8-venv

# Set Python 3.8 as the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install pip for Python 3.8
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.8

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


RUN apt-get update && apt-get install -y\
    ros-noetic-tf2-geometry-msgs\
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
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

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

# Ultralytics with  GPU
RUN python3 -m pip install ultralytics
RUN python3 -m pip install -r /requirements.txt

# Install Default models for YOLO
RUN curl -Lo /yolov8n.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt
RUN curl -Lo /yolov8s-world.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8s-world.pt

RUN apt-get install -y libeigen3-dev python3-tf2-kdl
RUN apt-get update && apt-get install -y ros-noetic-tf2-geometry-msgs

RUN echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> /root/.bashrc

COPY ./ /catkin_ws/src/hydrus-software-stack
WORKDIR /catkin_ws/src/hydrus-software-stack
RUN chmod +x ros-entrypoint.sh
CMD ["/catkin_ws/src/hydrus-software-stack/ros-entrypoint.sh"]

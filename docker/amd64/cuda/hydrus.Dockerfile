# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Update package list
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y lsb-release gnupg curl software-properties-common

# Setup ROS repositories
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic base and catkin build tools
RUN apt-get update && apt-get install -y ros-noetic-ros-base python3-catkin-tools ros-noetic-catkin

# Add the deadsnakes PPA and install Python 3.8
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.8 python3.8-distutils python3.8-venv

# Set Python 3.8 as the default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install pip for Python 3.8
RUN curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py | python3.8

#Add kisak-mesa PPA (for latest graphics drivers)
RUN add-apt-repository -y ppa:kisak/kisak-mesa


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


RUN sudo sh -c \
    'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
     curl https://packages.osrfoundation.org/gazebo.key | sudo apt-key add - &&\
     apt-get update && apt-get install -y ignition-fortress ros-noetic-ros-ign &&\
     echo "export GZ_VERSION=fortress" >> /root/.bashrc


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
    arduino-cli core install arduino:avr &&\
    arduino-cli lib install "Servo@1.2.1" && \
    arduino-cli lib install "BlueRobotics MS5837 Library@1.1.1"


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

# Install additional dependencies for the embedded node
# Install tmux, vim, git, and htop in a single RUN command
RUN apt-get update && apt-get install -y tmux vim git htop socat

# Copy embedded Arduino code in the Arduino libraries folder
COPY ./devices/Arduino/HydrusModule /root/Arduino/libraries/HydrusModule

COPY ./ /catkin_ws/src/hydrus-software-stack
WORKDIR /catkin_ws/src/hydrus-software-stack
CMD ["/bin/bash", "-c", "sleep infinity"]

# Use an official ROS image as a parent image
FROM ultralytics/ultralytics:latest-jetson-jetpack4

# Print Ubuntu version
RUN apt-get update && apt-get install -y lsb-release gnupg curl
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_PYTHON_VERSION=3

#Install ROS with melodic python3 workaround
#https://www.dhanoopbhaskar.com/blog/2020-05-07-working-with-python-3-in-ros-kinetic-or-melodic/
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
apt-get update && apt-get install -y python3-rospkg git python3-rosdep python3-distro &&\
apt-get install -y --fix-missing ros-melodic-ros-base
RUN rosdep init && rosdep update --rosdistro=melodic

# Camera and Computer Vision Dependencies Python-3
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libgl1-mesa-glx \
    ros-melodic-cv-bridge \
    ros-melodic-python-orocos-kdl \
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
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && \
    mkdir -p /home/catkin_ws/src && \
    cd /home/catkin_ws/ && \
    catkin_make'
    # Source ROS setup.bash script
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Install Arduino CLI and libraries
WORKDIR /usr/local/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh && \
    arduino-cli core update-index && \
    arduino-cli core install arduino:avr
RUN arduino-cli lib install "Rosserial Arduino Library@0.7.9" && \
    sed -i '/#include "ros\/node_handle.h"/a #include "geometry_msgs/Vector3.h"' /root/Arduino/libraries/Rosserial_Arduino_Library/src/ros.h && \
    arduino-cli lib install "Servo@1.2.1" && \
    arduino-cli lib install "BlueRobotics MS5837 Library@1.1.1"
RUN apt-get install -y ros-melodic-rosserial-arduino



# Copy embedded Arduino code in the Arduino libraries folder
COPY ./embedded_arduino /root/Arduino/libraries/embedded_arduino


# Copy the Python Dependencies and Install them
COPY ./requirements.txt /requirements.txt
RUN apt-get install -y python3
RUN python3 -m pip install -r /requirements.txt

# Install Default models for YOLO
RUN curl -Lo /yolov8n.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt
RUN curl -Lo /yolov8s-world.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8s-world.pt

COPY ./ /catkin_ws/src/hydrus-software-stack
WORKDIR /catkin_ws/src/hydrus-software-stack
RUN chmod +x ros-entrypoint.sh
CMD ["./ros-entrypoint.sh"]

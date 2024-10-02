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

# Install Python and detector node Dependencies
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
    ros-melodic-smach-ros \
    ros-melodic-executive-smach \
    ros-melodic-smach-viewer

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

# Source ROS setup.bash script
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# Copy embedded Arduino code
WORKDIR /root/Arduino/libraries
COPY ./embedded_arduino ./sensor_actuator_pkg

# Set the working directory
WORKDIR /catkin_ws
# Copy the rest of your application code
COPY ./requirements.txt /requirements.txt
# Install additional Python packages using pip
RUN pip install -r /requirements.txt

COPY ./ros-entrypoint.sh /catkin_ws/ros-entrypoint.sh
RUN chmod +x /catkin_ws/ros-entrypoint.sh

RUN curl -Lo /yolov8n.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8n.pt
RUN curl -Lo /yolov8s-world.pt https://github.com/ultralytics/assets/releases/latest/download/yolov8s-world.pt

# # Build the catkin workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set the entrypoint
CMD ["/catkin_ws/ros-entrypoint.sh"]


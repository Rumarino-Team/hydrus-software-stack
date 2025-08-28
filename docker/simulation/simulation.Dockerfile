FROM osrf/ros:humble-desktop-full-jammy
RUN apt-get update

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get upgrade -y && \
	apt-get install -y \
	libglm-dev \
	libfreetype6-dev \
	libsdl2-dev \
	python3-colcon-common-extensions \
	python3-rosdep \
	python3-vcstool

# ROS 2 workspace setup
SHELL ["/bin/bash", "-c"]

ENV ROS_WS=/home/ros2_ws

RUN mkdir -p /${ROS_WS}/src \
	&& echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 

WORKDIR $ROS_WS/src

RUN cd /${ROS_WS}/ \
	&& colcon build --symlink-install \
	&& source install/setup.bash

WORKDIR /home/

RUN git clone https://github.com/patrykcieslak/stonefish
RUN cd stonefish && mkdir build  && cd build  && cmake ..  && make -j$(nproc) && sudo make install

WORKDIR $ROS_WS/src

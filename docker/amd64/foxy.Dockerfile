# Hydrus ROS 2 (Foxy) layer built on shared base
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Common packages for all Hydrus images
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    cmake \
    git \
    wget \
    curl \
    lsb-release \
    gnupg \
    software-properties-common \
    ca-certificates \
    tmux \
    vim \
    htop \
    socat \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*


ENV ROS_DISTRO=foxy \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Configure locale and add ROS 2 apt repository; install core + dev tools
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-foxy-ros-base \
       ros-foxy-tf2-ros \
       ros-foxy-tf2-geometry-msgs \
       ros-foxy-image-transport \
       ros-foxy-rviz2 \
       ros-foxy-rqt \
       ros-foxy-rosbag2 \
       ros-foxy-ros2bag \
       ros-foxy-rqt-image-view \
       python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

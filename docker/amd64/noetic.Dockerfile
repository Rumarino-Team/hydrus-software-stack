# Hydrus ROS 1 (Noetic) layer built on shared base
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


# Add ROS 1 repo and install core + commonly used packages
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-noetic-ros-base \
       ros-noetic-tf2-geometry-msgs \
       python3-tf2-kdl \
       ros-noetic-rviz \
       ros-noetic-rqt \
       ros-noetic-rosbag \
       ros-noetic-image-view \
       ros-noetic-tf \
       ros-noetic-tf2-ros \
       ros-noetic-image-transport \
       ros-noetic-laser-proc \
    && rm -rf /var/lib/apt/lists/*

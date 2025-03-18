    # Specify the parent image from which we build
    FROM stereolabs/zed:4.1-devel-jetson-jp4.6.1

    # Print Ubuntu version
    RUN apt-get update && apt-get install -y lsb-release gnupg

    #Install ROS
    RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
    RUN apt-get update && apt-get install -y ros-melodic-ros-base
    RUN apt-get install -y git python-rosdep
    RUN rosdep init && rosdep update --rosdistro=melodic

    # Create workspace inside for zed-ros-interfaces
    RUN . /opt/ros/melodic/setup.sh && \
        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make

    # Download zed-ros-wrapper
    RUN git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git ~/catkin_ws/src/zed-ros-wrapper

    WORKDIR /root/catkin_ws
    RUN . /opt/ros/melodic/setup.sh && \
        rosdep install --from-paths src --ignore-src -r -y
    COPY ./camera-entrypoint.sh /camera-entrypoint.sh
    RUN chmod +x /camera-entrypoint.sh

    CMD ["/camera-entrypoint.sh"]

#!/bin/bash
set -e

ROS_DISTRO=jazzy
UBU_CODENAME=$(lsb_release -cs)

echo "🛠 Installing ROS 2 $ROS_DISTRO on Ubuntu $UBU_CODENAME..."

# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Repos & keys
sudo apt install -y software-properties-common curl gnupg
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $UBU_CODENAME main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-desktop \
  python3-colcon-common-extensions python3-pip python3-rosdep

# Rosdep init
sudo rosdep init || true
rosdep update

# Source on startup
if ! grep -F "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc >/dev/null; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

echo "✅ ROS 2 $ROS_DISTRO installation complete!"

#!/bin/bash
#
# Установка окружения для работы модели RobotFrame в симуляторе Gazebo
#
shopt -s nocasematch
OS=$(cat /etc/lsb-release | grep DISTRIB_ID | awk -F"=" '{print $2}')
if [[ "${OS}" != "Ubuntu" ]]; then echo "Supported operating system only UBUNTU"; exit 1; fi
OSVER=$(cat /etc/lsb-release | grep RELEASE | awk -F"=" '{print $2}')
MIN_VER="22.04"
if awk "BEGIN {exit !(${OSVER} < ${MIN_VER})}"; then
echo "Unsupported version of operation system Ubuntu"; exit 1;
fi

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Добавление репозитория ROS2
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Установка ROS2
sudo apt update
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc  \
&& echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc 
source ~/.bashrc

# Установка дополнительных пакетов 
pip3 install setuptools==58.2.0
apt install -y \
gazebo \
libqt5gui5 \
libgazebo11 \
libopencv-dev \
ros-humble-xacro \
python3-argcomplete \
joystick \
ros-dev-tools \
ros-humble-rqt \
ros-humble-gazebo-ros-pkgs \
ros-humble-joint-state-publisher-gui \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-twist-mux \
ros-humble-controller-manager \
ros-humble-usb-cam \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-gazebo-ros2-control \
ros-humble-gripper-controllers \
ros-humble-rviz2

echo "ROS2 Numble is successfully installed."

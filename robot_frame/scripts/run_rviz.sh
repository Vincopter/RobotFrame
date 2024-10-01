#!/bin/bash

CURRENT_DIR=(`pwd`)
PACKAGE_NAME="robot_frame"

source /opt/ros/humble/setup.bash
if [ -z "${COLCON_PREFIX_PATH}" ]; then 
    source ~/.bashrc
else
    source ${COLCON_PREFIX_PATH}/setup.bash; 
fi

RVIZ_PACKAGE=$(ros2 pkg list | grep "rviz2")
if [ -z "${RVIZ_PACKAGE}" ]; then 
    echo "Rviz2 application isn't installed!"
    echo "Stop simulation..."
    exit 1
fi

cd ${CURRENT_DIR} || { echo "Failure"; exit 1; }
clear

ros2 launch ${PACKAGE_NAME} rviz.launch.py
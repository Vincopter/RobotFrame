#!/bin/bash
#
# Run Gazebo with world and without robot_frame
#

which gazebo &> /dev/null
if [ $? -eq 1 ]; then echo "Gazebo not installed!"; exit 1; fi

source /opt/ros/humble/setup.bash
if [ -z "${COLCON_PREFIX_PATH}" ]; then 
    source ~/.bashrc
else
    source ${COLCON_PREFIX_PATH}/setup.bash; 
fi

PACKAGE_NAME="robot_frame"
PROJECT_ROOT=$(source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd ${PACKAGE_NAME} > /dev/null && pwd)
if [ -z "${PROJECT_ROOT}" ]; then 
    echo "Unable to get root '${PACKAGE_NAME}' package folder..."; exit 1
fi

PROJECT_MODELS_PATH="${PROJECT_ROOT}/worlds/models:${PROJECT_ROOT}/robot_frame/description/resources"
echo "Rewrite variable GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH}"

GAZEBO_WORLD_NAME="cube.world"
GAZEBO_WORLD_PATH="${PROJECT_ROOT}/worlds"

(GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH}" gazebo ${GAZEBO_WORLD_PATH}/${GAZEBO_WORLD_NAME} --verbose)

#!/bin/bash
#
# Run Gazebo with world and without robot_frame
#
PROJECT_ROOT="$(cd ../..; pwd)"

which gazebo &> /dev/null
if [ $? -eq 1 ]; then echo "Gazebo not installed!"; exit 1; fi

PROJECT_MODELS_PATH="${PROJECT_ROOT}/worlds/models:${PROJECT_ROOT}/robot_frame/description/resources"
echo "Rewrite variable GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH}"

GAZEBO_WORLD_NAME="cube.world"
GAZEBO_WORLD_PATH="${PROJECT_ROOT}/worlds"

(GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH}" gazebo ${GAZEBO_WORLD_PATH}/${GAZEBO_WORLD_NAME} --verbose)

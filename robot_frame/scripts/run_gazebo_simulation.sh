#!/bin/bash
#
# Run Gazebo simulation of RobotFrame
#
CURRENT_DIR=$('pwd')

INSTALLED=$(dpkg -s ros-humble-desktop)

PACKAGE_NAME="robot_frame"
PACKAGE_DIR=$(source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd $PACKAGE_NAME > /dev/null && pwd)
cd ${CURRENT_DIR} || { echo "Failure"; exit 1; }

ros2 launch $PACKAGE_NAME gazebo.launch.py \
    use_teleop:="True" \
    use_cameras:="False" \
    run_demo:="True" \
    run_rviz:='False' \
    world:="${PACKAGE_DIR}/worlds/cube.world"

 

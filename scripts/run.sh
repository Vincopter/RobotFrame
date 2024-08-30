#!/bin/bash
CURRENT_DIR=$('pwd')
RUN_CONTROL_SHELL_FILE="control_demo.sh"

clear

# ROS1
# Starting control on host
# start_script_as_X "/bin/bash ${CURRENT_DIR}/${RUN_CONTROL_SHELL_FILE}" "CONTROL"
#start_script_as_X "roslaunch xxx_description sitl_run.launch model:='$(find xxx_description)/description/robot.urdf.xacro'" "RVIZ"
#roslaunch xxx_description sitl_run.launch model:='$(find xxx_description)/description/robot.urdf.xacro'
#roslaunch xxx_description gazebo.launch

#ROS2
ros2 launch robot_frame rviz.launch.py
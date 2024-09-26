#!/bin/bash

# CMD AGRS:
# 1 - запуск публикатора и rviz
# 2 - запуск эмуляции газебо и помещение в нее робота (запуск по отдельности)
# 3 - запуск симуляции в газебо чере launch file

cd ~/dev_ws
colcon build --symlink-install --packages-select my_robot

sleep 3

source install/local_setup.bash

start_script() { 
    xterm -xrm 'XTerm.vt100.allowTitleOps: false' -T "${2}" -e "${1}" &
}


if [ $1 -eq 1 ]; then
    start_script 'ros2 launch my_robot rsp.launch.py use_sim_time:=true' "ROBOT LAUNCH"
    start_script 'ros2 run joint_state_publisher_gui joint_state_publisher_gui' "ROBOT CONFIG"
    start_script 'rviz2 -d /home/osin/.rviz2/my_robot.rviz' "RVIZ"
fi

if [ $1 -eq 2 ]; then
    start_script 'ros2 launch my_robot rsp.launch.py use_sim_time:=true' "ROBOT LAUNCH"
    start_script 'ros2 launch gazebo_ros gazebo.launch.py' "GAZEBO"
    # run entity spawn node in gazebo
    start_script 'ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot' "ROBOT TO GAZEBO"
fi

if [ $1 -eq 3 ]; then
    start_script 'ros2 launch my_robot launch_sim.launch.py' "ROBOT LAUNCH"
    #можно загружать свой мир, передав в launch_sim.launch.py world:=path/to/my.world
    #например, ros2 launch my_bot launch_sim.launch.py world:=path/to/my.world

    # управление через клавиатуру - teleop
    start_script 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' "CONTROL"
    # запуск rviz
    start_script 'rviz2 -d /home/osin/.rviz2/my_robot.rviz' "RVIZ"

    # управление через джостик 
    #start_script 'ros2 run teleop_twist_joy teleop_node' "CONTROL1"
    #start_script 'ros2 run joy joy_node' "CONTROL2"
fi
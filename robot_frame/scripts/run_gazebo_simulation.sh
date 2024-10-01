#!/bin/bash
#
# Run Gazebo simulation of RobotFrame
#
CURRENT_DIR=(`pwd`)
PACKAGE_NAME="robot_frame"

source /opt/ros/humble/setup.bash
if [ -z "${COLCON_PREFIX_PATH}" ]; then 
    source ~/.bashrc
else
    source ${COLCON_PREFIX_PATH}/setup.bash; 
fi

PACKAGE_DIR=$(source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd ${PACKAGE_NAME} > /dev/null && pwd)
if [ -z "${PACKAGE_DIR}" ]; then 
    echo "Unable to get package directory [${PACKAGE_NAME}]"
fi

ROS_PACKAGES=$(ros2 pkg list)

pkg_installed() {
    [[ $ROS_PACKAGES =~ (^|[[:space:]])$1($|[[:space:]]) ]] && return 0 || return 1
}

pkg_installed_or_exit() {
    pkg_installed $1
    if [ $? -eq 1 ]; then echo "ROS package '$1' not installed!"; exit 1; fi
}

app_installed() {
    dpkg -s $1 &> /dev/null
    local status=$?
    return $status
}

app_installed "gazebo"
if [ $? -eq 1 ]; then echo "Gazebo not installed!"; exit 1; fi

pkg_installed_or_exit "xacro"
pkg_installed_or_exit "twist_mux"
pkg_installed_or_exit "ros2_control"
pkg_installed_or_exit "ros2_controllers"
pkg_installed_or_exit "gazebo_ros_pkgs"
pkg_installed_or_exit "gazebo_ros2_control"
pkg_installed_or_exit "joint_state_publisher"
pkg_installed_or_exit "gripper_controllers"

app_installed "joystick"
if [ $? -eq 1 ]; then 
    echo "If there are plans to connect a joystick, then corresponding package must be installed!"; exit 1; 
fi

pkg_installed "usb_cam"
if [ $? -eq 1 ]; then 
    echo "If there are plans use cameras, then package 'ros-humble-usb-cam' must be installed!"; exit 1; 
fi

pkg_installed "rviz2"
if [ $? -eq 1 ]; then 
    echo "If you need to run with run_rviz:='True', need to first install Rviz2"; exit 1; 
fi

cd ${CURRENT_DIR} || { echo "Failure"; exit 1; }
#
# Опции запуска:
# use_teleop - использовать управление с клавиатуры
# use_cameras - использовать реальные камеры для подключения к эмуляции (в системе заведены как /dev/video0, /dev/video1)
# run_rviz - запуск дополнительно Rviz2
# run_demo - запуск демонстрации работы манипуляторов после прогрузки симулятора Gazebo
# world - карта мира, иcпользуемая для симуляции
#
ros2 launch ${PACKAGE_NAME} gazebo.launch.py \
    use_teleop:='True' \
    use_cameras:='False' \
    run_rviz:='False' \
    run_demo:='True' \
    world:="${PACKAGE_DIR}/worlds/cube.world"

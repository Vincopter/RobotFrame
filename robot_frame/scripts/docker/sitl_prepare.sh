#!/bin/bash
#
# [RobotFrame] Script of prepare demonstration.
#
CURRENT_DIR=$('pwd')
DOCKER_VOLUME="shared"
HOST_VOLUME_DIR=/${DOCKER_VOLUME}
PACKAGE_NAME="robot_frame"
DEMO_DOCKER_IMAGE="${PACKAGE_NAME}:demo"
DOCKER_FILENAME="CustomDockerfile"

# Создаем директорию для обмена между docker-контейнером в корневом окружении 
# (т.е. также как и в docker-контейнере). Это связано с некоторыми ограничением 
# в раздельном запуске эмулятора Gazebo на хост-сервере. Cтруктура размещения 
# данных робота (директорий/файлов) на хостовом сервере должна быть идентичной 
# структуре в docker-контейнере.
sudo mkdir -p ${HOST_VOLUME_DIR}
PACKAGE_GROUP=${PACKAGE_NAME}
if ! grep -q $PACKAGE_GROUP /etc/group; then 
    bound=1000
    lastId=$(grep -o '[0-9]\+' /etc/group | tail -1)
    if [[ $lastId -lt $bound ]]; then 
        ((lastId += bound))
    else
        ((lastId += ( bound / '100' )))
    fi
    sudo groupadd $PACKAGE_GROUP -g $lastId
fi
sudo gpasswd -a $USER $PACKAGE_GROUP > /dev/null
sudo chmod g+rwx ${HOST_VOLUME_DIR}/
sudo chown $USER:$PACKAGE_GROUP ${HOST_VOLUME_DIR}/

cd ${HOST_VOLUME_DIR} || { echo "Failure"; exit 1; }

(docker volume create $DOCKER_VOLUME)
if [ $? -eq 0 ]; then echo "Docker volume ($DOCKER_VOLUME) has been succesfully created"; fi
 
cat > ${HOST_VOLUME_DIR}/${DOCKER_FILENAME} << EOF
FROM osrf/ros:humble-desktop
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -y \
nano \
less \
xpra \
xterm \
netcat \
gazebo \
libqt5gui5 \
libgazebo11 \
libopencv-dev \
net-tools \
iproute2 \
iputils-ping \
ros-humble-xacro \
python3-argcomplete \
joystick \
ros-dev-tools \
ros-humble-gazebo-ros-pkgs \
ros-humble-joint-state-publisher-gui \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-twist-mux \
ros-humble-controller-manager \
ros-humble-usb-cam \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-gazebo-ros2-control \
ros-humble-gripper-controllers

RUN apt-get install -y openssh-server
RUN mkdir -p /var/run/sshd
RUN mkdir -p ~/.ssh
RUN sed -i 's/\#Port 22/Port 2202/' /etc/ssh/sshd_config
RUN sed -i 's/\#PermitRootLogin prohibit-password/PermitRootLogin without-password/' /etc/ssh/sshd_config
RUN sed -i 's/\#PermitEmptyPasswords no/PermitEmptyPasswords yes/' /etc/ssh/sshd_config
RUN sed -i 's/\#UseDNS no/UseDNS yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
EXPOSE 22
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc  \
&& echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc 
CMD ["bash"]
EOF

( docker build -t $DEMO_DOCKER_IMAGE -f ${HOST_VOLUME_DIR}/${DOCKER_FILENAME} . )
if [ ! $? -eq 0 ]; then 
    echo "Unable to build docker image!"
fi

cd ${HOST_VOLUME_DIR} || { echo "Failure"; exit 1; }
# TEST
#git clone -q https://github.com/Vincopter/RobotFrame.git
git clone -q git@github.com:Vincopter/RobotFrame.git

cd "${CURRENT_DIR}" || { echo "Failure to get current folder"; exit 1; }

echo "Demo preparation has been completed."




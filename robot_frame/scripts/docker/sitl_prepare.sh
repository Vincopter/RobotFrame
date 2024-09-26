#
# [RobotFrame] Script of prepare demonstration.
#
#!/bin/bash
CURRENT_DIR=$('pwd')
DOCKER_VOLUME="shared"
DOCKER_VOLUME_DIR=${CURRENT_DIR}/${DOCKER_VOLUME}
DEMO_DOCKER_IMAGE="robot_frame:demo"
DOCKER_FILENAME="CustomDockerfile"

mkdir -p ${DOCKER_VOLUME_DIR}
sudo chmod u+rwx $DOCKER_VOLUME_DIR/
sudo chown $USER $DOCKER_VOLUME_DIR/

cd $CURRENT_DIR || { echo "Failure"; exit 1; }

(docker volume create $DOCKER_VOLUME)
if [ $? -eq 0 ]; then echo "Docker volume ($DOCKER_VOLUME) has been succesfully created"; fi
 
cat > ${CURRENT_DIR}/${DOCKER_FILENAME} << EOF
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
RUN sed -i 's/\#Port 22/Port 2200/' /etc/ssh/sshd_config
RUN sed -i 's/\#PermitRootLogin prohibit-password/PermitRootLogin without-password/' /etc/ssh/sshd_config
RUN sed -i 's/\#PermitEmptyPasswords no/PermitEmptyPasswords yes/' /etc/ssh/sshd_config
RUN sed -i 's/\#UseDNS no/UseDNS yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
EXPOSE 22
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc  \
&& echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc 
CMD ["bash"]
EOF

( docker build -t $DEMO_DOCKER_IMAGE -f ${CURRENT_DIR}/${DOCKER_FILENAME} . )
if [ ! $? -eq 0 ]; then 
    echo "Unable to build docker image!"
fi

cd ${CURRENT_DIR}/${DOCKER_VOLUME}
#git clone -q https://github.com/Vincopter/RobotFrame.git
git clone -q git@github.com:Vincopter/RobotFrame.git

echo "Demo preparation has been completed."




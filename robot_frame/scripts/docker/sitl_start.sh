#!/bin/bash
#
# [RobotFrame] Start script of demonstration.
#
CURRENT_DIR=$('pwd')
DOCKER_VOLUME="shared"
HOST_SHARED_PATH=/${DOCKER_VOLUME}
PACKAGE_NAME="robot_frame"
PACKAGE_GROUP=${PACKAGE_NAME}
PROJECT_NAME="RobotFrame"
DEMO_DOCKER_IMAGE="${PACKAGE_NAME}:demo"
GEN_SCRIPTS_SIGN="#[{$PROJECT_NAME}] << Generated script >>"

cd ${HOST_SHARED_PATH} || { echo "Invalid host shared path"; exit 1; }

# Configure SSH
DOCKER_KEY_NAME="id_demo"
HOST_SSH_KEYS_DIR="${CURRENT_DIR}/Keys"
DOCKER_SSH_KEYS_DIR="${HOST_SHARED_PATH}/Keys"
mkdir -p "${HOST_SSH_KEYS_DIR}"
cd "${HOST_SSH_KEYS_DIR}" || { echo "Failure to access SSH directory"; exit 1; }
if [ ! -s "${HOST_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}" -o \
     ! -s "${HOST_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}.pub" ]
then
   ssh-keygen -t rsa -b 4096 -f ${DOCKER_KEY_NAME} -C root@$('hostname') -N "" > /dev/null
fi
mkdir -p ${DOCKER_SSH_KEYS_DIR}
cp -f "${HOST_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}.pub" ${DOCKER_SSH_KEYS_DIR}

cd "${CURRENT_DIR}" || { echo "Failure"; exit 1; }

# Working script names
DEMO_RUN_SHELL_FILE="run_demo.sh"
STOP_SIM_SHELL_FILE="sitl_stop.sh"
RUN_CONTROL_SHELL_FILE="run_control_demo.sh"
RUN_DEMO_MANIPULATORS_SHELL_FILE="run_manipulators_demo.sh"

# Service script for stop simulation
cat > "${CURRENT_DIR}/${STOP_SIM_SHELL_FILE}" << EOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
docker container stop gazebo_sim
PPIDx=\$( ps -a -o pid,ppid,cmd | grep "gzclient" | grep -v "grep" | awk '{print \$1}' )
if [[ \$PPIDx != "" ]]; then kill -KILL \${PPIDx}; fi
EOF
sudo chmod +x "${CURRENT_DIR}/${STOP_SIM_SHELL_FILE}"

cd ${HOST_SHARED_PATH} || { echo "Failure"; exit 1; }

# check repository
cd ${PROJECT_NAME} && (git pull)
sudo chown -R ${USER}:${PACKAGE_GROUP} .

# Simulation Runner script in container
lastGroupId=$(getent group ${PACKAGE_GROUP} | cut -d: -f3)
cat > ${HOST_SHARED_PATH}/${DEMO_RUN_SHELL_FILE} << FEOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
# ssh configure
mv -f "${DOCKER_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}.pub" ~/.ssh/authorized_keys
sudo chown \$USER: ~/.ssh/authorized_keys
service ssh restart
WORK_DIR=/${DOCKER_VOLUME}
# permissions configurations
if grep -q ${PACKAGE_GROUP} /etc/group; then sudo groupdel -f ${PACKAGE_GROUP} &> /dev/null; fi
sudo groupadd ${PACKAGE_GROUP} -g ${lastGroupId}
sudo gpasswd -a \$USER ${PACKAGE_GROUP} > /dev/null
sudo chmod -R g+rwx \${WORK_DIR}
sudo chown -R \$USER:${PACKAGE_GROUP} \${WORK_DIR}

# building ROS projects
cd /${DOCKER_VOLUME}/
colcon build --symlink-install --packages-select ${PACKAGE_NAME}
sudo chown -R \$USER:${PACKAGE_GROUP} \${WORK_DIR}/${PROJECT_NAME} \${WORK_DIR}/install \${WORK_DIR}/build \${WORK_DIR}/log
sudo chmod -R g+rwx \${WORK_DIR}/${PROJECT_NAME} \${WORK_DIR}/install \${WORK_DIR}/build \${WORK_DIR}/log
echo "source \${WORK_DIR}/install/setup.bash" >> ~/.bashrc
source install/setup.bash
ros2 launch ${PACKAGE_NAME} gazebo.launch.py gui:='False' use_teleop:='False' run_demo:='False' use_cameras:='False'
FEOF
sudo chmod +x ${HOST_SHARED_PATH}/${DEMO_RUN_SHELL_FILE}
#
# Run docker 
#
docker_hash=$(docker run \
   -d -it --rm -m=4g \
   --name=gazebo_sim \
   --network host \
   --privileged -v /dev/bus/usb:/dev/bus/usb \
   -v ${HOST_SHARED_PATH}:/${DOCKER_VOLUME} \
   ${DEMO_DOCKER_IMAGE} \
   /bin/bash /${DOCKER_VOLUME}/${DEMO_RUN_SHELL_FILE} \
   &)
echo "ID of running Docker container: ${docker_hash}"
sleep 5

# Getting ip data
docker_cont_ip=$(sudo docker inspect --format='{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ${docker_hash})
echo "Ip address of container is ${docker_cont_ip}"

# Script connect to container and run contol
cat > ${HOST_SHARED_PATH}/${RUN_CONTROL_SHELL_FILE} << FEOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
source /opt/ros/humble/setup.bash
source /${DOCKER_VOLUME}/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_keyboard
FEOF
sudo chmod +x ${HOST_SHARED_PATH}/${RUN_CONTROL_SHELL_FILE}

# Inviation on start manipulator's demo 
cat > ${HOST_SHARED_PATH}/${RUN_DEMO_MANIPULATORS_SHELL_FILE} << FEOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
red='\033[0;31m'
noColor='\033[0m'
bold='$(tput bold)'
normal='$(tput sgr0)'
printf "\n\${bold}When Gazebo loads, \${red}press any key\${noColor} to start the demonstration of manipulators...\${normal}"
read -n 1 -s -r -p ""
printf "\n"
clear
source /opt/ros/humble/setup.bash
source /${DOCKER_VOLUME}/install/setup.bash
ros2 run ${PACKAGE_NAME} demonstration.py
FEOF
sudo chmod +x ${HOST_SHARED_PATH}/${RUN_DEMO_MANIPULATORS_SHELL_FILE}

# Parameters:
# 1st - some command
# 2nd - title of x-window
# 3rd - size of x-window in specified style in format: width x height (optional)
start_script_as_X() {
	command=${1}
	tittle=${2}
	winsize=${3}
	windows_style=''
	if [ -n "$winsize" ]; then windows_style='-fn -misc-fixed-medium-r-normal--18-*-*-*-*-*-iso8859-15 -geometry ${winsize}'; fi
	xterm -xrm 'XTerm.vt100.allowTitleOps: false' ${windows_style} -T "${tittle}" -e "${command}" &
}

# Starting empty console
start_script_as_X "docker exec -it ${docker_hash} /bin/bash" "CONSOLE" "150x40"
sleep 3

# Starting control on host
start_script_as_X "docker exec -it ${docker_hash} /bin/bash /${DOCKER_VOLUME}/${RUN_CONTROL_SHELL_FILE}" "CONTROL"
sleep 3

# Starting ssh console
start_script_as_X "ssh -i ${HOST_SSH_KEYS_DIR}/${DOCKER_KEY_NAME} root@$('hostname') -p 2202" "SSH CONSOLE" "150x40"
sleep 3

# Starting gazebo on host
gazebo_port=11345
is_exist_port=$(sudo netstat -tulpn | grep ${gazebo_port} | grep -c gzserver)
if [ ${is_exist_port} -eq 1 ]; then
   PROJECT_ROOT=${HOST_SHARED_PATH}/${PROJECT_NAME}
   PROJECT_MODELS_PATH="${PROJECT_ROOT}/worlds/models"
   echo "Form value of variable GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH}"
   (GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PROJECT_MODELS_PATH} GAZEBO_MASTER_URI=http://localhost:${gazebo_port} nohup gzclient &) &> /dev/null
else
    echo "No specified port ${gazebo_port} for running Gazebo client!"
fi
sleep 3
echo "Gazebo (gzclient) is running, wait for it to load."

# Starting inviation on starting demo
start_script_as_X "docker exec -it ${docker_hash} /bin/bash /${DOCKER_VOLUME}/${RUN_DEMO_MANIPULATORS_SHELL_FILE}" "START DEMO"
# END

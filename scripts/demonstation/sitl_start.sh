#
# [RobotFrame]
# Start script of demonstration.
#
#!/bin/bash
CURRENT_DIR=$('pwd')
DOCKER_VOLUME="shared"
HOST_SHARED_PATH=${CURRENT_DIR}/${DOCKER_VOLUME}
DEMO_DOCKER_IMAGE="robot_frame:demo"
DEMO_PROJECT="RobotFrame"
GEN_SCRIPTS_SIGN="# [RobotFrame] << Generated script >>"

# Configure SSH
DOCKER_KEY_NAME="id_demo"
DOCKER_SSH_KEYS_DIR="${CURRENT_DIR}/Keys"
mkdir -p ${DOCKER_SSH_KEYS_DIR}
cd ${DOCKER_SSH_KEYS_DIR}
if [ ! -s "${DOCKER_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}" -o \
     ! -s "${DOCKER_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}.pub" ]
then
   ssh-keygen -t rsa -b 4096 -f ${DOCKER_KEY_NAME} -C root@$('hostname') -N "" > /dev/null
   # connect to
   # ssh -i Keys/id_demo root@Ubuntu2004 -p 2200
fi
cp -f "${DOCKER_SSH_KEYS_DIR}/${DOCKER_KEY_NAME}.pub" ${HOST_SHARED_PATH}

cd ${CURRENT_DIR} || { echo "Failure"; exit 1; }
# Working scripts
DEMO_RUN_SHELL_FILE="run_demo.sh"
STOP_SIM_SHELL_FILE="sitl_stop.sh"
RUN_CONTROL_SHELL_FILE="run_control_demo.sh"

# Script for stop simulation
cat > ${STOP_SIM_SHELL_FILE} << EOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
docker container stop gazebo_sim
PPIDx=\$( ps -a -o pid,ppid,cmd | grep "gzclient" | grep -v "grep" | awk '{print \$1}' )
if [[ \$PPIDx != "" ]]; then kill -KILL \${PPIDx}; fi
EOF
sudo chmod +x ${STOP_SIM_SHELL_FILE}

cd $HOST_SHARED_PATH || { echo "Failure"; exit 1; }

# check repository
cd ${DEMO_PROJECT} && (git pull)
cd $HOST_SHARED_PATH

# Simulation Runner script in container
cat > ${HOST_SHARED_PATH}/${DEMO_RUN_SHELL_FILE} << FEOF
${GEN_SCRIPTS_SIGN}
#!/bin/bash
# ssh configure
mv -f "/shared/${DOCKER_KEY_NAME}.pub" ~/.ssh/authorized_keys
chown root: ~/.ssh/authorized_keys
service ssh restart

# building ROS projects
mkdir -p ~/dev_ws/src
cd ~/dev_ws
cp -r /shared/RobotFrame src
colcon build --symlink-install
source install/setup.bash
ros2 launch RobotFrame launch_sim.launch.py
FEOF
sudo chmod +x ${HOST_SHARED_PATH}/${DEMO_RUN_SHELL_FILE}

# Run docker 
docker_hash=$(docker run \
   -d -it --rm -m=4g \
   --name=gazebo_sim \
   --network host \
   -v ${HOST_SHARED_PATH}:/${DOCKER_VOLUME} \
   ${DEMO_DOCKER_IMAGE} \
   /bin/bash ${DOCKER_VOLUME}/${DEMO_RUN_SHELL_FILE} \
   &)

echo "ID of running Docker container: ${docker_hash}"
sleep 5

# Getting ip data
docker_cont_ip=$(sudo docker inspect --format='{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ${docker_hash})
echo "Ip address of container is ${docker_cont_ip}"

# ScripConnect to container and run conrtol
cat > ${HOST_SHARED_PATH}/${RUN_CONTROL_SHELL_FILE} << FEOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/dev_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
FEOF
sudo chmod +x ${HOST_SHARED_PATH}/${RUN_CONTROL_SHELL_FILE}

# Parameters:
# 1st - some command
# 2nd - title of x-window
# 3rd - size of x-window in specified style in format: width x height (optional)
start_script_as_X() {
 command=${1}
 tittle=${2}
 winsize=${3}
 if [ ! -z ${winsize} ]; then windows_style = '-fn "-misc-fixed-medium-r-normal--18-*-*-*-*-*-iso8859-15" -geometry ${winsize}'; fi
 xterm -xrm 'XTerm.vt100.allowTitleOps: false' ${windows_style} -T "${tittle}" -e "${command}" &
}

# Starting console
start_script_as_X "ssh -i ${DOCKER_SSH_KEYS_DIR}/${DOCKER_KEY_NAME} root@$('hostname') -p 2200" "SSH CONSOLE" "150x40"
sleep 3

# Starting control on host
start_script_as_X "docker exec -it ${docker_hash} /${DOCKER_VOLUME}/${RUN_CONTROL_SHELL_FILE}" "CONTROL"
sleep 3

# Starting gazebo on host
gazebo_port=11345
is_exist_port=$(sudo netstat -tulpn | grep ${gazebo_port} | grep gzserver | wc -l)
if [ $is_exist_port -eq 1 ]; then
   (GAZEBO_MASTER_URI=http://localhost:${gazebo_port} nohup gzclient &) &> /dev/null
else
    echo 'No specified port ${gazebo_port} for running Gazebo client!'
fi
echo "Gazebo application started."






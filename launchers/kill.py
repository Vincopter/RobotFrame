import os
import subprocess
from launch import LaunchDescription

def getPackageName():
    return 'robot_frame'

def getRealPackageDirPath():
    return os.popen(
        '/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s > /dev/null && pwd"' % getPackageName()).read().strip()

def generate_launch_description():

    ksFilePath = os.path.join(getRealPackageDirPath(), 'launchers/_kill.sh')
    subprocess.run(["bash", ksFilePath])
    return LaunchDescription()


if __name__ == '__main__':
    
    try:

        generate_launch_description()

    except Exception as ex:
        print(f"[ERROR]: {ex}")


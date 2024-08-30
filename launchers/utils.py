import os
from launch_ros import substitutions

def getPackageName():
    return 'robot_frame'
    
def getRealPackageDirPath():
    return os.popen(
        '/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s > /dev/null && pwd"' % getPackageName()).read().strip()

def getPackageShareDir():
    return substitutions.FindPackageShare(package=getPackageName()).find(getPackageName())




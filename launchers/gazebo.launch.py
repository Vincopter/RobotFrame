import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
#from launchers.utils import getPackageShareDir
from launch_ros.actions import Node

def getPackageName():
    return 'robot_frame'

def getPackageShareDir():
    return get_package_share_directory(getPackageName())

def getRealPackageDirPath():
    return os.popen(
        '/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s > /dev/null && pwd"' % getPackageName()).read().strip()

def getLaunchArgs():
    return [
        DeclareLaunchArgument(
            name='run_rviz',
            default_value='False',
            description="Start also Rviz module if True"
        ),
        DeclareLaunchArgument(
            name='use_cameras',
            default_value='False',
            description="Connect to real web-cameras (front and back cameras) if True"
        ),
    ]

def generate_launch_description():
     
    launch_description = LaunchDescription(getLaunchArgs())

    run_rviz = LaunchConfiguration('run_rviz')
    use_cameras = LaunchConfiguration('use_cameras')
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(getPackageShareDir(), 'launchers', 'robot.launch.py')
            ]), 
        launch_arguments={
            'gui': 'True',
            'use_teleop': 'False',
            'use_sim_time': 'True',
            'use_ros2_control': 'True',
        }.items()
    )
    launch_description.add_action(robot)
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(getPackageShareDir(), 'launchers', 'rviz.launch.py')
        ]),
        condition=IfCondition(run_rviz),
        launch_arguments={
            'gui': 'True',
            'use_teleop': 'True',
            'use_sim_time': 'True',
            'run_publisher':'False',
        }.items(),
    )
    launch_description.add_action(rviz)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': os.path.join(getRealPackageDirPath(), 'worlds', 'cube.world'),
        }.items()
    )
    launch_description.add_action(gazebo)

    spawnIntoEnvironment = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'vc_robot'],
        output='screen'
    )
    launch_description.add_action(spawnIntoEnvironment)
    
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(getPackageShareDir(), 'launchers', 'cameras.launch.py')
        ]),
        condition=IfCondition(use_cameras),
        launch_arguments={
            'namespace': 'cameras',
        }.items(),
    )
    launch_description.add_action(cameras)
    
    return launch_description


""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        # [DEBUG]
        generate_launch_description()

    except Exception as ex:
        print(f"[ERROR]: {ex}")

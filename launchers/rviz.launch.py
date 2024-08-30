import os
import launch
import launch_ros

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def getPackageName():
    return 'robot_frame'

def getRealPackageDirPath():
    return os.popen(
        '/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s > /dev/null && pwd"' % getPackageName()).read().strip()

def getPackageShareDir():
    return launch_ros.substitutions.FindPackageShare(package=getPackageName()).find(getPackageName())

def generate_launch_description():
    
    run_publisher = LaunchConfiguration('run_publisher')
    run_publisher_arg = DeclareLaunchArgument(
        'run_publisher',
        default_value='True',
        description="Run robot state\'s publisher if True"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=[
            '-d', [os.path.join(getRealPackageDirPath(), 'rviz', 'environmentx.rviz')],
            '__log_level', ['debug']
        ]
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(getPackageShareDir(), 'launchers', 'robot.launch.py')
            ]), 
        launch_arguments={
            'gui': 'True',
            'use_teleop': 'False',
            'use_sim_time': 'True',
            'use_ros2_control': 'False',
        }.items()
    )
    
    return LaunchDescription([
        run_publisher_arg,
        launch.actions.GroupAction(
               condition = IfCondition(run_publisher),
               actions = [robot]
        ),
        rviz_node
    ])


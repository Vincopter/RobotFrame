import os, sys
import launch

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import utils 

def generate_launch_description():
    
    from utils import ArgumentsType
    run_publisher = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.RUN_PUBLISHER))
    run_publisher_arg = utils.getLaunchArgumentDeclaration(utils.ArgumentsType.RUN_PUBLISHER)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=[
            '-d', [os.path.join(utils.getRealPackageDirPath(), 'rviz', 'environment.rviz')],
            '__log_level', ['debug']
        ]
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'robot.launch.py')
            ]), 
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.USE_GUI): 'True',
            utils.getArgumentNameByType(ArgumentsType.USE_TELEOP): 'False',
            utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): 'True',
            utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL): 'False',
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

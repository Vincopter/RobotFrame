from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def getTeleopPkgName():
    return "teleop_twist_keyboard"

def getTeleopNodeName():
    return "teleop_twist_keyboard"

def generate_launch_description():

    """
    teleop_exec = ExecuteProcess(
        cmd=[[
            'ros2 run ',
            getTeleopPkgName() + " ",
            getTeleopNodeName(),
        ]],
        shell=True
    )
    """
     
    teleop_node = Node(
        package=getTeleopPkgName(),
        executable=getTeleopNodeName(),
        name='teleop',
        namespace='',
        prefix = 'xterm -e',
        output='screen'
    )
    
    return LaunchDescription([
        teleop_node
    ])
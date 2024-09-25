from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart

def getTeleopPkgName():
    return "teleop_twist_keyboard"

def getTeleopNodeName():
    return getTeleopPkgName()

def generate_launch_description():

    teleop_node = Node(
        package=getTeleopPkgName(),
        executable=getTeleopNodeName(),
        name='teleop_node_keyboard',
        prefix = 'xterm -e',
        output='screen',
        remappings=[
            ('/cmd_vel','/cmd_vel_keyboard')
        ]
    )
    
    return LaunchDescription([
        teleop_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=teleop_node,
                on_start=[
                    LogInfo(msg='[FRAME] Teleoperation keyboard is starting')
                ]
            )
        )
    ])
    
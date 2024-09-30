import os, sys

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node 
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction, ExecuteProcess
from launch.event_handlers import OnProcessStart
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import utils
from utils import ArgumentsType

def getTeleopPkgName():
    return "teleop_twist_keyboard"

def getTeleopNodeName():
    return getTeleopPkgName()

def getCustomNodeName():
    return "teleop_node_keyboard"

def getLaunchArgs():
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_DOCKER)
    ]

def getLaunchActions(context):
    lstNodes = []
    use_docker = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_DOCKER)).perform(context)
   
    topicRemapping = ('/cmd_vel', '/cmd_vel_keyboard')
    if use_docker.lower() == 'false':
        lstNodes.append(Node(
            package=getTeleopPkgName(),
            executable=getTeleopNodeName(),
            name=getCustomNodeName(),
            prefix = 'xterm -e',
            output='screen',
            remappings=[topicRemapping]
        ))
        lstNodes.append(RegisterEventHandler(
            OnProcessStart(
                target_action=lstNodes[-1],
                on_start=[
                    LogInfo(msg='[FRAME] Teleoperation keyboard is starting')
                ]
            )
        ))
    else:
        lstNodes.append(ExecuteProcess(
                name=getCustomNodeName(),
                cmd=[
                    FindExecutable(name='ros2'),
                    'run',
                    '{}'.format(getTeleopPkgName()) ,
                    '{}'.format(getTeleopNodeName()),
                    '--ros_args --remap /cmd_vel:=/cmd_vel_keyboard'
                ],
                shell=False,
                emulate_tty=False,
                output='both',
            )
        )
    return lstNodes   

def generate_launch_description():
    
    opaqueFunc = OpaqueFunction(function = getLaunchActions)
    launchDesc = LaunchDescription(getLaunchArgs())
    launchDesc.add_action(opaqueFunc)
    return launchDesc

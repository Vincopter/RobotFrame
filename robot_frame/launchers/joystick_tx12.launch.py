import os
import subprocess

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import LogInfo, RegisterEventHandler, DeclareLaunchArgument
import utils 

def getJoyParamsToYaml(deviceId: str, deviceName: str):
    joyParamsDict = {
        "joy_node": {
            "ros__parameters": {
                "device_id": int(deviceId),
                "device_name": f"'{deviceName}'",
                "deadzone": 0.05,
                "autorepeat_rate": 0.5
            }
        }
    }
    
    fileNamePath = os.path.join(utils.getRobotFrameConfigDir(), 'joy_params.yaml')
    utils.saveYaml(fileNamePath, joyParamsDict)
    return fileNamePath

def getJoyTeleopParamsToYaml():
    joyParamsDict = {
        "teleop_node_joystick": {
            "ros__parameters": {
                "require_enable_button": bool(False),
                "enable_button": 0,
                "enable_turbo_button": 1, # 'D' button
                "axis_linear": {"x": 1},
                "scale_linear": {"x": float(0.7)},
                "scale_linear_turbo": {"x": float(3)},
                "axis_angular": {"yaw": 5},
                "scale_angular": {"yaw": float(0.5)},
                "scale_angular_turbo": {"yaw": float(3)},                
            }
        }
    }

    fileNamePath = os.path.join(utils.getRobotFrameConfigDir(), 'joy_teleop_params.yaml')
    utils.saveYaml(fileNamePath, joyParamsDict)
    return fileNamePath

def getLaunchArgs():
    from utils import ArgumentsType
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_SIM_TIME)
    ]

def generate_launch_description():

    launch_description = LaunchDescription(getLaunchArgs())

    command = 'ros2 run joy joy_enumerate_devices | sed \'3q;d\' | awk -F":" \'{print $1,";",$5}\' | xargs'
    result = subprocess.check_output(command, shell=True, text=True).split(";")
    
    if result and len(result) == 2:
        
        joy_no = result[0].strip()
        joy_name = result[1].strip()
        
        joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[getJoyParamsToYaml(joy_no, joy_name)],
        )
        launch_description.add_action(joy_node)
        launch_description.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=joy_node,
            on_start=[LogInfo(msg='[FRAME] Joystick is found \'{}\' and node action started [deviceID=\'{}\']'.format(
                joy_name,
                joy_no))
            ])
        ))
        
        # checking axes by jstest (jstest-gtk) or evtest
        use_sim_time = LaunchConfiguration(utils.getArgumentNameByType(utils.ArgumentsType.USE_SIM_TIME))
        teleop_node = Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_node_joystick',
                parameters=[
                    getJoyTeleopParamsToYaml(), 
                    { 
                        utils.getArgumentNameByType(utils.ArgumentsType.USE_SIM_TIME): use_sim_time 
                    }
                ],
                remappings=[
                     ('/cmd_vel','/cmd_vel_joystick')
                ]
            )
        launch_description.add_action(teleop_node)
        launch_description.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=teleop_node,
            on_start=[LogInfo(msg='[FRAME] Joystick teleoperation has been started')])
        ))
    
    return launch_description


""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        # [DEBUG]
        generate_launch_description()
        ...

    except Exception as ex:
        print(f"[ERROR]: {ex}")
        

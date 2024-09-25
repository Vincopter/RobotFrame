import re
import os
import ast
import utils

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import LogInfo, RegisterEventHandler, OpaqueFunction
from controllers import ControllerType, getControllerSpecifierByType

def getMultiplexerParamsToYaml(muxDict: dict):
    
    if len(muxDict) == 0:
        return str()
    
    muxParamsDict = {
        "twist_mux": {
            "ros__parameters": {
                "topics": {}
            }
        }
    }

    lstTopics = muxParamsDict["twist_mux"]["ros__parameters"]["topics"]
    
    for k, v in muxDict.items():

        topicName = k
        match = re.match("^cmd_vel_([\w]+$)", topicName, re.IGNORECASE)
        if match:
            topicName = match.group(1)
        else:
            topicName += "_name"
        
        lstTopics[topicName]= {
            "topic": k, 
            "timeout": float(0.5), 
            "priority": int(v)
        }
        
    fileNamePath = os.path.join(utils.getRobotFrameConfigDir(), 'teleop_multiplexer_params.yaml')
    utils.saveYaml(fileNamePath, muxParamsDict)
    return fileNamePath

def getLaunchArgs():
    from utils import ArgumentsType
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_SIM_TIME, 'False'),
        utils.getLaunchArgumentDeclaration(ArgumentsType.TMUX_RULES)
    ]
    
def getLaunchActions(context):
    
    from utils import ArgumentsType
    use_sim_time = LaunchConfiguration(
        utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME)).perform(context)
    dictRules = ast.literal_eval(
        LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.TMUX_RULES)).perform(context))

    tmultiplexer = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            getMultiplexerParamsToYaml(dictRules), 
            { 
                utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): bool(use_sim_time) 
            }
        ],
        remappings=[(
            '/cmd_vel_out',
            '/{}/cmd_vel_unstamped'.format(getControllerSpecifierByType(ControllerType.DRIVE))
        )],
        # [DEBUG]
        # arguments=["--ros-args", "--log-level", "debug"],
        # output={"both": {"own_log"}},
    )
    
    lstNodes = [tmultiplexer]
    lstNodes.append(RegisterEventHandler(OnProcessStart(
        target_action=tmultiplexer,
        on_start=[LogInfo(msg=f'[FRAME] Twist multiplexer has been started [rules=\'{dictRules}\']')])
    ))

    return lstNodes

def generate_launch_description():
    opaqueFunc = OpaqueFunction(function = getLaunchActions)
    launchDesc = LaunchDescription(getLaunchArgs())
    launchDesc.add_action(opaqueFunc)
    return launchDesc

""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        # [DEBUG]
        xRules = "{'cmd_vel_navigation': '10', 'cmd_vel_joystick':'30', 'cmd_vel_keyboard':'100'}"
        dictRules = ast.literal_eval(xRules)
        yamlParamsFilePath = getMultiplexerParamsToYaml(dictRules)
        ...

    except Exception as ex:
        print(f"[ERROR]: {ex}")
        

import re
import xacro
import utils, controllers

import xml.etree.ElementTree as ET
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def convertGazeboColor(xmlBuffer):
    root = ET.fromstring(xmlBuffer)
        
    for linkElem in root.iter('link'):
        linkName = linkElem.get('name')
        
        linkMaterial = linkElem.find('visual/material')        
        if linkMaterial is None:
            continue
        
        linkMaterialName = linkMaterial.get("name")
        if linkMaterialName == "":
            continue
        
        gazeboElem = root.find("gazebo/[@reference='{}']".format(linkName))
        if gazeboElem is None:
            gazeboElem = ET.Element("gazebo")
            gazeboElem.set('reference', linkName) 
            root.append(gazeboElem)
        else: 
            if not gazeboElem.find("*/material") is None:
                continue
            ...
            
        if not gazeboElem is None:
            materialElem = ET.Element("material")
            materialElem.text = str("Gazebo/" + re.sub('One$', '', linkMaterialName, flags=0))
            gazeboElem.append(materialElem)

    return ET.tostring(root).decode()
    
def getLaunchArgs(urdfModelFilePath: str):
    from utils import ArgumentsType
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_GUI),
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_SIM_TIME),
        utils.getLaunchArgumentDeclaration(ArgumentsType.MODEL_SCHEMA, urdfModelFilePath),        
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_TELEOP),
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_ROS2_CONTROL, 'False'),
    ]
  
def getLaunchActions(context):

    from utils import ArgumentsType
    use_ros2_control = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL)).perform(context)
    
    robot_description = convertGazeboColor(xacro.process_file(
        utils.getRobotUrdfFilePath(), 
        mappings={
            utils.getArgumentNameByType(ArgumentsType.RESOURCES_DIR): 
                utils.getRobotResourceShareDir(),
            utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL): 
                use_ros2_control
        }
    ).toxml())
    
    if use_ros2_control.lower() == 'true':
        robot_description = controllers.configureGazeboROS2Control(robot_description)
        
    utils.saveXml(
        utils.getRobotDescXmlFilePath(),
        robot_description
    )
        
    use_sim_time = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME))
  
    params = {
        utils.getArgumentNameByType(ArgumentsType.ROBOT_DESCRIPTION): robot_description, 
        utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): use_sim_time
    }

    lstNodes = []
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    lstNodes.append(robot_state_publisher_node)
    
    if use_ros2_control.lower() != 'true':
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration(
                utils.getArgumentNameByType(utils.ArgumentsType.USE_GUI)
            ))
        )
        lstNodes.append(joint_state_publisher_gui_node)
        
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[params],
            condition=UnlessCondition(LaunchConfiguration(
                utils.getArgumentNameByType(utils.ArgumentsType.USE_GUI)
            ))
        )
        lstNodes.append(joint_state_publisher_node)
    
    use_teleop = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_TELEOP)).perform(context)
    if use_teleop.lower() == 'true':
        lstNodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([utils.getLaunchShareDir(), 'teleop.launch.py'
                ])
            ])
        ))

    return lstNodes   
  
def generate_launch_description():
    
    opaqueFunc = OpaqueFunction(function = getLaunchActions)
    launchDesc = LaunchDescription(getLaunchArgs(utils.getRobotUrdfFilePath()))
    launchDesc.add_action(opaqueFunc)
    return launchDesc

""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        
        # [DEBUG]
        ...

    except Exception as ex:
        print(f"[ERROR]: {ex}")


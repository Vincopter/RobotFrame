import os
import re
import xacro
import launch_ros

from pathlib import Path
import xml.etree.ElementTree as ET
from launch_ros.actions import Node
from launch_ros import substitutions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

def getPackageName():
    return 'robot_frame'

def getPackageShareDir():
    return substitutions.FindPackageShare(package=getPackageName()).find(getPackageName())

def getResourceDir():
    for p in Path(getPackageShareDir()).glob('**/resources'):
        if p.is_dir():
            return os.path.realpath(p)
    return str()

def getUrdfFilePath():
    return os.path.join(getPackageShareDir(), 'description/robot.urdf.xacro')

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
        elif not gazeboElem.find("material") is None:
            continue        
         
        if not gazeboElem is None:
            materialElem = ET.Element("material")
            materialElem.text = str("Gazebo/" + re.sub('One$', '', linkMaterialName, flags=0))
            gazeboElem.append(materialElem)

    return ET.tostring(root).decode()
    

def getLaunchArgs(urdfModelFilePath: str):
    return [
        DeclareLaunchArgument(
            name='gui', 
            default_value='False',
            description='This is a flag for joint_state_publisher_gui'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'),
        
        DeclareLaunchArgument(
            name='model', 
            default_value=urdfModelFilePath,
            description='Path to URDF file'),
        
        DeclareLaunchArgument(
            name='use_teleop', 
            default_value='False',
            description='Create teleop node if True'
            ),
        DeclareLaunchArgument(
            name='use_ros2_control', 
            default_value='False',
            description='Using of ROS2 controlling by specify plugin'
        ),
    ]
  
def getLaunchActions(context):
    
    robot_description = convertGazeboColor(xacro.process_file(
        getUrdfFilePath(), 
        mappings={'meshDir' : getResourceDir()}
        ).toxml())
    
    use_sim_time = LaunchConfiguration('use_sim_time')
  
    params = {
        'robot_description': robot_description, 
        'use_sim_time': use_sim_time
    }

    lstNodes = []
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    lstNodes.append(robot_state_publisher_node)
    
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)
    if use_ros2_control.lower() != 'true':
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        )
        lstNodes.append(joint_state_publisher_gui_node)
        
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[params],
            condition=UnlessCondition(LaunchConfiguration('gui'))
        )
        lstNodes.append(joint_state_publisher_node)
    
    use_teleop = LaunchConfiguration('use_teleop').perform(context)
    if use_teleop.lower() == 'true':
        lstNodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    launch_ros.substitutions.FindPackageShare(getPackageName()),
                    'launchers/teleop.launch.py'
                ])
            ])
        ))
    return lstNodes   
  
def generate_launch_description():
    
    opaqueFunc = OpaqueFunction(function = getLaunchActions)
    launchDesc = LaunchDescription(getLaunchArgs(getUrdfFilePath()))
    launchDesc.add_action(opaqueFunc)
    return launchDesc

""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        # [DEBUG]
        #generate_launch_description()
            
        with open("corrected.xml", "w") as file:
            file.write(convertGazeboColor(xacro.process_file(
                getUrdfFilePath(), 
                mappings={'meshDir' : getResourceDir()}).toxml()))

    except Exception as ex:
        print(f"[ERROR]: {ex}")


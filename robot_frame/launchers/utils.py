import os
from enum import Enum
from yaml import dump
from pathlib import Path
from launch_ros import substitutions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

class ArgumentsType(Enum):
    UNDEFINED = 0
    NAMESPACE = 1
    RUN_RVIZ = 2
    USE_CAMERAS = 3
    USE_TELEOP = 4
    USE_ROS2_CONTROL = 5
    RUN_DEMO = 6
    USE_SIM_TIME = 7
    TMUX_RULES = 8
    USE_GUI = 9
    MODEL_SCHEMA = 10
    RUN_PUBLISHER = 11
    PARAMS_YAML = 12
    WORLD_FILE = 13
    RESOURCES_DIR = 14
    ROBOT_DESCRIPTION = 15

argumentsDescByType = {
    # format: name arg, default value, description
    ArgumentsType.NAMESPACE: ['namespace', "", "ROS namespace name for cameras"],
    ArgumentsType.RUN_RVIZ: ['run_rviz', "False", "Start also Rviz module if True"],
    ArgumentsType.USE_CAMERAS: ['use_cameras','False', "Connect to real web-cameras (front and back cameras) if True"],
    ArgumentsType.USE_TELEOP: ['use_teleop', 'False', 'Create teleop node if True'],
    ArgumentsType.USE_ROS2_CONTROL: ['use_ros2_control', 'True', 'Using of ROS2 controlling by specify plugin'],
    ArgumentsType.RUN_DEMO: ['run_demo', 'False', "Run RobotFrame's demonstration if True (launch demo will be in 10 seconds)"],
    ArgumentsType.USE_SIM_TIME: ['use_sim_time', 'True', 'Use simulation time'],
    ArgumentsType.TMUX_RULES: ['tmux_rules', '', 'In the parameter need to specify list of nodes and their priorities. '\
                'Format: "{cmd_vel_<topic_trait>:priority}"'],
    ArgumentsType.USE_GUI: ['gui', 'False', 'This is a flag for joint_state_publisher_gui'],
    ArgumentsType.MODEL_SCHEMA: ['model', '', "Path to URDF file (robot's description)"],
    ArgumentsType.RUN_PUBLISHER: ['run_publisher', 'True', "Run robot state's publisher if True"],
    ArgumentsType.PARAMS_YAML: ['params_file', '', "Gazebo parameters in YAML-format"],
    ArgumentsType.WORLD_FILE: ['world', '', "Gazebo world file (SDF-file)"],
    ArgumentsType.RESOURCES_DIR: ['resDir', '', "Directory with resources (models, materials) for applying to robot (*.dae, *.png)"],
    ArgumentsType.ROBOT_DESCRIPTION: ['robot_description', '', "Robot's description file (xacro-format)"],
}

class SideType(Enum):
    UNDEFINED = 0
    LEFT = 1
    RIGHT = 2
    FRONT = 3
    BACK = 4

sideSpecifierByType = {
    SideType.LEFT: 'left',
    SideType.RIGHT: 'right',
    SideType.FRONT: 'front',
    SideType.BACK: 'back',
}

#
# /** ENUM HELPERS **/
# /*@{*/
#
def getSideTypeByName(side: str) -> SideType:
    """ Return side type by specifier
    """
    try:            
        if len(side) == 0:
            raise ValueError("Invalid side specifier")
        
        for _, (key, value) in enumerate(sideSpecifierByType.items()):
            if side.lower() == value:
                return key
    except ValueError as _:
        ...
    return SideType.UNDEFINED

def getSideNameByType(side: SideType) -> str:
    """ Return side specifier by type
    """
    try:
        side_ = SideType(side)
        if side_ in SideType:
            return sideSpecifierByType[side_]
    except (KeyError, ValueError) as _:
        ...
    return str()


def getArgumentNameByType(argType: ArgumentsType):
    try:
        if ArgumentsType(argType) in ArgumentsType:
            return argumentsDescByType[ArgumentsType(argType)][0]
    except (KeyError, ValueError) as _:
        ...
    return str()

def getLaunchArgumentDeclaration(argType: ArgumentsType, defaultValue = None):
    try:
        if ArgumentsType(argType) in ArgumentsType:
            
            argDesc = argumentsDescByType[ArgumentsType(argType)]
            assert not argDesc is None

            if defaultValue is None:
                defaultValue = argDesc[1]
                
            return DeclareLaunchArgument(
                name=argDesc[0], 
                default_value=defaultValue, 
                description=argDesc[2]
            )
    except (KeyError, ValueError) as _:
        ...
    return None
#
# /*@}*/
#

def getPackageName():
    return 'robot_frame'

def getRobotDescXmlFileName():
    return 'robot_description.xml'

def getBasisLinkName():
    """ Base link name by default
    """
    return 'base_link'

def getRobotFrameDescTopicName():
    """ Entity xml published on topic
    """
    return 'robot_description'

def getRobotFrameNameToSpawn():
    """ Name of entity to spawn
    """
    return 'vc_robot'

def getRobotDescXmlFilePath():
    return os.path.join(getRealRobotDescDir(), getRobotDescXmlFileName())

def getRobotUrdfFilePath():
    """ Descriptio in URDF format
    """
    return os.path.join(getRobotDescShareDir(), 'robot.urdf.xacro')

def getRealPackageDirPath():
    return os.popen(
        '/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && \
        colcon_cd %s > /dev/null && pwd"' % getPackageName()).read().strip()
    
def getRealRobotFrameDirPath():
    return os.path.join(getRealPackageDirPath(), getPackageName())

def getPackageShareDir():
    return substitutions.FindPackageShare(package=getPackageName()).find(getPackageName())

def getLaunchShareDir():
    return os.path.join(getPackageShareDir(), "launchers")

def getRobotFrameConfigDir():
    return os.path.join(getRealRobotFrameDirPath(), "configs")

def saveYaml(filename: str, content: dict):
    yamlContainer = dump(content, default_style='"', sort_keys=False)
    yamlContainer = yamlContainer.replace("\"", "").replace("'", '"')
    with open(filename, encoding='utf8', mode='w') as fl:
        fl.write('# [GENERATED YAML-FILE]\n')
        fl.write(yamlContainer)

def saveXml(filename: str, content: str):
    from lxml import etree
    root = etree.fromstring(content)
    root.addprevious(etree.Comment('[GENERATED XML-FILE]'))
    etree.ElementTree(root).write(
        filename,
        pretty_print=True,
        encoding="UTF-8",
        xml_declaration=True)

def getRealRobotDescDir():
    return os.path.join(getRealRobotFrameDirPath(), "description")

def getRobotDescShareDir():
    return os.path.join(getPackageShareDir(), "description")

def getRobotResourceShareDir():
    for p in Path(getRobotDescShareDir()).glob('**/resources'):
        if p.is_dir():
            return os.path.realpath(p)
    return str()

def getWorldsMaterialDir():
    return os.path.join(getRealPackageDirPath(), "worlds", "models")

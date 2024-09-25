#!/usr/bin/env python3
import os, xacro, datetime, copy, sys
import xml.etree.ElementTree as ET

from enum import Enum
from rclpy.impl import rcutils_logger
from launch import LaunchDescription
from launch_ros.actions import Node 
from rclpy.impl.logging_severity import LoggingSeverity
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction, RegisterEventHandler, LogInfo, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import utils
from utils import ArgumentsType, SideType

def getControllersConfigFileName():
    return "controllers.yaml"

def getControllersConfigFilePath():
    return os.path.join(utils.getRobotFrameConfigDir(), getControllersConfigFileName())

class ControllerType(Enum):
    UNDEFINED = 0
    DRIVE = 1
    JOINTS = 2
    MANIPULATOR = 3
    GRIPPER = 4
    GRIPPER_ACTION = 5
    GRIPPER_MIMIC = 6
    COVER = 7

def getControllerSpecifierByType(ctrlType: ControllerType) -> str:
    return CManager().getControllerSpecifierByType(ctrlType)

def getControllerTypeBySpecifier(specName: str) -> ControllerType:
    return CManager().getControllerTypeBySpecifier(specName)

def getRosDriverByControllerType(ctrlType: ControllerType) -> str:
    return CManager().getRosDriverNameByControllerType(ctrlType)

def getControllerTypeByRosDriverName(driver: str) -> ControllerType:
    return CManager().getControllerTypeByRosDriverName(driver)

def getRobotControllerXmlTag():
    # Кастомный тег для определения соединений в контроллерах на уровне URDF файла
    # формат заполнения тега: <vc_robot_controller type="..." side="..."/>
    # где:
    # type -  тип контроллера который будет управлять соединением, 
    #         из набора спецификаторв по типу из ControllerType
    # side -  направление/позиция соединения, для понимания к какому контроллеру 
    #         относится соединение (как правило, обозначает стороны изделия SideType)
    #
    return "vc_robot_controller"

class ControlLoadStates:
    def __init__(self):
        self.__isLoaded = {
            ControllerType.DRIVE: False,
            ControllerType.JOINTS: False,
            ControllerType.MANIPULATOR : False,
            ControllerType.GRIPPER: False,
            ControllerType.COVER: False,
        }
    
    def setState(self, ctrlType: ControllerType, state: bool):
        try:
            if ControllerType(ctrlType) in ControllerType:
                self.__isLoaded[ControllerType(ctrlType)] = bool(state)
        except (KeyError, ValueError) as _:
            pass
    
    def getState(self, ctrlType: ControllerType) -> bool:
        try:
            if ControllerType(ctrlType) in ControllerType:
                return bool(self.__isLoaded[ControllerType(ctrlType)])
        except (KeyError, ValueError) as _:
            pass
    
    def isAllLoaded(self) -> bool:
        for type_, value_ in self.__isLoaded.items():
            if bool(value_) == False:
                return False
        return True

class CManager:
    """ Класс-конфигуратор для инициализации 
        необходимых ROS2-контроллеров при launch-запуске, 
        в том числе создание файла конфигурации в yaml-формате.        
    """  
    def __init__(self):
        self.__dictRobotJointsByControllerTypes = dict()
        self.__logger = rcutils_logger.RcutilsLogger("CManager")
        self.__dictControllersByType = {
            ControllerType.DRIVE: "differential_drive",
            ControllerType.JOINTS: "joints_broadcaster",
            ControllerType.GRIPPER_MIMIC: "gripper_mimic",
            ControllerType.GRIPPER_ACTION: "gripper_action",
            ControllerType.MANIPULATOR: "manipulators",
            ControllerType.GRIPPER: "grippers",
            ControllerType.COVER: "covers",
        }
        self.__dictRosDriverByControllerType = {
            ControllerType.DRIVE : "diff_drive_controller/DiffDriveController",
            ControllerType.JOINTS : "joint_state_broadcaster/JointStateBroadcaster",
            ControllerType.GRIPPER_ACTION : "position_controllers/GripperActionController",
            ControllerType.MANIPULATOR : "joint_trajectory_controller/JointTrajectoryController",
            ControllerType.GRIPPER : "joint_trajectory_controller/JointTrajectoryController",
            ControllerType.COVER : "joint_trajectory_controller/JointTrajectoryController",
        }
        
    def logError(self, message):
        self.__logger.log("[{}] {}.".format(
                datetime.datetime.now().time(),
                message
            ),
            severity=LoggingSeverity.ERROR
        )

    def getControllerSpecifierByType(self, ctrlType: ControllerType) -> str:
        """ Return controller specifier by type
        """
        try:
            ctrlType_ = ControllerType(ctrlType)
            if ctrlType_ in ControllerType:
                return "{}_controller".format(self.__dictControllersByType[ctrlType_])
        except (KeyError, ValueError) as _:
            ...
        return str()
    
    def getControllerTypeBySpecifier(self, specName: str):
        """ Return controller type by string specifier 
        """
        try:            
            if len(specName) == 0:
                raise ValueError("Invalid controller specifier")
            
            for _, (key, value) in enumerate(self.__dictControllersByType.items()):
                if specName.lower() == value:
                    return key
        except ValueError as _:
            ...
        return ControllerType.UNDEFINED
    
    def getRosDriverNameByControllerType(self, ctrlType: ControllerType) -> str:
        """ Return ROS driver name for controller by type
        """
        try:
            ctrlType_ = ControllerType(ctrlType)
            if not ctrlType_ in ControllerType:
                raise KeyError("Invalid controller type")

            if ctrlType_ in self.__dictRosDriverByControllerType:
                return self.__dictRosDriverByControllerType[ctrlType]
            else:
                self.logError(
                    "Unable to find ROS driver for controller: '{}'".format(
                        self.getControllerSpecifierByType(ctrlType))
                )                
        except KeyError as _:
            ...
        return str()
    
    def getControllerTypeByRosDriverName(self, driver: str) -> ControllerType:
        if not driver is None and len(driver) > 0:
            for type_, name_ in self.__dictRosDriverByControllerType.items():
                if name_ == driver:
                    return type_
            
        return ControllerType.UNDEFINED

    def getSpawnedControllersTypeList(self):
        return [
            ControllerType.DRIVE,
            ControllerType.GRIPPER,
            ControllerType.MANIPULATOR,
            ControllerType.COVER,
            ControllerType.JOINTS,
        ]
    
    def getSpawnedControllersSpecList(self):
        return [ self.getControllerSpecifierByType(type_) for type_ in self.getSpawnedControllersTypeList() ]
    
    def setJointControllerByType(self, jointName: str, ctrlTypeName: str, sideName: str):
        try:
            if len(jointName) == 0:
                raise ValueError("Invalid joint name")
            
            ctrlType = self.getControllerTypeBySpecifier(ctrlTypeName)
            if ctrlType == ControllerType.UNDEFINED:
                raise ValueError("Unknown controller type [%s]"%ctrlTypeName)

            sideType = SideType.UNDEFINED
            if not sideName is None and len(sideName) > 0:
                sideType = utils.getSideTypeByName(sideName)
                if sideType == SideType.UNDEFINED:
                    raise ValueError("Unknown side [%s]"%sideName)

            self.__dictRobotJointsByControllerTypes.setdefault(ctrlType, []).append(
                tuple([jointName, sideType])
            )        
        except ValueError as ex:
            self.logError(
                "Add of joint element is failed: '{}'".format(ex)
            )

    def isAnyJointsData(self):
        return len(self.__dictRobotJointsByControllerTypes) > 0
    
    def getJointDataByControllerType(self, ctrlType: ControllerType):
        try:
            
            if not ControllerType(ctrlType) in ControllerType:
                raise KeyError("Unknown controller type")
            
            if ctrlType in self.__dictRobotJointsByControllerTypes:
                raise ValueError("No any data for controller type [%d]"%int(ctrlType))
            
            return self.__dictRobotJointsByControllerTypes[ctrlType]
            
        except (KeyError, ValueError) as ex:
            self.logError(
                "Unable to get joint data: '{}'".format(ex)
            )
        return None        
    
    def fetchRobotControllers(self):
        try:        
            xmlRobotDescription = xacro.process_file(
                utils.getRobotUrdfFilePath(), 
                mappings={
                    utils.getArgumentNameByType(ArgumentsType.RESOURCES_DIR): 
                        utils.getRobotResourceShareDir(),
                    utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL): 
                        'True'
                }
            ).toxml()
            
            xmlRoot = ET.fromstring(xmlRobotDescription)
            
            for jointElem in xmlRoot.findall('./joint[@type]'):
                jointName = jointElem.get('name')
                ctrlElems = jointElem.findall('./{}'.format(getRobotControllerXmlTag()))
                
                for ctrlElem in ctrlElems:
                    if not ctrlElem is None:
                        self.setJointControllerByType(
                            jointName, 
                            ctrlElem.get('type'),
                            ctrlElem.get('side')
                        )
            
            if not self.isAnyJointsData():
                raise Exception("Joints data does not fetched!")
        
        except Exception as ex:
            self.logError("Unable to fetch of controller's tag in URDF. {}.".format(ex))
            return False
        
        return True
    
    def createControllerConfig(self):
        
        if not self.fetchRobotControllers():
            return False

        yamlControllersConfig = {
            "controller_manager": {
                "ros__parameters": {
                    "update_rate": int(30),
                    "use_sim_time": bool(True),
                }
            }
        }
        
        yamlConfigByControllerDict = {            
            ControllerType.DRIVE: {
                "publish_rate": float(30),
                "base_frame_id": utils.getBasisLinkName(),
                "wheel_separation": 0.485,
                "wheel_radius": 0.0923,
                "use_stamped_vel": bool(False),
                "left_wheel_names": [],
                "right_wheel_names": []
            },
            ControllerType.GRIPPER: {
                "joints": [],
                "command_interfaces": ["position"],
                "state_interfaces": ["position"],
                "open_loop_control": bool(False),
                "allow_partial_joints_goal": bool(True),
                "allow_integration_in_goal_trajectories": bool(True),
                "allow_nonzero_velocity_at_trajectory_end": bool(False),
            },
            ControllerType.MANIPULATOR: {
                "joints": [],
                "command_interfaces": ["position"],
                "state_interfaces": ["position"],
                "open_loop_control": bool(True),
                "allow_partial_joints_goal": bool(True),
                "allow_integration_in_goal_trajectories": bool(False),
            },
            ControllerType.COVER: {
                "joints": [],
                "command_interfaces": ["position"],
                "state_interfaces": ["position"],
                "open_loop_control": bool(True),
                "allow_partial_joints_goal": bool(True),
                "allow_integration_in_goal_trajectories": bool(False),                
            },
            ControllerType.JOINTS: {
                "use_local_topics": bool(False),
            },
            ControllerType.GRIPPER_ACTION: {
                "joint": str(),
                "action_monitor_rate": float(20),
                "goal_tolerance": float(0.01),
                "max_effort": float(900),
                "allow_stalling": bool(False),
                "stall_velocity_threshold": float(0.001),
                "stall_timeout": float(1),
            }
        }
        
        try:

            def createConfig(ctrlType: ControllerType, ctrlSpec: str, jointsData: list = None):
                
                nonlocal yamlControllersConfig                
                yamlHeaderConfig = yamlControllersConfig["controller_manager"]["ros__parameters"]
                yamlHeaderConfig[ctrlSpec] = {
                    "type": self.getRosDriverNameByControllerType(ctrlType)
                }

                yamlControllersConfig[ctrlSpec] = { 
                    'ros__parameters': {} 
                }
                
                def getJoints(lstJoint: list, side: SideType = SideType.UNDEFINED) -> list:
                    if side == None or side == SideType.UNDEFINED:
                        return [j[0] for j in lstJoint]
                    
                    lstJointRes = []
                    for joint in lstJoint:
                        if joint[1] == side:
                            lstJointRes.append(joint[0])
                        
                    return lstJointRes

                try:
                    if not ctrlType in yamlConfigByControllerDict:
                        raise ValueError("No definition for '%s' controller"%self.getControllerSpecifierByType(ctrlType))

                    yamlControllersConfig[ctrlSpec]['ros__parameters'] = copy.deepcopy(yamlConfigByControllerDict[ctrlType])
                    
                    if not jointsData is None and len(jointsData) > 0:
                        
                        controllerDetails = yamlControllersConfig[ctrlSpec]['ros__parameters']
                        
                        if 'joint' in controllerDetails:
                            controllerDetails['joint'] = getJoints(jointsData)[0]
                        elif 'joints' in controllerDetails:
                            controllerDetails['joints'] = getJoints(jointsData)
                        elif 'left_wheel_names' in controllerDetails and \
                            'right_wheel_names' in controllerDetails:
                            controllerDetails['left_wheel_names'] = getJoints(jointsData, SideType.LEFT)
                            controllerDetails['right_wheel_names'] = getJoints(jointsData, SideType.RIGHT)

                except ValueError as ex:
                    self.logError(
                        "Unable to configure controller's yaml-file by cause: '{}'.".format(ex)
                    )
                
            for ctrlType, ctrlData in self.__dictRobotJointsByControllerTypes.items():
                
                if ctrlType == ControllerType.GRIPPER_MIMIC:
                    # configuration of this controller will be skipped
                    continue 
                if ctrlType == ControllerType.GRIPPER_ACTION:
                    for ctrlElem in ctrlData:
                        side = ctrlElem[1]
                        ctrlSpec = "{}_{}".format(
                            utils.getSideNameByType(side), 
                            self.getControllerSpecifierByType(ctrlType)
                        )
                        createConfig(ctrlType, ctrlSpec, [ctrlElem])
                else:
                    createConfig(
                        ctrlType, 
                        self.getControllerSpecifierByType(ctrlType), 
                        ctrlData
                    )
                
                createConfig(
                    ControllerType.JOINTS, 
                    self.getControllerSpecifierByType(ControllerType.JOINTS)
                )
           
            # Save controller config
            utils.saveYaml(getControllersConfigFilePath(), yamlControllersConfig)

        except Exception as ex:
            self.logError("Unable to create of controller config by cause: '{}'.".format(ex))
            return False

        return True

def generateRos2ControllerConfig():
    if not CManager().createControllerConfig():
        raise Exception("Unable to create controller's config file [%s]"%getControllersConfigFileName())

class ControllerNode(Node):
    def __init__(self, ctrlType: ControllerType):
        assert ctrlType != ControllerType.UNDEFINED
        super().__init__(
            name="{}_node".format(getControllerSpecifierByType(ctrlType)),
            package="controller_manager",
            executable="spawner",
            arguments=[getControllerSpecifierByType(ctrlType)],
            # [DEBUG]
            #arguments=[getControllerSpecifierByType(ctrlType), "--ros-args", "--log-level", "{}:=DEBUG".format(getControllerSpecifierByType(ctrlType))],
            #output={"both": {"own_log"}},
        )
        self.__type = ctrlType

    def getType(self):
        return self.__type
 
diff_drive_loaded_message = "Configured and activated differential_drive_controller"

def generate_launch_description():
    
    try:

        launchDescription = LaunchDescription()
        run_demo = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.RUN_DEMO))
        launchDescription.add_action(
            utils.getLaunchArgumentDeclaration(ArgumentsType.RUN_DEMO))

        controllerManager = CManager()
        if not os.path.exists(getControllersConfigFilePath()):
            if not controllerManager.createControllerConfig():
                raise Exception("Unable to create controller's config file [%s]"%getControllersConfigFileName())

        controlNodes = []
        for ctrlType in controllerManager.getSpawnedControllersTypeList():
            node = ControllerNode(ctrlType)
            controlNodes.append(node)
       
        spawnControlNodesGroup = GroupAction(controlNodes)
        launchDescription.add_action(spawnControlNodesGroup)
       
        runObstacleController = ExecuteProcess(
            name="run_obstacle_controller",
            cmd=[
                FindExecutable(name='ros2'),
                ' run ',
                utils.getPackageName(),
                ' obstacle_controller.py ',
                ' --config={} '.format(getControllersConfigFilePath()),
                # [DEBUG]
                # ' --verbose '
            ],
            shell=True
        )
        launchDescription.add_action(runObstacleController)

        runDemonstration = ExecuteProcess(
            name="demonstration",
            condition=IfCondition(run_demo),
            cmd=[
                FindExecutable(name='ros2'),
                ' run ',
                utils.getPackageName(),
                ' demonstration.py',
            ],
            shell=True
        )
        
        launchDescription.add_entity(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=runObstacleController,
                    on_start=[
                        LogInfo(msg='[FRAME] The demonstration\'s node will be start in 10 seconds...'),
                        TimerAction(
                            period=10.0,
                            actions=[runDemonstration],
                        )
                    ]
                )
            )
        )
        
        return launchDescription    

    except Exception as ex:
        print(f"Unable launch of controller manager: '{ex}'")
        return None

def configureGazeboROS2Control(xmlRobotDesc):

    xmlRoot = ET.fromstring(xmlRobotDesc)
    try:        

        # fill node "gazebo_ros2_control"
        gazeboControlPluginNode = xmlRoot.find('./gazebo/plugin[@name="gazebo_ros2_control"]')
        if gazeboControlPluginNode is None:
            gazeboNode = xmlRoot.find("./gazebo")
            if gazeboNode is None:
                gazeboNode = ET.SubElement(xmlRoot, 'gazebo')
                pluginNode = ET.SubElement(gazeboNode, 'plugin')
                pluginNode.set('name', 'gazebo_ros2_control')
                pluginNode.set('filename', 'libgazebo_ros2_control.so')
                gazeboControlPluginNode = pluginNode

        if not gazeboControlPluginNode is None:
            
            robotParam = gazeboControlPluginNode.find("./robot_param")
            if robotParam is None:
                robotParam = ET.SubElement(gazeboControlPluginNode, 'robot_param')
            robotParam.text = utils.getRobotFrameDescTopicName()
                
            robotParamNode = gazeboControlPluginNode.find("./robot_param_node") 
            if robotParamNode is None:
                robotParamNode = ET.SubElement(gazeboControlPluginNode, 'robot_param_node')
            # default publisher node name
            robotParamNode.text = 'robot_state_publisher'
            
            parametersNode = gazeboControlPluginNode.find("./parameters") 
            if parametersNode is None:
                parametersNode = ET.SubElement(gazeboControlPluginNode, 'parameters')
            # path to config controller file in yaml-format
            parametersNode.text = getControllersConfigFilePath()

        # fill node "gazebo_ros_joint_state_publisher"
        gazeboJointPluginNode = xmlRoot.find('./gazebo/plugin[@name="gazebo_ros_joint_state_publisher"]')
        if gazeboJointPluginNode is None:
            gazeboNode = xmlRoot.find("./gazebo")
            if gazeboNode is None:
                gazeboNode = ET.SubElement(xmlRoot, 'gazebo')
                pluginNode = ET.SubElement(gazeboNode, 'plugin')
                pluginNode.set('name', 'gazebo_ros_joint_state_publisher')
                pluginNode.set('filename', 'libgazebo_ros_joint_state_publisher.so')
                gazeboJointPluginNode = pluginNode

        if not gazeboJointPluginNode is None:
        
            lstJoints = []
            for jointElem in xmlRoot.findall('./joint[@type]'):
                ctrlElems = jointElem.findall('./{}'.format(getRobotControllerXmlTag()))
                for ctrlElem in ctrlElems:
                    if not ctrlElem is None:
                        ctrlType = getControllerTypeBySpecifier(ctrlElem.get('type'))
                        if  ctrlType == ControllerType.MANIPULATOR or \
                            ctrlType == ControllerType.GRIPPER or \
                            ctrlType == ControllerType.GRIPPER_MIMIC or \
                            ctrlType == ControllerType.COVER:
                            lstJoints.append(jointElem.get('name'))
                        
            for joint in lstJoints:
                jointNameNode = ET.SubElement(gazeboJointPluginNode, 'joint_name')
                jointNameNode.text = joint

        return ET.tostring(xmlRoot).decode()
        
    except Exception as ex:
        print(f"[ERROR]: {ex}")
    return None
   
""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        
        # [DEBUG]
        ...

    except Exception as ex:
        print(f"[ERROR]: {ex}")


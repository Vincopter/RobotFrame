#!/usr/bin/env python3
import os, subprocess, time, math
import datetime
import threading
import pathlib
import yaml
import rclpy
import logging
import argparse
from rclpy.impl import rcutils_logger

from enum import Enum
from rclpy.node import Node
from threading import Lock
from yaml import load, Loader
from typing import Dict
from collections import namedtuple
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Empty
import robot_frame.launchers.controllers as controller
from robot_frame.launchers.controllers import ControllerType

logging.basicConfig(
    level = logging.INFO,
    format = "[%(levelname)s] %(message)s"
)

class TopicTypes(Enum):
    UNKNOWN = 0
    LIDAR_FRONT = 1
    LIDAR_BACK = 2
    LIDAR_LEFT = 3
    LIDAR_RIGHT = 4
    ODOMETRY = 5
    NAVIGATION = 6

""" Name of topics by type
"""
topic_names_by_type = {
    TopicTypes.UNKNOWN: str(),
    # values will be updated at runtime
    TopicTypes.LIDAR_FRONT: "front",
    TopicTypes.LIDAR_BACK: "back",
    TopicTypes.LIDAR_LEFT: "left",
    TopicTypes.LIDAR_RIGHT: "right",
    TopicTypes.ODOMETRY: "odom",
    TopicTypes.NAVIGATION: "navigation",
}

class MovementType(Enum):
    UNKNOWN = 0
    FORWARD = 1
    BACKWARD = 2

class MessageTypes(Enum):
    UNKNOWN = 0
    SCANNER = 1
    MOVEMENT = 2
    ODOMETRY = 3

""" ROS Type of Messages
"""
ros_messages_by_type = {
    
    MessageTypes.UNKNOWN: Empty,
    MessageTypes.SCANNER: LaserScan,
    MessageTypes.MOVEMENT: Twist,
    MessageTypes.ODOMETRY: Odometry,
}
    
""" ROS message type by custom topic types
"""
message_type_by_custom_topics = {
    
    TopicTypes.LIDAR_FRONT: MessageTypes.SCANNER,
    TopicTypes.LIDAR_LEFT: MessageTypes.SCANNER,
    TopicTypes.LIDAR_RIGHT: MessageTypes.SCANNER,
    TopicTypes.LIDAR_BACK: MessageTypes.SCANNER,
    TopicTypes.NAVIGATION: MessageTypes.MOVEMENT,
    TopicTypes.ODOMETRY: MessageTypes.ODOMETRY,
}

class LidarSideType(Enum):
    UNKNOWN = 0
    FRONT = 1
    LEFT = 2
    RIGHT = 3
    BACK = 4

""" Lidar Frame ID by side type
"""
lidar_ids_by_type = {
    LidarSideType.FRONT: "front_lidar",
    LidarSideType.LEFT:"left_lidar",
    LidarSideType.RIGHT: "right_lidar",
    LidarSideType.BACK: "back_lidar"
}

""" UTILS
"""
#
# /** HELPERS **/
# /*@{*/
#
def getTopicNameByType(topicType: TopicTypes) -> str:
    """ Return topic name by type
    """
    try:
        if TopicTypes(topicType) in TopicTypes:
            return topic_names_by_type[TopicTypes(topicType)]
    except (KeyError, ValueError) as _:
        ...
    return TopicTypes.UNKNOWN

def updateTopicNameByType(topicType: TopicTypes, name: str):
    """ Change topic name by type
    """
    try:
        if len(name) == 0:
            raise ValueError("Invalid update topic name")
        
        if TopicTypes(topicType) in TopicTypes:
            topic_names_by_type[TopicTypes(topicType)] = name
    except KeyError as _:
        ...
    except ValueError as ex:
        raise ex

def getLidarIdByType(sideLidar: LidarSideType) -> str:
    """ Return lidar id (frame_id) by type
    """
    try:
        if LidarSideType(sideLidar) in LidarSideType:
            return lidar_ids_by_type[LidarSideType(sideLidar)]
    except (KeyError, ValueError) as _:
        ...
    return LidarSideType.UNKNOWN

def getLidarSideById(lidarId: str) -> LidarSideType:
    """ Return lidar side type by id (frame_id)
    """
    for _, (key, value) in enumerate(lidar_ids_by_type.items()):
        if lidarId == value:
            return key
        
    return LidarSideType.UNKNOWN

def getMovementDesc(movement: MovementType) -> str:
    if movement == MovementType.FORWARD:
        return "FORWARD"
    elif movement == MovementType.BACKWARD:
        return "BACKWARD"
    return "UNKNOWN"

def getOppositeMovement(movement: MovementType) -> MovementType:
    if movement == MovementType.FORWARD:
        return MovementType.BACKWARD
    elif movement == MovementType.BACKWARD:
        return MovementType.FORWARD
    return MovementType.UNKNOWN

def getMessageTypeByTopic(typeTopic: TopicTypes) -> MessageTypes:
    """ Return message type by custom topic type
    """
    try:
        if TopicTypes(typeTopic) in TopicTypes:
            return message_type_by_custom_topics[TopicTypes(typeTopic)]
    except (KeyError, ValueError) as _:
        ...
    return MessageTypes.UNKNOWN

def getRosMessageByCustomType(typeMsg: MessageTypes):
    """ Return ROS message type by custom type
    """
    try:
        if MessageTypes(typeMsg) in MessageTypes:
            return ros_messages_by_type[MessageTypes(typeMsg)]
    except (KeyError, ValueError) as _:
        ...
    return Empty

def getAngleGrad(rad: float):
        return (rad * 180) /math.pi
#
# /*@}*/
#

""" Odometry properties class description
"""
class OdomProps():

    __movementDir : MovementType
    
    def __init__(self, isVerbose: bool):
        self.__mutex = Lock()
        self.__verbose = isVerbose
        self.__movementDir = MovementType.UNKNOWN
        self.__coordinates = {}
        self.__logger = rcutils_logger.RcutilsLogger("ODOM")
    
    def isVerboseLogging(self):    
        return self.__verbose
    
    def processMessage(self, message: Odometry):
        try:
            if not isinstance(message, Odometry):
                raise TypeError("Unsupported message type!")

            xDir = message.twist.twist.linear.x
            yDir = message.twist.twist.linear.y
     
            movementDir = MovementType.UNKNOWN
            if  (xDir > 0.0 or math.isclose(xDir, 0.0)) and \
                (yDir > 0.0 or math.isclose(yDir, 0.0)):
                # robot going forward
                movementDir = MovementType.FORWARD
            else:
                movementDir = MovementType.BACKWARD
        
            self.saveAndPrintCoordinates(message.header.frame_id, xDir, yDir, movementDir)
            self.checkAndSetMovementType(movementDir)

        except TypeError as ex:
            print(f"[ERROR]: {ex}")
        except Exception as ex:
            print(f"[ERROR]: {ex}")
    
    def saveAndPrintCoordinates(self, frame_id, x, y, movement):
        if not self.isVerboseLogging():
            return
        
        if not frame_id in self.__coordinates:
            self.__coordinates[frame_id] = [.0, .0]

        xx, yy = round(x, 3), round(y, 3)
            
        lst_ = self.__coordinates[frame_id]
        if not math.isclose(lst_[0], xx) or not math.isclose(lst_[1], yy):
            self.__coordinates[frame_id] = [xx, yy]
            self.__logger.info("[{}] MOVE ({}): x:{} y:{} [{}]".format(
                datetime.datetime.now().time(),
                frame_id, 
                xx, 
                yy, 
                getMovementDesc(movement)))
            
    def checkAndSetMovementType(self, mvmt: MovementType):
        if mvmt != MovementType.UNKNOWN:
            with self.__mutex:
                self.__movementDir = mvmt

    def getMovementType(self):
        with self.__mutex:
            return self.__movementDir

""" Lidar properties and points datum class description
"""
class LidarProps:
    __type: LidarSideType = LidarSideType.UNKNOWN
    __wrapAngle = 0         # общий угол обхвата лидаром этого типа (в радианах)
    __angleResolution = 0   # угловое разрешение (в радианах)
    __rayMin = 0            # в метрах
    __rayMax = 0            # в метрах
    
    # массив точек для анализа
    __collissionPoints : Dict[int, list] = dict()

    def __init__(self, type: LidarSideType, angleResolution: float, angleMin: float, angleMax: float):
        self.__type = type
        self.__constants = (namedtuple(
            'Constants', [
                # кол-во точек на секцию 
                'PointsOneSector',
                # пороговое расстояние до препятствия
                "CollissionThreshold", 
                # угол отклонения для обхода препятствий (в градусах)
                "DeflectionAngleOnSector",  
            ]
        ))(
            30, 
            1.3,
            60
        )
        
        self.__angleResolution = angleResolution
        self.setWrapAngleInRad(angleMin, angleMax)
            
    def getType(self):
        return self.__type

    def setWrapAngleInRad(self, angleMin: float, angleMax: float):
        self.__wrapAngle = abs(angleMax) + abs(angleMin)

    def getAngleResolutionInRad(self):
        return self.__angleResolution

    def getWrapAngleRad(self):
        return self.__wrapAngle

    def getWrapAngleGrad(self):
        return self.__wrapAngle * 180/math.pi

    def setRayMinMax(self, *, min: float, max: float):
        self.__rayMin, self.__rayMax = min, max

    def getRayMin(self):
        return self.__rayMin

    def getRayMax(self):
        return self.__rayMax
    
    def getOneAngleShiftInRad(self):
        return float(self.__constants.DeflectionAngleOnSector) * math.pi / 180

    def getThreshold(self):
        return self.__constants.CollissionThreshold

    def getPointsForOneSector(self) -> int:
        return self.__constants.PointsOneSector
    
    def clearCollissionPoints(self):
        self.__collissionPoints.clear()

    def initCollissionPoints(self):
        allPointsCount = math.ceil(self.getWrapAngleRad() / self.getAngleResolutionInRad())
        sectorCount = math.ceil(allPointsCount / self.getPointsForOneSector())
        self.clearCollissionPoints()
        self.__collissionPoints = {x: list() for x in range(0, sectorCount)}
    
    def getCollissionSectorsCount(self):
        return len(self.__collissionPoints)

    def areThereCollissions(self):
        for _, v in self.__collissionPoints.items():
            if len(v) > 0:
                return True
        return False
        
    def setCollissionPoint(self, index: int, point: float):
        if index < self.getCollissionSectorsCount():
            self.__collissionPoints.setdefault(index, []).append(point)
    
    def calcObstacleAvoidance(self) -> float:
        """
            Метод рассчета угла обхода при наличии препятствия
            В случае если;
             0: путь прямо свободен 
             -1: пути нет, стоп (разворот)
        """
        collissionPoints = dict()
        collissionPoints = self.__collissionPoints.copy()

        median = math.ceil(len(collissionPoints)/2)
        center = [median, median-1]

        to_right = lambda x: median - (median - x)
        to_left = lambda x: median + (median - x)
        noPoints = lambda x: len(collissionPoints[x]) == 0
        checkArrBoundary = lambda x: x >= 0 and x < len(collissionPoints)    

        def isClear(points):
            if isinstance(points, list):
                points_ = [x for x in points if checkArrBoundary(x)]
                if points != points_:
                    return False
                return len([x for x in points if noPoints(x)]) == len(points)
            else:
                if not checkArrBoundary(points):
                    return False
            return noPoints(points)
            
        def getClearPointsIndex(index):
            resIndex = -1
            while index >= 0:
                if isClear(to_right(index)):
                    if checkArrBoundary(to_right(index-1)):
                        if isClear(to_right(index-1)):
                            resIndex = to_right(index)
                            break
                if isClear(to_left(index)):
                    if checkArrBoundary(to_left(index-1)):
                        if isClear(to_left(index-1)):
                            resIndex = to_left(index)
                            break
                index -= 1
            return resIndex

        clearIndex = -1
        if not isClear(center):
            for x in center:
                if not isClear(x):
                    clearIndex = getClearPointsIndex(x)
        else:
            clearIndex = center[0]
            # путь по ходу движения свободен

        if clearIndex == -1:
            # по курсу движения пути нет
            return None

        deflectionAngle = self.getOneAngleShiftInRad()
        if clearIndex > median:
            deflectionAngle = (clearIndex - median) * self.getOneAngleShiftInRad()    
        else:
            deflectionAngle = (median - clearIndex) * -self.getOneAngleShiftInRad()
        
        return deflectionAngle
        

""" Obstacle removal class description
"""
class ObstacleControl(Node):

    __odom: OdomProps
    __lidars : Dict[LidarSideType, LidarProps] = dict()
    __publishers = {}
    __subscriptions = {}
    __diffDriveControllerName = str()
    __verbose = False
    
    def __init__(self, configYamlFile, isVerbose):
        super().__init__('obstacle_control')    
        self.__lidarsMutex = Lock()
        self.__frame_id = "base_link"
        self.__verbose = isVerbose
        self._logger.info(f"Initialize of '{self.getNodeName()}' node.")
        self.__constants = (namedtuple(
            'Constants', [
                # базовая линейная скорость, применяемая для 
                # в случае обхода препятствия.
                'ServiceVelocity', 
            ]
        ))(
            0.1,
        )
        self._logger.info(f"Config controllers file is '{configYamlFile}'.")
        self.parseYamlConfigAndGetSpecificNodes(configYamlFile)
       
        self.__odom = OdomProps(isVerbose)
        self.createSubscription(
            TopicTypes.ODOMETRY, 
            self.__odom.processMessage,
        )

        self.createSubscription(
            TopicTypes.LIDAR_FRONT, 
            self.processLidarMessage
        )
        self.createSubscription(
            TopicTypes.LIDAR_BACK, 
            self.processLidarMessage
        )
       
        self.createPublisher(TopicTypes.NAVIGATION)
        
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def parseYamlConfigAndGetSpecificNodes(self, configYamlFile):

        def getDiffControllerName(configYamlDoc) -> str:
            for key, val in configYamlDoc['controller_manager']['ros__parameters'].items():
                if isinstance(val, dict):
                    if 'type' in val:
                        if val["type"].find("DiffDriveController") != -1:
                            nameDiffDriveController = key
            if len(nameDiffDriveController) == 0:
                raise SystemExit("Unable to fetch of drive controller name!")
            return nameDiffDriveController

        with open(configYamlFile) as fl:
            try:
                configYamlDoc = yaml.load(fl, Loader=Loader)
            except yaml.YAMLError as exc:
                self._logger.warning("Unable to load controller config [{}]".format(configYamlFile))
                raise SystemExit("Unable to get startup configuration")
            
            self.__diffDriveControllerName = getDiffControllerName(configYamlDoc)

        if len(self.getDiffDriveCtrlName()) > 0:
            self._logger.info(f"Drive controller has been found ['{self.getDiffDriveCtrlName()}']")

        command = "ros2 topic list --show-types | sed -e 's/\[//g' -e 's/]//g' | awk '{print $1,\";\",$2}'"
        lstTopics = subprocess.check_output(command, shell=True, text=True).split("\n")
        # time.sleep(1) # воркарраунд для метода get_topic_names_and_types
        # lstTopics = self.get_topic_names_and_types()
        for topic_info in lstTopics:
            if len(topic_info) == 0:
                continue
            name, type_ = topic_info.split(";")
            name, type_ = name.strip(), type_.strip()
            if name.find(self.getDiffDriveCtrlName()) != -1:
                if Odometry.__name__ in str(type_):
                    updateTopicNameByType(TopicTypes.ODOMETRY, name)
            elif name.find(getTopicNameByType(TopicTypes.NAVIGATION)) != -1:
                updateTopicNameByType(TopicTypes.NAVIGATION, name)
            else:
                if LaserScan.__name__ in str(type_):
                    for topicType in [
                        TopicTypes.LIDAR_FRONT, 
                        TopicTypes.LIDAR_BACK, 
                        TopicTypes.LIDAR_LEFT, 
                        TopicTypes.LIDAR_RIGHT]:
                        if name.find(getTopicNameByType(topicType)) != -1:
                            updateTopicNameByType(topicType, name)

    def getNodeName(self) -> str:
        lst_ = self.get_node_names()
        if len(lst_) > 0:
            return lst_[0]
        return str()
    
    def getFrameID(self):
        return self.__frame_id
    
    def isVerbose(self) -> bool:
        return self.__verbose    
        
    def getDiffDriveCtrlName(self):
        return self.__diffDriveControllerName        
    
    def getDefaultVelocity(self):
        return self.__constants.ServiceVelocity
        
    def getLidarPropsByType(self, type: LidarSideType):
        try:
            with self.__lidarsMutex:
                return self.__lidars[LidarSideType(type)]
        except KeyError as _:
            return None

    def setLidarPropsByType(self, lidarType: LidarSideType, data: LidarProps):
        try:
            with self.__lidarsMutex:
                self.__lidars[lidarType] = data
        except KeyError as _:
            ...
    
    def getMovement(self):
        return self.__odom.getMovementType()
    
    def timer_callback(self):
        try:
            
            self.checkPathAndDoWorkarround()

        finally:
            ...
    
    def createPublisher(self, *topics):
        
        for customTopic in topics:
            topicName = getTopicNameByType(customTopic)
            self._logger.info("Creating of publisher for topic '{}'.".format(topicName))
            self.__publishers[customTopic] = self.create_publisher(
                getRosMessageByCustomType(getMessageTypeByTopic(customTopic)),
                topicName, 
                1
            )

    def createSubscription(self, topicType: TopicTypes, callback):
        
        if topicType == TopicTypes.UNKNOWN:
            return
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        topicName = getTopicNameByType(topicType)
        self._logger.info("Creating of subscription on topic '{}'.".format(topicName))
        self.__subscriptions[topicType] = self.create_subscription(
            getRosMessageByCustomType(getMessageTypeByTopic(topicType)),
            topicName, 
            callback,
            qos_profile,
        )
    
    def sendMessageToTopic(self, topicType: TopicTypes, message):
        
        if not TopicTypes(topicType) in TopicTypes:
            return 

        try:
            pub = self.__publishers[topicType]
            msgType = getRosMessageByCustomType(getMessageTypeByTopic(topicType))
            if not isinstance(message, msgType):
                raise ValueError("Unsupported message type: '{}'.".format(type(message)))
            
            pub.publish(message)
            
        except KeyError as _:
            self._logger.warning("Unable to get publisher for topic '{}'.".format(
                getTopicNameByType(topicType)
            ))
        except ValueError as ex:
            self._logger.error("Unable to send message on topic '{}' [{}]".format(
                getTopicNameByType(topicType),
                ex
            ))        
            
    def processLidarMessage(self, message: LaserScan):
        
        try:
            if not isinstance(message, LaserScan):
                raise TypeError("Unsupported message type!")
            
            typeLidar = getLidarSideById(message.header.frame_id)
            if self.getLidarPropsByType(typeLidar) is None:
                lp = LidarProps(
                    typeLidar, 
                    message.angle_increment,
                    message.angle_min,
                    message.angle_max
                )
                lp.setRayMinMax(min=message.range_min, max=message.range_max)
                self.setLidarPropsByType(lp.getType(), lp)
            
            lidar = self.getLidarPropsByType(typeLidar)
            assert not lidar is None
            
            lidar.initCollissionPoints()
            for idxArray in range(0, lidar.getCollissionSectorsCount()):

                pointsSectorNum = lidar.getPointsForOneSector()
                pointsShift = int(idxArray*pointsSectorNum)
                ...
                
                for point in message.ranges[pointsShift:pointsShift + pointsSectorNum]:
                    if point < float('inf') and point < lidar.getThreshold():
                        lidar.setCollissionPoint(idxArray, point)

        except (TypeError, Exception) as ex:
            print(f"[ERROR]: {ex}")

    def areThereCollission(self, sideLidar: LidarSideType) -> False:
        if  sideLidar == LidarSideType.UNKNOWN or \
            sideLidar == LidarSideType.LEFT or \
            sideLidar == LidarSideType.RIGHT:
            return False
       
        lidar = self.getLidarPropsByType(sideLidar)
        if lidar != None:
            if lidar.areThereCollissions():
                return True
        
        return False
    
    def controlDirection(self, deflectionAngle: float, movement: MovementType):
        
        coeffMovement = 1.0
        velocity = self.getDefaultVelocity()
        if deflectionAngle is None:
            self._logger.info("Path is not allowed. Go back!")            
            movement = getOppositeMovement(movement)
            if movement == MovementType.BACKWARD:
                coeffMovement = -1.0
                velocity *= 2
        
        elif math.isclose(abs(deflectionAngle), 0.0):
            return
        else:
            if movement == MovementType.BACKWARD:
                coeffMovement = -1.0

        if deflectionAngle is None:
            deflectionAngle = 0.0

        x = velocity * coeffMovement
        z = float(deflectionAngle)

        twist = Twist()
        twist.linear.x = x
        twist.angular.z = z
        self._logger.info(
            "[{}] Send bypass [x={:.3f}, z={:.3f}][{}]".format(
                datetime.datetime.now().time(),
                x,
                getAngleGrad(z),
                getMovementDesc(movement)                
            ))
               
        self.sendMessageToTopic(TopicTypes.NAVIGATION, twist)
            
    def checkPathAndDoWorkarround(self):
        
        sideLidar = None
        movement = self.getMovement()
        if movement == MovementType.FORWARD:
            if self.areThereCollission(LidarSideType.FRONT):
                sideLidar = LidarSideType.FRONT
        elif movement == MovementType.BACKWARD:
            if self.areThereCollission(LidarSideType.BACK):
                sideLidar = LidarSideType.BACK

        if sideLidar is None:
            return
          
        try: 
            lidar = self.getLidarPropsByType(sideLidar)
            if lidar is None:
                return
        
            deflectionAngle = lidar.calcObstacleAvoidance()
            self.controlDirection(deflectionAngle, movement)
            ...
            
        except (TypeError, Exception) as ex:
            print(f"[ERROR]: {ex}")

def waitUntilControllersBecomeActive(configYamlFile):
       
    lstControllersSpec = []
    with open(configYamlFile) as fl:
        try:
            configYamlDoc = yaml.load(fl, Loader=Loader)
        except yaml.YAMLError as exc:
            raise SystemExit("Unable to get startup configuration")
        
        if not configYamlDoc is None:
            for key, val in configYamlDoc['controller_manager']['ros__parameters'].items():
                if isinstance(val, dict):
                    if 'type' in val:
                        type_ = controller.getControllerTypeByRosDriverName(val["type"])
                        if  type_ == ControllerType.DRIVE or \
                            type_ == ControllerType.COVER or \
                            type_ == ControllerType.GRIPPER or \
                            type_ == ControllerType.MANIPULATOR:
                            lstControllersSpec.append(key)
        
    if len(lstControllersSpec) == 0:
        raise Exception("Unable to fetch any controllers specifier for waiting for their activation!")

    stepCount = 60
    while len(lstControllersSpec) > 0:
        
        stepCount -= 1
        if stepCount < 0:
            raise SystemExit("Time has expired and attempts to obtain data about loaded controllers...")
        
        try:
            command = "ros2 control list_controllers | grep -i \"active\" | sed 's/\x1b\[[0-9;]\{1,\}[A-Za-z]//g' | awk '{print $1}'"
            resultSet = subprocess.check_output(command, shell=True, text=True).split("\n")
            for result in resultSet:
                if not result is None and len(result) > 0:
                    driver = result.strip()
                    try:
                        lstControllersSpec.remove(driver)
                    except ValueError:
                        continue
                    ...
            time.sleep(0.5)
        except ValueError:
            pass

def run(controllerConfigFile, isVerbose):
  
    rclpy.init(args=None)
  
    try:

        controller = ObstacleControl(controllerConfigFile, isVerbose)
        nodeName = controller.getNodeName()
        rclpy.logging.get_logger(nodeName).info(
            f"Starting node [time='{datetime.datetime.now().time()}'][TID='{threading.get_ident()}']")

        rclpy.spin(controller)
        ...
        
    except SystemExit:
        rclpy.logging.get_logger(nodeName).info(
            f"Stopping node [time='{datetime.datetime.now().time()}']"
        )

    rclpy.shutdown()

if __name__ == '__main__':
    
    try:

        parser = argparse.ArgumentParser(description='Optional app description')
        parser.add_argument('-c', '--config', required=True, type=str, help='Name and path to configuration ROS2 controllers file in yaml-format')
        parser.add_argument('-v', '--verbose', required=False, action='store_true', help='Display advanced diagnostics if specified')
        args = parser.parse_args()
        
        os.environ.pop('RCUTILS_CONSOLE_OUTPUT_FORMAT', None)
        os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{severity}][{name}]: {message}"
        
        isVerbose = args.verbose
        configControllerFile = str()

        if args.config is None or len(args.config) == 0:
            rootPkgDir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            configControllerFile = os.path.join(rootPkgDir, "configs", 'controllers.yaml')
        else:
            configControllerFile = args.config
        
        if not pathlib.Path(configControllerFile).is_file():
            raise Exception(f"Unable to find config file ['{configControllerFile}']")

        waitUntilControllersBecomeActive(configControllerFile)

        run(configControllerFile, isVerbose)
        pass
        
    except SystemExit as se:
        print(f"Node execution has stopped. {se}") 
    except Exception as ex:
        print(f"[ERROR]: {ex}") 

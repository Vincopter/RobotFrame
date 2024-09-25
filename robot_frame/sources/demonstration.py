#!/usr/bin/env python3
import sys
import rclpy
import datetime
import threading

from enum import Enum
from collections import UserList
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header, Empty

class TopicTypes(Enum):
    UNKNOWN = 0
    COVER = 1
    MANIPULATORS = 2
    GRIPPERS = 3
    WHEELS = 4
    LASERS = 5

class MessageTypes(Enum):
    UNKNOWN = 0
    TRAJECTORY = 1
    TWIST = 2
    SCAN = 3

class Devices(Enum):
    UNKNOWN = 0
    COVER = 1
    LEFT_MANIPULATOR = 2
    RIGHT_MANIPULATOR = 3
    LEFT_GRIPPER = 4
    RIGHT_GRIPPER = 5
    LEFT_WHEELS = 6
    RIGHT_WHEELS = 7
    LASER_FRONT = 8
    LASER_BACK = 8

""" Demonstration phase 
    (order is important)
"""
class PhaseEvents(Enum):
    UNKNOWN = 0
    ON_INIT = 1
    ON_PREPARE = 2
    ON_WORK_LEFT_SIDE = 3
    ON_WORK_RIGHT_SIDE = 4
    ON_DONE = 5
    STOP = 6

class Actions(Enum):
    INITIAL = 0
    ACTIVATE = 1
    DEACTIVATE = 2
    TESTING = 3

""" Name of topics by type (see controllers.yaml)
"""
topic_names_by_type = {
    
    TopicTypes.COVER: "covers_controller",
    TopicTypes.GRIPPERS: "grippers_controller",
    TopicTypes.MANIPULATORS: "manipulators_controller",
}

""" ROS Type of Messages
"""
ros_messages_by_type = {
    
    MessageTypes.UNKNOWN: Empty,
    MessageTypes.TRAJECTORY: JointTrajectory,
    MessageTypes.TWIST: Twist,
    MessageTypes.SCAN: LaserScan,
}

""" Name of topic message by type
"""
message_name_by_type = {
    
    MessageTypes.TRAJECTORY: "joint_trajectory",
    MessageTypes.TWIST: "twist",
    MessageTypes.SCAN: "scan",
}

""" Joints name by device type
"""
robot_frame_joints = {
    
    Devices.COVER: [
        'cover_hinge_joint'
    ],
    Devices.LEFT_MANIPULATOR: [
        'left_m1st_abase_joint',
        'left_a1st_m1st_joint',
        'left_a2nd_m2nd_joint',
    ],
    Devices.LEFT_GRIPPER: [
        'left_gripper_controller'
    ],
    Devices.RIGHT_MANIPULATOR: [
        'right_m1st_abase_joint',
        'right_a1st_m1st_joint',
        'right_a2nd_m2nd_joint',
    ],
    Devices.RIGHT_GRIPPER: [
        'right_gripper_controller'
    ],
    Devices.LEFT_WHEELS: [],
    Devices.RIGHT_WHEELS: [],
    Devices.LASER_FRONT: [],
    Devices.LASER_BACK: [],
}

""" Topic type by device type
"""
topic_type_by_device = {
    
    Devices.COVER: TopicTypes.COVER,
    Devices.LEFT_MANIPULATOR: TopicTypes.MANIPULATORS,
    Devices.LEFT_GRIPPER: TopicTypes.GRIPPERS,
    Devices.RIGHT_MANIPULATOR: TopicTypes.MANIPULATORS,
    Devices.RIGHT_GRIPPER: TopicTypes.GRIPPERS,
    Devices.LEFT_WHEELS: TopicTypes.WHEELS,
    Devices.RIGHT_WHEELS: TopicTypes.WHEELS,
    Devices.LASER_FRONT: TopicTypes.LASERS,
    Devices.LASER_BACK: TopicTypes.LASERS,
}

""" Message type by topic type
"""
message_type_by_device = {
    
    TopicTypes.COVER: MessageTypes.TRAJECTORY,
    TopicTypes.MANIPULATORS: MessageTypes.TRAJECTORY,
    TopicTypes.GRIPPERS: MessageTypes.TRAJECTORY,
    TopicTypes.WHEELS: MessageTypes.TWIST,
    TopicTypes.LASERS: MessageTypes.SCAN,
}

""" UTILS
"""
#
# /** HELPERS **/
# /*@{*/
#
def getMessageTypeByTopic(typeTopic: TopicTypes) -> MessageTypes:
    """ Return message type by topic type
    """
    try:
        if TopicTypes(typeTopic) in TopicTypes:
            return message_type_by_device[TopicTypes(typeTopic)]
    except (KeyError, ValueError) as _:
        ...
    return MessageTypes.UNKNOWN

def getCustomTopicTypeByDevice(typeDevice: Devices) -> TopicTypes:
    """ Return custom topic type by device type
    """
    try:
        if Devices(typeDevice) in Devices:
            return topic_type_by_device[Devices(typeDevice)]
    except (KeyError, ValueError) as _:
        ...
    return TopicTypes.UNKNOWN

def getDeviceTypeByJointName(jointName: str) -> Devices:
    """ Return device type by joint name
    """
    for _, (key, value) in enumerate(robot_frame_joints.items()):
        for joint in value:
            if jointName == joint:
                return key
    return Devices.UNKNOWN

def getCustomTopicTypeByJointName(jointName: str) -> TopicTypes:
    """ Return custom topic type by joint's name
    """    
    try:
        if len(jointName) == 0:
            return TopicTypes.UNKNOWN
        
        deviceType = getDeviceTypeByJointName(jointName)
        return getCustomTopicTypeByDevice(deviceType)
    except KeyError as _:
        ...
    return TopicTypes.UNKNOWN

def getTopicNameByType(typeTopic: TopicTypes) -> str:
    """ Return topic name by type
    """
    try:
        if TopicTypes(typeTopic) in TopicTypes:
            return topic_names_by_type[TopicTypes(typeTopic)]
    except (KeyError, ValueError) as _:
        ...
    return str()

def getJointsListByDevice(typeDevice: Devices) -> list:
    """ Return joints list by device type
    """
    try:
        if Devices(typeDevice) in Devices:
            return robot_frame_joints[Devices(typeDevice)][:]
    except (KeyError, ValueError) as _:
        ...
    return list()

def getMessageNameByType(typeMsg: MessageTypes) -> str:
    """ Return message name by message type
    """
    try:
        if MessageTypes(typeMsg) in MessageTypes:
            return message_name_by_type[MessageTypes(typeMsg)]
    except (KeyError, ValueError) as _:
        ...
    return str()

def getRosMessageByCustomType(typeMsg: MessageTypes):
    """ Return ROS message type by custom type
    """
    try:
        if MessageTypes(typeMsg) in MessageTypes:
            return ros_messages_by_type[MessageTypes(typeMsg)]
    except (KeyError, ValueError) as _:
        ...
    return Empty

def getActionsCopyByDevice(device: Devices, action: Actions) -> list:    
    """ Return list copy of commands (demo actions) 
        by device and actuon type
    """
    try:
        global joint_actions
        if  Devices(device) in Devices and \
            Actions(action) in Actions:
            return joint_actions[Devices(device)][Actions(action)].copy()
    except (KeyError, ValueError) as _:
        ...
    return list()

def linkDeviceJointsActionListToContainer(device: Devices, *actions, container: list):
    """ Get device actions for processing by type
    """
    lstActions = list()
    lstJoints = list()
    addJoints = lambda dev: lstJoints.append(getJointsListByDevice(dev))
    for action in actions:
        lstActions.append(getActionsCopyByDevice(device, action))

    addJoints(device)
    if len(lstActions) > 0:
        container.append(tuple([lstJoints, lstActions]))
#
# /*@}*/
#

""" Actions plan sequences for Joints by device type
"""
joint_actions = {
    
    Devices.COVER: {
        Actions.INITIAL: [[0.0] * len(getJointsListByDevice(Devices.COVER))], 
        Actions.TESTING: [[1.0], [1.5], [0.5], [0.0]], 
        Actions.ACTIVATE: [[1.35]], 
        Actions.DEACTIVATE: [[0.0]]
    },
    Devices.LEFT_GRIPPER: {
        Actions.INITIAL: [[0.0] * len(getJointsListByDevice(Devices.LEFT_GRIPPER))], 
        Actions.ACTIVATE: [[1.3]], 
        Actions.DEACTIVATE: [[0.0]],
        Actions.TESTING: [
            [0.0], 
            [1.0],
            [1.3], 
            [0.0]], 
    },
    Devices.RIGHT_GRIPPER: {
        Actions.INITIAL: [[0.0] * len(getJointsListByDevice(Devices.RIGHT_GRIPPER))], 
        Actions.DEACTIVATE: [[0.0]],
        Actions.ACTIVATE: [
                [1.2]
            ], 
        Actions.TESTING: [
                [0.0], 
                [1.0], 
                [1.2], 
                [0.0]
            ], 
    },
    Devices.LEFT_MANIPULATOR: {
        Actions.INITIAL: [[0.0] * len(getJointsListByDevice(Devices.LEFT_MANIPULATOR))], 
        Actions.TESTING: [ 
                [ 0.0, 1.5, 0.0],
                [-0.3, 1.5, -1.0],
                [-0.3, 1.5, -1.2],
                [-0.3, 1.5, -0.5],
                [-0.3, 0.5, 0.0],
                [-0.3, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
        Actions.ACTIVATE: [ 
                [-1.1, 0.0, 0.0],
                [-1.7, 0.0, 0.0],
                [-1.7, -1.0, 0.0],
                [-1.7, -1.3, -1.0], 
                [-1.7, -1.4, -1.5],
            ],                 
        Actions.DEACTIVATE: [ 
                [-1.7, -1.3, -1.0],
                [-1.7, -1.0, 0.0],
                [-1.7, 0.0, 0.0], 
                [0.0, 0.0, 0.0],
            ]                 
    },
    Devices.RIGHT_MANIPULATOR: {
        Actions.INITIAL: [[0.0] * len(getJointsListByDevice(Devices.RIGHT_MANIPULATOR))], 
        Actions.TESTING: [ 
                [-0.1, 0.5, 0.0],
                [-0.5, 1.3, 0.0],
                [-1.0, 1.5, 0.0],
                [-1.5, 1.0, -0.5],
                [-0.3, 1.5, -1.2],
                [-0.3, 1.5, -0.5],
                [-0.3, 0.5, 0.0],
                [-0.3, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
        Actions.ACTIVATE: [ 
                [0.0, 0.0, 0.0],
                [-1.7, 0.0, 0.0],
                [-1.7, -1.0, 0.0],
                [-1.7, -1.3, -1.0], 
                [-1.7, -1.3, -1.5],
            ],                 
        Actions.DEACTIVATE: [ 
                [-1.7, -1.3, -1.0],
                [-1.7, -1.0, 0.0],
                [-1.7, 0.0, 0.0], 
                [0.0, 0.0, 0.0],
            ]                
    },
}

""" Class implementing the demo plan
"""
class RobotFrameDemoPlan:
    
    __currentPhase : PhaseEvents
    
    def __init__(self, initialPhase: PhaseEvents, logger):
        self.__actionsQueue = list()
        self.__currentPhase = initialPhase
        self.__logger = logger
        
    def getPhase(self):
        return self.__currentPhase
    
    def isStopPhase(self) -> bool:
        return self.getPhase() == PhaseEvents.STOP

    def shiftPhase(self):
        self.__currentPhase = PhaseEvents(self.__currentPhase.value + 1)

    def isActionsQueueEmpty(self):
        if self.__actionsQueue == 0:
            return True
        for actionsByJoints in self.__actionsQueue:
            actions = actionsByJoints[1]
            if len(actions) > 0:
                return False
        return True
    
    def setActionsQueue(self, lstActions):
        self.clearActionsQueue()
        self.__actionsQueue = lstActions

    def getUnprocessedActionsCount(self):
        count = 0
        for actionsByJoints in self.__actionsQueue:
            count += len(actionsByJoints[1])
        return count

    def clearActionsQueue(self):
        return self.__actionsQueue.clear()
    
    def tryMovePhase(self) -> bool:
        currentPhase = self.getPhase()
        try:
            if not PhaseEvents(currentPhase.value) in PhaseEvents:
                self.clearActionsQueue()
                currentPhase = PhaseEvents.UNKNOWN
                return False
            
            if not self.isActionsQueueEmpty():
                return False

            _ = PhaseEvents(currentPhase.value + 1)
            ...
        except ValueError as ex:
            return False

        return True

    def movePhase(self) -> bool:
        if not self.tryMovePhase():
            return False
        
        if not self.isActionsQueueEmpty():
            self.__logger.warning("Move demo phase. Actions queue is not empties [count={}]".format(
                self.getUnprocessedActionsCount())
            )
        
        self.shiftPhase()            
        self.clearActionsQueue()
        if self.isStopPhase():
            self.__logger.info(
                "It is impossible to change phase. "\
                "The demonstration has reached its end. "\
                "The node will be stopped...")

        self.setActionsQueue(self.getActionsAndJointsByPhase(self.getPhase()))
        self.__logger.info("Demo phase has been successfully moved to '{}'. Count of new actions are {}.".format(
            self.getPhase().name,
            self.getUnprocessedActionsCount())
        )

        return True

    def getLastActionAndJoint(self):
        joints = list()
        actions = list()
        garbager = []

        for idx, actionsByJoints in enumerate(self.__actionsQueue):

            joints = actionsByJoints[0][0]
            allActions = actionsByJoints[1]
            
            if len(allActions) > 0:
                
                if len(allActions[0]) == 0:
                    allActions.pop(0)
                    garbager.append(idx)
                    continue
                else:
                    actions = allActions[0].pop(0)
                    break
                
        [self.__actionsQueue.pop(x) for x in garbager]
        if len(actions) == 0:
            actions = None
             
        return (joints, actions)

    def getActionsAndJointsByPhase(self, phase: PhaseEvents):
        
        container = list()
 
        if phase == PhaseEvents.ON_INIT:
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.INITIAL, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_MANIPULATOR, Actions.INITIAL, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_MANIPULATOR, Actions.INITIAL, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_GRIPPER, Actions.INITIAL, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_GRIPPER, Actions.INITIAL, container=container)
            
        elif phase == PhaseEvents.ON_PREPARE:
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.TESTING, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_GRIPPER, Actions.TESTING, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_MANIPULATOR, Actions.TESTING, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_GRIPPER, Actions.TESTING, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_MANIPULATOR, Actions.TESTING, container=container)
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.DEACTIVATE, container=container)
            
        elif phase == PhaseEvents.ON_WORK_LEFT_SIDE:
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_MANIPULATOR, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_GRIPPER, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_GRIPPER, Actions.DEACTIVATE, container=container)            
            linkDeviceJointsActionListToContainer(Devices.LEFT_MANIPULATOR, Actions.DEACTIVATE, container=container)

        elif phase == PhaseEvents.ON_WORK_RIGHT_SIDE:
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_MANIPULATOR, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_GRIPPER, Actions.ACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.RIGHT_GRIPPER, Actions.DEACTIVATE, container=container)            

        elif phase == PhaseEvents.ON_DONE:            
            linkDeviceJointsActionListToContainer(Devices.RIGHT_MANIPULATOR, Actions.DEACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.LEFT_MANIPULATOR, Actions.DEACTIVATE, container=container)
            linkDeviceJointsActionListToContainer(Devices.COVER, Actions.DEACTIVATE, container=container)
            
        elif phase == PhaseEvents.STOP:
            raise SystemExit("Stopping demonstation")

        return container

class RobotFrameDemonstrationNode(Node):

    __robotFrameDemoPlan: RobotFrameDemoPlan
    
    def __init__(self):
        
        super().__init__('robot_frame_demonstration')
        self._logger.info(f"Initialize of RobotFrame demostration [{self.getNodeName()}]")
        self.__robotFrameDemoPlan = RobotFrameDemoPlan(
            PhaseEvents.UNKNOWN,
            self.get_logger())
        
        self.__callNumbers = 0
        self.__publishers = {}
        self.createPublisher(
            Devices.COVER,
            Devices.LEFT_MANIPULATOR,
            Devices.RIGHT_MANIPULATOR,
            Devices.LEFT_GRIPPER,
            Devices.RIGHT_GRIPPER
        )

        self.__frame_id = "base_link"
        
        self.timer_period = 2
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # Задержка (в сек.) для выполнения перехода к указанным точкам.
        # Должна быть меньше чем timer_period
        self.duration_sec = 1.3
        self.duration_nanosec = 0.4 * 1e9

    def getNodeName(self) -> str:
        lst_ = self.get_node_names()
        if len(lst_) > 0:
            return lst_[0]
        return str()

    def getPlan(self):
        return self.__robotFrameDemoPlan

    def getCurrentPhase(self):
        return self.getPlan().getPhase()

    def checkOnChangePhase(self):
        return self.getPlan().tryMovePhase()

    def getFrameID(self):
        return self.__frame_id
    
    def getNextCallNumber(self) -> int:
        self.__callNumbers += 1
        return self.__callNumbers
    
    def createPublisher(self, *devices):
        topics = set()
        for device in devices:
            ct_ = getCustomTopicTypeByDevice(device)
            if ct_ != TopicTypes.UNKNOWN:
                topics.add(ct_)

        for customTopic in topics:
            msgType = getMessageTypeByTopic(customTopic)
            strTopic = "/{}/{}".format(getTopicNameByType(customTopic), getMessageNameByType(msgType))
            self._logger.info("Creating of publisher for topic '{}'.".format(strTopic))
            self.__publishers[customTopic] = self.create_publisher(
                getRosMessageByCustomType(msgType),
                strTopic, 
                1
            )
            
    def getPublisherByType(self, topicType: TopicTypes):
        try:
            return self.__publishers[topicType]
        except KeyError as _:
            self._logger.error("Unable to find publisher by type '{}'!".format(getTopicNameByType(topicType)))
            return None
    
    def on_activate(self, state):
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)
    
    def on_error(self, state):
        self.get_logger().warning("on_error() is called.")
        return super().on_activate(state)

    def timer_callback(self):
        callNumber_ = self.getNextCallNumber()
        self._logger.info(f"[{callNumber_}] Start of processing callback [{datetime.datetime.now().time()}][TID='{threading.get_ident()}']")

        currentPhase = self.getCurrentPhase()
        self._logger.info("[{}] Current phase is: '{}'.".format(
            callNumber_,
            currentPhase.name))
        
        def _tryMoveAndGetLastActions():
            if self.getPlan().tryMovePhase():
                result = self.getPlan().movePhase()
                if result:
                    currentPhase = self.getCurrentPhase()
                    self._logger.info(
                        "[{}] Enabled receiving new actions after changing the plan phase. "\
                        "New phase is '{}'.".format(
                            callNumber_,
                            currentPhase.name))
        
            jointsAndActions = self.getPlan().getLastActionAndJoint()
            return (jointsAndActions[0], jointsAndActions[1])
        
        try:
        
            joints, actions = _tryMoveAndGetLastActions()
            if  (joints is None or len(joints) == 0) and \
                (actions is None or len(actions) == 0):
                self._logger.error("[{}] Received invalid joints set ans actions for demostration plan. " \
                    "Process callback will be terminated...".format(
                    callNumber_)
                )
                return
            
            if (actions is None or len(actions) == 0):
                joints, actions = _tryMoveAndGetLastActions()

            if  joints is None or len(joints) == 0:
                self._logger.error("[{}] Received invalid joints configuration for demostration plan. " \
                    "Process callback will be terminated...".format(
                    callNumber_)
                )
                return
            
            if actions is None or len(actions) == 0:
                self._logger.warning("[{}] The actions are over for demostration plan for joints: '{}'. " \
                    "Process callback skipping...".format(
                    callNumber_,
                    ", ".join(map(str, joints))))
                return

            self._logger.info("[{}] Received new actions series for joints: '{}'.".format(
                callNumber_,
                ", ".join(map(str, joints))))

            positions = []
            for action in actions:
                positions.append(action)
            
            if len(positions) > 0:
                msgTrajectory = JointTrajectory()
                msgTrajectory.header = Header()  
                msgTrajectory.header.frame_id = self.getFrameID()  
                msgTrajectory.joint_names = UserList(joints)

                point = JointTrajectoryPoint()
                point.positions = UserList(positions)
                point.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))
                msgTrajectory.points.append(point)

                self._logger.info("[{}] Positions ({}) for processing have been added [{}]".format(
                    callNumber_,
                    len(msgTrajectory.points),
                    positions))

            if len(joints) > 0:
                # detect joint name by 1st custom topic
                topicType = getCustomTopicTypeByJointName(joints[0])
                pub = self.getPublisherByType(topicType)
                if not pub is None:
                    pub.publish(msgTrajectory)
                else:
                    self._logger.error("[{}] Unable to publish actions!".format(callNumber_))
        finally:            
            self._logger.info(
                f"[{callNumber_}] Stop processing callback [{datetime.datetime.now().time()}][TID='{threading.get_ident()}']")
   
def processDemo(args=None):
  
    rclpy.init(args=args)
  
    robot_frame = RobotFrameDemonstrationNode()
    nodeName = robot_frame.getNodeName()
    rclpy.logging.get_logger(nodeName).info(
        f"Starting node [time='{datetime.datetime.now().time()}'][TID='{threading.get_ident()}']")
  
    try:
        rclpy.spin(robot_frame)
        ...
    except SystemExit:
        rclpy.logging.get_logger(nodeName).info(
            f"Stopping node [time='{datetime.datetime.now().time()}']"
        )

    rclpy.shutdown()
 
if __name__ == '__main__':

    print("Installed Python version is: ", sys.version)
    try:
       
        processDemo()
        ...
    except Exception as ex:
        print(f"[ERROR]: {ex}")

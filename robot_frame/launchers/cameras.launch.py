import os
import cv2 as cv

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import utils 

# Можем подключить для робота только одну или две реальные камеры.
# Предполагается, что камеры идентифицируются как первое и второе устройство,
# если это не так, то следует предусмотреть смену индексов (0, 1) в словаре camOrientation.
camOrientation = {
    0: "front",
    1: "back"
}

class CameraConfig:
    __cameraName: str
    __cameraFrameName: str
    __parametersPath: Path
    __rosNamespace: str

    def __init__(self, cameraName):
        self.__cameraName = cameraName
    def getCameraName(self) -> str:
        return self.__cameraName
    def setCameraFrameName(self, frameName):
        self.__cameraFrameName = frameName
        return self
    def getCameraFrameName(self) -> str:
        return self.__cameraFrameName
    def setYamlParametersPath(self, paramPath):
        self.__parametersPath = paramPath
        return self
    def getYamlParametersPath(self) -> Path:
        return self.__parametersPath
    def setRosNamespace(self, ns):
        self.__rosNamespace = ns
        return self
    def getRosNamespace(self) -> str:
        return self.__rosNamespace

lstOfCameraConfigs = []

def enumCameras():

    arrCamIDs = []
    for idx in camOrientation.keys():
        cap = cv.VideoCapture(idx)        
        if cap.isOpened():
            arrCamIDs.append(idx)
            cap.release()
            
    return arrCamIDs

def getCamInfoYaml(cameraName: str):
    # Calibration parameters for WebCamera: "Sunplus Innovation Technology Inc." 
    # Checked through chessboard calibration (OpenCV).
    infoDict = {
        "image_width": 640,
        "image_height": 480,
        "camera_name": f"'{cameraName}'",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": "[438.8, 0.0, 305.6, 0.0, 437.3, 243.7, 0.0, 0.0, 1.0]",            
        },
        "distortion_model": 'plumb_bob',
        "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": "[-0.36, 0.11, 0.001, 0.000505, 0.0]",
            },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": "[0.99998, 0.0028, -0.00605, -0.0028, 0.999986, -0.0044, 0.006, 0.0044, 0.99997]"
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": "[393.6538, 0.0, 322.79794, 0.0, 0.0, 393.654, 241.091, 0.0, 0.0, 0.0, 1.0, 0.0]",
        }
    }
    
    fileNamePath = os.path.join(utils.getRobotFrameConfigDir(), 'sunplus_camera_info.yaml')
    utils.saveYaml(fileNamePath, infoDict)
    return fileNamePath

def getCamConfigYaml(camId: int, *, name: str, frame: str):
    
    cameraName = name
    cameraFrameId = frame
    deviceVideoId = "/dev/video{}".format(camId)
    
    # Config for WebCamera: "Sunplus Innovation Technology Inc." 
    configDict = {
        "/**": {
            "ros__parameters": {
                "video_device": f"'{deviceVideoId}'",
                "framerate": 30.0,
                "io_method": "'mmap'",
                "camera_name": f"'{cameraName}'",
                "frame_id": f"'{cameraFrameId}'",
                "pixel_format": "'mjpeg2rgb'",
                "av_device_format": "'YUV422P'",
                "image_width": 640,
                "image_height": 480,
                "camera_info_url": "'{}'".format(getCamInfoYaml(cameraName)),
                "brightness": -1,
                "contrast": -1,
                "saturation": -1,
                "sharpness": -1,
                "gain": -1,
                "focus": -1,
                "autofocus": bool(False),
                "auto_white_balance": bool(False),
                "white_balance": 4000,
                "autoexposure": bool(True),
                "exposure": 100,
            }
        }
    }
    
    fileNamePath = os.path.join(
        utils.getRobotFrameConfigDir(),
        "sunplus_param_camera_{}.yaml".format(camOrientation[camId]))
    utils.saveYaml(fileNamePath, configDict)
    return fileNamePath

def createCameraConfig(camId: int, *, namespace: str):
    
    cameraName = "optical_camera_{}".format(camOrientation[camId])
    cameraFrameId = "{}_camera_optical".format(camOrientation[camId])
    
    cameraConfigPathFile = getCamConfigYaml(
        camId, 
        name=cameraName, 
        frame=cameraFrameId
    )
    
    global lstOfCameraConfigs
    lstOfCameraConfigs.append(
        CameraConfig(cameraName).
            setRosNamespace(f"/{namespace}/{cameraName}").
            setYamlParametersPath(cameraConfigPathFile).
            setCameraFrameName(cameraFrameId)
    )

def getLaunchArgs():
    from utils import ArgumentsType
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.NAMESPACE)
    ]
    
def getLaunchActions(context):
    #
    # Check camera before use with guvcview app...
    #
    ros_namespace = LaunchConfiguration(
        utils.getArgumentNameByType(utils.ArgumentsType.NAMESPACE).perform(context))
    
    for camId in enumCameras():
        createCameraConfig(camId, namespace=ros_namespace)

    cameraNodes = [
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name=camera.getCameraName(),
            namespace=camera.getRosNamespace(),
            parameters=[camera.getYamlParametersPath()],
        )
        for camera in lstOfCameraConfigs
    ]
    
    return cameraNodes

def generate_launch_description():
    try:
        opaqueFunc = OpaqueFunction(function = getLaunchActions)
        launchDescription = LaunchDescription(getLaunchArgs())
        launchDescription.add_action(opaqueFunc)
        return launchDescription
    except Exception as _:
        return LaunchDescription()
    

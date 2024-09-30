import os
import cv2 as cv
from yaml import dump
from ament_index_python.packages import get_package_share_directory
import robot_frame.launchers.utils as utils

camOrientation = {
    0: "front",
    1: "back"
}

def saveYaml(filename: str, content: dict):
    yamlContainer = dump(content, default_style='"')
    yamlContainer = yamlContainer.replace("\"", "").replace("'", '"')
    with open(filename, encoding='utf8', mode='w') as fl:
        fl.write('# [GENERATED YAML-FILE]\n')
        fl.write(yamlContainer)

def enumCameras():

    arrCamIDs = []
    for idx in camOrientation.keys():
        cap = cv.VideoCapture(idx)
        _ = cap.read()
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
    
    fileNamePath = os.path.join(utils.getRealRobotFrameDirPath(), 'configs', 'sunplus_camera_info.yaml')
    saveYaml(fileNamePath, infoDict)
    return fileNamePath
       
def getCamConfigYaml(camId: int):
    
    deviceVideoId = "/dev/video{}".format(camId)
    cameraName = "camera_{}".format(camOrientation[camId])
    cameraFrameId = "{}_camera_optical".format(camOrientation[camId])
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
        utils.getRealRobotFrameDirPath(), 
        'configs', 
        "sunplus_param_camera_{}.yaml".format(camOrientation[camId]))
    saveYaml(fileNamePath, configDict)
    return fileNamePath
   
""" [DEBUG]
"""    
if __name__ == '__main__':
    
    try:
        for camId in enumCameras():
            getCamConfigYaml(camId)
        
        ...

    except Exception as ex:
        print(f"[ERROR]: {ex}")



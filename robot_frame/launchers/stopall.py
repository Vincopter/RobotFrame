#!/usr/bin/env python3
import os, sys, subprocess, rclpy
from rclpy.node import Node

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import utils 

def run():

    ksFilePath = os.path.join(utils.getRealRobotFrameDirPath(), 'scripts/stopall.sh')
    subprocess.run(["bash", ksFilePath])
    
if __name__ == '__main__':
    
    try:
        run()
        ...
    except (Exception, SystemExit) as ex:
        print(f"[ERROR]: {ex}") 

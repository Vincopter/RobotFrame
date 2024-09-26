import os, sys, pathlib

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
import utils, controllers

def getGazeboParamToYaml():
    gazeboParamsDict = {
        "gazebo": {
            "ros__parameters": {
                "publish_rate": 400.0                
            }
        }
    }
    
    fileNamePath = os.path.join(utils.getRobotFrameConfigDir(), 'gazebo_params.yaml')
    utils.saveYaml(fileNamePath, gazeboParamsDict)
    return fileNamePath

def getMultiplexerArgs():
    multiplexer = {}
    multiplexer["cmd_vel_navigation"] = '10'
    multiplexer["cmd_vel_keyboard"] = '40'
    multiplexer["cmd_vel_joystick"] = '50'
    return str(multiplexer)

def resetLogSetting():
    """ Reset ROS2 log's settings
    """
    import launch.logging
    launch.logging.launch_config.reset()
    os.environ.pop('ROS_LOG_DIR', None)
    home = pathlib.Path.home()
    custom_log_dir = str(pathlib.Path(home / utils.getPackageName() / 'logs'))
    os.makedirs(custom_log_dir, exist_ok=True)

    def delete_outdated_files(dirPath):
        try:
            with os.scandir(dirPath) as entries:
                for entry in entries:
                    if entry.is_file():
                        os.unlink(entry.path)
                    elif entry.is_dir():
                        delete_outdated_files(entry.path)
                        os.rmdir(entry.path)
        except OSError:
            ...

    delete_outdated_files(custom_log_dir)
    os.environ['ROS_LOG_DIR'] = custom_log_dir
    print("Logs path for all modules has been changed on '{}'".format(custom_log_dir))

def changeEnvironmentVariables():
    
    variable = "GAZEBO_MODEL_PATH"
    lstMaterialPaths = [
        utils.getWorldsMaterialDir(),
        utils.getRobotResourceShareDir(),
    ]
    
    if os.environ.get(variable) is None:
        os.environ[variable] = os.pathsep.join(lstMaterialPaths)
    else:
        os.environ[variable] += os.pathsep + os.pathsep.join(lstMaterialPaths)

    print("Environment '{}' = '{}'".format(variable, os.environ[variable]))

def getLaunchArgs():
    from utils import ArgumentsType
    return [
        utils.getLaunchArgumentDeclaration(ArgumentsType.RUN_RVIZ),
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_CAMERAS),
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_TELEOP),
        utils.getLaunchArgumentDeclaration(ArgumentsType.USE_ROS2_CONTROL),
        utils.getLaunchArgumentDeclaration(ArgumentsType.RUN_DEMO),
    ]
    
def generate_launch_description():
    
    resetLogSetting()
    changeEnvironmentVariables()

    launch_description = LaunchDescription(getLaunchArgs())

    from utils import ArgumentsType
    use_gui = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_GUI))
    run_rviz = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.RUN_RVIZ))
    run_demo = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.RUN_DEMO))
    use_cameras = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_CAMERAS))
    use_teleop = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_TELEOP))
    use_ros2_control = LaunchConfiguration(utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL))
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'robot.launch.py')
            ]), 
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.USE_GUI): use_gui,
            utils.getArgumentNameByType(ArgumentsType.USE_TELEOP): use_teleop,
            utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): 'True',
            utils.getArgumentNameByType(ArgumentsType.USE_ROS2_CONTROL): use_ros2_control,
        }.items()
    )
    launch_description.add_action(robot)
    
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'rviz.launch.py')
        ]),
        condition=IfCondition(run_rviz),
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.USE_TELEOP): 'True',
            utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): 'True',
            utils.getArgumentNameByType(ArgumentsType.RUN_PUBLISHER): 'False',
        }.items(),
    )
    launch_description.add_action(rviz)

    twist_mult = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'multiplexer.launch.py')
        ]),
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.TMUX_RULES): getMultiplexerArgs(),
            utils.getArgumentNameByType(ArgumentsType.USE_SIM_TIME): 'True',
        }.items(),
    )
    launch_description.add_action(twist_mult)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.PARAMS_YAML): 
                getGazeboParamToYaml(),
            utils.getArgumentNameByType(ArgumentsType.WORLD_FILE): 
                os.path.join(utils.getRealPackageDirPath(), 'worlds', 'cube.world'),
            # [DEBUG]
            #'verbose': 'true'
        }.items()
    )
    launch_description.add_action(gazebo)

    spawnIntoEnvironment = Node(
         package='gazebo_ros', 
         executable='spawn_entity.py',
         arguments=[
             '-topic', utils.getRobotFrameDescTopicName(), 
             '-entity', utils.getRobotFrameNameToSpawn()],
         output='screen'
    )
    spawnIntoEnvironmentTimer = TimerAction(
        period=3.0, 
        actions=[spawnIntoEnvironment])
    launch_description.add_action(spawnIntoEnvironmentTimer)
    launch_description.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=spawnIntoEnvironment,
            on_start=[LogInfo(msg='[FRAME] Spawn action started')]
        )
    ))
    
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'cameras.launch.py')
        ]),
        condition=IfCondition(use_cameras),
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.NAMESPACE): 'cameras',
        }.items(),
    )
    launch_description.add_action(cameras)

    ros2Controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'controllers.py')
        ]),
        condition=IfCondition(use_ros2_control),
        launch_arguments={
            utils.getArgumentNameByType(ArgumentsType.RUN_DEMO): run_demo,
        }.items(),
    )
    launch_description.add_action(ros2Controller)

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(utils.getLaunchShareDir(), 'joystick_tx12.launch.py')
        ])
    )
    launch_description.add_action(joystick)

    return launch_description

""" [DEBUG]
"""
if __name__ == '__main__':
    
    try:
        # [DEBUG]
        generate_launch_description()

    except Exception as ex:
        print(f"[ERROR]: {ex}")

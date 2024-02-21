from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction, DeclareLaunchArgument
import time
import sys





################################################################################
######################## LAUNCH DESCRIPTION ####################################
################################################################################

def generate_launch_description():
    
    ld = LaunchDescription()

    print("\033[92m"+
    "  _____                _         _                  \n"+
    " |  __ \              | |       | |                 \n"+
    " | |__) |___  __ _  __| |_   _  | |_ ___            \n"+
    " |  _  // _ \/ _` |/ _` | | | | | __/ _ \           \n"+
    " | | \ \  __/ (_| | (_| | |_| | | || (_) |          \n"+
    " |_|  \_\___|\__,_|\__,_|\__, |  \__\___/   _       \n"+
    " | |     | |              __/ |            | |      \n"+
    " | |_ ___| | ___  ___  _ |___/___ _ __ __ _| |_ ___ \n"+
    " | __/ _ \ |/ _ \/ _ \| '_ \ / _ \ '__/ _` | __/ _ \ \n"+
    " | ||  __/ |  __/ (_) | |_) |  __/ | | (_| | ||  __/\n"+
    "  \__\___|_|\___|\___/| .__/ \___|_|  \__,_|\__\___|\n"
    "                      | |                           \n"+
    "                      |_|                           \n\033[00m")

    # time.sleep(1)
    # Get path to haption_ws from command line
    for arg in sys.argv:
        if arg.startswith("haption_ws_path:="):
            haption_ws_path = str(arg.split(":=")[1])




    print("\n\n\033[91mREMEMBER TO ACTIVATE FORCE FEEDBACK BUTTON ON HAPTIC DEVICE\033[00m\n\n")
    time.sleep(1)

    ############################## HAPTIC DEVICE CONTROL NODE ##################
    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period = 2.0,
        actions = [Node(
        package="haption_raptor_api",
        executable="raptor_api_wrapper"
        )]

    )

    haptic_parameters_calibration = haption_ws_path + "src/test_calibration/parameters.yaml"


    # CALIBRATION NODE
    haptic_calibration_node = Node(
        package="test_calibration",
        executable="test_calibration",
        parameters=[
            ParameterFile(haptic_parameters_calibration)
        ]   
    )

    # SET RIGHT PATH TO YAML
    haptic_parameters_control = haption_ws_path + "src/haptic_control/parameters.yaml"

    haptic_control_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="haptic_control",
                executable="haptic_control",
                parameters=[
                    ParameterFile(haptic_parameters_control)
                ]
            )
        ]
    )


    ld.add_action(haptic_wrapper)
    ld.add_action(haptic_calibration_node)
    ld.add_action(haptic_control_node)
    return ld



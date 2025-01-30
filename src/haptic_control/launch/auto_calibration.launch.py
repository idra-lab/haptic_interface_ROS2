from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction
# get package share directory
from ament_index_python.packages import get_package_share_directory



################################################################################
######################## LAUNCH DESCRIPTION ####################################
################################################################################

def generate_launch_description():
    
    ld = LaunchDescription()

    print("\033[92mCalibration started\033[00m")


    ############################## HAPTIC DEVICE CONTROL NODE ##################
    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period = 0.0,
        actions = [Node(
        package="haption_raptor_api",
        executable="raptor_api_wrapper"
        )]

    )
    haption_calibration_package_share = get_package_share_directory('test_calibration')
    haptic_parameters_calibration = haption_calibration_package_share + "/config/parameters.yaml"


    # CALIBRATION NODE
    haptic_calibration_node = TimerAction(
        period = 3.0,
        actions = [Node(
        package="test_calibration",
        executable="test_calibration",
        parameters=[ParameterFile(haptic_parameters_calibration)]
        )]
    )

    
    ld.add_action(haptic_wrapper)
    ld.add_action(haptic_calibration_node)
    return ld



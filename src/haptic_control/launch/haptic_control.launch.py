from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

import time
import sys




#################### NODE FOR CALLING CALIBRATION HAPTIC DEVICE #################
import rclpy
from rclpy.node import Node as RCLNode
from rokubimini_msgs.srv import ResetWrench

class FTReset(RCLNode):
    def __init__(self):
        super().__init__('reset_wrench')
        self.cli = self.create_client(ResetWrench, '/bus0/ft_sensor0/reset_wrench')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ResetWrench.Request()

    def send_request(self):
        self.req.desired_wrench.force.x = 0.0
        self.req.desired_wrench.force.y = 0.0
        self.req.desired_wrench.force.z = 0.0
        self.req.desired_wrench.torque.x = 0.0
        self.req.desired_wrench.torque.y = 0.0
        self.req.desired_wrench.torque.z = 0.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
################################################################################



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

    calibration_pkg_share = get_package_share_directory('test_calibration')
    haptic_control_pkg_share = get_package_share_directory('haptic_control')

    ############################## HAPTIC DEVICE CALIBRATION ###################
    print("\n\n\n\033[93m WARNING: ROBOT MUST NOT BE IN CONTACT WITH ANYTHING DURING CALIBRATION\033[00m\n\n")
    # time.sleep(3)
    print("\n\n\033[92mRESETTING FORCE SENSOR...\033[00m\n")
    rclpy.init()
    reset_wrench_node = FTReset()
    response = reset_wrench_node.send_request()
    reset_wrench_node.get_logger().info('\n\n'+str(response))
    while 'success=True' not in str(response):
        print("\n\n\nERROR: Force sensor not reset, trying again...\n\n\n")
        time.sleep(0.5)
        response = reset_wrench_node.send_request()
    reset_wrench_node.destroy_node()
    rclpy.shutdown()

    print("\n\n\033[91mREMEMBER TO ACTIVATE FORCE FEEDBACK BUTTON ON HAPTIC DEVICE\033[00m\n\n")

    ############################## HAPTIC DEVICE CONTROL NODE ##################
    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period = 2.0,
        actions = [Node(
        package="haption_raptor_api",
        executable="raptor_api_wrapper"
        )]

    )

    haptic_parameters_calibration = calibration_pkg_share + "/config/parameters.yaml"


    # CALIBRATION NODE
    haptic_calibration_node = TimerAction(
        period = 2.0,
        actions = [Node(
        package="test_calibration",
        executable="test_calibration",
        parameters=[ParameterFile(haptic_parameters_calibration)]
        )]
    )

    # SET RIGHT PATH TO YAML
    haptic_parameters_control = haptic_control_pkg_share + "/config/parameters.yaml"

    haptic_control_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="haptic_control",
                executable="haptic_control",
                # remappings=[('/target_frame', '/target_frame_haptic')],
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



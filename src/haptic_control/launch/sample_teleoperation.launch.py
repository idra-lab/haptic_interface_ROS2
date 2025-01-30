from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction

# get package share directory
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

# PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch.actions import LogInfo


def generate_launch_description():

    ld = LaunchDescription()

    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period=0.0,
        actions=[Node(package="haption_raptor_api", 
                      executable="raptor_api_wrapper",
                      # prefix=['xterm  -fa "Monospace" -fs 14 -e gdb -ex run --args']
                      )
        ],
    )

    # SET RIGHT PATH TO YAML
    haptic_params = get_package_share_directory("haptic_control") + "/config/haptic_parameters.yaml"
    robot_params = get_package_share_directory("haptic_control") + "/config/ur3e_parameters.yaml"
    
    sample_teleoperation = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="haptic_control",
                executable="sample_teleoperation",
                # remappings=[('/target_frame', '/target_frame_haptic')],
                parameters=[
                    ParameterFile(haptic_params), ParameterFile(robot_params)
                ],
                remappings=[
                    (
                        # remember to set the right ft_sensor_rate in the haptic yaml file
                        "bus0/ft_sensor0/ft_sensor_readings/wrench", # 1 kHz
                        "/force_torque_sensor_broadcaster/wrench", # 0.5 kHz
                    )
                ],
            )
        ],
    )
    ld.add_action(haptic_wrapper)
    ld.add_action(sample_teleoperation)
    return ld

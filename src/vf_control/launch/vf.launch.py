from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction, DeclareLaunchArgument

# get package share directory
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

# PythonExpression
from launch.conditions import LaunchConfigurationEquals
from launch.actions import LogInfo
import time
import sys


def generate_launch_description():

    ld = LaunchDescription()

    # LOGINFO
    ld.add_action(
        DeclareLaunchArgument(
            "use_fixtures",
            default_value="false",
            description="Use this argument to activate the fixtures (true or false)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "delay",
            default_value="0.0",
            description="Use this argument to simulate a delay in the system (in seconds), use 0.0 for no delay",
        )
    )

    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period=0.0,
        actions=[Node(package="haption_raptor_api", executable="raptor_api_wrapper")],
    )

    haptic_parameters_calibration = (
        get_package_share_directory("test_calibration") + "/config/parameters.yaml"
    )

    # CALIBRATION NODE
    haptic_calibration_node = TimerAction(
        period=0.0,
        actions=[
            Node(
                package="test_calibration",
                executable="test_calibration",
                parameters=[ParameterFile(haptic_parameters_calibration)],
            )
        ],
    )

    # SET RIGHT PATH TO YAML
    vf_params = get_package_share_directory("vf_control") + "/config/parameters.yaml"
    vf_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="vf_control",
                executable="vf_control",
                # remappings=[('/target_frame', '/target_frame_haptic')],
                parameters=[
                    ParameterFile(vf_params),
                    {"use_fixtures": LaunchConfiguration("use_fixtures")},
                    {"delay": LaunchConfiguration("delay")},
                ],
                remappings=[
                    (
                        "bus0/ft_sensor0/ft_sensor_readings/wrench",
                        "/force_torque_sensor_broadcaster/wrench",
                    )
                ],
                # prefix=["xterm -hold -fa 'Monospace' -fs 14 -e "],
                # output='screen',
                # emulate_tty=True,
                # arguments=[('__log_level:=debug')]
            )
        ],
    )
    # load rviz if use_fixtures is true using ifcondition
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", get_package_share_directory("vf_control") + "/rviz/vf.rviz"],
        # condition=LaunchConfigurationEquals("use_fixtures", "true"),
    )
    # ld.add_action(haptic_wrapper)
    # ld.add_action(haptic_calibration_node)
    ld.add_action(vf_node)
    # ld.add_action(rviz)
    return ld

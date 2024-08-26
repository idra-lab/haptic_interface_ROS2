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
import time
import sys


def generate_launch_description():
    
    ld = LaunchDescription()

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", get_package_share_directory("vf_control") + "/config/vf.rviz"],
        output={'both': 'log'}
    )

    # SET RIGHT PATH TO YAML
    vf_params = get_package_share_directory('vf_control') + "/config/parameters.yaml"
    print("VF PARAMETERS FILE: ",vf_params)
    vf_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="vf_control",
                executable="vf_control_test",
                # remappings=[('/target_frame', '/target_frame_haptic')],
                parameters=[
                    ParameterFile(vf_params)
                ],
                prefix=["xterm -hold -fa 'Monospace' -fs 14 -e "],
                output='screen',
                emulate_tty=True,
                arguments=[('__log_level:=debug')]
            )
        ]
    )

    
    ld.add_action(rviz)
    ld.add_action(vf_node)
    return ld



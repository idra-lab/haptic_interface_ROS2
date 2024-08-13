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
        arguments=["-d", get_package_share_directory("virtual_fixture_test") + "/config/vf.rviz"],
        output={'both': 'log'}
    )
    vf_node = TimerAction(
        period = 1.0,
        actions=[Node(
        package="virtual_fixture_test",
        executable="virtual_fixture_test",
        
        )
        ]
    )

    
    # ld.add_action(rviz)
    ld.add_action(vf_node)
    return ld



from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, OpaqueFunction
# PythonExpression
from launch.conditions import LaunchConfigurationEquals




launch_args = [
    DeclareLaunchArgument("delay", default_value="0.0", description="Use this argument to simulate a delay in the system (in seconds), use 0.0 for no delay"),
    DeclareLaunchArgument("use_fixtures", default_value="false", description="Use this argument to activate the fixtures (true or false)"),
    DeclareLaunchArgument("robot_type", default_value="kuka", description="Use this argument to select the robot type (ur3e, kuka)"),
]

def launch_setup(context):
    # SET RIGHT PATH TO YAML
    haptic_params = get_package_share_directory("haptic_control") + "/config/haptic_parameters.yaml"
    robot_params = get_package_share_directory("haptic_control") + "/config/" + LaunchConfiguration('robot_type').perform(context) + "_parameters.yaml"

    # haptic_wrapper
    haptic_wrapper = TimerAction(
        period=0.0,
        actions=[Node(package="haption_raptor_api", 
                      executable="raptor_api_wrapper",
                      # prefix=['xterm  -fa "Monospace" -fs 14 -e gdb -ex run --args']
                      )
        ],
    )
    
    vf_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package="haptic_control",
                executable="haptic_control",
                # remappings=[('/target_frame', '/target_frame_haptic')],
                parameters=[
                    ParameterFile(haptic_params), ParameterFile(robot_params),
                    {"use_fixtures": LaunchConfiguration("use_fixtures")},
                    {"delay": LaunchConfiguration("delay")},
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
        arguments=["-d", get_package_share_directory("haptic_control") + "/rviz/vf.rviz"],
        condition=LaunchConfigurationEquals("use_fixtures", "true"),
    )
    return [haptic_wrapper, vf_node, rviz]
    


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    opfunc = OpaqueFunction(function = launch_setup)
    ld.add_action(opfunc)
    
    return ld

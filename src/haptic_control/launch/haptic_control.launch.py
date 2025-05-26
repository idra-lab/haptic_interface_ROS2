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
import subprocess
import os
import datetime
DEFAULT_INPUT_MESH_PATH = "/home/nardi/temp/vf_output/virtual_fixtures.obj"
DEFAULT_SKIN_MESH_PATH = "/home/nardi/temp/vf_output/skin_mesh.obj"
DEFAULT_BAG_PATH = "/home/nardi/temp/vf_output/exp_bag"
DEFAULT_SKIN_PARAMS_PATH = "/home/nardi/temp/skel_output/optimized_skel_params.json"

launch_args = [
    DeclareLaunchArgument("delay", default_value="0.0", description="Use this argument to simulate a delay in the system (in seconds), use 0.0 for no delay"),
    DeclareLaunchArgument("use_fixtures", default_value="false", description="Use this argument to activate the fixtures (true or false)"),
    DeclareLaunchArgument("bag_path", default_value=DEFAULT_BAG_PATH, description="Use this argument to select the path to the bag file"),
    DeclareLaunchArgument("input_mesh_path", default_value=DEFAULT_INPUT_MESH_PATH, description="Use this argument to select the file to the virtual fixtures mesh"),
    DeclareLaunchArgument("skin_mesh_path", default_value=DEFAULT_SKIN_MESH_PATH, description="Use this argument to select the file to the patient skin mesh"),
    DeclareLaunchArgument("skin_params_path", default_value=DEFAULT_SKIN_PARAMS_PATH, description="Use this argument to select the file to the skin parameters"),
    DeclareLaunchArgument("robot_type", default_value="kuka", description="Use this argument to select the robot type (ur3e, kuka)"),
]

def launch_setup(context):
    # SET RIGHT PATH TO YAML
    haptic_params = get_package_share_directory("haptic_control") + "/config/haptic_parameters.yaml"
    robot_params = get_package_share_directory("haptic_control") + "/config/" + LaunchConfiguration('robot_type').perform(context) + "_parameters.yaml"
    use_fixtures = True if LaunchConfiguration("use_fixtures").perform(context) == "true" else False

    bag_path = LaunchConfiguration("bag_path").perform(context)
    # remove folder if exists
    os.system("rm -r " + "adjacency_dict*.txt")
    if os.path.exists(bag_path):
        datetime_now = datetime.datetime.now()
        datetime_str = datetime_now.strftime("%Y-%m-%d_%H-%M-%S")
        bag_path = bag_path[:-1] + "_" + datetime_str
        print("\033[91m \n\nBag file already exists, creating another with suffix: ", datetime_str, "\033[0m")
        # exit(0)
    #     os.system("rm -r " + bag_path)
    CMD = "ros2 bag record /current_frame /lbr/target_frame /desired_frame /lbr/force_torque_broadcaster/wrench /lbr/state /us_image/compressed -o " + bag_path + " & "

    subprocess.Popen(CMD, shell=True)

    print("\033[92m \n\nuse_fixtures: ", use_fixtures, "\033[0m")
    print("\033[92m Recording bag in: ", bag_path, "\033[0m")
    input_mesh_path = LaunchConfiguration("input_mesh_path").perform(context)
    skin_mesh_path = LaunchConfiguration("skin_mesh_path").perform(context)
    params_path = LaunchConfiguration("skin_params_path").perform(context)
    if use_fixtures:
        print("\033[92m \n\nUsing fixtures \033[0m")
        print("\033[92m input_mesh_path: ", input_mesh_path, "\033[0m")
        print("\033[92m skin_mesh_path: ", skin_mesh_path, "\033[0m \n\n")
    else:
        print("\033[92m Not using fixtures \033[0m")
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
                    {"vf_parameters.input_mesh_path": input_mesh_path},
                    {"vf_parameters.skin_mesh_path": skin_mesh_path},
                    {"vf_parameters.skin_params_path": params_path},
                    {"robot_type": LaunchConfiguration("robot_type")},
                    {"delay": LaunchConfiguration("delay")},
                ],
                # prefix=["xterm -hold -fa 'Monospace' -fs 14 -e "],
                output='screen',
                emulate_tty=True,
                arguments=[('__log_level:=debug')]
            )
        ],
    )
    # load rviz if use_fixtures is true using ifcondition
    rviz_vf = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", get_package_share_directory("haptic_control") + "/rviz/vf.rviz"],
        condition=LaunchConfigurationEquals("use_fixtures", "true"),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", get_package_share_directory("haptic_control") + "/rviz/rviz.rviz"],
        condition=LaunchConfigurationEquals("use_fixtures", "false"),
    )
    return [haptic_wrapper, vf_node, rviz_vf, rviz]
    


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    opfunc = OpaqueFunction(function = launch_setup)
    ld.add_action(opfunc)
    
    return ld

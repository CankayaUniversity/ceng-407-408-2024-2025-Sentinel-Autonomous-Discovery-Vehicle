from typing import List
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def declare_args() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="true",
            description="Use ros2_control instead of gazebo_control plugin",
        ),
        DeclareLaunchArgument(
            "use_autonomous",
            default_value="true",
            description="Enable autonomous movement with exploration",
        ),
        DeclareLaunchArgument(
            "use_manual",
            default_value="false",
            description="Enable manual movement with teleop nodes",
        ),
        DeclareLaunchArgument(
            "use_builtin",
            default_value="true",
            description="Use builtin manual controller (teleop nodes)",
        ),

        DeclareLaunchArgument(
            "timer_period",
            default_value="30.0",
            description="Timer Period",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = "false"
    use_3d_map = "false"

    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_autonomous = LaunchConfiguration("use_autonomous")
    use_builtin = LaunchConfiguration("use_builtin")
    timer_period = LaunchConfiguration("timer_period")

    model = get_launch_file(
        package_name="model",
        launch_file_name="rsp.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            "use_3d_map": use_3d_map,
        },
    )

    movement = get_launch_file(
        package_name="movement",
        launch_file_name="movement.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_builtin": use_builtin,
            "use_autonomous": use_autonomous,
        },
    )

    mapping = get_launch_file(
        package_name="mapping",
        launch_file_name="mapping.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_3d_map": use_3d_map,
        },
    )

    lidar = load_lidar()


    joint_spwaner,skid_spwaner,controller_manager = load_ros2_control(use_ros2_control)

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    delayed_skid_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[skid_spwaner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_spwaner],
        )
    )



    ld = LaunchDescription(declare_args())
    ld.add_action(model)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_skid_spawner)
    ld.add_action(delayed_joint_broad_spawner)
    ld.add_action(movement)
    ld.add_action(mapping)
    ld.add_action(lidar)
    return ld


def get_launch_file(
    package_name: str, launch_file_name: str, launch_arguments: dict
) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), "launch", launch_file_name]
            )
        ),
        launch_arguments=launch_arguments.items(),
    )

def load_lidar():
    params_file = os.path.join(get_package_share_directory("sentinel"),'config','lidar_params.yaml')
    return Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_node",
        parameters=[params_file],
        remappings=[
            ("/scan", "/scan"),
        ],
    )


def load_ros2_control(use_ros2_control: LaunchConfiguration):
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory("model"),'config','ros2_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
    joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        condition=IfCondition(use_ros2_control),
    )
    skid_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["skid_steer_cont"],
        condition=IfCondition(use_ros2_control),
    )
    return [joint_spawner, skid_spawner,controller_manager]
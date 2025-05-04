from typing import List

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
            default_value="true",
            description="Enable manual movement with teleop nodes",
        ),
        DeclareLaunchArgument(
            "use_builtin",
            default_value="true",
            description="Use builtin manual controller (teleop nodes)",
        ),
        DeclareLaunchArgument(
            "use_3d_map",
            default_value="true",
            description="Enable 3D map",
        ),
        DeclareLaunchArgument(
            "timer_period",
            default_value="30.0",
            description="Timer Period",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="edifice.sdf",
            description="World name to load",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = "true"
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_autonomous = LaunchConfiguration("use_autonomous")
    use_manual = LaunchConfiguration("use_manual")
    use_builtin = LaunchConfiguration("use_builtin")
    use_3d_map = LaunchConfiguration("use_3d_map")
    timer_period = LaunchConfiguration("timer_period")
    world = LaunchConfiguration("world")

    model = get_launch_file(
        package_name="model",
        launch_file_name="rsp.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
            "use_3d_map": use_3d_map,
        },
    )

    gazebo = get_launch_file(
        package_name="simulation",
        launch_file_name="gazebo.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": world,
        },
    )

    movement = get_launch_file(
        package_name="movement",
        launch_file_name="movement.launch.py",
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_builtin": use_builtin,
            "use_autonomous": use_autonomous,
            "use_manual": use_manual,
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

    object_detection = Node(
        package="object_detection",
        executable="run",
        name="object_detection",
    )

    rviz2 = get_rviz2(use_3d_map)

    ros2_control = load_ros2_control(use_ros2_control)

    timer_action = TimerAction(
        period=timer_period, actions=ros2_control + [movement, mapping, object_detection] + rviz2
    )

    ld = LaunchDescription(declare_args())
    ld.add_action(model)
    ld.add_action(gazebo)
    ld.add_action(timer_action)

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


def get_rviz2(use_3d_map: LaunchConfiguration) -> List[Node]:
    rviz2_2d = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("sentinel"), "config", "simulation.rviz"]
            ),
        ],
        condition=UnlessCondition(use_3d_map),
    )

    rviz2_3d = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("sentinel"), "config", "3d_map.rviz"]
            ),
        ],
        condition=IfCondition(use_3d_map),
    )

    return [rviz2_2d, rviz2_3d]


def load_ros2_control(use_ros2_control: LaunchConfiguration):
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
    return [joint_spawner, skid_spawner]

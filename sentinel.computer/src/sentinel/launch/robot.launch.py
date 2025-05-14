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
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare


def declare_args() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "use_autonomous",
            default_value="false",
            description="Enable autonomous movement with exploration",
        ),
        DeclareLaunchArgument(
            "use_manual",
            default_value="true",
            description="Enable manual movement",
        ),
        DeclareLaunchArgument(
            "use_builtin",
            default_value="true",
            description="Use builtin manual controller (teleop nodes)",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = "false"
    use_3d_map = "false"

    use_autonomous = LaunchConfiguration("use_autonomous")
    use_manual = LaunchConfiguration("use_manual")
    use_builtin = LaunchConfiguration("use_builtin")

    camera_tunnel = Node(
        package="camera_tunnel",
        executable="run",
        name="camera_tunnel",
        output="screen",
        remappings=[
            ("raspicam/raw", "camera/image_raw"),
            ("raspicam/compressed", "camera/image_raw/compressed"),
            ("raspicam/camera_info", "camera/camera_info"),
        ],
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

    object_detection = Node(
        package="object_detection",
        executable="run",
        name="object_detection",
        output="screen",
    )

    path_finder = Node(
        package="path_finder", executable="run", name="path_finder", output="screen"
    )

    rviz2 = get_rviz2(use_3d_map)

    ld = LaunchDescription(declare_args())
    ld.add_action(camera_tunnel)
    ld.add_action(movement)
    ld.add_action(rviz2)
    ld.add_action(object_detection)
    ld.add_action(path_finder)
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


def get_rviz2(use_3d_map: LaunchConfiguration):
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

    return rviz2_2d

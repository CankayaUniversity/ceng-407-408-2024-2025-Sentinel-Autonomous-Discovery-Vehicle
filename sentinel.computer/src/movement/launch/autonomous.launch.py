import os

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = "movement"

    nav2_config = os.path.join(
        get_package_share_directory(package_name), "config", "nav2_params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    map_yaml_file = LaunchConfiguration(
        "map",
        default="",
    )
    # map_yaml_file = LaunchConfiguration(
    #     "map",
    #     default=os.path.join(
    #         get_package_share_directory("mapping"),
    #         "result",
    #         "map_one.yaml",
    #     ),
    # )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml_file,
            "params_file": nav2_config,
        }.items(),
    )

    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("explore_lite"),
                "launch",
                "explore.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Full path to map yaml file to load",
            ),
            nav2,
            explore,
        ]
    )

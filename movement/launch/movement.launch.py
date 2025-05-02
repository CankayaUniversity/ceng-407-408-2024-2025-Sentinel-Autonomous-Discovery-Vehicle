import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_builtin = LaunchConfiguration("use_builtin")
    use_autonomous = LaunchConfiguration("use_autonomous")
    use_manual = LaunchConfiguration("use_manual")
    package = "movement"

    manual_movement = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                "launch",
                "manual.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_builtin": use_builtin,
        }.items(),
        condition=IfCondition(use_manual),
    )
    autonomous_movement = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package),
                "launch",
                "autonomous.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_autonomous),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}, {"use_stamped": False}],
        remappings=[("/cmd_vel_out", "/skid_steer_cont/cmd_vel_unstamped")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "use_builtin",
                default_value="false",
                description="Uses joy,teleop and twist_stamper nodes",
            ),
            DeclareLaunchArgument(
                "use_autonomous",
                default_value="true",
                description="Use autonomous mode",
            ),
            DeclareLaunchArgument(
                "use_manual",
                default_value="true",
                description="Use manual mode",
            ),
            manual_movement,
            autonomous_movement,
            twist_mux,
        ]
    )

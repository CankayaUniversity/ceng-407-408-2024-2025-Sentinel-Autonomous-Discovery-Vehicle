import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_3d_map = LaunchConfiguration("use_3d_map")

    pkg_path = os.path.join(get_package_share_directory("model"))
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")

    robot_description_config = Command(
        [
            "xacro ",
            xacro_file,
            " use_ros2_control:=",
            use_ros2_control,
            " use_3d_map:=",
            use_3d_map,
            " sim_mode:=",
            use_sim_time,
        ]
    )

    params = {
        "robot_description": robot_description_config,
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="Use ros2_control if true",
            ),
            DeclareLaunchArgument(
                "use_3d_map",
                default_value="false",
                description="Use 3d map if true",
            ),
            node_robot_state_publisher,
        ]
    )

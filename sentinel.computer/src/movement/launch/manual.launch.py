import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "movement"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_builtin = LaunchConfiguration("use_builtin")

    joy_params = os.path.join(
        get_package_share_directory(package_name), "config", "joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_builtin),
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params, {"use_sim_time": use_sim_time}],
        remappings=[("/cmd_vel", "/cmd_vel_joy")],
        condition=IfCondition(use_builtin),
    )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/cmd_vel_in", "/skid_steer_cont/cmd_vel_unstamped"),
            ("/cmd_vel_out", "/skid_steer_cont/cmd_vel"),
        ],
        condition=IfCondition(use_builtin),
    )

    manual_movement_twist_publisher = Node(
        name="movement",
        package="movement",
        executable="twist",
        remappings=[("/movement", "/cmd_vel_joy")],
        condition=UnlessCondition(use_builtin),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "use_builtin",
                default_value="false",
                description="Uses joy,teleop and twist_stamper nodes",
            ),
            joy_node,
            teleop_node,
            twist_stamper,
            manual_movement_twist_publisher,
        ]
    )

import os
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def declare_args() -> List[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params.yaml"),
            description="Path to the params file",
        ),
        DeclareLaunchArgument(
            "tune_detection",
            default_value="false",
            description="Enables tuning mode for the detection",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Enables sim time for the follow node.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="The name of the input image topic.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="The name of the output command vel topic.",
        ),
        DeclareLaunchArgument(
            "enable_3d_tracker",
            default_value="false",
            description="Enables the 3D tracker node",
        ),
    ]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    tune_detection = LaunchConfiguration("tune_detection")
    use_sim_time = LaunchConfiguration("use_sim_time")
    image_topic = LaunchConfiguration("image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_3d_tracker = LaunchConfiguration("enable_3d_tracker")

    detect_node = Node(
        package='ball_tracker',
        executable='detect_ball',
        parameters=[params_file, {'tuning_mode': tune_detection}],
        remappings=[('/image_in', image_topic)],
    )

    detect_3d_node = Node(
        package='ball_tracker',
        executable='detect_ball_3d',
        parameters=[params_file],
        condition=IfCondition(enable_3d_tracker)
    )

    follow_node = Node(
        package='ball_tracker',
        executable='follow_ball',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', cmd_vel_topic)],
    )

    return LaunchDescription([
        *declare_args(),
        detect_node,
        detect_3d_node,
        follow_node,
    ])

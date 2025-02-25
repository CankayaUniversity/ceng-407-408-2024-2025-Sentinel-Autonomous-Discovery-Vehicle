from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import subprocess


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"/home/sentinel/YDLIDAR/ydlidar_ros2_driver/launch/ydlidar_launch.py"
                )
            ),
            Node(
                package="camera_publisher",
                executable="camera",
                name="camera_publisher",
                parameters=[
                    {
                        "computer_host": "192.168.171.94",
                        "computer_port": 9000,
                    }
                ],
                output="screen",
            ),
            Node(
                package="movement",
                executable="run",
                name="movement",
                output="screen",
            ),
        ]
    )

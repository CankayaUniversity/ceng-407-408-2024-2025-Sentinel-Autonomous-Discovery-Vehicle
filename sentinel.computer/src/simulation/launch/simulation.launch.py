import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('simulation'), 'sentinel.urdf')
    rviz_config_path = os.path.join(get_package_share_directory('simulation'), 'simulation.rviz')
    return LaunchDescription([
        # Robot State Publisher to read and publish robot's URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),
        # Joint State Publisher to continuously publish joint states
        Node(
            package='simulation',
            executable='state_publisher',
            name='state_publisher',
            output='screen'
        ),
        #Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])

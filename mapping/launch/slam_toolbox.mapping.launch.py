from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    # static_map = os.path.join(
    #     get_package_share_directory("mapping"),
    #     "result",
    #     "map_one_serial",
    # )

    slam_config_dir = os.path.join(
        get_package_share_directory("mapping"),
        "config",
        "mapper_params_online_async.yaml",
    )

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[
            slam_config_dir,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": use_lifecycle_manager,
                # "map_file_name": static_map,
            },
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup the slam toolbox."
            "Ignored when use_lifecycle_manager is true.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_lifecycle_manager",
            default_value="false",
            description="Enable bond connection during node activation",
        )
    )

    ld.add_action(slam_toolbox_node)

    ld.add_action(
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
            ),
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_toolbox_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    LogInfo(msg="[LifecycleLaunch] SlamToolbox node is activating."),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_toolbox_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            ),
            condition=IfCondition(
                AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
            ),
        )
    )
    return ld

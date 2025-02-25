from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(                                              # static transform publisher that publishes the transform between the camera and the base_link
            package="tf2_ros",                             # adds other links to the tf tree
            executable="static_transform_publisher",
            name="camera_base_link",
            output="screen",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "1.57", "base_link", "camera_link"] # x, y, z, roll, pitch, yaw, parent_frame, child_frame 
        ),
        
        TimerAction(
            actions=[
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    output="screen",
                    parameters=[{
                        "rtabmap_args": "--delete_db_on_start", # delete the database on start, useful for development
                        "frame_id": "base_link",
                        "odom_frame_id": "base_link",
                        "map_frame_id": "base_link", 
                        "subscribe_depth": False,          # rtab-map subscribes to depth image data (default True)
                        "subscribe_rgb": True,             # rtab-map subscribes to camera data
                        "subscribe_scan": True,            # rtab-map subscribes to lidar data 
                        "subscribe_odom": False,           # rtab-map subscribes to odometry data (default True)
                        "approx_sync": True,               # rtab-map uses approximate time synchronization for multiple data sources (default False)
                        "topic_queue_size": 30,            # the topic queue size must be greater than the Hz of topics (check topic Hz by rostopic hz {topic_name})
                        "sync_queue_size": 30,             # the topic queue size must be greater than the Hz of topics (check topic Hz by rostopic hz {topic_name})
                        "qos_image": 2,                    # reliability = best effort
                        "static_map": False,
                        "Rtabmap/DetectionRate": "100.0",  # detection rate of rtabmap default 1.0 Hz
                        "Rtabmap/PublishRate": "100.0"     # publish rate of rtabmap default 1.0 Hz
                    }],
                    remappings=[
                        ("/rgb/image", "/raspicam/raw"), # default subscription of camera topic is /rgb/image remap to out camera topic
                        ("/rgb/camera_info", "/raspicam/camera_info"), # default subscription of camera_info topic is /rgb/camera_info remap to out camera_info topic
                        ("/scan", "/scan") # default subscription of lidar topic is /scan remap to out lidar topic
                    ]
                ),
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    output="screen",
                    parameters=[{
                        "rtabmap_args": "--delete_db_on_start",
                        "frame_id": "base_link",
                        "odom_frame_id": "base_link",
                        "map_frame_id": "base_link", 
                        "subscribe_depth": False,
                        "subscribe_rgb": True,
                        "subscribe_scan": True,
                        "subscribe_odom": False,
                        "approx_sync": True,
                        "topic_queue_size": 30,
                        "sync_queue_size": 30,
                        "qos_image": 2,
                        "static_map": False,
                    }],
                    remappings=[
                        ("/rgb/image", "/raspicam/raw"),
                        ("/rgb/camera_info", "/raspicam/camera_info"),
                        ("/scan", "/scan")
                    ]
                )
            ],
            period=2.0
        ),
    ])

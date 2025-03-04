from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([

        # Start the camera_tunnel for camera data
        Node(
            package="camera_tunnel",
            executable="run",
            name="camera_tunnel",
            output="screen"
        ),

        # Start the manual_movement node for movement data
        Node(
            package="manual_movement",
            executable="run",
            name="manual_movement",
            output="screen"
        ),
        
        # Start the state_publisher for the odometry data
        Node(
            package="simulation",
            executable="state_publisher",
            name="state_publisher",
            output="screen"
        ),
        
        # Add camera link to base link in tf tree
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_base_link",
            output="screen",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "1.57", "base_link", "camera_link"]
        ),
        
        # Add lidar link to base link in tf tree
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_base_link",
            output="screen",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "base_link", "laser_frame"]
        ),
        
        TimerAction(
            actions=[
                Node(
                    package="rtabmap_slam",
                    executable="rtabmap",
                    name="rtabmap",
                    output="screen",
                    parameters=[{
                        "rtabmap_args": "--delete_db_on_start",     # Delete the database on each launch, useful for development
                        "frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "map_frame_id": "map", 
                        "subscribe_depth": False,                   # rtab-map subscribes to depth image data (default True)
                        "subscribe_rgb": True,                      # rtab-map subscribes to camera data
                        "subscribe_scan": True,                     # rtab-map subscribes to lidar data 
                        "subscribe_odom": True,                     # rtab-map subscribes to odometry data
                        "approx_sync": True,                        # rtab-map uses approximate time synchronization for multiple data sources (default False)
                        "topic_queue_size": 30,                     # the topic queue size must be greater than the Hz of topics (check topic Hz by rostopic hz {topic_name})
                        "sync_queue_size": 30,                      # the topic queue size must be greater than the Hz of topics (check topic Hz by rostopic hz {topic_name})
                        "qos_image": 2,                             # reliability = best effort
                        "Rtabmap/DetectionRate": "100.0",           # detection rate of rtabmap default 1.0 Hz
                        "Rtabmap/PublishRate": "100.0",             # publish rate of rtabmap default 1.0 Hz

                        "Odom/Strategy": "1",                       # 0=Odometry, 1=Odometry+IMU, 2=Odometry+IMU+Lidar (Inertial Measurement Unit)
                        "Odom/ResetCountdown": "1",                 # Resets odometry after a certain number of failures
                        
                        "Reg/Strategy": "1",                        # 0=Odometry, 1=Odometry+IMU, 2=Odometry+IMU+Lidar
                        "Reg/Force3DoF": "true",                    # 2D Mapping
                        "RGBD/CreateOccupancyGrid": "true",         # Generate occupancy grid
                        "Grid/FromDepth": "false",                  # Don't create grid from depth data
                        "Grid/RayTracing": "true",                  # Use ray tracing for grid mapping
                        "Grid/CellSize": "0.05",                    # Set grid cell size (5 cm)
                        
                        "Mem/STMSize": "30",                        # Short-term memory size
                        "Mem/IncrementalMemory": "true",            # Enable incremental mapping 
                        "RGBD/LinearUpdate": "0.05",                # Minimum linear movement to update map
                        "RGBD/AngularUpdate": "0.01",               # Minimum angular movement to update map
                        "Icp/VoxelSize": "0.05",                    # Voxel size for ICP (Iterative Closest Point)
                        "Icp/MaxCorrespondenceDistance": "0.1",     # Maximum correspondence distance for ICP
                        
                        "scan_cloud_max_points": 5000,              # Maximum number of points in scan cloud (scan cloud = collection of scan data points)  
                        "scan_cloud_normal_k": "5"                  # Neighborhood size for normal estimation
                    }],
                    remappings=[
                        ("/rgb/image", "/raspicam/raw"),
                        ("/rgb/camera_info", "/raspicam/camera_info"),
                        ("/scan", "/scan"),
                        ("/odom", "/tf")
                    ]
                ),
                Node(
                    package="rtabmap_viz",
                    executable="rtabmap_viz",
                    name="rtabmap_viz",
                    output="screen",
                    parameters=[{
                        "frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "map_frame_id": "map", 
                        "subscribe_depth": False,
                        "subscribe_rgb": True,
                        "subscribe_scan": True,
                        "subscribe_odom": True,
                        "subscribe_map_data": True,
                        "approx_sync": True,
                        "topic_queue_size": 30,
                        "sync_queue_size": 30,
                        "qos_image": 2,
                        "static_map": False,
                    }],
                    remappings=[
                        ("/rgb/image", "/raspicam/raw"),
                        ("/rgb/camera_info", "/raspicam/camera_info"),
                        ("/scan", "/scan"),
                        ("/odom", "/tf")
                    ]
                )
            ],
            period=2.0
        ),
    ])
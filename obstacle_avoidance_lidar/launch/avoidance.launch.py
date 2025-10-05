"""
Launch file for obstacle avoidance system.

This launch file starts the complete obstacle avoidance stack including
the frame adapter, avoidance node, and optional fake scan publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Generate launch description for obstacle avoidance system.
    """
    
    # Declare launch arguments
    use_fake_scan_arg = DeclareLaunchArgument(
        'use_fake_scan',
        default_value='false',
        description='Use fake scan publisher instead of real LiDAR'
    )
    
    use_frame_adapter_arg = DeclareLaunchArgument(
        'use_frame_adapter',
        default_value='true',
        description='Use frame adapter to wrap frame_grabber module'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            FindPackageShare('obstacle_avoidance_lidar').find('obstacle_avoidance_lidar'),
            'config',
            'params.yaml'
        ),
        description='Path to configuration file'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='2.0',
        description='Safe distance threshold in meters'
    )
    
    stop_distance_arg = DeclareLaunchArgument(
        'stop_distance',
        default_value='0.5',
        description='Stop distance threshold in meters'
    )
    
    base_speed_arg = DeclareLaunchArgument(
        'base_speed',
        default_value='1.0',
        description='Base forward speed in m/s'
    )
    
    # Get launch configurations
    use_fake_scan = LaunchConfiguration('use_fake_scan')
    use_frame_adapter = LaunchConfiguration('use_frame_adapter')
    config_file = LaunchConfiguration('config_file')
    safe_distance = LaunchConfiguration('safe_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    base_speed = LaunchConfiguration('base_speed')
    
    # Frame adapter node
    frame_adapter_node = Node(
        package='obstacle_avoidance_lidar',
        executable='frame_adapter',
        name='frame_adapter',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(
            PythonExpression([
                use_frame_adapter, ' and not ', use_fake_scan
            ])
        ),
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # Fake scan publisher node
    fake_scan_publisher_node = Node(
        package='obstacle_avoidance_lidar',
        executable='fake_scan_publisher',
        name='fake_scan_publisher',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(use_fake_scan),
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # Obstacle avoidance node
    avoidance_node = Node(
        package='obstacle_avoidance_lidar',
        executable='avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[
            config_file,
            {'safe_distance': safe_distance},
            {'stop_distance': stop_distance},
            {'base_speed': base_speed}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    # Log info about configuration
    log_info = LogInfo(
        msg=PythonExpression([
            "'Launching obstacle avoidance system with: '",
            "'safe_distance=', ", safe_distance, " 'm, '",
            "'stop_distance=', ", stop_distance, " 'm, '",
            "'base_speed=', ", base_speed, " 'm/s'"
        ])
    )
    
    return LaunchDescription([
        # Launch arguments
        use_fake_scan_arg,
        use_frame_adapter_arg,
        config_file_arg,
        safe_distance_arg,
        stop_distance_arg,
        base_speed_arg,
        
        # Log info
        log_info,
        
        # Nodes
        frame_adapter_node,
        fake_scan_publisher_node,
        avoidance_node,
    ])


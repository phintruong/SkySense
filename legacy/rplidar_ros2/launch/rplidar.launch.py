#!/usr/bin/env python3
"""
Launch file for RPLIDAR ROS 2 node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='',
        description='Serial port for RPLIDAR (empty for auto-detect)'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for LaserScan messages'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect',
        default_value='true',
        description='Auto-detect RPLIDAR port'
    )
    
    # RPLIDAR node
    rplidar_node = Node(
        package='rplidar_ros2',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'auto_detect': LaunchConfiguration('auto_detect'),
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'time_increment': 0.0,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 12.0,
            'motor_pwm': 660,
        }]
    )
    
    return LaunchDescription([
        port_arg,
        frame_id_arg,
        auto_detect_arg,
        rplidar_node,
    ])


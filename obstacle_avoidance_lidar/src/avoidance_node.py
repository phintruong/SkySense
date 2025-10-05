"""
Core obstacle avoidance node implementing VFH-style algorithm.

This node subscribes to LiDAR scan data, processes it to detect obstacles,
and publishes velocity commands for obstacle avoidance.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from typing import Optional, List, Tuple
import time
import math

# Import our utility functions
from .utils import (
    clean_scan, moving_median_filter, segment_scan, vfh_avoidance,
    calculate_velocity_commands, get_front_sector_index, normalize_angle,
    create_histogram_data
)


class ObstacleAvoidanceNode(Node):
    """
    Main obstacle avoidance node implementing VFH-style algorithm.
    
    This node processes LiDAR scans, detects obstacles, and generates
    velocity commands for safe navigation.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._load_parameters()
        
        # Initialize state
        self.last_scan_time = None
        self.current_heading = 0.0
        self.watchdog_timeout = 0.5  # seconds
        
        # Create subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        
        # Diagnostic publishers
        self.min_distance_publisher = self.create_publisher(
            Float32,
            '~/min_distance',
            10
        )
        
        self.chosen_sector_publisher = self.create_publisher(
            Int32,
            '~/chosen_sector',
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            '~/velocity_debug',
            10
        )
        
        # Visualization publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '~/avoidance_markers',
            10
        )
        
        # Create timer for watchdog and rate limiting
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # Initialize control variables
        self.last_cmd_vel = Twist()
        self.scan_data = None
        self.sector_distances = None
        self.sector_angles = None
        
        self.get_logger().info('Obstacle avoidance node initialized')
        self.get_logger().info(f'Subscribing to: {self.scan_topic}')
        self.get_logger().info(f'Publishing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Safe distance: {self.safe_distance} m')
        self.get_logger().info(f'Stop distance: {self.stop_distance} m')
    
    def _declare_parameters(self):
        """Declare all ROS parameters."""
        # Topic names
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        # Frame IDs
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_link')
        
        # Control parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('safe_distance', 2.0)
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        
        # Processing parameters
        self.declare_parameter('sector_count', 36)
        self.declare_parameter('median_window', 5)
        
        # Safety parameters
        self.declare_parameter('enable_hard_brake', True)
        self.declare_parameter('watchdog_timeout', 0.5)
    
    def _load_parameters(self):
        """Load parameters from ROS parameter server."""
        # Topic names
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Frame IDs
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        
        # Control parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        
        # Processing parameters
        self.sector_count = self.get_parameter('sector_count').value
        self.median_window = self.get_parameter('median_window').value
        
        # Safety parameters
        self.enable_hard_brake = self.get_parameter('enable_hard_brake').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
    
    def scan_callback(self, msg: LaserScan):
        """
        Callback for incoming LiDAR scan messages.
        
        Args:
            msg: LaserScan message containing range data
        """
        try:
            # Update last scan time
            self.last_scan_time = time.time()
            
            # Process the scan
            self._process_scan(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {e}')
    
    def _process_scan(self, msg: LaserScan):
        """
        Process incoming LiDAR scan data.
        
        Args:
            msg: LaserScan message
        """
        try:
            # Clean the scan data
            ranges = clean_scan(
                msg.ranges,
                msg.range_min,
                msg.range_min
            )
            
            # Apply median filter
            filtered_ranges = moving_median_filter(
                ranges,
                self.median_window
            )
            
            # Segment scan into sectors
            self.sector_distances = segment_scan(
                filtered_ranges,
                msg.angle_min,
                msg.angle_max,
                msg.angle_increment,
                self.sector_count
            )
            
            # Calculate sector angles
            from .utils import calculate_sector_angles
            self.sector_angles = calculate_sector_angles(
                self.sector_count,
                msg.angle_min,
                msg.angle_max
            )
            
            # Store scan data for control loop
            self.scan_data = {
                'ranges': filtered_ranges,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'timestamp': msg.header.stamp
            }
            
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {e}')
    
    def control_loop(self):
        """
        Main control loop that runs at the specified rate.
        """
        try:
            # Check watchdog
            if not self._check_watchdog():
                return
            
            # Check if we have valid scan data
            if self.scan_data is None or self.sector_distances is None:
                self.get_logger().warn('No valid scan data available')
                return
            
            # Perform obstacle avoidance
            cmd_vel = self._compute_avoidance_commands()
            
            # Publish commands
            self.cmd_vel_publisher.publish(cmd_vel)
            self.last_cmd_vel = cmd_vel
            
            # Publish diagnostic data
            self._publish_diagnostics()
            
            # Publish visualization markers
            self._publish_visualization()
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {e}')
    
    def _check_watchdog(self) -> bool:
        """
        Check if scan data is recent enough.
        
        Returns:
            True if data is recent, False if watchdog triggered
        """
        if self.last_scan_time is None:
            self.get_logger().warn('No scan data received yet')
            return False
        
        time_since_scan = time.time() - self.last_scan_time
        
        if time_since_scan > self.watchdog_timeout:
            self.get_logger().warn(
                f'Watchdog triggered: no scan for {time_since_scan:.2f}s'
            )
            
            # Publish zero velocities
            zero_cmd = Twist()
            self.cmd_vel_publisher.publish(zero_cmd)
            self.last_cmd_vel = zero_cmd
            
            return False
        
        return True
    
    def _compute_avoidance_commands(self) -> Twist:
        """
        Compute velocity commands using VFH-style avoidance.
        
        Returns:
            Twist message with velocity commands
        """
        try:
            # Find minimum distance for emergency stop
            min_distance = min(self.sector_distances)
            
            # Apply VFH avoidance algorithm
            chosen_angle, chosen_sector = vfh_avoidance(
                self.sector_distances,
                self.sector_angles,
                self.safe_distance,
                self.current_heading
            )
            
            # Get front clearance
            front_sector_idx = get_front_sector_index(self.sector_count)
            front_clearance = self.sector_distances[front_sector_idx]
            
            # Calculate velocity commands
            vx, vy, yaw_rate = calculate_velocity_commands(
                chosen_angle,
                self.current_heading,
                front_clearance,
                self.base_speed,
                self.max_yaw_rate,
                self.stop_distance,
                min_distance
            )
            
            # Create twist message
            cmd_vel = Twist()
            cmd_vel.linear.x = vx
            cmd_vel.linear.y = vy
            cmd_vel.angular.z = yaw_rate
            
            # Update current heading (simplified integration)
            self.current_heading += yaw_rate * (1.0 / self.control_rate)
            self.current_heading = normalize_angle(self.current_heading)
            
            # Log control decisions
            self.get_logger().debug(
                f'Control: min_dist={min_distance:.2f}m, '
                f'chosen_angle={math.degrees(chosen_angle):.1f}°, '
                f'chosen_sector={chosen_sector}, '
                f'vx={vx:.2f}m/s, yaw_rate={yaw_rate:.2f}rad/s'
            )
            
            return cmd_vel
            
        except Exception as e:
            self.get_logger().error(f'Error computing avoidance commands: {e}')
            # Return zero velocities on error
            return Twist()
    
    def _publish_diagnostics(self):
        """Publish diagnostic information."""
        try:
            if self.sector_distances is not None:
                # Publish minimum distance
                min_dist_msg = Float32()
                min_dist_msg.data = min(self.sector_distances)
                self.min_distance_publisher.publish(min_dist_msg)
                
                # Publish chosen sector (if available)
                if hasattr(self, '_last_chosen_sector'):
                    sector_msg = Int32()
                    sector_msg.data = self._last_chosen_sector
                    self.chosen_sector_publisher.publish(sector_msg)
                
                # Publish velocity debug
                self.velocity_publisher.publish(self.last_cmd_vel)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing diagnostics: {e}')
    
    def _publish_visualization(self):
        """Publish visualization markers for RViz."""
        try:
            if self.sector_distances is None or self.sector_angles is None:
                return
            
            marker_array = MarkerArray()
            
            # Create sector markers
            for i, (angle, distance) in enumerate(zip(self.sector_angles, self.sector_distances)):
                marker = Marker()
                marker.header.frame_id = self.laser_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                # Set position and orientation
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.0
                
                # Calculate arrow endpoint
                end_x = distance * math.cos(angle)
                end_y = distance * math.sin(angle)
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = math.sin(angle / 2.0)
                marker.pose.orientation.w = math.cos(angle / 2.0)
                
                # Set scale
                marker.scale.x = distance * 0.8  # Arrow length
                marker.scale.y = 0.05  # Arrow width
                marker.scale.z = 0.05  # Arrow height
                
                # Set color based on distance
                if distance < self.stop_distance:
                    # Red for obstacles
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif distance < self.safe_distance:
                    # Yellow for caution
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                else:
                    # Green for clear
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                
                marker.color.a = 0.7
                
                marker_array.markers.append(marker)
            
            # Create chosen direction marker
            if hasattr(self, '_last_chosen_angle'):
                chosen_marker = Marker()
                chosen_marker.header.frame_id = self.laser_frame
                chosen_marker.header.stamp = self.get_clock().now().to_msg()
                chosen_marker.id = 1000
                chosen_marker.type = Marker.ARROW
                chosen_marker.action = Marker.ADD
                
                chosen_marker.pose.position.x = 0.0
                chosen_marker.pose.position.y = 0.0
                chosen_marker.pose.position.z = 0.1
                
                chosen_marker.pose.orientation.x = 0.0
                chosen_marker.pose.orientation.y = 0.0
                chosen_marker.pose.orientation.z = math.sin(self._last_chosen_angle / 2.0)
                chosen_marker.pose.orientation.w = math.cos(self._last_chosen_angle / 2.0)
                
                chosen_marker.scale.x = 2.0
                chosen_marker.scale.y = 0.1
                chosen_marker.scale.z = 0.1
                
                chosen_marker.color.r = 0.0
                chosen_marker.color.g = 0.0
                chosen_marker.color.b = 1.0
                chosen_marker.color.a = 1.0
                
                marker_array.markers.append(chosen_marker)
            
            self.marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing visualization: {e}')


def main(args=None):
    """
    Main function to run the obstacle avoidance node.
    """
    rclpy.init(args=args)
    
    try:
        avoidance_node = ObstacleAvoidanceNode()
        rclpy.spin(avoidance_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in avoidance node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


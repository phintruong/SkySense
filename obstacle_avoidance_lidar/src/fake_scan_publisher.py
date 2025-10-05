"""
Fake scan publisher for testing obstacle avoidance without real LiDAR hardware.

This node publishes synthetic LaserScan messages with configurable obstacles
to test the obstacle avoidance system in simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
import math
import time
from typing import List, Tuple, Dict, Any


class FakeScanPublisher(Node):
    """
    Node that publishes fake LiDAR scan data for testing.
    
    This node generates synthetic LaserScan messages with configurable
    obstacles to test the obstacle avoidance system.
    """
    
    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self._load_parameters()
        
        # Create publisher
        self.scan_publisher = self.create_publisher(
            LaserScan,
            self.scan_topic,
            10
        )
        
        # Create timer for publishing
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.publish_scan)
        
        # Initialize obstacle simulation
        self._initialize_obstacles()
        
        # Calculate number of measurements
        self.num_measurements = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Time tracking for dynamic obstacles
        self.start_time = time.time()
        
        self.get_logger().info('Fake scan publisher initialized')
        self.get_logger().info(f'Publishing on topic: {self.scan_topic}')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Number of measurements: {self.num_measurements}')
        self.get_logger().info(f'Number of obstacles: {self.obstacle_count}')
    
    def _declare_parameters(self):
        """Declare all ROS parameters."""
        # Topic names
        self.declare_parameter('scan_topic', '/scan')
        
        # Frame IDs
        self.declare_parameter('frame_id', 'laser_link')
        
        # Publishing parameters
        self.declare_parameter('publish_rate', 10.0)
        
        # LiDAR scan parameters
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('scan_time', 0.0)
        self.declare_parameter('time_increment', 0.0)
        
        # Obstacle simulation parameters
        self.declare_parameter('obstacle_count', 3)
        self.declare_parameter('obstacle_distance', 2.0)
        self.declare_parameter('noise_level', 0.1)
        self.declare_parameter('dynamic_obstacles', True)
    
    def _load_parameters(self):
        """Load parameters from ROS parameter server."""
        # Topic names
        self.scan_topic = self.get_parameter('scan_topic').value
        
        # Frame IDs
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publishing parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # LiDAR scan parameters
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        self.time_increment = self.get_parameter('time_increment').value
        
        # Obstacle simulation parameters
        self.obstacle_count = self.get_parameter('obstacle_count').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.noise_level = self.get_parameter('noise_level').value
        self.dynamic_obstacles = self.get_parameter('dynamic_obstacles').value
    
    def _initialize_obstacles(self):
        """Initialize obstacle positions and properties."""
        self.obstacles = []
        
        for i in range(self.obstacle_count):
            obstacle = {
                'angle': (i * 2 * math.pi / self.obstacle_count) + np.random.uniform(-0.5, 0.5),
                'distance': self.obstacle_distance + np.random.uniform(-0.5, 0.5),
                'width': np.random.uniform(0.2, 0.8),  # angular width
                'height': np.random.uniform(0.3, 1.0),  # height variation
                'speed': np.random.uniform(0.1, 0.5) if self.dynamic_obstacles else 0.0,
                'direction': np.random.choice([-1, 1])  # movement direction
            }
            self.obstacles.append(obstacle)
        
        self.get_logger().info(f'Initialized {len(self.obstacles)} obstacles')
    
    def _update_obstacles(self):
        """Update obstacle positions for dynamic simulation."""
        if not self.dynamic_obstacles:
            return
        
        current_time = time.time() - self.start_time
        
        for obstacle in self.obstacles:
            if obstacle['speed'] > 0:
                # Move obstacle in a circular pattern
                obstacle['angle'] += obstacle['speed'] * obstacle['direction'] * 0.1
                
                # Keep angle in valid range
                while obstacle['angle'] > math.pi:
                    obstacle['angle'] -= 2 * math.pi
                while obstacle['angle'] < -math.pi:
                    obstacle['angle'] += 2 * math.pi
                
                # Vary distance slightly
                obstacle['distance'] += np.random.uniform(-0.05, 0.05)
                obstacle['distance'] = max(0.5, min(5.0, obstacle['distance']))
    
    def _generate_scan_ranges(self) -> List[float]:
        """
        Generate synthetic range measurements.
        
        Returns:
            List of range measurements
        """
        ranges = []
        
        for i in range(self.num_measurements):
            angle = self.angle_min + i * self.angle_increment
            
            # Start with maximum range
            range_val = self.range_max
            
            # Check for obstacles
            for obstacle in self.obstacles:
                obstacle_angle = obstacle['angle']
                obstacle_distance = obstacle['distance']
                obstacle_width = obstacle['width']
                
                # Check if this angle is within the obstacle
                angle_diff = abs(angle - obstacle_angle)
                # Handle angle wrapping
                angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
                
                if angle_diff < obstacle_width / 2.0:
                    # This angle hits the obstacle
                    range_val = min(range_val, obstacle_distance)
            
            # Add some noise
            if self.noise_level > 0:
                range_val += np.random.normal(0, self.noise_level)
            
            # Clip to valid range
            range_val = max(self.range_min, min(self.range_max, range_val))
            
            ranges.append(range_val)
        
        return ranges
    
    def publish_scan(self):
        """Publish a synthetic LaserScan message."""
        try:
            # Update dynamic obstacles
            self._update_obstacles()
            
            # Generate scan ranges
            ranges = self._generate_scan_ranges()
            
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            
            # Set scan parameters
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = self.time_increment
            scan_msg.scan_time = self.scan_time
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            # Set ranges
            scan_msg.ranges = ranges
            
            # Publish the scan
            self.scan_publisher.publish(scan_msg)
            
            # Log debug info occasionally
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            if self._debug_counter % 50 == 0:  # Log every 50 scans
                min_range = min(ranges) if ranges else 0.0
                max_range = max(ranges) if ranges else 0.0
                self.get_logger().debug(
                    f'Published fake scan: min_range={min_range:.2f}m, '
                    f'max_range={max_range:.2f}m, obstacles={len(self.obstacles)}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error publishing fake scan: {e}')


def main(args=None):
    """
    Main function to run the fake scan publisher.
    """
    rclpy.init(args=args)
    
    try:
        fake_publisher = FakeScanPublisher()
        rclpy.spin(fake_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in fake scan publisher: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


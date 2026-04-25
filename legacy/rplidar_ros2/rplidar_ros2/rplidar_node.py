#!/usr/bin/env python3
"""
RPLIDAR ROS 2 Node

ROS 2 node that publishes LaserScan messages from RPLIDAR A1 sensor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import glob
import logging
import time
from typing import Optional

from .rplidar_protocol import RPLidarProtocol, RPLidarHealth

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def find_rplidar_port() -> Optional[str]:
    """
    Auto-detect RPLIDAR port.
    
    Returns:
        Port path (e.g., '/dev/ttyUSB0') or None if not found
    """
    # Common Linux USB serial ports
    possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    for port in sorted(possible_ports):
        try:
            # Try to connect and get info
            lidar = RPLidarProtocol(port=port)
            if lidar.connect():
                info = lidar.get_info()
                lidar.disconnect()
                if info:
                    logger.info(f"Found RPLIDAR on {port}")
                    return port
        except Exception:
            continue
    
    logger.warning("RPLIDAR not found on any port")
    return None


class RPLidarNode(Node):
    """
    ROS 2 node for RPLIDAR A1 sensor.
    
    Publishes sensor_msgs/msg/LaserScan messages on /scan topic.
    """
    
    def __init__(self):
        """Initialize RPLIDAR ROS 2 node."""
        super().__init__('rplidar_node')
        
        # Declare parameters
        self.declare_parameter('port', '')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.00872665)
        self.declare_parameter('time_increment', 0.0)
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('motor_pwm', 660)
        self.declare_parameter('auto_detect', True)
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.time_increment = self.get_parameter('time_increment').get_parameter_value().double_value
        self.scan_time = self.get_parameter('scan_time').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        motor_pwm = self.get_parameter('motor_pwm').get_parameter_value().integer_value
        auto_detect = self.get_parameter('auto_detect').get_parameter_value().bool_value
        
        # Auto-detect port if not specified
        if not port and auto_detect:
            port = find_rplidar_port()
            if not port:
                self.get_logger().error("Failed to auto-detect RPLIDAR port")
                raise RuntimeError("RPLIDAR port not found")
        
        if not port:
            port = '/dev/ttyUSB0'  # Default fallback
            self.get_logger().warn(f"Using default port: {port}")
        
        self.get_logger().info(f"Using RPLIDAR port: {port}")
        
        # Initialize RPLIDAR
        self.lidar = RPLidarProtocol(port=port)
        
        if not self.lidar.connect():
            self.get_logger().error("Failed to connect to RPLIDAR")
            raise RuntimeError("RPLIDAR connection failed")
        
        # Get device info
        info = self.lidar.get_info()
        if info:
            self.get_logger().info(f"RPLIDAR Model: {info['model']}, "
                                  f"Firmware: {info['firmware']}, "
                                  f"Serial: {info['serial']}")
        
        # Check health
        health = self.lidar.get_health()
        if health:
            if health['status'] == RPLidarHealth.GOOD:
                self.get_logger().info("RPLIDAR health: GOOD")
            elif health['status'] == RPLidarHealth.WARNING:
                self.get_logger().warn("RPLIDAR health: WARNING")
            else:
                self.get_logger().error("RPLIDAR health: ERROR")
        
        # Start motor
        if not self.lidar.start_motor(pwm=motor_pwm):
            self.get_logger().error("Failed to start RPLIDAR motor")
            self.lidar.disconnect()
            raise RuntimeError("Failed to start motor")
        
        # Wait for motor to stabilize
        time.sleep(2.0)
        
        # Create publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # Create timer for publishing scans
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info("RPLIDAR node started, publishing on /scan")
        
        # Track last scan time
        self.last_scan_time = time.time()
        self.scan_data_cache = []
    
    def publish_scan(self) -> None:
        """Publish LaserScan message."""
        try:
            # Collect scan data
            scan_points = []
            scan_start_time = time.time()
            
            # Get one complete scan
            try:
                for points in self.lidar.iter_scans():
                    scan_points = points
                    break  # Get first complete scan
            except Exception as e:
                self.get_logger().warn(f"Error reading scan: {e}")
                # Try to reconnect
                self.reconnect_lidar()
                return
            
            if not scan_points:
                return
            
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = self.time_increment
            scan_msg.scan_time = self.scan_time
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            # Convert points to ranges array
            # Create array with NaN for missing angles
            import math
            num_points = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
            ranges = [float('nan')] * num_points
            intensities = [0.0] * num_points
            
            for angle_deg, distance_mm, quality in scan_points:
                # Convert to radians
                angle_rad = math.radians(angle_deg)
                
                # Normalize angle to [-pi, pi]
                while angle_rad > math.pi:
                    angle_rad -= 2 * math.pi
                while angle_rad < -math.pi:
                    angle_rad += 2 * math.pi
                
                # Convert distance to meters
                distance_m = distance_mm / 1000.0
                
                # Find index in ranges array
                if self.angle_min <= angle_rad <= self.angle_max:
                    idx = int((angle_rad - self.angle_min) / self.angle_increment)
                    if 0 <= idx < num_points:
                        ranges[idx] = distance_m
                        intensities[idx] = float(quality)
            
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            
            # Publish
            self.publisher.publish(scan_msg)
            
            # Update scan time
            actual_scan_time = time.time() - scan_start_time
            self.last_scan_time = time.time()
            
            self.get_logger().debug(f"Published scan with {len(scan_points)} points "
                                   f"(scan time: {actual_scan_time:.3f}s)")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing scan: {e}")
            self.reconnect_lidar()
    
    def reconnect_lidar(self) -> None:
        """Attempt to reconnect to RPLIDAR."""
        self.get_logger().warn("Attempting to reconnect to RPLIDAR...")
        
        try:
            self.lidar.disconnect()
            time.sleep(1.0)
            
            if self.lidar.connect():
                if self.lidar.start_motor():
                    time.sleep(2.0)
                    self.get_logger().info("RPLIDAR reconnected successfully")
                else:
                    self.get_logger().error("Failed to restart motor after reconnect")
            else:
                self.get_logger().error("Failed to reconnect to RPLIDAR")
        except Exception as e:
            self.get_logger().error(f"Error during reconnect: {e}")
    
    def destroy_node(self) -> None:
        """Cleanup on node shutdown."""
        self.get_logger().info("Shutting down RPLIDAR node...")
        if self.lidar:
            self.lidar.disconnect()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = RPLidarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


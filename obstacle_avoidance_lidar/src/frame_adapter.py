"""
Frame adapter node to wrap frame_grabber module and publish LiDAR data as ROS 2 LaserScan.

This node acts as a bridge between the frame_grabber module and ROS 2, converting
the module's output to sensor_msgs/LaserScan format and publishing it on /scan topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
from typing import Optional, Dict, Any
import time


class FrameAdapter(Node):
    """
    ROS 2 node that adapts frame_grabber module output to LaserScan messages.
    
    This node provides a clean interface between the frame_grabber module and
    the obstacle avoidance system, handling data conversion and publishing.
    """
    
    def __init__(self):
        super().__init__('frame_adapter')
        
        # Declare parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'laser_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('scan_time', 0.0)
        self.declare_parameter('time_increment', 0.0)
        
        # Get parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value
        self.time_increment = self.get_parameter('time_increment').value
        
        # Create publisher
        self.scan_publisher = self.create_publisher(
            LaserScan,
            self.scan_topic,
            10
        )
        
        # Create timer for publishing
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.publish_scan)
        
        # Initialize frame_grabber
        self.frame_grabber = None
        self._initialize_frame_grabber()
        
        # Calculate number of measurements
        self.num_measurements = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        self.get_logger().info(f'Frame adapter initialized')
        self.get_logger().info(f'Publishing on topic: {self.scan_topic}')
        self.get_logger().info(f'Frame ID: {self.frame_id}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Number of measurements: {self.num_measurements}')
    
    def _initialize_frame_grabber(self):
        """
        Initialize the frame_grabber module.
        
        This method attempts to import and initialize the frame_grabber module.
        If the module is not available, it will log a warning and use a stub.
        """
        try:
            # TODO: Replace this with actual frame_grabber import
            # import frame_grabber
            # self.frame_grabber = frame_grabber.FrameGrabber()
            
            # For now, use a stub implementation
            self.frame_grabber = FrameGrabberStub()
            self.get_logger().info('Using frame_grabber stub - replace with actual implementation')
            
        except ImportError as e:
            self.get_logger().warn(f'Could not import frame_grabber: {e}')
            self.get_logger().warn('Using stub implementation')
            self.frame_grabber = FrameGrabberStub()
        except Exception as e:
            self.get_logger().error(f'Error initializing frame_grabber: {e}')
            self.frame_grabber = FrameGrabberStub()
    
    def publish_scan(self):
        """
        Publish a LaserScan message with data from frame_grabber.
        """
        try:
            # Get scan data from frame_grabber
            scan_data = self.frame_grabber.get_scan_data()
            
            if scan_data is None:
                self.get_logger().warn('No scan data available from frame_grabber')
                return
            
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
            if 'ranges' in scan_data:
                ranges = scan_data['ranges']
                # Ensure we have the right number of measurements
                if len(ranges) != self.num_measurements:
                    self.get_logger().warn(
                        f'Expected {self.num_measurements} measurements, got {len(ranges)}'
                    )
                    # Pad or truncate as needed
                    if len(ranges) < self.num_measurements:
                        ranges.extend([self.range_max] * (self.num_measurements - len(ranges)))
                    else:
                        ranges = ranges[:self.num_measurements]
                
                scan_msg.ranges = ranges
            else:
                # Create dummy ranges if not provided
                scan_msg.ranges = [self.range_max] * self.num_measurements
                self.get_logger().warn('No ranges in scan data, using dummy values')
            
            # Publish the scan
            self.scan_publisher.publish(scan_msg)
            
            # Log debug info occasionally
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            if self._debug_counter % 100 == 0:  # Log every 100 scans
                min_range = min(scan_msg.ranges) if scan_msg.ranges else 0.0
                max_range = max(scan_msg.ranges) if scan_msg.ranges else 0.0
                self.get_logger().debug(
                    f'Published scan: min_range={min_range:.2f}m, max_range={max_range:.2f}m'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error publishing scan: {e}')


class FrameGrabberStub:
    """
    Stub implementation of frame_grabber for testing and development.
    
    This class provides a minimal interface that mimics what the actual
    frame_grabber module should provide.
    """
    
    def __init__(self):
        self.get_logger().info('FrameGrabberStub initialized')
    
    def get_scan_data(self) -> Optional[Dict[str, Any]]:
        """
        Get scan data in the expected format.
        
        Returns:
            Dictionary containing scan data with 'ranges' key
        """
        # Generate dummy scan data
        # This simulates a 360-degree scan with some obstacles
        num_measurements = 628  # ~360 degrees with 0.01 rad increment
        
        # Create ranges with some obstacles
        ranges = []
        for i in range(num_measurements):
            angle = -3.14159 + i * 0.01
            
            # Add some obstacles at specific angles
            if abs(angle) < 0.5:  # Front obstacle
                range_val = 2.0 + 0.5 * np.sin(angle * 10)
            elif abs(angle - 1.57) < 0.3:  # Left obstacle
                range_val = 1.5 + 0.3 * np.cos(angle * 5)
            elif abs(angle + 1.57) < 0.3:  # Right obstacle
                range_val = 1.8 + 0.4 * np.sin(angle * 7)
            else:
                # Clear path
                range_val = 10.0 + 2.0 * np.sin(angle * 2)
            
            # Add some noise
            range_val += np.random.normal(0, 0.1)
            
            # Clip to valid range
            range_val = max(0.1, min(30.0, range_val))
            
            ranges.append(range_val)
        
        return {
            'ranges': ranges,
            'timestamp': time.time(),
            'frame_id': 'laser_link'
        }


def main(args=None):
    """
    Main function to run the frame adapter node.
    """
    rclpy.init(args=args)
    
    try:
        frame_adapter = FrameAdapter()
        rclpy.spin(frame_adapter)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in frame adapter: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


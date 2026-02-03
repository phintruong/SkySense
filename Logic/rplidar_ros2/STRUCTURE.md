# Package Structure

```
rplidar_ros2/
├── rplidar_ros2/
│   ├── __init__.py
│   ├── rplidar_protocol.py    # Serial protocol implementation
│   └── rplidar_node.py         # ROS 2 node
├── launch/
│   └── rplidar.launch.py       # Launch file
├── resource/
│   └── rplidar_ros2            # Package marker
├── package.xml                  # ROS 2 manifest
├── setup.py                     # Python setup
├── setup.cfg
├── CMakeLists.txt               # Build configuration
└── README.md                    # Documentation
```

## Components

**rplidar_protocol.py**: Implements RPLIDAR A1 serial protocol, handles commands/responses, motor control via DTR, scan data parsing.

**rplidar_node.py**: ROS 2 node using rclpy, publishes `sensor_msgs/msg/LaserScan` on `/scan`, auto-detects port, handles disconnects.

**rplidar.launch.py**: Launch file with configurable parameters.

## Dependencies

- ROS 2 Humble
- Python 3.8+
- pyserial
- rclpy
- sensor_msgs

# RPLIDAR ROS 2 Node

ROS 2 (Humble) node for RPLIDAR A1 sensor using the official serial protocol.

## Requirements

- ROS 2 Humble
- Python 3.8+
- pyserial
- RPLIDAR A1 connected via USB

## Installation

### Dependencies

```bash
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install pyserial
```

### Build Package

```bash
cd ~/ros2_ws/src
# Copy rplidar_ros2 package here
cd ~/ros2_ws
colcon build --packages-select rplidar_ros2
source install/setup.bash
```

### USB Permissions

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

Optional udev rule:
```bash
sudo nano /etc/udev/rules.d/99-rplidar.rules
```
Add: `KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"`
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Usage

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch with auto-detection
ros2 launch rplidar_ros2 rplidar.launch.py

# Specify port manually
ros2 launch rplidar_ros2 rplidar.launch.py port:=/dev/ttyUSB0

# Run node directly
ros2 run rplidar_ros2 rplidar_node
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `''` | Serial port (empty for auto-detect) |
| `frame_id` | string | `'laser_frame'` | Frame ID for LaserScan messages |
| `auto_detect` | bool | `true` | Auto-detect RPLIDAR port |
| `angle_min` | double | `-3.14159` | Minimum scan angle (radians) |
| `angle_max` | double | `3.14159` | Maximum scan angle (radians) |
| `angle_increment` | double | `0.00872665` | Angular resolution (radians) |
| `time_increment` | double | `0.0` | Time between measurements (seconds) |
| `scan_time` | double | `0.1` | Time per scan (seconds) |
| `range_min` | double | `0.15` | Minimum range (meters) |
| `range_max` | double | `12.0` | Maximum range (meters) |
| `motor_pwm` | int | `660` | Motor PWM value (0-1023) |

## Verification

```bash
# Check topics
ros2 topic list
ros2 topic echo /scan

# Check node
ros2 node list

# Visualize in RViz2
rviz2
# Add LaserScan display, set topic to /scan, frame_id to laser_frame
```

## Troubleshooting

**Permission denied**: Add user to dialout group and log out/in.

**Device not found**: Verify USB connection with `lsusb` and `ls -l /dev/ttyUSB*`.

**Motor not starting**: Verify power supply, USB cable quality, and DTR pin control.

**No scan data**: Wait 2-3 seconds for motor stabilization, verify motor rotation, check node logs.

## Architecture

- `rplidar_protocol.py`: Serial protocol implementation
- `rplidar_node.py`: ROS 2 node publishing `sensor_msgs/msg/LaserScan` on `/scan`
- Motor control via DTR pin
- Auto-reconnect on USB disconnect

## License

MIT License

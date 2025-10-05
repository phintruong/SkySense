# Obstacle Avoidance LiDAR for Quadcopters

A comprehensive ROS 2 Humble-based obstacle detection and avoidance system for quadcopters running on Raspberry Pi 4. This package implements a VFH-style algorithm for real-time obstacle avoidance using LiDAR data.

## Features

- **Real-time LiDAR Processing**: Clean and filter LiDAR scans with moving median filtering
- **VFH-style Avoidance**: Vector Field Histogram algorithm for intelligent path planning
- **Configurable Parameters**: YAML-based configuration for easy tuning
- **Safety Features**: Watchdog timer, emergency stop, and hard brake functionality
- **Visualization**: RViz2 integration with real-time obstacle visualization
- **Testing Tools**: Fake scan publisher and matplotlib debugging scripts
- **Unit Tests**: Comprehensive test suite for all utility functions

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble distribution
- **Python**: 3.8 or higher
- **Hardware**: Raspberry Pi 4 (recommended)
- **LiDAR**: Compatible with frame_grabber module

### Required ROS 2 Packages

```bash
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-std-msgs
sudo apt install ros-humble-nav-msgs ros-humble-visualization-msgs
sudo apt install ros-humble-rviz2 ros-humble-rviz-common
```

### Python Dependencies

```bash
pip3 install numpy matplotlib pytest
```

## Installation

1. **Clone the repository**:
```bash
git clone <your-github-repo-url>
cd obstacle_avoidance_lidar
```

2. **Build the ROS 2 package**:
```bash
cd /path/to/your/ros2/workspace/src
cp -r /path/to/obstacle_avoidance_lidar .
cd ..
colcon build --packages-select obstacle_avoidance_lidar
source install/setup.bash
```

3. **Install Python dependencies**:
```bash
pip3 install -r requirements.txt  # If you create one
```

## Configuration

### Parameter Tuning

Edit `config/params.yaml` to adjust system behavior:

```yaml
obstacle_avoidance_node:
  ros__parameters:
    safe_distance: 2.0    # Safe distance threshold (meters)
    stop_distance: 0.5    # Emergency stop distance (meters)
    base_speed: 1.0       # Base forward speed (m/s)
    max_yaw_rate: 1.0     # Maximum yaw rate (rad/s)
    sector_count: 36      # Number of scan sectors
    control_rate: 20.0    # Control loop frequency (Hz)
```

### Frame_grabber Integration

The system includes a frame adapter that wraps your `frame_grabber` module. To integrate with your specific LiDAR:

1. **If frame_grabber is a Python module**:
   - Edit `src/frame_adapter.py`
   - Replace the stub implementation with your actual frame_grabber import
   - Update the `get_scan_data()` method to match your module's API

2. **If frame_grabber is already a ROS 2 node**:
   - Set `use_frame_adapter: false` in the launch file
   - Ensure your node publishes on `/scan` topic

## Usage

### Running with Real LiDAR

```bash
# Launch the complete system
ros2 launch obstacle_avoidance_lidar avoidance.launch.py

# Or with custom parameters
ros2 launch obstacle_avoidance_lidar avoidance.launch.py safe_distance:=3.0 stop_distance:=0.8
```

### Running with Fake Scan Publisher (Testing)

```bash
# Launch with fake scan data
ros2 launch obstacle_avoidance_lidar avoidance.launch.py use_fake_scan:=true

# Or run components separately
ros2 run obstacle_avoidance_lidar fake_scan_publisher
ros2 run obstacle_avoidance_lidar avoidance_node
```

### Visualization

```bash
# Launch RViz2 with pre-configured visualization
ros2 run rviz2 rviz2 -d src/obstacle_avoidance_lidar/rviz/obstacle_avoidance.rviz
```

### Debugging and Analysis

```bash
# Run the histogram plotter for real-time analysis
ros2 run obstacle_avoidance_lidar plot_histogram

# View diagnostic topics
ros2 topic echo /obstacle_avoidance_node/min_distance
ros2 topic echo /obstacle_avoidance_node/chosen_sector
ros2 topic echo /cmd_vel
```

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│   frame_grabber │───▶│  frame_adapter   │───▶│  avoidance_node     │
│   (LiDAR)       │    │  (ROS 2 bridge)  │    │  (VFH algorithm)    │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
                                                         │
                                                         ▼
                                               ┌─────────────────────┐
                                               │     /cmd_vel        │
                                               │  (velocity commands)│
                                               └─────────────────────┘
```

### Key Components

- **frame_adapter.py**: Bridges frame_grabber module to ROS 2 LaserScan messages
- **avoidance_node.py**: Core VFH-style obstacle avoidance algorithm
- **utils.py**: Utility functions for scan processing and filtering
- **fake_scan_publisher.py**: Synthetic LiDAR data for testing
- **plot_histogram.py**: Real-time visualization and debugging

## Algorithm Details

### VFH-Style Avoidance

1. **Scan Processing**:
   - Clean invalid values (NaN, inf)
   - Apply moving median filter (window size 5)
   - Clip to valid range [range_min, range_max]

2. **Sector Analysis**:
   - Divide 360° scan into configurable sectors
   - Calculate minimum distance in each sector
   - Mark sectors as blocked if distance < safe_distance

3. **Decision Making**:
   - Build histogram of free sectors
   - Choose direction maximizing clearance
   - Prefer directions closest to current heading
   - Scale forward speed based on front clearance

4. **Safety Features**:
   - Emergency stop if any reading < stop_distance
   - Watchdog timer (0.5s timeout)
   - Rate-limited control (20 Hz)

## Testing

### Unit Tests

```bash
# Run all tests
cd obstacle_avoidance_lidar
python3 -m pytest test/ -v

# Run specific test file
python3 -m pytest test/test_utils.py -v
```

### Integration Testing

```bash
# Test with fake scans
ros2 launch obstacle_avoidance_lidar avoidance.launch.py use_fake_scan:=true

# Verify output
ros2 topic echo /cmd_vel --once
```

### Expected Behavior

- **Clear path**: Non-zero vx, minimal yaw_rate
- **Obstacle detected**: Reduced vx, increased yaw_rate for avoidance
- **Emergency stop**: All velocities zero when obstacle < stop_distance
- **No data**: Zero velocities after 0.5s timeout

## Integration with Flight Controllers

### PX4 Integration

```bash
# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Launch with MAVROS bridge
ros2 launch obstacle_avoidance_lidar avoidance.launch.py
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
```

### ArduPilot Integration

```bash
# Use MAVROS with ArduPilot
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@127.0.0.1:14555
```

## Troubleshooting

### Common Issues

1. **No scan data received**:
   - Check frame_grabber module integration
   - Verify topic names in configuration
   - Test with fake_scan_publisher

2. **No velocity commands**:
   - Check safe_distance and stop_distance parameters
   - Verify scan data quality
   - Monitor diagnostic topics

3. **Poor avoidance behavior**:
   - Tune sector_count and safe_distance
   - Adjust base_speed and max_yaw_rate
   - Check scan filtering parameters

### Debug Topics

```bash
# Monitor system health
ros2 topic echo /obstacle_avoidance_node/min_distance
ros2 topic echo /obstacle_avoidance_node/chosen_sector
ros2 topic echo /obstacle_avoidance_node/velocity_debug

# Check scan quality
ros2 topic echo /scan --once
```

## Performance Considerations

- **Control Rate**: 20 Hz provides good balance of responsiveness and computational load
- **Sector Count**: 36 sectors (10° each) recommended for most applications
- **Filter Window**: Window size 5 provides good noise reduction without excessive smoothing
- **Memory Usage**: ~50MB typical for Raspberry Pi 4

## Known Limitations

1. **2D Only**: Assumes planar LiDAR scan (no vertical obstacle detection)
2. **Static Obstacles**: Algorithm optimized for static environments
3. **No Path Planning**: Reactive avoidance only, no global path planning
4. **Single LiDAR**: Designed for single LiDAR sensor

## Future Enhancements

- [ ] 3D obstacle detection with multi-layer LiDAR
- [ ] Dynamic obstacle tracking and prediction
- [ ] Integration with global path planners
- [ ] Machine learning-based obstacle classification
- [ ] Multi-sensor fusion (LiDAR + camera + IMU)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

MIT License - see LICENSE file for details

## Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review ROS 2 documentation for general ROS issues

## Acknowledgments

- ROS 2 Humble community
- VFH algorithm research papers
- Raspberry Pi Foundation
- Open source robotics community


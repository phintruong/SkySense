# SkySense - Obstacle Detection System

Real-time obstacle detection using RPLIDAR A1 and HC-SR04 ultrasonic sensors.

## Features

- **360 deg LiDAR Detection**: RPLIDAR A1 for full environmental scanning
- **Ultrasonic Sensing**: HC-SR04 for close-range distance measurement
- **Forward Cone Analysis**: Focuses on +/-80 deg forward-facing cone
- **Tilt Compensation**: Accounts for drone roll angle (IMU ready)
- **Navigation Decisions**: Determines forward, left, right, backward, or hover actions

## Project Structure

```
Logic/
├── lidar/                 # LiDAR sensor module
│   ├── __init__.py
│   ├── rplidar_reader.py  # RPLIDAR A1 hardware interface
│   ├── visualization.py   # Pygame-based radar visualization
│   └── ros2/              # ROS2 integration (optional)
├── ultrasonic/            # Ultrasonic sensor module
│   ├── __init__.py
│   └── hc_sr04_distance.py # HC-SR04 hardware interface
├── core/                  # Core logic modules
│   ├── __init__.py
│   └── logic.py           # Obstacle detection and decision logic
├── main.py                # Main program (real hardware)
├── README.md
└── requirements.txt
```

## Usage

```bash
python main.py
```

## Configuration

Edit `core/logic.py` to adjust:
- `DANGER_RADIUS = 0.5` - Danger zone radius (meters)
- `FORWARD_CONE_HALF_ANGLE = 80` - Forward cone half-angle (degrees)

## Hardware Setup

### LiDAR (RPLIDAR A1)

```python
from lidar import RPLidarReader
from core import process_scan

reader = RPLidarReader(port='/dev/ttyUSB0')
reader.connect()
reader.start()

for scan_data in reader.iter_scans():
    action, obstacles = process_scan(scan_data)
    # Process navigation action...
```

### Ultrasonic (HC-SR04)

```python
# GPIO pins: TRIGGER=24, ECHO=23
python ultrasonic/hc_sr04_distance.py
```

### Visualization

```python
from lidar.visualization import visualize_lidar

visualize_lidar(port='/dev/ttyUSB0', title="SkySense LiDAR Radar")
```

## Algorithm

1. Reads 360 deg scan from RPLIDAR
2. Filters obstacles to forward-facing cone (+/-80 deg)
3. Adjusts cone based on drone tilt angle
4. Classifies obstacles: center, left-front, right-front
5. Determines navigation action based on obstacle positions

## License

MIT License

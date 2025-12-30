# SkySense - LiDAR Obstacle Detection Simulation

Python simulation system for testing LiDAR-based obstacle detection logic before connecting real hardware.

## Features

- **360° LiDAR Simulation**: Generates realistic scan data (0.2-3.0m range)
- **Forward Cone Detection**: Analyzes obstacles in ±80° forward-facing cone
- **Tilt Compensation**: Accounts for drone roll angle (MPU/IMU ready)
- **Navigation Decisions**: Determines forward, left, right, backward, or hover actions

## Project Structure

```
SkySense/
├── hardware/              # Hardware interface modules
│   ├── __init__.py
│   ├── rplidar_reader.py   # RPLIDAR A1 sensor interface
│   └── hc_sr04_distance.py  # HC-SR04 ultrasonic sensor
├── simulation/            # Simulation modules
│   ├── __init__.py
│   └── lidar_sim.py       # LiDAR scan data simulation
├── core/                  # Core logic modules
│   ├── __init__.py
│   └── logic.py           # Obstacle detection and decision logic
├── visualization/         # Visualization modules
│   ├── __init__.py
│   └── lidar_visualization.py  # Pygame-based radar visualization
├── main.py                # Main simulation loop
├── README.md              # Project documentation
└── requirements.txt       # Python dependencies
```

## Usage

```bash
python main.py
```

## Configuration

Edit `core/logic.py` to adjust:
- `DANGER_RADIUS = 0.5` - Danger zone radius (meters)
- `FORWARD_CONE_HALF_ANGLE = 80` - Forward cone half-angle (degrees)

## Integration with Real Hardware

### LiDAR Integration

Use the `RPLidarReader` class from `hardware.rplidar_reader` to read from your RPLIDAR sensor:

```python
from hardware import RPLidarReader
from core import process_scan

reader = RPLidarReader()
reader.connect()
reader.start()

for scan_data in reader.iter_scans():
    action, obstacles = process_scan(scan_data)
    # Process navigation action...
```

### MPU/IMU Integration

In `main.py`, replace the random tilt angle with your MPU reading:

```python
# Replace: tilt_angle = random.uniform(-30, 30)
# With: tilt_angle = mpu.get_roll_angle()
```

### Visualization

Run real-time LiDAR visualization:

```python
from visualization import visualize_lidar

visualize_lidar(port='/dev/ttyUSB0', title="SkySense LiDAR Radar")
```

## Algorithm

1. Filters obstacles to forward-facing cone (±80°)
2. Adjusts cone based on drone tilt angle
3. Classifies obstacles: center, left-front, right-front
4. Chooses action based on obstacle positions and distances

## License

MIT License

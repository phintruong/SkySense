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
├── lidar_sim.py      # LiDAR scan data simulation
├── logic.py          # Obstacle detection and decision logic
├── main.py           # Main simulation loop
├── hc_sr04_distance.py  # HC-SR04 ultrasonic sensor (optional)
└── requirements.txt  # Python dependencies
```

## Usage

```bash
python main.py
```

## Configuration

Edit `logic.py` to adjust:
- `DANGER_RADIUS = 0.5` - Danger zone radius (meters)
- `FORWARD_CONE_HALF_ANGLE = 80` - Forward cone half-angle (degrees)

## Integration with Real Hardware

### LiDAR Integration

Replace `generate_scan()` in `lidar_sim.py` to read from your LiDAR sensor. Maintain the same return format: `[(angle, distance), ...]`

### MPU/IMU Integration

In `main.py`, replace the random tilt angle with your MPU reading:

```python
# Replace: tilt_angle = random.uniform(-30, 30)
# With: tilt_angle = mpu.get_roll_angle()
```

## Algorithm

1. Filters obstacles to forward-facing cone (±80°)
2. Adjusts cone based on drone tilt angle
3. Classifies obstacles: center, left-front, right-front
4. Chooses action based on obstacle positions and distances

## License

MIT License

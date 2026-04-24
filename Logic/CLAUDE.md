# Logic — Python Backend

## Overview
Autonomous drone obstacle detection and navigation system. Reads 360° LiDAR scans, detects obstacles in a forward cone, and outputs navigation decisions. Streams data to the React frontend via WebSocket.

## Entry Points
- `main.py` — Console-only loop: reads real RPLIDAR hardware, runs detection, prints results
- `server.py` — FastAPI WebSocket server on port 8000: streams LiDAR + nav decisions to frontend. Falls back to demo data if no hardware present
- `visualization/lidar_visualization.py` — Standalone Pygame radar display

## Module Layout
```
Logic/
├── core/logic.py           # Obstacle detection algorithm (the brain)
├── hardware/
│   ├── rplidar_reader.py   # RPLIDAR A1 driver (PyRPlidar, USB serial)
│   └── hc_sr04_distance.py # HC-SR04 ultrasonic sensor (GPIO)
├── visualization/
│   └── lidar_visualization.py  # Pygame radar display
├── rplidar_ros2/
│   ├── rplidar_node.py     # ROS 2 LaserScan publisher
│   └── rplidar_protocol.py # Low-level RPLIDAR serial protocol
├── main.py                 # Hardware detection loop (console)
├── server.py               # FastAPI WebSocket server
└── requirements.txt
```

## Key Data Flow
```
RPLIDAR A1 → rplidar_reader.iter_scans() → [(angle, distance), ...]
    → process_scan(scan_data, tilt_angle) → (action, obstacle_info)
    → WebSocket JSON to frontend OR console output
```

## Core Algorithm (core/logic.py)
- Forward cone: ±80° from heading, adjusted by tilt angle
- Region classification: center (±20°), left-front (20-80°), right-front (280-340°)
- Danger radius: 0.5m
- Actions: MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, MOVE_BACKWARD, HOVER
- Decision tree: center blocked → compare sides → choose clearer path

## Hardware Config
- RPLIDAR A1: USB serial at 460800 baud, `/dev/ttyUSB0`, range 50mm-3000mm
- HC-SR04: GPIO 24 (trigger), GPIO 23 (echo), threshold 30cm
- MPU-6050 IMU: **not implemented** — tilt_angle hardcoded to 0

## Commands
```bash
# Run console detection loop (requires RPLIDAR hardware)
python main.py

# Run WebSocket server (works without hardware via demo mode)
python server.py
# or: uvicorn server:app --host 0.0.0.0 --port 8000

# Run ROS 2 node
ros2 launch rplidar_ros2 rplidar.launch.py
```

## Dependencies
- fastapi, uvicorn (WebSocket server)
- pyrplidar (RPLIDAR driver)
- gpiozero (GPIO sensors)
- pygame (visualization)
- rclpy, sensor_msgs (ROS 2)

## Known Issues
- `Logic/rplidar_reader.py` at root is a duplicate of `hardware/rplidar_reader.py` — delete it
- HC-SR04 driver is standalone, not integrated into main pipeline
- No real IMU driver — tilt compensation exists but uses 0
- No actual motor/ESC output — decisions are computed but not actuated

# SkySense — Project Status

*Last updated: 2026-04-24*

## What Is SkySense?

A self-piloting autonomous drone that navigates obstacles using 360° LiDAR scanning, powered by a Raspberry Pi 4. The project has two halves: a **Python backend** (sensor reading, obstacle logic, WebSocket server) and a **React/Three.js frontend** (3D drone showcase with live LiDAR overlay).

---

## What's Working

| Feature | Status |
|---------|--------|
| LiDAR obstacle detection algorithm (±80° forward cone, tilt compensation, nav decisions) | **Done** |
| RPLIDAR A1 hardware driver (serial, motor control, scan acquisition) | **Done** |
| Low-level RPLIDAR serial protocol (rplidar_protocol.py) | **Done** |
| ROS 2 package (LaserScan publisher on `/scan`) | **Done** |
| HC-SR04 ultrasonic distance sensor driver | **Done** |
| Pygame radar visualization | **Done** |
| FastAPI WebSocket server (streams LiDAR + nav decisions, demo fallback) | **Done** |
| Frontend WebSocket client (useLidarSocket hook, auto-reconnect) | **Done** |
| Live LiDAR overlay on 3D viewer (2D radar, color-coded points, action display) | **Done** |
| 3D drone model viewer (glTF, orbit controls, part selection/highlighting) | **Done** |
| Explode/collapse animation for drone parts | **Done** |
| Drive/Fly mode (WASD + Space/Ctrl physics, boost, brake) | **Done** |
| Interactive parts catalog (25+ components, search, category filter, metadata) | **Done** |
| Sound effects (UI + per-component audio) | **Done** |
| Infinite grass ground with lazy-loaded chunks + instanced grass blades | **Done** |
| Guided tour mode (8 steps through major systems) | **Done** |
| First-person / third-person camera toggle | **Done** |

---

## File Map

### Logic (Python Backend)

| File | Purpose |
|------|---------|
| `Logic/main.py` | Main loop — reads real RPLIDAR hardware, runs detection, displays console results |
| `Logic/server.py` | FastAPI WebSocket server — streams LiDAR data + nav decisions to frontend, demo fallback |
| `Logic/core/logic.py` | **Core brain** — obstacle detection in forward cone, classifies obstacles (center/left/right), outputs nav decisions (FORWARD, TURN_LEFT, TURN_RIGHT, BACKWARD, HOVER) |
| `Logic/visualization/lidar_visualization.py` | Pygame real-time radar display with distance-based coloring |
| `Logic/hardware/rplidar_reader.py` | RPLIDAR A1 sensor interface — connection, motor, scan acquisition, data export |
| `Logic/hardware/hc_sr04_distance.py` | HC-SR04 ultrasonic sensor reader via GPIO, alerts within 30 cm |
| `Logic/rplidar_ros2/rplidar_node.py` | ROS 2 node publishing LaserScan on `/scan` topic |
| `Logic/rplidar_ros2/rplidar_protocol.py` | Low-level serial protocol for RPLIDAR A1 (packet parsing, motor control) |
| `Logic/requirements.txt` | Python dependencies |

### Showcase (React/Three.js Frontend)

| File | Purpose |
|------|---------|
| `showcase/client/src/App.tsx` | Root layout — combines viewer + panels |
| `showcase/client/src/components/RobotViewer.tsx` | **Main component** (~930 lines) — 3D model, explode, propeller spin, drive/fly physics, grass ground |
| `showcase/client/src/components/ControlPanel.tsx` | UI controls — search, category filter, explode slider, drive toggle |
| `showcase/client/src/components/PartInfoPanel.tsx` | Description/role/category for selected parts |
| `showcase/client/src/components/RightSidebar.tsx` | Parts list sidebar + guided tour mode |
| `showcase/client/src/components/LidarOverlay.tsx` | Live 2D radar overlay — scan points, forward cone, action display |
| `showcase/client/src/config/robotParts.ts` | Drone parts metadata — groups, categories, keywords, relationships |
| `showcase/client/src/config/sounds.ts` | Sound file mappings |
| `showcase/client/src/hooks/useRobotModel.ts` | Zustand global state — selections, drone physics, camera, LiDAR data |
| `showcase/client/src/hooks/useLidarSocket.ts` | WebSocket client — connects to Python backend, auto-reconnect |
| `showcase/client/src/hooks/useUISounds.ts` | Audio playback hook for UI interactions |
| `showcase/client/src/services/api.ts` | API service stub |
| `showcase/client/src/types/robot.ts` | TypeScript type definitions |

---

## Architecture

```
 DRONE HARDWARE (Raspberry Pi 4)
 ├── RPLIDAR A1 (360° laser scanner, USB serial)
 ├── HC-SR04 (ultrasonic backup, GPIO)
 └── MPU-6050 IMU (planned, not implemented)
          │
          ▼
 PYTHON BACKEND
 ├── Hardware Layer ──── reads real sensors (rplidar_reader, hc_sr04)
 ├── Core Logic ──────── obstacle detection + nav decisions
 ├── Visualization ───── Pygame radar display
 ├── ROS 2 Node ──────── publishes LaserScan to /scan topic
 └── WebSocket Server ── FastAPI on port 8000 (streams to frontend)
          │
          ▼ WebSocket /ws/lidar (connected)
          │
 REACT FRONTEND
 ├── 3D Drone Viewer ─── interactive glTF model with explode/drive
 ├── LiDAR Overlay ───── live 2D radar with scan points + nav action
 ├── Parts System ────── 25+ components with metadata + search
 ├── Drive/Fly Mode ──── WASD physics simulation
 └── Control Panel ───── UI settings, tour mode, camera controls
```

---

## What's NOT Working / Missing

| Gap | Details |
|-----|---------|
| **No MPU/IMU driver** | Tilt compensation logic exists but uses hardcoded `tilt_angle = 0` — no real IMU hardware code |
| **HC-SR04 not integrated** | Driver works standalone but isn't called from main.py or server.py |
| **main.py and server.py are separate** | main.py is console-only; server.py handles frontend streaming — no unified entry point |
| **No deployment pipeline** | No systemd services, Docker setup, or startup scripts for Raspberry Pi |
| **No test suite** | Inline test cases in `logic.py` but no pytest/unittest framework |
| **Duplicate file** | `Logic/rplidar_reader.py` (root) duplicates `Logic/hardware/rplidar_reader.py` |
| **`nul` file in repo root** | Empty artifact — should be removed |
| **No actual motor/ESC control** | Nav decisions are computed but never sent to flight controller or ESCs |
| **No path planning** | Reactive obstacle avoidance only — no waypoint navigation or SLAM |
| **No sensor fusion** | LiDAR and ultrasonic operate independently, no Kalman/complementary filter |

---

## Next Steps (Suggested Priority)

### 1. Implement MPU-6050 IMU Driver
Write `hardware/mpu6050.py` to read real tilt/roll/yaw angles via I2C. Replace the hardcoded `tilt_angle = 0` in both main.py and server.py.

### 2. Integrate HC-SR04 into Detection Pipeline
Use ultrasonic as secondary close-range confirmation alongside LiDAR for obstacles < 30 cm.

### 3. Unify Entry Points
Merge main.py console logic into server.py, or create a unified launcher that runs both the detection loop and WebSocket server.

### 4. Add Sensor Fusion
Implement complementary or Kalman filter to combine LiDAR, ultrasonic, and IMU data into a unified state estimate.

### 5. Flight Controller Interface
Bridge nav decisions to actual motor/ESC commands (PWM via PCA9685 or direct GPIO).

### 6. Deployment Scripts
Create systemd service + setup script for auto-start on Raspberry Pi at boot.

### 7. Clean Up
- Delete the duplicate `Logic/rplidar_reader.py`
- Remove the `nul` file from the repo
- Add proper unit tests with pytest

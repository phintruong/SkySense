# SkySense — Project Status

## What Is SkySense?

A self-piloting drone that navigates obstacles autonomously using 360° LiDAR scans, powered by a Raspberry Pi 4. The project has two halves: a **Python backend** (sensor reading + obstacle logic) and a **React/Three.js frontend** (3D drone showcase).

---

## What's Working

| Feature | Status |
|---------|--------|
| LiDAR obstacle detection algorithm (±80° forward cone, tilt compensation, nav decisions) | **Done** |
| ~~Simulated LiDAR scan data generation~~ | **Removed** |
| RPLIDAR A1 hardware driver (serial, motor control, scan acquisition) | **Done** |
| ROS 2 package (LaserScan publisher on `/scan`) | **Done** |
| HC-SR04 ultrasonic distance sensor driver | **Done** |
| Pygame radar visualization | **Done** |
| 3D drone model viewer (glTF, orbit controls, part selection) | **Done** |
| Explode/collapse animation for drone parts | **Done** |
| Drive/Fly mode (WASD + Space/Ctrl physics) | **Done** |
| Interactive parts catalog (25+ components, search, metadata) | **Done** |
| Sound effects (UI + component interactions) | **Done** |
| Infinite grass ground with lazy-loaded chunks | **Done** |

---

## File Map

### Logic (Python Backend)

| File | Purpose |
|------|---------|
| `Logic/main.py` | Main loop — reads real RPLIDAR hardware, runs detection, displays results |
| `Logic/core/logic.py` | **Core brain** — obstacle detection in forward cone, classifies obstacles (center/left/right), outputs nav decisions (FORWARD, TURN_LEFT, TURN_RIGHT, BACKWARD, HOVER) |
| ~~`Logic/simulation/lidar_sim.py`~~ | **Removed** — was generating fake data |
| `Logic/visualization/lidar_visualization.py` | Pygame real-time radar display with distance-based coloring |
| `Logic/hardware/rplidar_reader.py` | RPLIDAR A1 sensor interface — connection, motor, scan acquisition, data export |
| `Logic/hardware/hc_sr04_distance.py` | HC-SR04 ultrasonic sensor reader via GPIO, alerts within 30 cm |
| `Logic/rplidar_ros2/rplidar_node.py` | ROS 2 node publishing LaserScan on `/scan` topic |
| `Logic/rplidar_ros2/rplidar_protocol.py` | Low-level serial protocol for RPLIDAR A1 |
| `Logic/requirements.txt` | Python dependencies |

### Showcase (React/Three.js Frontend)

| File | Purpose |
|------|---------|
| `showcase/client/src/App.tsx` | Root layout — combines viewer + panels |
| `showcase/client/src/components/RobotViewer.tsx` | **Main component** (~930 lines) — loads 3D model, orbit controls, part highlighting, explode animation, propeller spin, drive/fly physics, grass ground |
| `showcase/client/src/components/ControlPanel.tsx` | UI controls (explode slider, ground toggle, camera mode, drive mode) |
| `showcase/client/src/components/PartInfoPanel.tsx` | Shows description/role/category for selected parts |
| `showcase/client/src/components/RightSidebar.tsx` | Searchable parts list sidebar |
| `showcase/client/src/config/robotParts.ts` | Drone parts metadata — 4 motor groups, 4 arms, 4 columns, Raspberry Pi + 20 sub-components |
| `showcase/client/src/config/sounds.ts` | Sound file mappings |
| `showcase/client/src/hooks/useRobotModel.ts` | Zustand global state — selections, drone physics, camera, settings |
| `showcase/client/src/hooks/useUISounds.ts` | Audio playback hook for UI interactions |
| `showcase/client/src/services/api.ts` | API service stub (not connected) |
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
 ├── Hardware Layer ──── reads real sensors
 ├── Simulation Layer ── generates fake data for testing
 ├── Core Logic ──────── obstacle detection + nav decisions
 ├── Visualization ───── Pygame radar display
 └── ROS 2 Node ──────── publishes to /scan topic
          │
          ╳  ← NOT CONNECTED YET
          │
 REACT FRONTEND
 ├── 3D Drone Viewer ─── interactive glTF model
 ├── Parts System ────── 25+ components with metadata
 ├── Drive/Fly Mode ──── WASD physics simulation
 └── Control Panel ───── UI settings and controls
```

---

## What's NOT Working / Missing

| Gap | Details |
|-----|---------|
| **Backend ↔ Frontend not connected** | No data flows from Python logic to React frontend. `api.ts` exists but has no real endpoint. |
| **No MPU/IMU driver** | Tilt compensation exists in the algorithm but uses random values — no actual IMU hardware code. |
| **HC-SR04 not integrated into main pipeline** | Driver works standalone but isn't used in obstacle detection. |
| **No deployment pipeline** | No systemd services, Docker setup, or startup scripts for Raspberry Pi. |
| **No end-to-end tests** | Some inline test cases in `logic.py` but no test suite. |
| **Duplicate file** | `Logic/rplidar_reader.py` (root) duplicates `Logic/hardware/rplidar_reader.py`. |
| **`nul` file in repo root** | Empty artifact — should be removed. |

---

## Next Steps (Suggested Priority)

### 1. Connect Backend to Frontend
Build a WebSocket or REST API server (FastAPI or Flask) in Python that streams LiDAR scan data and nav decisions to the React frontend in real time. This is the **biggest gap**.

### 2. Add Live LiDAR Visualization to Frontend
Render scan points on the 3D viewer — show the LiDAR sweep around the drone model with color-coded distances (red/yellow/green).

### 3. Implement MPU-6050 IMU Driver
Write a `hardware/mpu6050.py` driver to read real tilt/roll angles. Plug it into `main.py` to replace the random tilt simulation.

### 4. Integrate HC-SR04 into Detection Pipeline
Use the ultrasonic sensor as a secondary "ground truth" check alongside LiDAR, especially for close-range obstacles.

### 5. Deployment Scripts
Create a systemd service + setup script so everything auto-starts on the Raspberry Pi at boot.

### 6. Clean Up
- Delete the duplicate `Logic/rplidar_reader.py`
- Remove the `nul` file from the repo
- Add proper unit tests

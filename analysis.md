# SkySense — Deep Analysis

*Generated: 2026-04-24*

---

## 1. REPO STRUCTURE

### Full Folder Tree
```
SkySense/
├── Logic/                                  # Python backend
│   ├── core/
│   │   ├── __init__.py
│   │   └── logic.py                        # Obstacle detection algorithm
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── rplidar_reader.py               # RPLIDAR A1 driver
│   │   └── hc_sr04_distance.py             # HC-SR04 ultrasonic driver
│   ├── visualization/
│   │   ├── __init__.py
│   │   └── lidar_visualization.py          # Pygame radar display
│   ├── rplidar_ros2/
│   │   ├── launch/rplidar.launch.py
│   │   ├── rplidar_ros2/
│   │   │   ├── __init__.py
│   │   │   ├── rplidar_node.py             # ROS 2 LaserScan publisher
│   │   │   └── rplidar_protocol.py         # Low-level serial protocol
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── setup.py / setup.cfg
│   │   └── STRUCTURE.md / README.md
│   ├── main.py                             # Console detection loop
│   ├── server.py                           # FastAPI WebSocket server
│   ├── rplidar_reader.py                   # DUPLICATE — delete
│   ├── requirements.txt
│   └── README.md
├── showcase/                               # React/Three.js frontend
│   └── client/
│       ├── src/
│       │   ├── App.tsx                     # Root layout
│       │   ├── components/
│       │   │   ├── RobotViewer.tsx         # Main 3D scene (~930 lines)
│       │   │   ├── ControlPanel.tsx        # UI controls
│       │   │   ├── PartInfoPanel.tsx       # Part details panel
│       │   │   ├── RightSidebar.tsx        # Parts list + tour
│       │   │   └── LidarOverlay.tsx        # Live 2D radar overlay
│       │   ├── hooks/
│       │   │   ├── useRobotModel.ts        # Zustand global state
│       │   │   ├── useLidarSocket.ts       # WebSocket to backend
│       │   │   └── useUISounds.ts          # Sound playback
│       │   ├── config/
│       │   │   ├── robotParts.ts           # 25+ parts metadata
│       │   │   └── sounds.ts
│       │   ├── services/api.ts
│       │   ├── types/robot.ts
│       │   └── utils/componentSounds.ts
│       ├── public/                         # Static assets (3D models, sounds)
│       ├── vite.config.ts
│       ├── tailwind.config.js
│       └── package.json
├── PROJECT_STATUS.md
├── analysis.md                             # This file
├── nul                                     # ARTIFACT — delete
└── .claude/settings.local.json
```

### Main Entry Points
1. **`Logic/main.py`** — Console-only loop: connects to RPLIDAR hardware, runs obstacle detection, prints results
2. **`Logic/server.py`** — FastAPI server on port 8000: streams LiDAR + nav decisions over WebSocket, falls back to demo data
3. **`showcase/client/`** — `npm run dev` starts Vite on port 5173, connects to backend WebSocket

### Module Organization
- **`Logic/core/`** — Pure algorithm (no I/O dependencies). Contains the obstacle detection brain.
- **`Logic/hardware/`** — Sensor drivers. Each sensor gets its own module.
- **`Logic/visualization/`** — Pygame-based display (standalone from the frontend).
- **`Logic/rplidar_ros2/`** — ROS 2 package for integration with robotics middleware.
- **`showcase/client/src/`** — React app organized by concern: components, hooks, config, types.

---

## 2. CURRENT FUNCTIONALITY

### End-to-End Flow

**Path A — Console (main.py):**
```
RPLIDAR A1 hardware
  → rplidar_reader.iter_scans() yields [(angle, distance), ...]
    → process_scan(scan_data, tilt_angle=0)
      → filter to ±80° forward cone within 0.5m
      → classify: center / left-front / right-front
      → determine_action() → MOVE_FORWARD | TURN_LEFT | TURN_RIGHT | MOVE_BACKWARD | HOVER
    → print to console
    → sleep 0.1s → repeat
```

**Path B — Frontend-connected (server.py):**
```
RPLIDAR A1 (or demo generator if no hardware)
  → process_scan()
  → WebSocket JSON: {scan, action, obstacles, tiltAngle, timestamp}
    → React frontend receives via useLidarSocket
      → Updates Zustand store
      → LidarOverlay renders 2D radar
      → 3D viewer shows drone model (independent of LiDAR data currently)
```

**Path C — ROS 2 (rplidar_node.py):**
```
RPLIDAR A1
  → rplidar_protocol.iter_scans()
  → Publish sensor_msgs/LaserScan to /scan topic at ~10 Hz
```

### Inputs → Outputs

| Input | Format | Output |
|-------|--------|--------|
| RPLIDAR A1 360° scan | List of (angle_deg, distance_m) | Navigation action string + obstacle details |
| HC-SR04 ultrasonic | Single distance value (m) | Alert if < 30cm (standalone, not integrated) |
| Keyboard (frontend) | WASD + Space/Ctrl | Drone position/heading in 3D scene |

### Real-Time or Event-Based?
**Real-time, polling-based.** The main loop polls the LiDAR at ~10 Hz (0.1s sleep). The WebSocket streams continuously. The frontend renders at 60 FPS with `useFrame()`. There is no event-driven architecture or message queue — it's a synchronous scan-process-decide loop.

---

## 3. DATA / STATE

### Python Backend State Variables

**core/logic.py — Module-level constants (no mutable state):**
```python
DANGER_RADIUS = 0.5          # meters
FORWARD_CONE_HALF_ANGLE = 80 # degrees
```

**hardware/rplidar_reader.py — RPLidarReader instance state:**
```python
self.port              # str: serial port path
self.baudrate          # int: 460800
self.motor_pwm         # int: 0-1023 (default 500)
self.lidar             # PyRPlidar instance (or None)
self._running          # bool: motor active flag
self._scan_generator   # generator: yields complete sweeps
```

**server.py — Module-level globals:**
```python
_hardware_available    # bool: True if RPLIDAR connected
_lidar                 # RPLidarReader instance (or None)
```

**hc_sr04_distance.py — Module-level constants:**
```python
TRIGGER_PIN = 24       # GPIO output
ECHO_PIN = 23          # GPIO input
THRESHOLD_CM = 30      # Alert threshold
```

### React Frontend State (Zustand store in useRobotModel.ts)

```
highlightedParts: string[]       # Currently highlighted mesh IDs
selectedPart: string | null      # Focused part for camera
parts: RobotPart[]               # All 25+ parts metadata
explodeStrength: number           # 0-100 slider
showGround: boolean              # Drive/Fly mode active
cameraMode: 'third' | 'first'
dronePosition: {x, y, z}        # Current drive position
droneOffset: {x, y, z} | null   # Initial center offset
droneSpeed: number               # Current forward speed
isDroneMoving: boolean
soundEnabled: boolean
soundVolume: number              # 0-1
lidar.connected: boolean
lidar.scan: [{angle, distance}]
lidar.action: string
lidar.obstacles: [{angle, distance, region}]
lidar.tiltAngle: number
```

### How State is Stored
- **Python:** Classes (RPLidarReader), module globals (server.py), pure functions with no state (logic.py)
- **React:** Zustand store (single global atom), React refs for per-frame physics (keysRef, speedRef, headingRef in RobotViewer)

### Where State is Updated
- **RPLidarReader:** `connect()` sets `self.lidar`, `start()` sets `self._running` and `self._scan_generator`
- **server.py:** Module init sets `_hardware_available` and `_lidar` once at startup
- **Zustand store:** Updated by UI interactions (click, slider, toggle) and WebSocket messages (lidar data)
- **Physics refs:** Updated every frame in `useFrame()` callback inside RobotViewer

---

## 4. LOOPS / TIMING

### Main Loops

**Python — main.py:**
```python
for scan_data in lidar.iter_scans():  # Blocking generator
    process_scan(scan_data, tilt_angle=0)
    time.sleep(0.1)                   # ~10 Hz
```

**Python — server.py WebSocket handler:**
```python
while True:
    scan_data = _lidar.get_scan() or _demo_scan(tick)
    action, obstacles = process_scan(scan_data)
    await ws.send_text(json.dumps(payload))
    await asyncio.sleep(0.1)          # ~10 Hz real, 0.25s demo (~4 Hz)
```

**React — RobotViewer useFrame():**
```typescript
useFrame((state, delta) => {
    // Runs every frame (~60 FPS)
    // Updates drive physics, propeller rotation, explode lerp, camera animation
})
```

### Loop Frequency
| Loop | Frequency | Blocking? |
|------|-----------|-----------|
| LiDAR scan acquisition | ~5-10 Hz (hardware-dependent) | Yes (generator blocks until sweep complete) |
| process_scan + WebSocket send | ~10 Hz (real) / ~4 Hz (demo) | Async |
| React render loop | ~60 FPS | No (requestAnimationFrame) |

### How Time is Handled
- **Python:** `time.sleep(0.1)` for loop pacing, `time.time()` for timestamps in WebSocket payloads
- **React:** `delta` from `useFrame()` (seconds since last frame) drives all physics calculations
- No fixed timestep or dt accumulator — physics is frame-rate dependent in the frontend

---

## 5. SENSORS

### Simulated and Real Sensor Inputs

| Sensor | Real Hardware? | Simulated? | Integrated? |
|--------|---------------|------------|-------------|
| RPLIDAR A1 (360° LiDAR) | Yes — USB serial driver | Yes — `_demo_scan()` in server.py | Yes — main pipeline |
| HC-SR04 (ultrasonic) | Yes — GPIO driver | No | **No** — standalone only |
| MPU-6050 (IMU) | **No driver** | No | **No** — `tilt_angle=0` hardcoded |

### Data Format of Sensor Outputs

**RPLIDAR A1:**
```python
# Per scan: list of (angle_degrees, distance_meters) tuples
[(0.0, 2.31), (2.0, 2.28), (4.0, 1.95), ..., (358.0, 2.44)]
# Range: 0-359° angle, 0.05-3.0m distance
# One complete sweep = ~180-360 points depending on scan mode
```

**HC-SR04:**
```python
# Single float: distance in meters
0.247  # (converted to cm internally for threshold check)
```

**Demo scan generator:**
```python
# Same format as real RPLIDAR, 180 points (every 2°)
# Base distance ~2m with sinusoidal noise
# One rotating close obstacle cluster at ~0.25m
```

### Is Noise Modeled?
- **Real sensors:** Natural hardware noise present in raw readings
- **Demo mode:** Sinusoidal variation (`0.3 * sin(deg * 3)`) simulates noise, but it's deterministic — not stochastic
- **No explicit noise model** (no Gaussian noise, no sensor error model, no covariance matrices)

---

## 6. CONTROL / INPUT

### How Decisions Are Made

**Obstacle avoidance (core/logic.py):**
```
1. Filter scan to forward cone (±80° from heading, adjusted by tilt)
2. Filter to danger zone (< 0.5m)
3. Classify each obstacle: center / left-front / right-front
4. Decision tree:
   - No obstacles → MOVE_FORWARD
   - Center blocked, one side clear → turn to clear side
   - Center blocked, both sides blocked → turn toward farther side
   - Center very close + both sides very close → HOVER
   - Sides blocked, center clear → if both < 0.3m → MOVE_BACKWARD
```

This is **reactive** obstacle avoidance. No path planning, no memory of previous states, no lookahead.

### Control Logic Implemented
- **Navigation decisions:** Yes — 5 discrete actions based on obstacle geometry
- **Motor actuation:** **No** — decisions are computed but never sent to hardware
- **PID control:** No
- **Waypoint following:** No
- **State machine:** No (stateless per-scan)

### Where Inputs Come From
- **LiDAR scan data:** RPLIDAR A1 hardware via USB serial (or demo generator)
- **Tilt angle:** Hardcoded to 0 (should come from MPU-6050 IMU)
- **User keyboard (frontend only):** WASD/Space/Ctrl for 3D viewer drive mode — purely cosmetic, not connected to backend

---

## 7. MATH / PHYSICS

### Backend — LiDAR Processing

**Coordinate transforms:**
```python
# Polar to Cartesian
x = distance * cos(radians(angle))
y = distance * sin(radians(angle))

# Tilt compensation (cone rotation)
adjusted_angle = (angle - tilt_angle) % 360
```

**Forward cone geometry:**
```
Cone spans: [360 - 80, 360] ∪ [0, 80] degrees (after tilt adjustment)
Center: ±20° from forward
Left-front: 20° to 80°
Right-front: 280° to 340°
```

**Distance thresholds:**
```
Danger zone: < 0.5m (triggers avoidance)
Very close:  < 0.3m (triggers HOVER or BACKWARD)
```

### Frontend — Drive Physics

**Horizontal motion (XZ plane):**
```
acceleration = 8 m/s² (forward), 4 m/s² (backward)
max_speed = 7 m/s (forward), 4 m/s (backward)
damping = exponential decay when no input
boost = 1.6× acceleration multiplier
brake = 2× damping

speed += throttle × accel × delta × boost
if no_input: speed *= exp(-damping × delta)
speed = clamp(speed, -max_reverse, max_forward)

forward_dir = (cos(heading), 0, sin(heading))
position += forward_dir × speed × delta
```

**Rotation:**
```
turn_rate = 1.2 rad/s
heading -= steer × turn_rate × delta
```

**Vertical (Y axis):**
```
climb_accel = 12 m/s², max_climb = 5.5 m/s
damping = 5.0
Same pattern as horizontal but single-axis
```

**Camera animation:**
```
easing = smoothstep: t² × (3 - 2t)
duration = 0.65s
camera.position.lerp(start, end, eased_t)
```

### Position/Velocity Tracking
- **Backend:** No. Each scan is processed independently. No position or velocity state.
- **Frontend:** Yes. `dronePosition`, `droneSpeed`, heading tracked per frame via React refs. But this is for the 3D viewer only — not connected to real drone state.

---

## 8. OUTPUT / VISUALIZATION

### Outputs

| Output | Type | Location |
|--------|------|----------|
| Console log | Text (formatted scan summary + action) | `main.py` → stdout |
| WebSocket JSON | `{scan, action, obstacles, tiltAngle, timestamp}` | `server.py` → port 8000 |
| Pygame radar | 800×800 window, color-coded points + range rings | `lidar_visualization.py` |
| ROS 2 LaserScan | Standard `/scan` topic | `rplidar_node.py` |
| 3D drone viewer | Interactive Three.js scene | `RobotViewer.tsx` |
| LiDAR overlay | 220×220 canvas, 2D radar with action label | `LidarOverlay.tsx` |

### Graphs / Logs / UI

**Pygame radar (lidar_visualization.py):**
- 800×800 dark background
- Concentric range rings every 50cm
- Points colored by distance: red (<1m), yellow (1-2m), green (>2m)
- 60 FPS refresh

**React LiDAR overlay (LidarOverlay.tsx):**
- Range rings every 1m (3m max)
- Forward cone shaded light blue
- Points: red (<0.3m), yellow (0.3-0.5m), green (>0.5m)
- Action label (color-coded: green=forward, yellow=hover, cyan=turn)
- Obstacle count and nearest distance
- Connection status badge

**React 3D viewer (RobotViewer.tsx):**
- Interactive glTF model with orbit controls
- Part highlighting with magenta glow
- Explode animation
- Drive mode with grass ground
- Guided tour (8 steps)

### Where Output is Generated
- Python: `print()` statements in main.py, `ws.send_text()` in server.py
- Pygame: `lidar_visualization.py` draws directly to screen surface
- React: Components render via React Three Fiber and HTML canvas

---

## 9. TECH STACK

### Languages
- **Python 3.x** — Backend (sensor drivers, algorithm, server)
- **TypeScript** — Frontend (React components, hooks, types)
- **XML/YAML** — ROS 2 package config (package.xml, launch files)

### Key Libraries / Frameworks

**Python:**
| Library | Purpose |
|---------|---------|
| FastAPI + uvicorn | WebSocket server, REST API |
| PyRPlidar | RPLIDAR A1 sensor driver |
| gpiozero | GPIO abstraction for HC-SR04 |
| pygame | Real-time radar visualization |
| rclpy + sensor_msgs | ROS 2 node and message types |
| pyserial | Serial communication (underlying) |

**TypeScript/React:**
| Library | Purpose |
|---------|---------|
| React 18+ | UI framework |
| React Three Fiber | Three.js declarative wrapper |
| Three.js | 3D rendering engine |
| Zustand | Global state management |
| Tailwind CSS | Utility-first styling |
| Vite | Build tool and dev server |

### Hardware Interaction
| Hardware | Interface | Library | Status |
|----------|-----------|---------|--------|
| RPLIDAR A1 | USB serial (460800 baud) | PyRPlidar / custom protocol | Working |
| HC-SR04 | GPIO 23/24 | gpiozero | Working (standalone) |
| MPU-6050 IMU | I2C | **None** | Not implemented |
| Motors/ESCs | PWM | **None** | Not implemented |
| Raspberry Pi 4 | Target platform | N/A | Development target |

---

## 10. CONSTRAINTS

### What Must Be Kept
- **core/logic.py** — The obstacle detection algorithm is correct and well-structured. The forward cone + region classification + decision tree pattern is solid.
- **hardware/rplidar_reader.py** — Production-quality RPLIDAR driver with proper error handling and logging.
- **rplidar_ros2/** — ROS 2 integration is standard and reusable.
- **server.py** — WebSocket streaming pattern and demo fallback are well-designed.
- **Frontend component architecture** — Clean separation of 3D viewer, overlay, state, and WebSocket hook.

### What Can Be Refactored
- **main.py** — Should be unified with server.py or become a CLI mode flag.
- **Decision logic** — Currently stateless and reactive. Can be replaced with a state machine or planner without affecting the rest.
- **Duplicate rplidar_reader.py** at Logic root — Delete it.
- **Drive physics** — Frontend-only and cosmetic. Can be replaced with real drone state from backend.
- **HC-SR04 module** — Needs integration into the main pipeline.
- **Visualization** — Pygame viewer is redundant with the React overlay. Could be dropped or kept for headless debugging.

### Performance Limits
- **RPLIDAR A1 scan rate:** ~5-10 Hz (hardware limit, not software)
- **WebSocket throughput:** JSON serialization of 180-360 points per scan at 10 Hz is lightweight
- **Frontend:** Three.js scene with ~930-line component is complex but runs at 60 FPS. Instanced grass rendering is the heaviest operation.
- **Raspberry Pi 4:** 4GB RAM, quad-core ARM — adequate for Python backend but not for running the React frontend simultaneously (frontend runs on a separate machine/browser)

---

## 11. GOAL ALIGNMENT

### Target Roles
- **Space Systems Flight Software**
- **Space Systems GNC (Guidance, Navigation, and Control)**

### Current State vs. Target

| GNC Component | Current State | Gap |
|---------------|--------------|-----|
| **Guidance** (where to go) | None — no waypoints, no path planning, no mission objectives | Full build needed |
| **Navigation** (where am I) | Partial — LiDAR reads environment but no position estimation, no SLAM, no state estimation | Major build needed |
| **Control** (how to get there) | Partial — reactive obstacle avoidance produces discrete actions, but no PID, no actuator interface | Significant build needed |
| **Sensor fusion** | None — LiDAR and ultrasonic are independent, no Kalman filter, no IMU integration | Full build needed |
| **State estimation** | None — no position, velocity, or attitude tracking in backend | Full build needed |
| **Simulation** | Minimal — demo scan generator exists but is not a physics simulation | Major build needed |
| **Flight dynamics** | Frontend only — arcade-style WASD physics, not realistic | Replace entirely |
| **Telemetry** | Partial — WebSocket streams sensor data, but no logging, no playback, no analysis | Build needed |

### Objective: Convert Into

**1. Simulation System**
- Replace demo scan generator with a proper 2D/3D environment simulator
- Model drone kinematics (6-DOF or simplified)
- Simulate sensor outputs (LiDAR with noise model, IMU with bias/drift)
- Add configurable environments (obstacles, terrain, wind)

**2. Control System**
- Replace reactive decision tree with proper GNC architecture
- Implement PID controllers for attitude and position
- Add state machine for mission phases (takeoff, cruise, avoid, land)
- Interface with motor/ESC hardware (or simulated actuators)

**3. Sensor Fusion System**
- Implement Extended Kalman Filter or complementary filter
- Fuse LiDAR, ultrasonic, and IMU into unified state estimate
- Track position, velocity, and attitude
- Propagate uncertainty (covariance)

---

## 12. REQUIRED OUTPUT FROM CLAUDE

### Current Code → Missing GNC Components Map

```
EXISTING                              MISSING (GNC)
─────────────────────────────────     ─────────────────────────────────
core/logic.py (obstacle detect)   →   guidance/path_planner.py
                                      guidance/waypoint_manager.py
                                      guidance/mission_state_machine.py

(nothing)                         →   navigation/state_estimator.py
                                      navigation/ekf.py (Extended Kalman Filter)
                                      navigation/slam.py (optional)

core/logic.py (action decisions)  →   control/pid_controller.py
                                      control/attitude_controller.py
                                      control/position_controller.py
                                      control/motor_mixer.py

hardware/rplidar_reader.py        →   sensors/lidar_interface.py (abstract)
hardware/hc_sr04_distance.py      →   sensors/ultrasonic_interface.py (abstract)
(nothing)                         →   sensors/imu_interface.py (MPU-6050)
                                      sensors/sensor_fusion.py

server.py (demo scan)             →   simulation/environment.py
                                      simulation/drone_dynamics.py
                                      simulation/sensor_models.py
                                      simulation/noise_models.py

server.py (WebSocket)             →   telemetry/data_logger.py
                                      telemetry/playback.py
                                      telemetry/real_time_stream.py

(nothing)                         →   config/drone_params.py
                                      config/pid_gains.py
                                      config/sensor_config.py
```

### Proposed Ideal File Structure

```
SkySense/
├── src/                                # Core flight software
│   ├── guidance/
│   │   ├── path_planner.py             # A* / RRT path planning
│   │   ├── waypoint_manager.py         # Mission waypoints + sequencing
│   │   └── mission_fsm.py             # State machine: IDLE→TAKEOFF→CRUISE→AVOID→LAND
│   ├── navigation/
│   │   ├── state_estimator.py          # Unified state: position, velocity, attitude
│   │   ├── ekf.py                      # Extended Kalman Filter
│   │   └── obstacle_detector.py        # Evolved from current logic.py
│   ├── control/
│   │   ├── pid.py                      # Generic PID controller class
│   │   ├── attitude_controller.py      # Roll/pitch/yaw PID loops
│   │   ├── position_controller.py      # XYZ position PID loops
│   │   └── motor_mixer.py             # Desired forces → individual motor PWMs
│   ├── sensors/
│   │   ├── base.py                     # Abstract sensor interface
│   │   ├── lidar.py                    # RPLIDAR A1 driver (from current)
│   │   ├── ultrasonic.py              # HC-SR04 driver (from current)
│   │   ├── imu.py                      # MPU-6050 driver (new)
│   │   └── fusion.py                  # Sensor fusion (EKF-based)
│   ├── actuators/
│   │   ├── motor_driver.py             # ESC/PWM interface
│   │   └── servo_driver.py            # If applicable
│   ├── simulation/
│   │   ├── environment.py              # 2D/3D obstacle world
│   │   ├── drone_model.py             # 6-DOF dynamics (or simplified quad model)
│   │   ├── sensor_sim.py              # Simulated LiDAR/IMU/ultrasonic with noise
│   │   └── visualizer.py             # Matplotlib/Pygame sim visualization
│   ├── telemetry/
│   │   ├── logger.py                   # CSV/binary flight data logging
│   │   ├── stream.py                  # WebSocket real-time stream (from server.py)
│   │   └── playback.py               # Replay logged flights
│   └── config/
│       ├── drone.py                    # Physical params: mass, arm length, motor constants
│       ├── pid_gains.py               # Tuning parameters
│       └── sensors.py                 # Port configs, calibration offsets
├── main.py                             # Unified entry point (real hardware or sim)
├── showcase/                           # React frontend (keep as-is, enhance later)
│   └── client/
├── tests/
│   ├── test_obstacle_detection.py
│   ├── test_pid.py
│   ├── test_ekf.py
│   ├── test_motor_mixer.py
│   └── test_simulation.py
├── ros2/                               # ROS 2 packages (moved from Logic/rplidar_ros2)
├── PROJECT_STATUS.md
└── requirements.txt
```

### Step-by-Step Refactor Plan

**Phase 1 — Foundation (Week 1-2)**
1. Create `src/` directory with the proposed structure
2. Move `core/logic.py` → `src/navigation/obstacle_detector.py`
3. Move `hardware/rplidar_reader.py` → `src/sensors/lidar.py`
4. Move `hardware/hc_sr04_distance.py` → `src/sensors/ultrasonic.py`
5. Create `src/sensors/base.py` with abstract sensor interface
6. Create `src/config/drone.py` with physical parameters
7. Delete duplicate `Logic/rplidar_reader.py` and `nul` file
8. Create unified `main.py` with CLI flags for hardware vs. sim mode

**Phase 2 — Sensor Fusion + IMU (Week 3-4)**
9. Write `src/sensors/imu.py` — MPU-6050 driver (I2C, accelerometer + gyro)
10. Write `src/navigation/ekf.py` — Extended Kalman Filter
    - State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    - Prediction: IMU-driven (accelerometer + gyro integration)
    - Update: LiDAR range measurements, ultrasonic distance
11. Write `src/sensors/fusion.py` — orchestrates EKF with all sensor inputs
12. Wire IMU into main pipeline (replace `tilt_angle=0`)

**Phase 3 — Control System (Week 5-6)**
13. Write `src/control/pid.py` — generic PID with anti-windup
14. Write `src/control/attitude_controller.py` — inner loop (roll/pitch/yaw)
15. Write `src/control/position_controller.py` — outer loop (x/y/z → desired attitude)
16. Write `src/control/motor_mixer.py` — desired torques → 4 motor PWMs
17. Create `src/config/pid_gains.py` with initial tuning values

**Phase 4 — Simulation (Week 7-8)**
18. Write `src/simulation/drone_model.py` — simplified quadrotor dynamics
    - Inputs: 4 motor forces
    - State: position, velocity, attitude, angular velocity
    - Integration: Euler or RK4
19. Write `src/simulation/sensor_sim.py` — generate noisy LiDAR/IMU/ultrasonic from sim state
20. Write `src/simulation/environment.py` — configurable obstacle fields
21. Integrate: sim mode runs the full GNC stack against simulated sensors

**Phase 5 — Guidance + Mission (Week 9-10)**
22. Write `src/guidance/waypoint_manager.py` — define waypoints, track progress
23. Write `src/guidance/path_planner.py` — A* or RRT for obstacle-free paths
24. Write `src/guidance/mission_fsm.py` — state machine: IDLE → TAKEOFF → NAVIGATE → AVOID → LAND
25. Wire guidance outputs into position controller

**Phase 6 — Integration + Testing (Week 11-12)**
26. Write unit tests for PID, EKF, motor mixer, obstacle detection
27. Run full sim loop: mission → guidance → navigation → control → sim dynamics → sensors → repeat
28. Tune PID gains in simulation
29. Update React frontend to display state estimation data (position, attitude, control outputs)
30. Create deployment scripts for Raspberry Pi

### Reusable vs. Replaceable Components

| Component | Verdict | Reason |
|-----------|---------|--------|
| `core/logic.py` obstacle detection | **Reusable** — evolve into `navigation/obstacle_detector.py` | Algorithm is sound, just needs to feed into a planner instead of directly outputting actions |
| `hardware/rplidar_reader.py` | **Reusable** — move to `sensors/lidar.py` | Production-quality driver, just needs abstract interface |
| `hardware/hc_sr04_distance.py` | **Reusable** — move to `sensors/ultrasonic.py` | Works, just needs integration into pipeline |
| `rplidar_ros2/rplidar_protocol.py` | **Reusable** — keep for low-level serial work | Clean protocol implementation |
| `rplidar_ros2/rplidar_node.py` | **Reusable** — keep for ROS 2 integration | Standard pattern, minimal changes needed |
| `server.py` WebSocket streaming | **Reusable** — move to `telemetry/stream.py` | Pattern is good, just needs to stream richer state data |
| `server.py` demo scan generator | **Replaceable** — replace with `simulation/` package | Too simple for real simulation needs |
| `main.py` | **Replaceable** — rewrite as unified entry point | Current version is console-only with no mode switching |
| `Logic/rplidar_reader.py` (root duplicate) | **Delete** | Exact duplicate |
| Frontend drive physics | **Replaceable** — should reflect real drone state from backend | Currently cosmetic arcade physics |
| Frontend LiDAR overlay | **Reusable** — enhance to show state estimation data | Good visualization pattern, just needs more data |
| Frontend 3D viewer | **Reusable** — keep as showcase | Impressive visualization, independent of GNC work |

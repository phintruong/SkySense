# Orchestration Plan

## Objective
Transform SkySense from a LiDAR obstacle detection system into a full Guidance, Navigation, and Control (GNC) flight software stack with physics simulation, 13-state EKF, cascaded PID control, sensor failure recovery, and a live React telemetry dashboard.

## Context

### Prior Decisions (from /grill-me session)
All design decisions have been locked. This plan is ground truth for all future sessions.

| Decision | Choice |
|---|---|
| Purpose | Real drone project, sim-first then hardware deployment |
| Hardware | Raspberry Pi 4, MPU-6050 IMU, Emax ECO II 2807 1300KV, quad-X |
| Physical params | Configurable in config file, 7-inch quad defaults until confirmed |
| Coordinate system | NED (North-East-Down). Gravity = [0, 0, +9.81]. Z-down. |
| Rotation | Quaternions [qw, qx, qy, qz] internal, Euler [roll, pitch, yaw] for display/PID |
| Existing code | Evolve forward. ROS2 node -> `legacy/`. Keep server.py, showcase, sensor drivers. |
| Python / deps | 3.11+, requirements.txt, numpy/scipy/matplotlib/fastapi/uvicorn |
| Sim rate | 200 Hz fixed (dt=0.005). EKF update at sensor arrival rate. |
| Motor model | First-order lag: thrust += (target - thrust) * dt / tau, tau ~0.03s |
| Drag model | Linear: F_drag = -b * v, b ~0.3 (configurable) |
| EKF state vector | 13-state: [x,y,z, vx,vy,vz, qw,qx,qy,qz, bwx,bwy,bwz] |
| EKF sensors | IMU accel+gyro (prediction+update), ultrasonic (altitude Z). No mag, no GPS. |
| LiDAR | Obstacle avoidance only, NOT in EKF |
| Observability | Roll/pitch/altitude observable. Yaw drifts (no mag). XY drifts (no GPS). Accepted for v1. |
| Control | Cascaded: position PID -> desired angles + thrust, attitude PID -> torques. No rate controller v1. |
| v1 success | Hover + altitude hold, disturbance rejection, sensor failure recovery, telemetry plots, live frontend |
| Failure demos | (1) Ultrasonic dropout, (2) Accelerometer degradation, (3) Full IMU failure -> emergency descent |
| Frontend | 3D attitude viz, sensor status panel, true vs estimated overlay, sim control panel, motor bars |
| WebSocket | Typed JSON messages. Server->client: telemetry. Client->server: disturbance/failure/reset commands. |
| Testing | pytest. Unit tests on math, PID, motor mixer, EKF. Visual validation for behavior. |
| Timeline | None. Build it right. |
| User math level | Linear algebra solid. Quaternions, EKF, PID, rigid body dynamics are new. |

### Current Codebase State (after Wave 1)
```
src/
  config/
    drone_params.py         # DroneParams dataclass (DONE)
    sim_params.py           # SimParams dataclass (DONE)
  math_utils/
    quaternion.py           # 7 quaternion ops (DONE, 13 tests passing)
    rotations.py            # 6 rotation conversions (DONE, 12 tests passing)
  simulation/               # empty, Wave 2
  sensors/
    lidar.py                # moved RPLIDAR driver (DONE)
    ultrasonic.py           # moved HC-SR04 driver (DONE)
    imu_sim.py              # Wave 2
    ultrasonic_sim.py       # Wave 2
  navigation/
    obstacle_detector.py    # moved logic.py (DONE)
    ekf.py                  # Wave 3
    state_estimator.py      # Wave 3
  control/                  # Wave 2
  telemetry/                # Wave 3
  server/                   # Wave 3
tests/
  conftest.py               # DroneParams/SimParams fixtures (DONE)
  test_quaternion.py        # 13 tests (DONE)
  test_rotations.py         # 12 tests (DONE)
requirements.txt            # (DONE)
legacy/
  rplidar_ros2/             # preserved ROS2 node (DONE)
Logic/                      # legacy — server.py, main.py, visualization still here
showcase/client/            # React Three.js frontend (untouched, Wave 4)
```

### Target Project Structure
```
src/
  config/
    __init__.py
    drone_params.py         # mass, arm_length, inertia, motor constants, drag
    sim_params.py           # dt, duration, noise levels, failure scenarios
  simulation/
    __init__.py
    drone_model.py          # DroneState + rigid body physics update
    environment.py          # obstacles, wind field, disturbance injection
    motor_model.py          # first-order lag motor model (4 motors)
  sensors/
    __init__.py
    imu_sim.py              # simulated IMU (accel + gyro + noise + bias)
    ultrasonic_sim.py       # simulated downward ultrasonic + noise
    lidar.py                # moved from hardware/rplidar_reader.py (real hardware)
    ultrasonic.py           # moved from hardware/hc_sr04_distance.py (real hardware)
  navigation/
    __init__.py
    ekf.py                  # 13-state Extended Kalman Filter
    state_estimator.py      # wraps EKF, manages sensor update scheduling
    obstacle_detector.py    # moved from core/logic.py
  control/
    __init__.py
    pid.py                  # generic PID with anti-windup
    attitude_controller.py  # inner loop: angle error -> torques
    position_controller.py  # outer loop: position error -> desired angles + thrust
    motor_mixer.py          # quad-X mixing matrix: thrust+torques -> 4 motor commands
  telemetry/
    __init__.py
    logger.py               # DataLogger: records true state, est state, control, sensors
    plotter.py              # matplotlib: trajectory, error, control, failure event plots
  server/
    __init__.py
    app.py                  # evolved FastAPI server with new protocol
    protocol.py             # message type definitions
  math_utils/
    __init__.py
    quaternion.py           # quaternion multiply, normalize, inverse, rotate_vector
    rotations.py            # quat<->euler, quat<->rotation_matrix, NED conventions
main.py                     # main simulation loop (200 Hz)
tests/
  __init__.py
  test_quaternion.py
  test_rotations.py
  test_pid.py
  test_motor_mixer.py
  test_ekf.py
  test_drone_model.py
  conftest.py               # shared fixtures
requirements.txt
legacy/
  rplidar_ros2/             # preserved ROS2 node
showcase/client/            # evolved React frontend
```

## Agents

Agents are assigned per-wave. The table below shows the primary assignment:

| Agent | Assigned Tool | Role Summary |
|-------|--------------|--------------|
| Agent 1 | Claude | Cross-cutting architecture, complex math (EKF), integration |
| Agent 2 | Codex | Self-contained modules with testable interfaces |
| Agent 3 | Codex/Cursor | Independent modules (Codex) or frontend work (Cursor) |

---

## Wave Progress

| Wave | Status | Notes |
|------|--------|-------|
| Wave 1: Foundation | COMPLETE | 25/25 tests pass. Config + math utils verified. |
| Wave 2: Core Modules | IN_PROGRESS | 3 agents assigned: dynamics, control, sensors |
| Wave 3: Estimation + Integration | NOT_STARTED | Blocked on Wave 2 |
| Wave 4: Frontend + Tests + Demos | NOT_STARTED | Blocked on Wave 3 |

---

## Wave 1: Foundation (COMPLETE)

Everything downstream depends on this wave. Project restructure, config system, and math utilities.

### Task 1.1: Project Restructure + Config System
- **Agent:** Agent 1
- **Tool:** Claude
- **Status:** DONE
- **Description:**
  1. Create the full `src/` directory tree (all `__init__.py` files)
  2. Move existing files:
     - `Logic/core/logic.py` -> `src/navigation/obstacle_detector.py`
     - `Logic/hardware/rplidar_reader.py` -> `src/sensors/lidar.py`
     - `Logic/hardware/hc_sr04_distance.py` -> `src/sensors/ultrasonic.py`
  3. Move `Logic/rplidar_ros2/` -> `legacy/rplidar_ros2/`
  4. Delete `Logic/rplidar_reader.py` (duplicate)
  5. Delete `nul` file if it exists
  6. Create `src/config/drone_params.py` with all physical parameters as a dataclass:
     - mass (default 1.8 kg)
     - arm_length (default 0.18 m)
     - inertia matrix Ixx, Iyy, Izz (defaults for 7-inch quad)
     - motor tau (0.03 s)
     - motor_max_thrust (per motor, derived from Emax 2807 specs)
     - drag_coefficient (0.3)
     - gravity (9.81)
     - propeller_ct, propeller_cq (thrust and torque coefficients)
  7. Create `src/config/sim_params.py`:
     - dt (0.005)
     - duration (30.0 s default)
     - imu_accel_noise_std, imu_gyro_noise_std, imu_gyro_bias_std
     - ultrasonic_noise_std
     - lidar_rate_hz (10), imu_rate_hz (200), ultrasonic_rate_hz (20)
  8. Create `requirements.txt` (numpy, scipy, matplotlib, fastapi, uvicorn, pytest)
  9. Create `tests/conftest.py` with shared config fixtures
  10. Create `tests/__init__.py`
- **Produces:** Complete directory structure, config system, requirements.txt. All other agents depend on this.
- **Depends on:** Nothing
- **Files in scope:** `src/config/`, `src/*/__init__.py`, `legacy/`, `tests/conftest.py`, `tests/__init__.py`, `requirements.txt`, file moves from `Logic/`
- **Files to avoid:** `src/math_utils/quaternion.py`, `src/math_utils/rotations.py` (owned by Agent 2)
- **Completion notes:** Created full src/ tree, moved 3 files, ROS2 to legacy, deleted duplicate. DroneParams + SimParams dataclasses with all defaults. requirements.txt and test fixtures ready.

### Task 1.2: Math Utilities + Pytest Setup
- **Agent:** Agent 2
- **Tool:** Codex
- **Status:** DONE
- **Description:**
  1. Create `src/math_utils/quaternion.py`:
     - `quat_multiply(q1, q2) -> q`: Hamilton product. q = [qw, qx, qy, qz].
     - `quat_conjugate(q) -> q`: [qw, -qx, -qy, -qz]
     - `quat_normalize(q) -> q`: q / ||q||
     - `quat_inverse(q) -> q`: conjugate / ||q||^2 (same as conjugate for unit quats)
     - `quat_rotate_vector(q, v) -> v`: rotate 3-vector v by quaternion q. Formula: v' = q * [0,v] * q_conj
     - `quat_from_axis_angle(axis, angle) -> q`: [cos(a/2), sin(a/2)*axis]
     - `quat_identity() -> q`: [1, 0, 0, 0]
     - All functions operate on numpy arrays. q is always shape (4,), v is shape (3,).
  2. Create `src/math_utils/rotations.py`:
     - `quat_to_euler(q) -> [roll, pitch, yaw]`: NED convention. Roll=phi (X), Pitch=theta (Y), Yaw=psi (Z). 3-2-1 rotation order (ZYX).
     - `euler_to_quat(roll, pitch, yaw) -> q`: inverse of above
     - `quat_to_rotation_matrix(q) -> R`: 3x3 DCM
     - `rotation_matrix_to_quat(R) -> q`: Shepperd's method
     - `body_to_ned(q, v_body) -> v_ned`: rotate body-frame vector to NED
     - `ned_to_body(q, v_ned) -> v_body`: rotate NED vector to body frame
     - NED convention: X=North, Y=East, Z=Down. Gravity in NED = [0, 0, 9.81].
  3. Create `src/math_utils/__init__.py` exporting all public functions
  4. Create `tests/test_quaternion.py`:
     - Identity quaternion rotates vector unchanged
     - 90-degree rotation about Z-axis maps [1,0,0] -> [0,1,0]
     - quat_multiply is associative
     - normalize produces unit quaternion
     - rotate then inverse-rotate returns original vector
     - quat_to_euler -> euler_to_quat round-trip (test at 0, 45, 90 degrees per axis)
  5. Create `tests/test_rotations.py`:
     - Euler [0,0,0] -> identity quaternion
     - Known rotation: 90 pitch -> expected quaternion
     - body_to_ned with identity quat is passthrough
     - Round-trip: quat -> matrix -> quat
     - Gimbal lock edge case: pitch = +/-90 degrees handled gracefully
- **Produces:** Quaternion + rotation math library with full test coverage. Used by EKF, drone model, attitude controller.
- **Depends on:** Nothing (only needs numpy, no project imports)
- **Files in scope:** `src/math_utils/quaternion.py`, `src/math_utils/rotations.py`, `src/math_utils/__init__.py`, `tests/test_quaternion.py`, `tests/test_rotations.py`
- **Files to avoid:** Everything in `src/config/`, `src/simulation/`, `src/sensors/`, `src/navigation/`, `src/control/`, `src/telemetry/`, `src/server/`
- **Completion notes:** 7 quaternion functions + 6 rotation functions. 25 tests total (13 quaternion, 12 rotation), all passing. Hamilton convention, NED 3-2-1, Shepperd's method for matrix-to-quat.

---

## Wave 2: Core Simulation Modules (IN_PROGRESS)

Three independent modules developed in parallel. All depend on Wave 1 (config + math_utils).

### Task 2.1: Drone Dynamics Model + Environment
- **Agent:** Agent 1
- **Tool:** Claude
- **Status:** NOT_STARTED
- **Description:**
  1. Create `src/simulation/drone_model.py`:
     - `DroneState` dataclass: position(3), velocity(3), quaternion(4), angular_velocity(3) — all numpy arrays. NED frame.
     - `DroneModel` class:
       - Constructor takes `DroneParams` from config
       - `step(motor_thrusts: np.array[4], dt: float, external_force: np.array[3]) -> DroneState`
       - Physics update order:
         a. Compute total thrust in body frame: F_body = [0, 0, -sum(thrusts)] (NED: thrust is -Z in body)
         b. Rotate thrust to NED: F_ned = body_to_ned(q, F_body)
         c. Add gravity: F_ned += [0, 0, mass * g]
         d. Add external force (disturbances)
         e. Add linear drag: F_ned += -drag_coeff * velocity
         f. Linear acceleration: a = F_ned / mass
         g. Update velocity: v += a * dt
         h. Update position: p += v * dt
         i. Compute torques from motor thrusts (differential thrust -> roll/pitch/yaw torques)
         j. Angular acceleration: alpha = I_inv @ (torques - omega x (I @ omega))
         k. Update angular velocity: omega += alpha * dt
         l. Update quaternion: q += 0.5 * quat_multiply([0, omega], q) * dt, then normalize
       - Motor torque computation for quad-X:
         - roll_torque = arm_length * (T1 - T2 - T3 + T4) / sqrt(2)
         - pitch_torque = arm_length * (-T1 - T2 + T3 + T4) / sqrt(2)
         - yaw_torque = cq * (-T1 + T2 - T3 + T4)  (counter-rotating props)
         - Motor numbering: front-right=1, back-right=2, back-left=3, front-left=4
  2. Create `src/simulation/motor_model.py`:
     - `MotorModel` class (manages 4 motors):
       - Each motor: current_thrust, first-order lag with tau
       - `update(commanded_thrusts: np.array[4], dt: float) -> np.array[4]` (actual thrusts)
       - `thrust += (commanded - thrust) * dt / tau`
       - Clamp to [0, max_thrust]
  3. Create `src/simulation/environment.py`:
     - `Environment` class:
       - `get_wind_force(position, time) -> np.array[3]`: configurable wind (constant + gusts)
       - `inject_disturbance(force: np.array[3])`: one-shot push (from frontend command)
       - `get_obstacles() -> list`: static obstacle list for LiDAR sim
       - `get_ground_height(x, y) -> float`: flat ground at z=0 (NED: ground is z=0, drone is z<0 when airborne)
- **Produces:** Complete physics simulation. Consumed by main loop and sensor sim.
- **Depends on:** Task 1.1 (config), Task 1.2 (math_utils)
- **Files in scope:** `src/simulation/drone_model.py`, `src/simulation/motor_model.py`, `src/simulation/environment.py`, `src/simulation/__init__.py`
- **Files to avoid:** `src/sensors/`, `src/control/`, `src/navigation/`
- **Completion notes:**

### Task 2.2: Control System
- **Agent:** Agent 2
- **Tool:** Codex
- **Status:** NOT_STARTED
- **Description:**
  1. Create `src/control/pid.py`:
     - `PIDController` class:
       - Constructor: Kp, Ki, Kd, output_min, output_max, integral_max (anti-windup)
       - `update(error: float, dt: float) -> float`
       - `reset()`: zero integral and previous error
       - Derivative-on-measurement (not error) to avoid kick on setpoint change
       - Integral clamping for anti-windup
  2. Create `src/control/attitude_controller.py`:
     - `AttitudeController` class:
       - 3 PID controllers: roll, pitch, yaw
       - `compute(current_euler: [r,p,y], desired_euler: [r,p,y], dt: float) -> torques[3]`
       - Angle wrapping for yaw error (handle 359->1 degree crossing)
       - Default gains: Kp=8.0, Ki=0.5, Kd=3.0 (starting point, will need tuning)
  3. Create `src/control/position_controller.py`:
     - `PositionController` class:
       - 3 PID controllers: x, y, z (NED frame)
       - `compute(current_pos, desired_pos, current_vel, current_yaw, dt) -> (desired_roll, desired_pitch, thrust)`
       - Z controller outputs thrust directly (altitude hold)
       - X/Y controllers output desired roll/pitch angles (rotated by yaw to align with body frame)
       - Clamp desired angles to +-30 degrees
       - Default gains: altitude Kp=5.0, Ki=1.0, Kd=3.0; position Kp=1.0, Ki=0.1, Kd=0.8
  4. Create `src/control/motor_mixer.py`:
     - `MotorMixer` class for quad-X:
       - `mix(thrust: float, torque_roll: float, torque_pitch: float, torque_yaw: float) -> np.array[4]`
       - Mixing matrix (quad-X, NED):
         ```
         M1 (front-right) = thrust - roll + pitch - yaw
         M2 (back-right)  = thrust - roll - pitch + yaw
         M3 (back-left)   = thrust + roll - pitch - yaw
         M4 (front-left)  = thrust + roll + pitch + yaw
         ```
         (signs derived from motor positions and rotation directions)
       - Clamp outputs to [0, max_thrust]
       - If any motor saturates, scale all proportionally to preserve torque ratios
  5. Create `src/control/__init__.py`
  6. Create `tests/test_pid.py`:
     - Step response converges to setpoint
     - Integral windup is bounded
     - Zero error produces zero output (after transient)
  7. Create `tests/test_motor_mixer.py`:
     - Pure thrust -> all motors equal
     - Pure roll torque -> correct differential (motors 3,4 up, motors 1,2 down)
     - Pure yaw torque -> diagonal pairs differential
     - Clamping works, ratios preserved
- **Produces:** Complete control stack. Consumed by main loop.
- **Depends on:** Task 1.1 (config for gain defaults and limits)
- **Files in scope:** `src/control/pid.py`, `src/control/attitude_controller.py`, `src/control/position_controller.py`, `src/control/motor_mixer.py`, `src/control/__init__.py`, `tests/test_pid.py`, `tests/test_motor_mixer.py`
- **Files to avoid:** `src/simulation/`, `src/sensors/`, `src/navigation/`, `src/math_utils/`
- **Completion notes:**

### Task 2.3: Sensor Simulation
- **Agent:** Agent 3
- **Tool:** Codex
- **Status:** NOT_STARTED
- **Description:**
  1. Create `src/sensors/imu_sim.py`:
     - `IMUSim` class:
       - Constructor takes noise params from SimParams
       - `read(true_accel_body: np.array[3], true_gyro_body: np.array[3], dt: float) -> dict`
         - Returns `{"accel": noisy_accel, "gyro": noisy_gyro}`
         - Accel noise: true_accel + N(0, accel_noise_std)
         - Gyro noise: true_gyro + gyro_bias + N(0, gyro_noise_std)
         - Gyro bias: random walk, bias += N(0, bias_std) * sqrt(dt)
       - `inject_failure(mode: str)`: "off" -> returns zeros, "noisy" -> 10x noise, "recover" -> normal
       - `is_healthy() -> bool`
     - In body frame. True accel includes gravity rotated to body frame (accel = specific_force = a_ned - g_ned rotated to body).
     - IMU measures SPECIFIC FORCE, not acceleration. In hover: accel reads [0, 0, -9.81] in body frame (NED).
  2. Create `src/sensors/ultrasonic_sim.py`:
     - `UltrasonicSim` class:
       - `read(true_altitude: float) -> dict`
         - Returns `{"altitude": noisy_altitude, "valid": bool}`
         - true_altitude = -position_z (NED: z is negative when above ground)
         - Add noise: N(0, ultrasonic_noise_std)
         - Range limits: 0.02m to 4.0m (HC-SR04 spec), return valid=False outside
       - `inject_failure(mode: str)`: "off" -> returns valid=False, "recover" -> normal
       - `is_healthy() -> bool`
  3. Create `src/sensors/__init__.py` (exports sim classes + keeps real hardware imports conditional)
- **Produces:** Simulated sensor interfaces. Consumed by EKF and main loop.
- **Depends on:** Task 1.1 (config for noise params), Task 1.2 (rotations for gravity-to-body transform in IMU)
- **Files in scope:** `src/sensors/imu_sim.py`, `src/sensors/ultrasonic_sim.py`, `src/sensors/__init__.py`
- **Files to avoid:** `src/sensors/lidar.py`, `src/sensors/ultrasonic.py` (moved real hardware files, don't modify), `src/simulation/`, `src/control/`, `src/navigation/`
- **Completion notes:**

---

## Wave 3: Estimation + Integration

The EKF is the most complex module. Main loop wires everything together. Server evolves for the new protocol.

### Task 3.1: 13-State Extended Kalman Filter
- **Agent:** Agent 1
- **Tool:** Claude
- **Status:** NOT_STARTED
- **Description:**
  1. Create `src/navigation/ekf.py`:
     - `EKF` class with state vector x (13,), covariance P (13x13):
       - State: [x, y, z, vx, vy, vz, qw, qx, qy, qz, bwx, bwy, bwz]
       - Indices: pos=0:3, vel=3:6, quat=6:10, gyro_bias=10:13
     - `predict(gyro: np.array[3], accel: np.array[3], dt: float)`:
       a. Subtract gyro bias: omega = gyro - x[10:13]
       b. Quaternion integration: q_new = q + 0.5 * quat_multiply([0, omega], q) * dt, normalize
       c. Rotate accel to NED: a_ned = body_to_ned(q, accel) + [0, 0, g] (accel is specific force, add gravity to get true accel)
       d. Velocity: v_new = v + a_ned * dt
       e. Position: p_new = p + v * dt
       f. Gyro bias: unchanged (random walk modeled in Q)
       g. Compute Jacobian F (13x13) analytically
       h. Process noise Q (13x13): tunable, diagonal blocks for pos/vel/quat/bias
       i. Covariance: P = F @ P @ F.T + Q
     - `update_accel(accel_measurement: np.array[3])`:
       - Measurement model: expected accel in body = R_ned_to_body @ [0, 0, -g] (hover specific force)
       - Innovation: y = accel_meas - h(x)
       - Innovation check: if ||y|| > threshold, reduce gain (degraded sensor)
       - Jacobian H (3x13): partial derivatives of measurement w.r.t. state (primarily quaternion components)
       - Kalman gain: K = P @ H.T @ inv(H @ P @ H.T + R_accel)
       - State update: x += K @ y
       - Normalize quaternion after update
       - Covariance: P = (I - K @ H) @ P
     - `update_altitude(altitude: float)`:
       - Measurement model: h = -x[2] (altitude = -z in NED)
       - H = [0,0,-1, 0,...,0] (1x13)
       - Standard Kalman update
       - Innovation check for ultrasonic dropout
     - `get_state() -> dict`: returns pos, vel, euler, gyro_bias as named dict
     - `get_covariance_trace() -> float`: trace(P) for health monitoring
     - `get_innovation_norm() -> float`: last innovation magnitude for frontend display
  2. Create `src/navigation/state_estimator.py`:
     - `StateEstimator` class:
       - Wraps EKF
       - `predict(imu_data: dict, dt: float)`: calls ekf.predict every step
       - `update_altitude(ultrasonic_data: dict)`: calls ekf.update_altitude when sensor has new data
       - `update_accel(imu_data: dict)`: calls ekf.update_accel (can run at IMU rate or sub-sampled)
       - Manages sensor health flags
       - Emergency detection: if all sensors unhealthy -> flag emergency state
  3. Create `src/navigation/__init__.py`
  4. Create `tests/test_ekf.py`:
     - Zero-input predict: state unchanged (except gravity on velocity)
     - Known constant accel: position grows quadratically
     - Altitude update pulls state toward measurement
     - Innovation check rejects outlier measurements
     - Quaternion stays normalized through predict+update cycles
     - Gyro bias converges when true bias is injected
- **Produces:** State estimation system. Consumed by main loop and control.
- **Depends on:** Task 1.2 (math_utils), Task 1.1 (config)
- **Files in scope:** `src/navigation/ekf.py`, `src/navigation/state_estimator.py`, `src/navigation/__init__.py`, `tests/test_ekf.py`
- **Files to avoid:** `src/navigation/obstacle_detector.py` (already moved, don't touch), `src/simulation/`, `src/control/`, `src/sensors/`
- **Completion notes:**

### Task 3.2: Main Simulation Loop + Telemetry Logger
- **Agent:** Agent 2
- **Tool:** Codex
- **Status:** NOT_STARTED
- **Description:**
  1. Create `main.py`:
     - `Simulation` class:
       - Constructor wires all modules: DroneModel, MotorModel, Environment, IMUSim, UltrasonicSim, StateEstimator, PositionController, AttitudeController, MotorMixer
       - `step(dt)`:
         a. Read simulated IMU (from true state)
         b. Read simulated ultrasonic (at its rate, using counter/timer)
         c. EKF predict (every step)
         d. EKF update altitude (when ultrasonic fires)
         e. EKF update accel (every step)
         f. Position controller: estimated state -> desired angles + thrust
         g. Attitude controller: estimated angles vs desired -> torques
         h. Motor mixer: thrust + torques -> 4 commands
         i. Motor model: commanded -> actual (first-order lag)
         j. Drone model: actual thrusts + environment forces -> new true state
         k. Log everything
       - `run(duration)`: loop for duration at dt=0.005
       - `get_telemetry_snapshot() -> dict`: current state for WebSocket streaming
       - `inject_disturbance(force)`: pass to environment
       - `inject_sensor_failure(sensor, mode)`: pass to sensor sim
       - `reset()`: reinitialize all modules
     - `main()` function: runs standalone sim, prints summary, shows plots
     - Target: [0, 0, -2.0] in NED (hover at 2m altitude)
  2. Create `src/telemetry/logger.py`:
     - `DataLogger` class:
       - `log(timestamp, true_state, estimated_state, control_output, sensor_status)`: append to internal lists
       - `get_log() -> dict of np.arrays`: for plotting
       - `clear()`: reset logs
       - `save(filename)`: save to numpy .npz file
  3. Create `src/telemetry/__init__.py`
- **Produces:** Working end-to-end simulation. This is the integration point.
- **Depends on:** All Wave 2 tasks (simulation, control, sensors) + Task 3.1 (EKF)
- **Files in scope:** `main.py`, `src/telemetry/logger.py`, `src/telemetry/__init__.py`
- **Files to avoid:** All `src/simulation/`, `src/control/`, `src/sensors/`, `src/navigation/` (read, don't modify)
- **Completion notes:**

### Task 3.3: WebSocket Server Evolution
- **Agent:** Agent 3
- **Tool:** Codex
- **Status:** NOT_STARTED
- **Description:**
  1. Create `src/server/protocol.py`:
     - Define message types as dataclasses or TypedDicts:
       - `TelemetryMessage`: type="telemetry", timestamp, true_state, estimated_state, control (thrust, torques, motor_outputs), sensors (imu_status, ultrasonic_status, lidar_status), ekf (innovation_norm, covariance_trace), scan, obstacles
       - `DisturbanceCommand`: type="inject_disturbance", force=[fx,fy,fz]
       - `SensorFailureCommand`: type="sensor_failure", sensor="ultrasonic"|"accel"|"imu", mode="off"|"noisy"|"recover"
       - `ResetCommand`: type="reset_sim"
       - `SetTargetCommand`: type="set_target", altitude=float
  2. Create `src/server/app.py`:
     - Evolve from `Logic/server.py`
     - FastAPI app with lifespan
     - `GET /api/status`: health + sim state summary
     - `WS /ws/telemetry`: bidirectional
       - Server sends TelemetryMessage at ~20 Hz (every 10th sim step at 200 Hz)
       - Client sends commands (disturbance, failure, reset, set_target)
       - Parse incoming JSON, dispatch to Simulation methods
     - Keep `/ws/lidar` endpoint for backwards compat with existing frontend (optional, can deprecate)
     - Runs Simulation in background asyncio task
  3. Create `src/server/__init__.py`
- **Produces:** New WebSocket server. Consumed by React frontend.
- **Depends on:** Task 3.2 (main loop Simulation class interface)
- **Files in scope:** `src/server/app.py`, `src/server/protocol.py`, `src/server/__init__.py`
- **Files to avoid:** `Logic/server.py` (legacy, don't modify), `main.py`, `showcase/`
- **Completion notes:**

---

## Wave 4: Frontend + Tests + Demos

Final wave: React frontend evolution, failure demo scenarios, test suite, analysis plots.

### Task 4.1: Failure Injection + Demo Scenarios + Analysis Plots
- **Agent:** Agent 1
- **Tool:** Claude
- **Status:** NOT_STARTED
- **Description:**
  1. Create demo runner script `demos/run_demos.py`:
     - `demo_hover()`: Start sim, target 2m altitude, run 10s, plot convergence
     - `demo_disturbance()`: Hover, inject [5,0,0]N push at t=3s, show recovery
     - `demo_ultrasonic_dropout()`: Hover, kill ultrasonic at t=3s, show altitude drift, recover at t=8s
     - `demo_accel_degradation()`: Hover, inject 10x noise at t=3s, show EKF innovation spike and trust reduction
     - `demo_imu_failure()`: Hover, kill full IMU at t=3s, trigger emergency descent, show controlled landing
     - Each demo: run sim, collect telemetry, generate matplotlib plots, save to `demos/output/`
  2. Create `src/telemetry/plotter.py`:
     - `plot_trajectory_3d(log)`: 3D position plot (true vs estimated)
     - `plot_attitude(log)`: roll/pitch/yaw over time (true vs estimated)
     - `plot_altitude(log)`: altitude over time with failure events marked
     - `plot_control_output(log)`: thrust + torques + motor outputs over time
     - `plot_ekf_health(log)`: innovation norm + covariance trace over time
     - `plot_sensor_status(log)`: binary sensor health timeline
     - All plots: proper labels, legends, grid, timestamps. Publication quality.
  3. Verify end-to-end integration: run each demo, ensure no crashes, plots look correct
- **Produces:** Demo scenarios + analysis plots. Portfolio-ready outputs.
- **Depends on:** All Wave 3 tasks
- **Files in scope:** `demos/run_demos.py`, `demos/output/`, `src/telemetry/plotter.py`
- **Files to avoid:** `src/simulation/`, `src/control/`, `src/navigation/`, `src/sensors/`, `main.py`, `showcase/`
- **Completion notes:**

### Task 4.2: React Frontend Evolution
- **Agent:** Agent 2
- **Tool:** Cursor
- **Status:** NOT_STARTED
- **Description:**
  1. Update Zustand store (`useRobotModel.ts`) with GNC telemetry state:
     - true_state, estimated_state, control outputs, sensor statuses, ekf health, motor outputs
  2. Create new WebSocket hook `useTelemetrySocket.ts`:
     - Connects to `ws://localhost:8000/ws/telemetry`
     - Parses TelemetryMessage, updates store
     - Sends commands (disturbance, failure, reset) to server
  3. Create `DroneAttitudeViewer` component:
     - 3D quad model that rotates in real-time based on estimated roll/pitch/yaw
     - Ghost/transparent model showing true state for comparison
     - This replaces or augments the existing RobotViewer for GNC mode
  4. Create `SensorStatusPanel` component:
     - Shows each sensor: IMU (accel), IMU (gyro), Ultrasonic, LiDAR
     - Color: green=healthy, yellow=degraded, red=failed
     - Shows last reading values
  5. Create `StateOverlay` component:
     - Side-by-side or overlaid: true state vs estimated state
     - Position, velocity, attitude as numeric readouts
     - Divergence highlighted when estimation error grows
  6. Create `SimControlPanel` component:
     - Buttons: "Inject Wind Gust", "Kill Ultrasonic", "Degrade Accelerometer", "Full IMU Failure", "Recover All", "Reset Sim"
     - Altitude target slider
     - Each button sends the corresponding WebSocket command
  7. Create `MotorOutputBars` component:
     - 4 vertical bars (M1-M4) showing current thrust percentage
     - Color-coded: green normal, orange high, red saturated
  8. Update `App.tsx` to compose new layout with all GNC components
  9. Keep existing LidarOverlay functional (it still works with obstacle data in telemetry)
- **Produces:** Interactive GNC dashboard. The demo and portfolio showpiece.
- **Depends on:** Task 3.3 (WebSocket server + protocol)
- **Files in scope:** `showcase/client/src/` (all files)
- **Files to avoid:** All `src/` Python files, `main.py`, `demos/`
- **Completion notes:**

### Task 4.3: Unit Test Suite
- **Agent:** Agent 3
- **Tool:** Codex
- **Status:** NOT_STARTED
- **Description:**
  1. Create `tests/test_drone_model.py`:
     - Free fall: no thrust, position increases in Z (NED down) with gravity
     - Hover thrust: thrust = mass * g, velocity stays ~zero
     - Drag: constant thrust above hover, velocity converges to terminal velocity
     - Torque: asymmetric thrust produces angular acceleration in correct axis
  2. Create `tests/test_motor_model.py`:
     - Step response: commanded thrust, actual thrust converges with time constant tau
     - Clamping: commanded > max results in max output
  3. Expand `tests/test_ekf.py` (if not complete from Task 3.1):
     - Convergence test: start with wrong initial state, observe convergence with measurements
     - Bias estimation: inject known gyro bias, verify filter estimates it
     - Sensor dropout: skip altitude updates, verify Z covariance grows
     - Innovation gating: large outlier measurement is rejected
  4. Run full test suite, ensure all pass: `pytest tests/ -v`
- **Produces:** Comprehensive test coverage. Confidence for future changes.
- **Depends on:** All prior waves (tests import all modules)
- **Files in scope:** `tests/` (all files)
- **Files to avoid:** All `src/` files (read only), `main.py`, `showcase/`, `demos/`
- **Completion notes:**

---

## Dependencies

### Wave 1 (no internal dependencies)
- Task 1.1 and Task 1.2 are independent (Agent 2 creates math_utils files directly, Agent 1 creates the parent `src/math_utils/` directory)
- **Coordination:** Agent 1 must create `src/math_utils/` directory. Agent 2 must create the `__init__.py`, `quaternion.py`, `rotations.py` files within it.

### Wave 2 depends on Wave 1
- Task 2.1 depends on Task 1.1 (config) + Task 1.2 (math_utils)
- Task 2.2 depends on Task 1.1 (config)
- Task 2.3 depends on Task 1.1 (config) + Task 1.2 (rotations for IMU gravity transform)

### Wave 3 depends on Wave 2
- Task 3.1 depends on Task 1.2 (math_utils) + Task 1.1 (config)
- Task 3.2 depends on ALL Wave 2 tasks + Task 3.1
- Task 3.3 depends on Task 3.2 (needs Simulation class interface)

### Wave 4 depends on Wave 3
- Task 4.1 depends on Task 3.2 (main loop) + Task 3.1 (EKF)
- Task 4.2 depends on Task 3.3 (WebSocket server)
- Task 4.3 depends on all prior modules existing

## Collision Risks

1. **`src/math_utils/` directory**: Agent 1 (Wave 1) creates the directory, Agent 2 (Wave 1) creates files in it. Low risk — directory creation is idempotent.
2. **`src/sensors/__init__.py`**: Created by Agent 1 (Wave 1, restructure) but Agent 3 (Wave 2) may need to update it. Resolution: Agent 1 creates a minimal `__init__.py`, Agent 3 updates exports in Wave 2.
3. **`src/navigation/__init__.py`**: Created by Agent 1 (Wave 1) but updated by Agent 1 (Wave 3). Same agent, no conflict.
4. **`tests/test_ekf.py`**: Created by Agent 1 (Wave 3, Task 3.1), potentially expanded by Agent 3 (Wave 4, Task 4.3). Different waves, no conflict.
5. **`main.py`**: Owned by Agent 2 in Wave 3. No other agent touches it after that.

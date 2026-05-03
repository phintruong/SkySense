# Codex Context: SkySense GNC

Read this before touching the codebase. `.orchestrator/plan.md` is the historical master plan, but this file summarizes the current working context for Codex.

## Project Purpose
SkySense is being transformed from a LiDAR obstacle detection project into a simulation-first drone Guidance, Navigation, and Control stack. The target hardware is Raspberry Pi 4, MPU-6050 IMU, HC-SR04 ultrasonic sensor, RPLIDAR A1, Emax ECO II 2807 motors, and a quad-X frame.

## Critical Conventions
- Coordinate frame: NED. X = North, Y = East, Z = Down. Gravity is `[0, 0, +9.81]`. A 2 m altitude hover target is `z = -2.0`.
- Body frame: X forward, Y right, Z down.
- Quaternions: `[qw, qx, qy, qz]`, scalar-first, Hamilton convention.
- The project stores orientation as body-to-NED. Body-frame angular velocity propagation uses:
  `q_dot = 0.5 * quat_multiply(q, omega_quat)`.
- Euler angles are `[roll, pitch, yaw]` in radians, 3-2-1 / ZYX order, for display and PID inputs.
- IMU accelerometer measures specific force, not inertial acceleration. In level hover it reads approximately `[0, 0, -9.81]` in body frame.
- Ultrasonic measures altitude as positive-up distance to ground: `altitude = -position_ned[2]`.

## Current Architecture
Main flight stack:
- `main.py`: `Simulation` class, 200 Hz closed-loop sim.
- `src/config/`: `DroneParams`, `SimParams`.
- `src/math_utils/`: quaternion and rotation helpers.
- `src/simulation/`: `DroneModel`, `MotorModel`, `Environment`.
- `src/sensors/`: simulated IMU/ultrasonic plus optional real hardware drivers.
- `src/navigation/`: 13-state EKF and `StateEstimator`; legacy `obstacle_detector.py`.
- `src/control/`: PID, attitude controller, position controller, motor mixer.
- `src/telemetry/`: `DataLogger`.
- `src/server/`: FastAPI telemetry server and protocol.
- `showcase/client/`: React/Three.js GNC telemetry dashboard.

Legacy/reference areas:
- `Logic/`: original LiDAR backend; do not use for new GNC work except as reference.
- `legacy/rplidar_ros2/`: preserved ROS 2 RPLIDAR node.

## Wave Status
- Wave 1 foundation: complete.
- Wave 2 dynamics/control/sensors: complete.
- Wave 3 EKF/main loop/telemetry/server: complete.
- Closed-loop divergence bug: resolved.
- Wave 4 frontend: complete. GNC dashboard rebuilt from scratch (parts explorer replaced).
- Wave 5 demos/tests/docs: in progress.

## Closed-Loop Stability Fix
The old docs may mention active altitude divergence. The resolved source of the final instability was attitude feedback coupling:
- `src/simulation/drone_model.py` and `src/navigation/ekf.py` now use the correct body-rate quaternion multiplication order.
- `src/control/attitude_controller.py` accepts optional `current_rates_body` and uses gyro body-rate damping, `-kd * body_rate`, when provided.
- `main.py` passes bias-corrected IMU gyro rates to the attitude controller.
- `tests/test_closed_loop.py` runs a seeded 10 s EKF-in-loop hover regression and requires target error `< 0.5 m`.

Latest verified commands:
```bash
pytest tests -q
python main.py
```
Expected current result: all tests pass, and `python main.py` ends within `0.5 m` of `[0, 0, -2]`.

## Control Loop Notes
- Cascaded flow: position controller -> desired roll/pitch/thrust -> attitude controller -> body torques -> motor mixer -> motor model -> drone dynamics.
- Outer position loop runs at 50 Hz; inner attitude loop runs at 200 Hz.
- For v1, X/Y position hold is intentionally disabled in `main.py` because there is no GPS/external position sensor. Altitude uses direct ultrasonic feedback.
- Position controller altitude sign is corrected for NED: negative z error means increase thrust.
- Emergency mode sets zero roll/pitch and uses reduced thrust for controlled descent.

## EKF Notes
- State vector: `[pos(3), vel(3), quat(4), gyro_bias(3)]`.
- Predict uses IMU gyro and accelerometer specific force.
- Updates use ultrasonic altitude and accelerometer gravity observation.
- Yaw and X/Y position are not fully observable without magnetometer/GPS. Treat drift as expected in v1.
- Innovation and covariance metrics are exposed for telemetry/frontend health.

## Server And Frontend
- New server entrypoint: `python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000`.
- Telemetry WebSocket: `/ws/telemetry`.
- Frontend is the Wave 4 GNC dashboard and connects to `ws://localhost:8000/ws/telemetry`.
- Dashboard features include 3D attitude visualization, true-vs-estimated ghost model, system health, failure controls, motor bars, and telemetry graphs.
- Frontend dev command:
  ```bash
  cd showcase/client
  npm run dev
  ```

## Test Commands
```bash
pytest tests -q
pytest tests/test_closed_loop.py -q
python main.py
```

## Development Rules
- Prefer new development in `src/`, `tests/`, `demos/`, and `showcase/client/src/`.
- Do not modify `Logic/` for GNC work unless explicitly asked; it is legacy reference.
- Do not change frame conventions casually. Most bugs in this repo come from NED/body/quaternion sign mistakes.
- If a doc conflicts with `AGENTS.md` and `.orchestrator/bugs/closed_loop_divergence.md`, trust the resolved bug note and current tests.
- Keep generated caches (`__pycache__`, `.pytest_cache`, frontend build output) out of meaningful changes.

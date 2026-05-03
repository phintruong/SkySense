# SkySense — GNC Flight Software

## What This Is
A real drone GNC (Guidance, Navigation, Control) flight software stack. Started as a LiDAR obstacle detection system, now being transformed into a full flight control system with physics simulation, state estimation, and control.

## Ground Truth
**`AGENTS.md` is the current Codex context summary.** **`.orchestrator/plan.md` is the historical master plan.** Read both in any new session; if older plan/status notes conflict with the resolved bug note or current tests, trust `AGENTS.md` and `.orchestrator/bugs/closed_loop_divergence.md`.

## Architecture Conventions (CRITICAL)
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81]. Altitude 2m = z=-2.0.
- **Quaternion**: [qw, qx, qy, qz] scalar-first, Hamilton convention. Internal representation.
- **Euler**: [roll, pitch, yaw] in radians, 3-2-1 (ZYX) rotation order. For display and PID only.
- **Body frame**: X=forward, Y=right, Z=down.
- **Sim rate**: 200 Hz (dt=0.005), EKF updates at sensor arrival rate.

## Hardware Target
Raspberry Pi 4, MPU-6050 IMU, Emax ECO II 2807 1300KV motors, quad-X frame. Building in simulation first, then deploying to hardware.

## Project Structure
```
src/
  config/          # DroneParams, SimParams dataclasses
  math_utils/      # quaternion ops, rotation conversions (NED, body<->NED)
  simulation/      # DroneModel (rigid body), MotorModel (first-order lag), Environment
  sensors/         # IMUSim, UltrasonicSim (with failure injection), real hardware drivers
  navigation/      # 13-state EKF, StateEstimator, obstacle_detector (legacy)
  control/         # PID, AttitudeController, PositionController, MotorMixer (quad-X)
  telemetry/       # DataLogger, matplotlib plotter
  server/          # FastAPI WebSocket server (telemetry + commands)
main.py            # 200 Hz simulation loop
tests/             # pytest suite
legacy/            # ROS2 node (preserved)
Logic/             # original LiDAR system (reference)
showcase/client/   # React Three.js frontend (evolving to GNC dashboard)
demos/             # failure recovery demo scripts + output plots
```

## Commands
```bash
# Run tests
pytest tests/ -v

# Run simulation
python main.py

# Run WebSocket server
python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000

# Run frontend
cd showcase/client && npm run dev
```

## Orchestration
The project is built in waves using `.orchestrator/`. Each wave has instruction files for parallel agents in `.orchestrator/instructions/`. Status files go in `.orchestrator/status/`. See plan.md for current wave progress.

### Wave Status
- Wave 1 (Foundation): COMPLETE — config + math_utils, 25 tests
- Wave 2 (Core Modules): COMPLETE — dynamics, control, sensors, 38 tests
- Wave 3 (Estimation + Integration): COMPLETE — EKF, main loop, telemetry, server, 47 tests
- Wave 4 (Frontend + Tests + Demos): NOT STARTED

### Resolved Bug: Closed-Loop Altitude Divergence
The full sim loop (`python main.py`) now holds the 2 m hover target within the documented 0.5 m threshold. The final fix corrected quaternion propagation order for body-rate integration and changed attitude derivative damping to use bias-corrected gyro body rates. See `.orchestrator/bugs/closed_loop_divergence.md` for details.

## Key Design Decisions
- 13-state EKF: [pos(3), vel(3), quat(4), gyro_bias(3)]
- Sensors in EKF: IMU (accel+gyro), ultrasonic (altitude). No magnetometer, no GPS.
- Yaw and XY position drift accepted for v1 (unobservable without mag/GPS).
- Cascaded control: position PID -> desired angles + thrust, attitude PID -> torques.
- Motor model: first-order lag (tau=0.03s). Drag: linear (b=0.3).
- v1 goal: hover stability, disturbance rejection, sensor failure recovery demos.
- Failure demos: ultrasonic dropout, accelerometer degradation, full IMU failure -> emergency descent.

## Dependencies
numpy, scipy, matplotlib, fastapi, uvicorn, pytest (see requirements.txt)

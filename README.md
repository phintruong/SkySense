# SkySense — Drone GNC Flight Software

Simulation-first Guidance, Navigation, and Control stack for a quad-X drone. Features a 13-state Extended Kalman Filter, cascaded PID control, sensor failure recovery, and a real-time 3D telemetry dashboard.

## Prerequisites

- Python 3.10+
- Node.js 18+
- npm

## Setup

```bash
# Install Python dependencies
pip install -r requirements.txt

# Install frontend dependencies
cd showcase/client
npm install
cd ../..
```

## Running

### Option 1: Terminal Simulation

Run a 10-second hover simulation and print results:

```bash
python main.py
```

Expected output: the drone holds a 2m altitude target with < 0.5m error.

### Option 2: Live Dashboard

Open two terminals in the project root.

**Terminal 1 — Backend (simulation server):**

```bash
python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000
```

**Terminal 2 — Frontend (dev server):**

```bash
cd showcase/client
npm run dev
```

Open **http://localhost:5173** in your browser.

You will see:
- A 3D drone model rotating in real time based on EKF-estimated attitude
- Live telemetry readouts (orientation, position, velocity, gyro bias)
- Four real-time graphs (attitude, altitude, angular velocity, motor thrust)
- System status badge (NOMINAL / DEGRADED / DRIFTING / EMERGENCY)
- Motor thrust bars color-coded by saturation level

### Things to try in the dashboard

| Button | What happens |
|--------|-------------|
| **Wind Gust** | Applies a 5N push. Watch the drone recover and the graphs spike. |
| **Kill Ultrasonic** | Disables altitude sensor. Status goes DEGRADED, altitude estimate drifts. |
| **Degrade Accel** | Injects 10x accelerometer noise. EKF innovation norm spikes. |
| **IMU Failure** | Full IMU loss. Triggers EMERGENCY mode and controlled descent. |
| **Recover All** | Restores all sensors to normal. |
| **Reset Sim** | Resets the simulation to initial hover state. |
| **Altitude Slider** | Changes the hover target between 0.5m and 4.0m. |

A semi-transparent ghost drone appears when the true state diverges from the estimated state, letting you visually compare estimation accuracy.

### Option 3: Demo Plots

Generate publication-quality failure recovery plots:

```bash
python demos/run_demos.py
```

Outputs 35 PNG files to `demos/output/` covering five scenarios:

1. **Hover Stability** — baseline convergence to 2m target
2. **Disturbance Rejection** — 5N wind gust recovery
3. **Ultrasonic Dropout** — altitude sensor loss and recovery
4. **Accelerometer Degradation** — 10x noise, EKF resilience
5. **IMU Failure** — full sensor loss, emergency descent

Each scenario produces 7 plots: altitude, attitude, 3D trajectory, control output, EKF health, sensor status, and position error.

## Running Tests

```bash
pytest tests/ -v
```

76 tests covering quaternion math, rotations, PID controller, motor mixer, drone dynamics, motor model, EKF, and closed-loop hover regression.

## Project Structure

```
main.py                    # 200 Hz simulation loop
src/
  config/                  # DroneParams, SimParams dataclasses
  math_utils/              # Quaternion ops, rotation conversions (NED, body-to-NED)
  simulation/              # DroneModel (rigid body), MotorModel (first-order lag), Environment
  sensors/                 # IMUSim, UltrasonicSim (with failure injection)
  navigation/              # 13-state EKF, StateEstimator
  control/                 # PID, AttitudeController, PositionController, MotorMixer (quad-X)
  telemetry/               # DataLogger, matplotlib plotter
  server/                  # FastAPI WebSocket server
demos/                     # Failure recovery demo scripts + output plots
showcase/client/           # React + Three.js + Recharts telemetry dashboard
tests/                     # pytest suite (76 tests)
```

## Architecture

- **Coordinate system:** NED (North-East-Down). Gravity = [0, 0, +9.81].
- **Quaternion convention:** [qw, qx, qy, qz], scalar-first, Hamilton.
- **Control loop:** Position PID (50 Hz) -> desired angles + thrust -> Attitude PID (200 Hz) -> torques -> Motor mixer -> Motor lag model -> Rigid body dynamics.
- **State estimation:** 13-state EKF fusing IMU (accel + gyro) and ultrasonic (altitude). Yaw and XY position drift are accepted without magnetometer/GPS.
- **Hardware target:** Raspberry Pi 4, MPU-6050 IMU, HC-SR04 ultrasonic, Emax ECO II 2807 motors, quad-X frame.

## Tech Stack

**Backend:** Python, NumPy, SciPy, FastAPI, Matplotlib

**Frontend:** React 19, TypeScript, React Three Fiber, Zustand, Recharts, Tailwind CSS, Vite

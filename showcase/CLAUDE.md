# Showcase - GNC Telemetry Dashboard

## Overview
Interactive real-time GNC telemetry dashboard. Connects to the Python simulation backend via WebSocket. Features 3D drone attitude visualization, live telemetry readouts, real-time graphs, sensor failure injection, and system health monitoring.

## Run
```bash
# Start backend first
python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000

# Then start frontend
cd showcase/client
npm install
npm run dev
# Vite dev server on http://localhost:5173
# Connects to WebSocket at ws://localhost:8000/ws/telemetry
```

## Module Layout
```text
showcase/client/src/
|-- App.tsx                         # Root layout: three-column + bottom graphs
|-- components/
|   |-- DroneViewer.tsx             # 3D R3F drone with IMU rotation + ghost model
|   |-- SystemStatus.tsx            # NOMINAL/DEGRADED/DRIFTING/EMERGENCY badge
|   |-- FailureControls.tsx         # 6 failure buttons + altitude slider
|   |-- TelemetryPanel.tsx          # Orientation, position, velocity, gyro bias readouts
|   |-- MotorBars.tsx               # 4 vertical motor thrust bars, color-coded
|   |-- TelemetryGraphs.tsx         # 4 Recharts time-series charts
|   `-- ConnectionBadge.tsx         # WebSocket connection indicator
|-- hooks/
|   |-- useGNCStore.ts              # Zustand store, single source of truth
|   `-- useTelemetrySocket.ts       # WebSocket client with auto-reconnect
|-- types/
|   `-- telemetry.ts                # TypeScript interfaces matching backend protocol
`-- utils/
    `-- statusClassifier.ts         # Frontend-side system status classification
```

## Key Features

### 3D Viewer (`DroneViewer.tsx`)
- Loads glTF drone model, rotates in real time from estimated Euler angles.
- Semi-transparent ghost model shows true state when estimation diverges.
- Motor meshes are color-coded: teal for normal, orange above 70%, red above 90% thrust.
- Propeller spin is proportional to motor thrust.
- OrbitControls, dark background, and subtle grid support attitude inspection.

### Telemetry Panel
- Live orientation in degrees, NED position, velocity, and gyro bias.
- Values highlight red when exceeding safe ranges, including tilt above 30 degrees or altitude error above 1 m.
- Monospace numeric readouts show `--` when disconnected.

### System Status
- Frontend-computed states: NOMINAL, DEGRADED, DRIFTING, EMERGENCY.
- Shows EKF innovation norm, covariance trace, and per-sensor health dots.
- Emergency state pulses red.

### Failure Controls
- Wind Gust one-shot 5 N push.
- Kill Ultrasonic, Degrade Accel, and IMU Failure toggles.
- Recover All and Reset Sim controls.
- Altitude target slider from 0.5 m to 4.0 m.
- Active failures are highlighted; controls are disabled when disconnected.

### Real-Time Graphs
- Recharts line charts for Attitude, Altitude with target, Angular Velocity, and Motor Thrust.
- 10-second rolling window, about 200 points at 20 Hz.
- Dark theme with entry animations disabled for streaming performance.

## Tech Stack
- React 19 / TypeScript 5.9
- React Three Fiber 9.5 + Three.js 0.182
- Zustand 5 for state management
- Recharts for time-series graphs
- Tailwind CSS 3.4
- Vite 7
- Native browser WebSocket API with auto-reconnect

## Layout
Three-column dashboard with bottom graph strip:
- Left, 220 px: system status and failure controls.
- Center, flexible: 3D drone viewer and connection badge.
- Right, 260 px: telemetry readouts and motor bars.
- Bottom, about 200 px: four real-time graphs side by side.

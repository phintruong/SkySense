# Agent C Instructions — Wave 5 (Documentation Update)

## Assignment
- **Role:** Update documentation to reflect Wave 4 frontend rebuild and current project state
- **Wave:** 5 (runs in parallel with Agents A and B)

## Objective
Update `showcase/CLAUDE.md` and `AGENTS.md` to accurately reflect the new GNC dashboard frontend that replaced the old parts explorer. Also update the Wave status in `.orchestrator/plan.md`.

## Context
The frontend was completely rebuilt in Wave 4:
- Old components deleted: `ControlPanel.tsx`, `PartInfoPanel.tsx`, `RightSidebar.tsx`, `LidarOverlay.tsx`, `RobotViewer.tsx`
- Old hooks/utils deleted: `useLidarSocket.ts`, `useUISounds.ts`, `useRobotModel.ts`, `componentSounds.ts`, `robotParts.ts`, `api.ts`, `robot.ts`
- New GNC dashboard components: `DroneViewer.tsx`, `SystemStatus.tsx`, `FailureControls.tsx`, `TelemetryPanel.tsx`, `MotorBars.tsx`, `TelemetryGraphs.tsx`, `ConnectionBadge.tsx`
- New data layer: `useGNCStore.ts`, `useTelemetrySocket.ts`, `telemetry.ts` (types), `statusClassifier.ts`
- TypeScript compiles clean, connects to `ws://localhost:8000/ws/telemetry`

## Tasks

### 1. Rewrite `showcase/CLAUDE.md`
Replace the entire file to document the new GNC dashboard. Structure:

```markdown
# Showcase — GNC Telemetry Dashboard

## Overview
Interactive real-time GNC telemetry dashboard. Connects to the Python simulation backend via WebSocket. Features 3D drone attitude visualization, live telemetry readouts, real-time graphs, sensor failure injection, and system health monitoring.

## Run
\`\`\`bash
# Start backend first
python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000

# Then start frontend
cd showcase/client
npm install
npm run dev
# Vite dev server on http://localhost:5173
# Connects to WebSocket at ws://localhost:8000/ws/telemetry
\`\`\`

## Module Layout
\`\`\`
showcase/client/src/
├── App.tsx                          # Root layout — three-column + bottom graphs
├── components/
│   ├── DroneViewer.tsx              # 3D R3F drone with IMU rotation + ghost model
│   ├── SystemStatus.tsx             # NOMINAL/DEGRADED/DRIFTING/EMERGENCY badge
│   ├── FailureControls.tsx          # 6 failure buttons + altitude slider
│   ├── TelemetryPanel.tsx           # Orientation, position, velocity, gyro bias readouts
│   ├── MotorBars.tsx                # 4 vertical motor thrust bars (color-coded)
│   ├── TelemetryGraphs.tsx          # 4 Recharts time-series (attitude, altitude, angular vel, motors)
│   └── ConnectionBadge.tsx          # WebSocket connection indicator
├── hooks/
│   ├── useGNCStore.ts               # Zustand store — single source of truth
│   └── useTelemetrySocket.ts        # WebSocket client with auto-reconnect
├── types/
│   └── telemetry.ts                 # TypeScript interfaces matching backend protocol
└── utils/
    └── statusClassifier.ts          # Frontend-side system status classification
\`\`\`

## Key Features

### 3D Viewer (DroneViewer.tsx)
- Loads glTF drone model, rotates in real-time from estimated euler angles
- Semi-transparent ghost model showing true state (visible during estimation divergence)
- Motor meshes color-coded: teal (normal), orange (>70%), red (>90% thrust)
- Propeller spin proportional to motor thrust
- OrbitControls for camera manipulation, dark background with grid

### Telemetry Panel
- Live orientation (roll/pitch/yaw degrees), position (NED), velocity, gyro bias
- Values highlight red when exceeding safe ranges (tilt >30 deg, altitude error >1m)
- Monospace font, shows "--" when disconnected

### System Status
- Computed from telemetry: NOMINAL (green), DEGRADED (yellow), DRIFTING (orange), EMERGENCY (red, pulsing)
- Shows EKF innovation norm, covariance trace, and per-sensor health dots

### Failure Controls
- Wind Gust (one-shot 5N push), Kill Ultrasonic, Degrade Accel, IMU Failure (toggles)
- Recover All, Reset Sim
- Altitude target slider (0.5–4.0m)
- Active failures highlighted, all buttons disabled when disconnected

### Real-Time Graphs
- Recharts line charts: Attitude, Altitude (with target), Angular Velocity, Motor Thrust
- 10-second rolling window (~200 points at 20 Hz)
- Dark theme, no entry animations

## Tech Stack
- React 19 / TypeScript 5.9
- React Three Fiber 9.5 + Three.js 0.182
- Zustand 5 (state management)
- Recharts (time-series graphs)
- Tailwind CSS 3.4
- Vite 7 (build tool)
- WebSocket (native browser API, auto-reconnect)

## Layout
Three-column with bottom graph strip:
- Left (220px): System status + failure controls
- Center (flex): 3D drone viewer + connection badge
- Right (260px): Telemetry readouts + motor bars
- Bottom (200px): 4 real-time graphs side-by-side
```

### 2. Update `AGENTS.md` — Wave Status section
Change Wave 4 from "next major work" to complete. Add Wave 5 as current.

Update the "Wave Status" section to:
```
## Wave Status
- Wave 1 foundation: complete.
- Wave 2 dynamics/control/sensors: complete.
- Wave 3 EKF/main loop/telemetry/server: complete.
- Closed-loop divergence bug: resolved.
- Wave 4 frontend: complete. GNC dashboard rebuilt from scratch (parts explorer replaced).
- Wave 5 demos/tests/docs: in progress.
```

Also update the "Server And Frontend" section to mention the new dashboard instead of the old parts explorer.

### 3. Update `.orchestrator/plan.md` — mark Wave 4 task status
Find Task 4.2 and change its status to DONE. Add completion notes:
```
- **Completion notes:** Frontend rebuilt as GNC telemetry dashboard. 13 new files: DroneViewer, SystemStatus, FailureControls, TelemetryPanel, MotorBars, TelemetryGraphs, ConnectionBadge, useGNCStore, useTelemetrySocket, telemetry types, statusClassifier. TypeScript clean. Old parts-explorer components deleted.
```

Also update the Wave Progress table at the top to show Wave 4.2 as COMPLETE.

## Verification
1. `showcase/CLAUDE.md` accurately describes the new frontend (file paths, features, tech stack)
2. `AGENTS.md` wave status is up to date
3. `.orchestrator/plan.md` Wave 4.2 marked DONE

## Files you own
- `showcase/CLAUDE.md` (REWRITE)
- `AGENTS.md` (EDIT — wave status + frontend sections only)
- `.orchestrator/plan.md` (EDIT — status updates only)

## Files NOT to touch
- All `src/` Python files
- All `showcase/client/src/` files (complete, don't modify)
- `tests/` (owned by Agent B)
- `demos/` (owned by Agent A)
- `main.py`
- `CLAUDE.md` (root project CLAUDE.md — different from showcase/CLAUDE.md)

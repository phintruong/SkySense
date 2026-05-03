# Wave 4 Frontend — Orchestration Plan

## Overview
Rebuild the `showcase/client/` React frontend from a parts-explorer into a GNC telemetry dashboard. The backend (FastAPI WebSocket server at `src/server/app.py`) is complete and streams telemetry at 20 Hz.

## Decisions (locked)
- **Framework**: Keep existing Vite + React + React Three Fiber + Zustand + Tailwind CSS. No Next.js.
- **Old components**: Delete `ControlPanel.tsx`, `PartInfoPanel.tsx`, `RightSidebar.tsx`, `LidarOverlay.tsx`. Rewrite `RobotViewer.tsx`, store, and socket hook.
- **3D model**: Keep glTF (`/site_models/site_models.gltf`). IMU rotation on root group. Transparent ghost clone for true-vs-estimated. Emissive overlays on motor meshes for status.
- **Layout**: Three-column + bottom graph strip. Left: status + controls (~220px). Center: 3D viewer (flex). Right: telemetry + motor bars (~260px). Bottom: graphs (~200px).
- **Graphs**: Recharts. 10-second rolling window (~200 points at 20 Hz). Failure event markers.
- **System status**: Frontend-computed from telemetry thresholds. NOMINAL / DEGRADED / DRIFTING / EMERGENCY.
- **Failure controls**: 6 buttons + altitude slider. Wind Gust, Kill Ultrasonic, Degrade Accel, IMU Failure, Recover All, Reset Sim.
- **No backend fallback**: Show disconnected state with retry loop, no mock data.
- **Styling**: Dark mode, minimal, responsive, smooth animations.

## New dependency needed
```bash
cd showcase/client && npm install recharts
```

## Agents

| Agent | Role | Files |
|-------|------|-------|
| Agent A | Data layer: types, Zustand store, WebSocket hook, status logic | `types/`, `hooks/`, `utils/` |
| Agent B | 3D Viewer: rewrite RobotViewer for GNC | `components/DroneViewer.tsx` |
| Agent C | UI panels + graphs + App.tsx layout | `components/` (all panels), `App.tsx` |

### Dependency order
1. **Agent A runs first** — store types and hook are imported by everything else.
2. **Agent B and Agent C run in parallel** — both depend on Agent A's types/store, not on each other.

## File plan

### Delete
- `src/components/ControlPanel.tsx`
- `src/components/PartInfoPanel.tsx`
- `src/components/RightSidebar.tsx`
- `src/components/LidarOverlay.tsx`
- `src/hooks/useLidarSocket.ts`
- `src/hooks/useUISounds.ts`
- `src/services/api.ts`
- `src/utils/componentSounds.ts`
- `src/config/robotParts.ts`

### Keep (reference only, will be heavily rewritten)
- `src/components/RobotViewer.tsx` → rewritten as `DroneViewer.tsx`
- `src/hooks/useRobotModel.ts` → rewritten as `useGNCStore.ts`

### Create
- `src/types/telemetry.ts` — TypeScript interfaces for telemetry protocol
- `src/hooks/useGNCStore.ts` — Zustand store for GNC state
- `src/hooks/useTelemetrySocket.ts` — WebSocket hook
- `src/utils/statusClassifier.ts` — NOMINAL/DEGRADED/DRIFTING/EMERGENCY logic
- `src/components/DroneViewer.tsx` — 3D R3F drone with IMU rotation + ghost
- `src/components/TelemetryPanel.tsx` — Numeric readouts (accel, gyro, euler, position)
- `src/components/SystemStatus.tsx` — Status badge with color
- `src/components/FailureControls.tsx` — 6 buttons + altitude slider
- `src/components/MotorBars.tsx` — 4 vertical motor thrust bars
- `src/components/TelemetryGraphs.tsx` — Recharts time-series strip
- `src/components/ConnectionBadge.tsx` — WebSocket connection indicator
- `src/App.tsx` — New layout composing all panels

## Telemetry protocol (from backend)
```typescript
interface TelemetryMessage {
  type: "telemetry";
  timestamp: number;
  true_state: {
    position: [number, number, number];    // NED meters
    velocity: [number, number, number];
    quaternion: [number, number, number, number]; // [qw, qx, qy, qz]
    euler: [number, number, number];       // [roll, pitch, yaw] radians
    angular_velocity: [number, number, number];
  };
  estimated_state: {
    position: [number, number, number];
    velocity: [number, number, number];
    quaternion: [number, number, number, number];
    euler: [number, number, number];
    gyro_bias: [number, number, number];
  };
  control: {
    thrust: number;
    torques: [number, number, number];
    motor_commands: [number, number, number, number];
    motor_actual: [number, number, number, number];
  };
  sensors: {
    imu_healthy: boolean;
    ultrasonic_healthy: boolean;
  };
  ekf: {
    innovation_norm: number;
    covariance_trace: number;
  };
  emergency: boolean;
}
```

## Commands (frontend -> backend)
```typescript
// Wind gust
{ type: "inject_disturbance", force: [5, 0, 0], duration: 0.5 }

// Sensor failures
{ type: "sensor_failure", sensor: "ultrasonic", mode: "off" }
{ type: "sensor_failure", sensor: "accel", mode: "noisy" }
{ type: "sensor_failure", sensor: "imu", mode: "off" }

// Recovery
{ type: "sensor_failure", sensor: "ultrasonic", mode: "recover" }
{ type: "sensor_failure", sensor: "accel", mode: "recover" }
{ type: "sensor_failure", sensor: "imu", mode: "recover" }

// Reset
{ type: "reset_sim" }

// Altitude target (value in meters, converted to NED z = -value)
{ type: "set_target", position: [0, 0, -2.0] }
```

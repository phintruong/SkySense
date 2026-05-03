# Agent A Instructions — Wave 4 (Data Layer)

## Assignment
- **Role:** Types, Zustand store, WebSocket hook, status classifier
- **Wave:** 4 (runs FIRST — B and C depend on your output)

## Objective
Build the data foundation for the GNC dashboard. Every UI component reads from the Zustand store you create. The WebSocket hook you write is the only connection to the backend. Get the types and store shape right — the other agents import from your files.

## Context

**Working directory:** `showcase/client/src/`
**Existing stack:** Vite + React 19 + TypeScript 5.9 + Zustand 5 + Tailwind 3.4
**Backend:** FastAPI WebSocket at `ws://localhost:8000/ws/telemetry`, streams JSON at ~20 Hz.

**Coordinate conventions (CRITICAL):**
- NED: X=North, Y=East, Z=Down. Altitude 2m = z=-2.0.
- Quaternion: [qw, qx, qy, qz] scalar-first.
- Euler: [roll, pitch, yaw] in radians.
- Body frame: X=forward, Y=right, Z=down.

## Tasks

### 1. Delete old files
Remove these (they're replaced by your new files):
- `src/hooks/useLidarSocket.ts`
- `src/hooks/useUISounds.ts`
- `src/hooks/useRobotModel.ts`
- `src/types/robot.ts`
- `src/services/api.ts`
- `src/utils/componentSounds.ts`
- `src/config/robotParts.ts`

### 2. Create `src/types/telemetry.ts`
Define TypeScript interfaces matching the backend protocol exactly:

```typescript
export interface Vec3 {
  0: number; 1: number; 2: number;
  length: 3;
}

export interface TelemetryMessage {
  type: "telemetry";
  timestamp: number;
  true_state: {
    position: [number, number, number];
    velocity: [number, number, number];
    quaternion: [number, number, number, number]; // [qw, qx, qy, qz]
    euler: [number, number, number];              // [roll, pitch, yaw] radians
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

export type SystemStatus = "NOMINAL" | "DEGRADED" | "DRIFTING" | "EMERGENCY";

export type SensorName = "imu" | "accel" | "ultrasonic";
export type FailureMode = "off" | "noisy" | "recover";

export interface DisturbanceCommand {
  type: "inject_disturbance";
  force: [number, number, number];
  duration: number;
}

export interface SensorFailureCommand {
  type: "sensor_failure";
  sensor: SensorName;
  mode: FailureMode;
}

export interface ResetCommand {
  type: "reset_sim";
}

export interface SetTargetCommand {
  type: "set_target";
  position: [number, number, number];
}

export type Command = DisturbanceCommand | SensorFailureCommand | ResetCommand | SetTargetCommand;

// For graph history
export interface TelemetrySnapshot {
  timestamp: number;
  roll: number;    // degrees (converted from radians for display)
  pitch: number;
  yaw: number;
  accelX: number;
  accelY: number;
  accelZ: number;
  altitude: number; // positive-up meters (converted from NED z)
}
```

### 3. Create `src/utils/statusClassifier.ts`

```typescript
import { TelemetryMessage, SystemStatus } from "../types/telemetry";

export function classifyStatus(msg: TelemetryMessage): SystemStatus {
  if (msg.emergency) return "EMERGENCY";

  const posErr = Math.sqrt(
    msg.true_state.position.reduce((sum, v, i) =>
      sum + (v - msg.estimated_state.position[i]) ** 2, 0)
  );
  if (posErr > 0.5) return "DRIFTING";

  if (!msg.sensors.imu_healthy || !msg.sensors.ultrasonic_healthy) return "DEGRADED";
  if (msg.ekf.innovation_norm > 2.0) return "DEGRADED";
  if (msg.ekf.covariance_trace > 1.0) return "DEGRADED";

  return "NOMINAL";
}
```

### 4. Create `src/hooks/useGNCStore.ts`
Zustand store. This is the single source of truth for the entire UI.

```typescript
import { create } from "zustand";
import { TelemetryMessage, TelemetrySnapshot, SystemStatus, Command } from "../types/telemetry";
import { classifyStatus } from "../utils/statusClassifier";

const MAX_HISTORY = 200; // 10 seconds at 20 Hz
const RAD_TO_DEG = 180 / Math.PI;

interface FailureState {
  ultrasonic: boolean;
  accel: boolean;
  imu: boolean;
}

interface GNCState {
  // Connection
  connected: boolean;
  setConnected: (v: boolean) => void;

  // Latest telemetry
  telemetry: TelemetryMessage | null;
  status: SystemStatus;
  updateTelemetry: (msg: TelemetryMessage) => void;

  // Graph history (rolling 10s window)
  history: TelemetrySnapshot[];

  // Active failures (for button highlighting)
  activeFailures: FailureState;
  setFailureActive: (sensor: keyof FailureState, active: boolean) => void;

  // Target altitude (positive-up meters, for slider display)
  targetAltitude: number;
  setTargetAltitude: (alt: number) => void;

  // Command sender (set by WebSocket hook)
  sendCommand: ((cmd: Command) => void) | null;
  setSendCommand: (fn: (cmd: Command) => void) => void;

  // Failure event log (for graph markers)
  events: { timestamp: number; label: string }[];
  addEvent: (label: string) => void;
}

export const useGNCStore = create<GNCState>((set, get) => ({
  connected: false,
  setConnected: (v) => set({ connected: v }),

  telemetry: null,
  status: "NOMINAL",
  updateTelemetry: (msg) => {
    const est = msg.estimated_state;
    const snapshot: TelemetrySnapshot = {
      timestamp: msg.timestamp,
      roll: est.euler[0] * RAD_TO_DEG,
      pitch: est.euler[1] * RAD_TO_DEG,
      yaw: est.euler[2] * RAD_TO_DEG,
      accelX: msg.true_state.angular_velocity[0], // or use a different field if preferred
      accelY: msg.true_state.angular_velocity[1],
      accelZ: msg.true_state.angular_velocity[2],
      altitude: -est.position[2],
    };
    // NOTE: accelX/Y/Z above should use actual acceleration data if available.
    // The backend sends angular_velocity, not raw accel in true_state.
    // Use estimated_state velocity differences or true_state.velocity for accel approx.
    // For v1, angular_velocity is a useful signal to graph. Rename fields if desired.

    const prev = get().history;
    const history = [...prev, snapshot].slice(-MAX_HISTORY);

    set({
      telemetry: msg,
      status: classifyStatus(msg),
      history,
    });
  },

  history: [],

  activeFailures: { ultrasonic: false, accel: false, imu: false },
  setFailureActive: (sensor, active) =>
    set((s) => ({
      activeFailures: { ...s.activeFailures, [sensor]: active },
    })),

  targetAltitude: 2.0,
  setTargetAltitude: (alt) => set({ targetAltitude: alt }),

  sendCommand: null,
  setSendCommand: (fn) => set({ sendCommand: fn }),

  events: [],
  addEvent: (label) => {
    const timestamp = get().telemetry?.timestamp ?? 0;
    set((s) => ({
      events: [...s.events.slice(-50), { timestamp, label }],
    }));
  },
}));
```

**Important note on acceleration data:** The backend `true_state` includes `angular_velocity` but not raw accelerometer readings in the telemetry snapshot. For the accel graph, use velocity differences between consecutive frames (`(v_new - v_old) / dt`) or ask Agent C to compute it in the graph component. Alternatively, if the backend is updated to include raw accel, use that directly.

### 5. Create `src/hooks/useTelemetrySocket.ts`
WebSocket hook that connects to the backend and feeds the store.

```typescript
import { useEffect, useRef, useCallback } from "react";
import { useGNCStore } from "./useGNCStore";
import { TelemetryMessage, Command } from "../types/telemetry";

const WS_URL = "ws://localhost:8000/ws/telemetry";
const RECONNECT_DELAY = 2000;

export function useTelemetrySocket() {
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const { setConnected, updateTelemetry, setSendCommand } = useGNCStore();

  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) return;

    const ws = new WebSocket(WS_URL);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      setSendCommand((cmd: Command) => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify(cmd));
        }
      });
    };

    ws.onmessage = (event) => {
      try {
        const msg: TelemetryMessage = JSON.parse(event.data);
        if (msg.type === "telemetry") {
          updateTelemetry(msg);
        }
      } catch { /* ignore malformed messages */ }
    };

    ws.onclose = () => {
      setConnected(false);
      wsRef.current = null;
      reconnectTimer.current = setTimeout(connect, RECONNECT_DELAY);
    };

    ws.onerror = () => {
      ws.close();
    };
  }, [setConnected, updateTelemetry, setSendCommand]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
      wsRef.current?.close();
    };
  }, [connect]);
}
```

## Verification
After completing all files:
1. `cd showcase/client && npx tsc --noEmit` should pass with no type errors on your files (other components will have errors until Agents B/C finish).
2. The store should be importable: `import { useGNCStore } from "./hooks/useGNCStore"`.
3. The types should cover every field in the backend's `TelemetryMessage` from `src/server/protocol.py`.

## Files you own
- `src/types/telemetry.ts` (CREATE)
- `src/utils/statusClassifier.ts` (CREATE)
- `src/hooks/useGNCStore.ts` (CREATE)
- `src/hooks/useTelemetrySocket.ts` (CREATE)

## Files to delete
- `src/hooks/useLidarSocket.ts`
- `src/hooks/useUISounds.ts`
- `src/hooks/useRobotModel.ts`
- `src/types/robot.ts`
- `src/services/api.ts`
- `src/utils/componentSounds.ts`
- `src/config/robotParts.ts`

## Files NOT to touch
- `src/components/*` (owned by Agents B and C)
- `src/App.tsx` (owned by Agent C)
- Anything outside `showcase/client/src/`

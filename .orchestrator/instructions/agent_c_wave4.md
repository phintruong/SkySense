# Agent C Instructions — Wave 4 (UI Panels + Graphs + Layout)

## Assignment
- **Role:** All UI panel components, Recharts graphs, App.tsx layout
- **Wave:** 4 (runs AFTER Agent A completes, PARALLEL with Agent B)

## Objective
Build every UI component around the 3D viewer: telemetry readouts, system status, failure controls, motor bars, real-time graphs, connection badge, and the App.tsx layout that composes them all.

## Context

**Working directory:** `showcase/client/src/`
**Stack:** React 19 + TypeScript 5.9 + Tailwind 3.4 + Zustand 5
**New dependency needed:** `recharts` (install with `npm install recharts` before starting)
**Store:** Import from `../hooks/useGNCStore` (created by Agent A)
**Types:** Import from `../types/telemetry` (created by Agent A)

**Styling requirements:**
- Dark mode throughout. Background: `#0a0a0f` or `bg-gray-950`. Panels: `bg-gray-900/80` with `backdrop-blur`.
- Text: `text-gray-100` primary, `text-gray-400` secondary.
- Monospace font for numeric values (`font-mono`).
- Smooth transitions on state changes (Tailwind `transition-colors duration-300`).
- Responsive: panels should not overlap on reasonable screen sizes (1280px+).

## Zustand Store API (from Agent A)

```typescript
import { useGNCStore } from "../hooks/useGNCStore";

// Connection
const connected = useGNCStore((s) => s.connected);

// Latest telemetry (null when disconnected)
const telemetry = useGNCStore((s) => s.telemetry);

// Computed status
const status = useGNCStore((s) => s.status); // "NOMINAL" | "DEGRADED" | "DRIFTING" | "EMERGENCY"

// Graph data (rolling 10s window)
const history = useGNCStore((s) => s.history);
// Each entry: { timestamp, roll, pitch, yaw, accelX, accelY, accelZ, altitude }
// roll/pitch/yaw already in degrees

// Failure state (for button highlighting)
const activeFailures = useGNCStore((s) => s.activeFailures);
// { ultrasonic: boolean, accel: boolean, imu: boolean }

// Actions
const sendCommand = useGNCStore((s) => s.sendCommand);
const setFailureActive = useGNCStore((s) => s.setFailureActive);
const addEvent = useGNCStore((s) => s.addEvent);
const targetAltitude = useGNCStore((s) => s.targetAltitude);
const setTargetAltitude = useGNCStore((s) => s.setTargetAltitude);
```

## Tasks

### 1. Delete old components
- `src/components/ControlPanel.tsx`
- `src/components/PartInfoPanel.tsx`
- `src/components/RightSidebar.tsx`
- `src/components/LidarOverlay.tsx`

### 2. Create `src/components/ConnectionBadge.tsx`
Small badge showing WebSocket connection state. Placed in the top-right corner of the 3D viewer.

```
Connected: green dot + "LIVE"
Disconnected: red dot + "DISCONNECTED" + pulsing animation
```

~30 lines. Use absolute positioning within the viewer container.

### 3. Create `src/components/SystemStatus.tsx`
Large status badge for the left panel.

| Status | Color | Label |
|--------|-------|-------|
| NOMINAL | green (`bg-green-500/20 text-green-400 border-green-500/50`) | NOMINAL |
| DEGRADED | yellow (`bg-yellow-500/20 text-yellow-400`) | DEGRADED |
| DRIFTING | orange (`bg-orange-500/20 text-orange-400`) | DRIFTING |
| EMERGENCY | red (`bg-red-500/20 text-red-400`) + pulsing | EMERGENCY |

Also show secondary info below the badge:
- EKF innovation norm (1 decimal)
- EKF covariance trace (2 decimals)
- Sensor health: IMU / Ultrasonic (green/red dots)

~60 lines.

### 4. Create `src/components/FailureControls.tsx`
Left panel failure injection buttons + altitude slider.

**Buttons (stacked vertically):**

| Button | Action on click | Active highlight |
|--------|----------------|-----------------|
| Wind Gust | `sendCommand({ type: "inject_disturbance", force: [5, 0, 0], duration: 0.5 })` then `addEvent("Wind Gust")` | No toggle (one-shot) |
| Kill Ultrasonic | Toggle: sends `mode: "off"` or `mode: "recover"`, updates `setFailureActive("ultrasonic", ...)` | Red when active |
| Degrade Accel | Toggle: sends `mode: "noisy"` or `mode: "recover"`, updates `setFailureActive("accel", ...)` | Orange when active |
| IMU Failure | Toggle: sends `mode: "off"` or `mode: "recover"`, updates `setFailureActive("imu", ...)` | Red when active |
| Recover All | Sends recover for all three sensors, clears all `activeFailures` | Green flash |
| Reset Sim | `sendCommand({ type: "reset_sim" })`, clears `activeFailures` | Blue |

**Altitude slider:**
- Range: 0.5 to 4.0 meters, step 0.1
- Default: 2.0
- On change: `sendCommand({ type: "set_target", position: [0, 0, -value] })` and `setTargetAltitude(value)`
- Show current value as label

All buttons disabled when `connected === false`.

~120 lines.

### 5. Create `src/components/TelemetryPanel.tsx`
Right panel numeric readouts. Display estimated state values in a compact grid.

**Sections:**

**Orientation (degrees, 1 decimal):**
```
Roll     12.3°
Pitch    -1.5°
Yaw       0.8°
```

**Position (meters, 2 decimals, converted from NED):**
```
North    0.02 m
East     0.01 m
Alt      2.01 m    ← show -z (positive up)
```

**Velocity (m/s, 2 decimals):**
```
Vn       0.01
Ve       0.00
Vd      -0.02
```

**Gyro Bias (rad/s, 4 decimals):**
```
Bx       0.0012
By      -0.0003
Bz       0.0001
```

Use monospace font. Color values red if they exceed reasonable ranges (e.g., tilt > 30 deg, altitude error > 1m from target).

When disconnected, show dashes (`--`) instead of numbers.

~100 lines.

### 6. Create `src/components/MotorBars.tsx`
Four vertical bars showing motor thrust. Placed below telemetry panel in the right sidebar.

```
  M1   M2   M3   M4
  ██   ██   ██   ██   ← vertical bars, filled from bottom
  ██   ██   ██   ██
  ░░   ██   ░░   ██
  ░░   ░░   ░░   ░░
 4.2  5.1  4.0  5.3   ← Newton value below each bar
```

- Max height represents `motor_max_thrust = 12.0 N`
- Fill percentage = `motor_actual[i] / 12.0 * 100`
- Colors:
  - < 70% (8.4N): green/teal (`bg-teal-400`)
  - 70-90% (8.4-10.8N): orange (`bg-orange-400`)
  - > 90% (10.8N): red (`bg-red-400`)
- Use `motor_actual` (post-lag), not `motor_commands`
- Label layout: Front-Right, Back-Right, Back-Left, Front-Left (matching quad-X numbering)

~80 lines.

### 7. Create `src/components/TelemetryGraphs.tsx`
Bottom strip with real-time scrolling Recharts graphs.

**Four charts side by side** in a horizontal flex row:

1. **Attitude** — roll (red), pitch (green), yaw (blue) in degrees. Y-axis: -45 to +45.
2. **Altitude** — estimated altitude (solid) vs target altitude (dashed). Y-axis: 0 to 5m.
3. **Angular Velocity** — p, q, r from `true_state.angular_velocity`. Y-axis: auto-scale.
4. **Motor Thrust** — m1-m4 from `control.motor_actual`. Y-axis: 0 to 12N.

**Chart config:**
- Use `<LineChart>` from recharts with `<Line>` per series
- Data from `useGNCStore((s) => s.history)` for charts 1-3
- Chart 4 needs motor data — either add motor values to `TelemetrySnapshot` in the store or compute locally
- X-axis: timestamp (relative seconds, formatted as "Xs")
- Dark styling: `stroke="#1a1a2e"` for grid, transparent background
- Line strokes: use distinct colors per series, `strokeWidth={1.5}`, `dot={false}`
- Animate: `isAnimationActive={false}` (performance — no entry animations on streaming data)
- `<ResponsiveContainer width="100%" height={160}>`

**Note:** The store's `TelemetrySnapshot` has `accelX/Y/Z` which Agent A mapped to angular velocity. For chart 3, use those fields and label as "Angular Velocity" (p/q/r). For chart 4 (motors), you'll need to extend the snapshot or read `telemetry.control.motor_actual` directly and maintain a separate small buffer.

~150 lines.

### 8. Create `src/App.tsx`
The root layout that composes everything.

```tsx
import { useTelemetrySocket } from "./hooks/useTelemetrySocket";
import { DroneViewer } from "./components/DroneViewer";
import { SystemStatus } from "./components/SystemStatus";
import { FailureControls } from "./components/FailureControls";
import { TelemetryPanel } from "./components/TelemetryPanel";
import { MotorBars } from "./components/MotorBars";
import { TelemetryGraphs } from "./components/TelemetryGraphs";
import { ConnectionBadge } from "./components/ConnectionBadge";

export default function App() {
  useTelemetrySocket(); // establishes WebSocket connection

  return (
    <div className="h-screen w-screen bg-gray-950 text-gray-100 flex flex-col overflow-hidden">
      {/* Main area: three columns */}
      <div className="flex-1 flex min-h-0">

        {/* Left panel */}
        <div className="w-56 flex flex-col gap-4 p-4 border-r border-gray-800 overflow-y-auto">
          <SystemStatus />
          <FailureControls />
        </div>

        {/* Center: 3D viewer */}
        <div className="flex-1 relative min-w-0">
          <DroneViewer />
          <ConnectionBadge />
        </div>

        {/* Right panel */}
        <div className="w-64 flex flex-col gap-4 p-4 border-l border-gray-800 overflow-y-auto">
          <TelemetryPanel />
          <MotorBars />
        </div>
      </div>

      {/* Bottom: graphs */}
      <div className="h-48 border-t border-gray-800 flex-shrink-0">
        <TelemetryGraphs />
      </div>
    </div>
  );
}
```

Remove all old imports. The App should be clean — just the socket hook call and the layout.

### 9. Clean up `index.css` / `main.tsx`
- Ensure Tailwind dark mode styles are applied (should already be since existing setup uses Tailwind)
- Remove any old CSS that referenced the parts explorer components
- `main.tsx` should just render `<App />` — no changes likely needed

## Verification
1. All components render without error when `telemetry` is null (disconnected state shows dashes, disabled buttons, no graph data).
2. When connected to backend (`python -m uvicorn src.server.app:app --host 0.0.0.0 --port 8000`), all panels show live data.
3. Failure buttons send correct WebSocket commands and toggle highlighting.
4. Graphs scroll smoothly without jank at 20 Hz update rate.
5. Layout doesn't overflow or overlap at 1280x720 and above.
6. No TypeScript errors: `npx tsc --noEmit`.

## Files you own
- `src/components/ConnectionBadge.tsx` (CREATE)
- `src/components/SystemStatus.tsx` (CREATE)
- `src/components/FailureControls.tsx` (CREATE)
- `src/components/TelemetryPanel.tsx` (CREATE)
- `src/components/MotorBars.tsx` (CREATE)
- `src/components/TelemetryGraphs.tsx` (CREATE)
- `src/App.tsx` (REWRITE)

## Files to delete
- `src/components/ControlPanel.tsx`
- `src/components/PartInfoPanel.tsx`
- `src/components/RightSidebar.tsx`
- `src/components/LidarOverlay.tsx`

## Files NOT to touch
- `src/hooks/*` (owned by Agent A)
- `src/types/*` (owned by Agent A)
- `src/utils/*` (owned by Agent A)
- `src/components/DroneViewer.tsx` (owned by Agent B)
- Anything outside `showcase/client/src/`

# Showcase — React/Three.js Frontend

> **Wave 4** will evolve this into a full GNC telemetry dashboard. See `.orchestrator/plan.md` Task 4.2 for the spec: 3D attitude viz, sensor status panel, true vs estimated state overlay, sim control panel, motor output bars.

## Overview
Currently: interactive 3D drone model viewer with live LiDAR visualization. Built with React, Three.js (via React Three Fiber), Zustand for state, and Tailwind for styling. Connects to the Python backend via WebSocket for real-time sensor data.

## Run
```bash
cd showcase/client
npm install
npm run dev
# Vite dev server on http://localhost:5173
# Expects Python backend WebSocket on ws://localhost:8000/ws/lidar
```

## Module Layout
```
showcase/client/src/
├── App.tsx                          # Root layout
├── components/
│   ├── RobotViewer.tsx              # Main 3D scene (~930 lines)
│   ├── ControlPanel.tsx             # Left panel: search, filter, explode, drive toggle
│   ├── PartInfoPanel.tsx            # Selected part details
│   ├── RightSidebar.tsx             # Parts list + guided tour
│   └── LidarOverlay.tsx            # Live 2D radar overlay (bottom-left)
├── hooks/
│   ├── useRobotModel.ts            # Zustand store (all global state)
│   ├── useLidarSocket.ts           # WebSocket client to Python backend
│   └── useUISounds.ts              # Audio playback
├── config/
│   ├── robotParts.ts               # 25+ parts metadata, groups, search
│   └── sounds.ts                   # Sound mappings
├── services/api.ts                 # API stub
├── types/robot.ts                  # TypeScript types
└── utils/componentSounds.ts        # Per-part audio mapping
```

## Key Features

### 3D Viewer (RobotViewer.tsx)
- Loads glTF model from `/site_models/site_models.gltf`
- Part selection with group highlighting (magenta glow, non-selected fade)
- Explode animation: parts move outward from center, adjustable 0-100
- Propeller spin: 60 rad/s, diagonal pairs counter-rotate, only when moving
- Focus-on-selection camera animation (smoothstep easing, 0.65s)

### Drive/Fly Mode
- WASD: forward/back/steer, Space/Ctrl: up/down, B: boost, Shift: brake
- Physics: acceleration, damping, max speed clamping
- Infinite grass ground with lazy-loaded chunks and instanced grass blades

### LiDAR Overlay (LidarOverlay.tsx)
- 220×220px canvas, bottom-left
- Range rings (1m intervals), forward cone visualization (±80°)
- Color-coded scan points: red (<0.3m), yellow (0.3-0.5m), green (>0.5m)
- Connection status badge, action label, obstacle count

### State (useRobotModel.ts — Zustand)
- Part selection/highlighting, explode strength, camera mode
- Drone position/speed/heading for drive mode
- LiDAR state: connected, scan data, action, obstacles, tilt angle
- Sound settings

### WebSocket (useLidarSocket.ts)
- Connects to `ws://localhost:8000/ws/lidar`
- Auto-reconnect on disconnect (2s delay)
- Parses JSON: scan, action, obstacles, tiltAngle, timestamp

## Tech Stack
- React 18+ / TypeScript
- React Three Fiber + Three.js
- Zustand (state management)
- Tailwind CSS
- Vite (build tool)
- WebSocket (native browser API)

# Agent 3 Instructions — Wave 3

## Assignment
- **Assigned Tool:** Codex
- **Wave:** 3
- **Role:** WebSocket server evolution — new GNC telemetry protocol

## Objective
Build the FastAPI WebSocket server that streams GNC telemetry to the React frontend and receives commands (disturbance injection, sensor failures, sim reset). This replaces/extends the legacy `Logic/server.py`.

## Context

**The server wraps a Simulation class** (built by Agent 2, Task 3.2). You can define the server assuming this interface exists — you don't need to wait for it. Here's the exact interface:

```python
from main import Simulation

sim = Simulation()

# Run one step
sim.step()

# Run for duration
sim.run(duration=10.0)

# Get current state snapshot (for WebSocket streaming)
snapshot = sim.get_telemetry_snapshot()
# Returns:
# {
#     "timestamp": float,
#     "true_state": {"position": [3], "velocity": [3], "quaternion": [4], "euler": [3], "angular_velocity": [3]},
#     "estimated_state": {"position": [3], "velocity": [3], "quaternion": [4], "euler": [3], "gyro_bias": [3]},
#     "control": {"thrust": float, "torques": [3], "motor_commands": [4], "motor_actual": [4]},
#     "sensors": {"imu_healthy": bool, "ultrasonic_healthy": bool},
#     "ekf": {"innovation_norm": float, "covariance_trace": float},
#     "emergency": bool
# }

# Commands
sim.inject_disturbance(np.array([5.0, 0.0, 0.0]), duration=0.1)
sim.inject_sensor_failure("ultrasonic", "off")   # sensor: "imu"|"accel"|"ultrasonic", mode: "off"|"noisy"|"recover"
sim.set_target(np.array([0.0, 0.0, -3.0]))       # NED position
sim.reset()
```

**Legacy server reference:** `Logic/server.py` has the old FastAPI + WebSocket pattern. You can reference its structure but build fresh.

## Tasks

### Task 3.3a: Protocol Definitions
- **Description:** Create `src/server/protocol.py`

```python
from dataclasses import dataclass, asdict
from typing import Literal
import numpy as np

# Server -> Client messages

@dataclass
class TelemetryMessage:
    type: str = "telemetry"
    timestamp: float = 0.0
    true_state: dict = None      # position, velocity, quaternion, euler, angular_velocity
    estimated_state: dict = None  # position, velocity, quaternion, euler, gyro_bias
    control: dict = None          # thrust, torques, motor_commands, motor_actual
    sensors: dict = None          # imu_healthy, ultrasonic_healthy
    ekf: dict = None              # innovation_norm, covariance_trace
    emergency: bool = False
    
    def to_json(self) -> dict:
        """Convert to JSON-serializable dict. Convert numpy arrays to lists."""
        d = asdict(self)
        return _convert_numpy(d)

# Client -> Server messages (parsed from JSON)

VALID_COMMANDS = {"inject_disturbance", "sensor_failure", "reset_sim", "set_target"}

def parse_command(data: dict) -> dict:
    """
    Parse incoming WebSocket command.
    
    Expected formats:
    {"type": "inject_disturbance", "force": [fx, fy, fz], "duration": 0.1}
    {"type": "sensor_failure", "sensor": "ultrasonic"|"accel"|"imu", "mode": "off"|"noisy"|"recover"}
    {"type": "reset_sim"}
    {"type": "set_target", "position": [x, y, z]}  # NED
    
    Returns parsed dict or raises ValueError.
    """

def _convert_numpy(obj):
    """Recursively convert numpy arrays/types to Python lists/types for JSON."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, dict):
        return {k: _convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_convert_numpy(i) for i in obj]
    return obj
```

- **Acceptance criteria:**
  - [ ] TelemetryMessage.to_json() produces JSON-serializable dict
  - [ ] parse_command() validates and parses all 4 command types
  - [ ] parse_command() raises ValueError for unknown commands
  - [ ] numpy arrays converted to lists in JSON output

### Task 3.3b: FastAPI WebSocket Server
- **Description:** Create `src/server/app.py`

```python
import asyncio
import json
import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from contextlib import asynccontextmanager

from .protocol import TelemetryMessage, parse_command, _convert_numpy

app = FastAPI(title="SkySense GNC Server")

# Global simulation instance (created on startup)
simulation = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize simulation on startup."""
    global simulation
    from main import Simulation
    simulation = Simulation()
    yield
    # Cleanup

app = FastAPI(title="SkySense GNC Server", lifespan=lifespan)

# Background sim task
sim_task = None

async def run_simulation():
    """
    Run simulation loop in background.
    Steps at ~200 Hz (5ms per step).
    In practice, run as fast as possible and let asyncio schedule.
    """
    global simulation
    while True:
        simulation.step()
        # Yield to event loop periodically (every step)
        if simulation.step_count % 10 == 0:
            await asyncio.sleep(0)  # yield to event loop
        else:
            await asyncio.sleep(0)  # always yield to avoid blocking

@app.get("/api/status")
async def get_status():
    """Health check + sim state summary."""
    if simulation is None:
        return {"status": "not_initialized"}
    snapshot = simulation.get_telemetry_snapshot()
    return {
        "status": "running",
        "time": snapshot["timestamp"],
        "emergency": snapshot["emergency"],
        "position": _convert_numpy(snapshot["true_state"]["position"]),
    }

@app.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    """
    Bidirectional WebSocket for GNC telemetry.
    
    Server -> Client: TelemetryMessage at ~20 Hz
    Client -> Server: commands (disturbance, failure, reset, set_target)
    """
    global sim_task, simulation
    await websocket.accept()
    
    # Start sim if not running
    if sim_task is None or sim_task.done():
        sim_task = asyncio.create_task(run_simulation())
    
    try:
        while True:
            # Send telemetry snapshot
            snapshot = simulation.get_telemetry_snapshot()
            msg = TelemetryMessage(
                timestamp=snapshot["timestamp"],
                true_state=snapshot["true_state"],
                estimated_state=snapshot["estimated_state"],
                control=snapshot["control"],
                sensors=snapshot["sensors"],
                ekf=snapshot["ekf"],
                emergency=snapshot["emergency"],
            )
            await websocket.send_json(msg.to_json())
            
            # Check for incoming commands (non-blocking)
            try:
                data = await asyncio.wait_for(
                    websocket.receive_json(), 
                    timeout=0.05  # 50ms = ~20 Hz send rate
                )
                handle_command(data)
            except asyncio.TimeoutError:
                pass  # No command, continue sending telemetry
    except WebSocketDisconnect:
        pass

def handle_command(data: dict):
    """Dispatch incoming command to simulation."""
    global simulation
    try:
        cmd = parse_command(data)
        cmd_type = cmd["type"]
        
        if cmd_type == "inject_disturbance":
            simulation.inject_disturbance(
                np.array(cmd["force"]),
                duration=cmd.get("duration", 0.1)
            )
        elif cmd_type == "sensor_failure":
            simulation.inject_sensor_failure(cmd["sensor"], cmd["mode"])
        elif cmd_type == "reset_sim":
            simulation.reset()
        elif cmd_type == "set_target":
            simulation.set_target(np.array(cmd["position"]))
    except (ValueError, KeyError) as e:
        print(f"Invalid command: {e}")
```

- **Acceptance criteria:**
  - [ ] `uvicorn src.server.app:app` starts without error
  - [ ] `GET /api/status` returns sim state
  - [ ] `WS /ws/telemetry` streams TelemetryMessage JSON at ~20 Hz
  - [ ] Client can send disturbance/failure/reset/set_target commands
  - [ ] Simulation runs in background asyncio task
  - [ ] WebSocket disconnect handled gracefully

### Task 3.3c: Module Init
- **Description:** Update `src/server/__init__.py`
```python
from .app import app
from .protocol import TelemetryMessage, parse_command
```

- **Acceptance criteria:**
  - [ ] `from src.server import app` works

## Scope

### Files you OWN:
- `src/server/app.py`
- `src/server/protocol.py`
- `src/server/__init__.py`

### Files to AVOID:
- `Logic/server.py` (legacy, don't modify)
- `main.py` (Agent 2 owns this)
- `src/simulation/`, `src/control/`, `src/sensors/`, `src/navigation/`
- `showcase/` (Wave 4)

### Files you may READ:
- `Logic/server.py` (reference for legacy patterns)
- `src/config/`
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- FastAPI + uvicorn installed (in requirements.txt)
- You do NOT need Task 3.2 to be complete — the Simulation interface is fully specified above. Just import from `main` and it will work at runtime.

### What depends on YOUR output:
- **Task 4.2 (React Frontend)**: connects to `/ws/telemetry` and sends commands
- The frontend team needs your exact JSON message format — `TelemetryMessage.to_json()` output IS the API contract.

## Status Updates
Write to `.orchestrator/status/agent_3_wave3_status.md`.

## Definition of Done
- [ ] Protocol defines TelemetryMessage with numpy->JSON conversion
- [ ] parse_command() handles all 4 command types with validation
- [ ] FastAPI app starts with `uvicorn src.server.app:app`
- [ ] `/api/status` endpoint works
- [ ] `/ws/telemetry` streams telemetry and receives commands
- [ ] No files outside scope were modified

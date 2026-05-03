# Agent A Instructions — Wave 5 (Demos + Plotter)

## Assignment
- **Role:** Matplotlib plotter library + failure recovery demo scripts
- **Wave:** 5 (runs in parallel with Agents B and C)

## Objective
Create publication-quality demo scenarios that showcase the GNC system's capabilities: hover stability, disturbance rejection, and sensor failure recovery. Each demo runs the sim, collects telemetry via DataLogger, generates plots, and saves them to `demos/output/`. These are the portfolio showpiece.

## Context

**Working directory:** `C:\Users\phine\OneDrive\Laptop\CODESTUFF\DRONE\SkySense`
**Python:** 3.10+, numpy, scipy, matplotlib already installed (see requirements.txt)

**Simulation API** (from `main.py`):
```python
from main import Simulation

sim = Simulation()  # Creates full stack with default params
sim.run(duration=10.0)  # Run for 10 seconds at 200 Hz
sim.inject_disturbance(np.array([5, 0, 0]), duration=0.5)  # Wind gust in NED
sim.inject_sensor_failure("ultrasonic", "off")    # Kill ultrasonic
sim.inject_sensor_failure("accel", "noisy")       # 10x accelerometer noise
sim.inject_sensor_failure("imu", "off")           # Full IMU failure → emergency descent
sim.inject_sensor_failure("ultrasonic", "recover") # Recover sensor
sim.set_target(np.array([0, 0, -3.0]))           # Change altitude target

log = sim.logger.get_log()  # Returns dict of numpy arrays
```

**DataLogger fields** (all numpy arrays after `get_log()`):
```python
log["time"]              # (N,) timestamps in seconds
log["true_position"]     # (N, 3) NED meters
log["true_velocity"]     # (N, 3) NED m/s
log["true_quaternion"]   # (N, 4) [qw, qx, qy, qz]
log["true_euler"]        # (N, 3) [roll, pitch, yaw] radians
log["est_position"]      # (N, 3) NED meters
log["est_velocity"]      # (N, 3) NED m/s
log["est_euler"]         # (N, 3) [roll, pitch, yaw] radians
log["est_gyro_bias"]     # (N, 3) rad/s
log["thrust"]            # (N,) total thrust Newtons
log["torques"]           # (N, 3) [roll, pitch, yaw] Nm
log["motor_commands"]    # (N, 4) pre-lag motor commands N
log["motor_actual"]      # (N, 4) post-lag motor thrusts N
log["imu_healthy"]       # (N,) boolean
log["ultrasonic_healthy"]# (N,) boolean
log["innovation_norm"]   # (N,) float
log["covariance_trace"]  # (N,) float
log["emergency"]         # (N,) boolean
```

**Conventions:**
- NED: X=North, Y=East, Z=Down. Altitude = -z. Hover at z=-2.0 means altitude=2.0m.
- Euler: [roll, pitch, yaw] in radians. Convert to degrees for plot labels.
- Sim rate: 200 Hz (dt=0.005). 10 seconds = 2000 data points.

## Tasks

### 1. Create `src/telemetry/plotter.py`

A reusable plotting library. Each function takes a `log` dict (from `DataLogger.get_log()`) and returns a matplotlib `Figure`. All plots should be publication quality: proper labels, legends, grid, consistent dark style.

**Style:**
```python
import matplotlib.pyplot as plt
plt.style.use("dark_background")
# Use consistent colors:
# True state: solid lines, blue/green family
# Estimated state: dashed lines, orange/red family
# Failure events: vertical red dashed lines
# Target: horizontal gray dashed line
```

**Functions to implement:**

```python
def plot_altitude(log: dict, events: list[tuple[float, str]] = None) -> plt.Figure:
    """
    Altitude over time: true (solid blue) vs estimated (dashed orange) vs target (gray dashed).
    Mark failure events as vertical red dashed lines with labels.
    Y-axis: altitude in meters (positive up, computed as -z).
    """

def plot_attitude(log: dict, events: list[tuple[float, str]] = None) -> plt.Figure:
    """
    Roll, pitch, yaw over time. 3 subplots stacked vertically.
    Each: true (solid) vs estimated (dashed). Convert radians to degrees.
    """

def plot_trajectory_3d(log: dict) -> plt.Figure:
    """
    3D position plot. True path (solid blue) vs estimated (dashed orange).
    Start marker (green dot) and end marker (red dot).
    Convert NED to display: X=North, Y=East, Z=Altitude(-Down).
    """

def plot_control_output(log: dict) -> plt.Figure:
    """
    2x2 subplots:
    - Top-left: total thrust over time
    - Top-right: torques (roll/pitch/yaw) over time
    - Bottom-left: motor commands (4 lines) over time
    - Bottom-right: motor actual (4 lines) over time
    """

def plot_ekf_health(log: dict, events: list[tuple[float, str]] = None) -> plt.Figure:
    """
    2 subplots stacked:
    - Top: innovation norm over time
    - Bottom: covariance trace over time (log scale)
    Mark failure events.
    """

def plot_sensor_status(log: dict, events: list[tuple[float, str]] = None) -> plt.Figure:
    """
    Binary timeline showing sensor health.
    Two horizontal bands: IMU and Ultrasonic.
    Green = healthy, red = failed. Step function style.
    """

def plot_position_error(log: dict, events: list[tuple[float, str]] = None) -> plt.Figure:
    """
    Position estimation error (||true - estimated||) over time.
    Also show per-axis errors as thinner lines.
    """
```

Each function should:
- Accept an optional `events` list of `(time, label)` tuples for failure event markers
- Use `tight_layout()` 
- Set figure size to `(12, 6)` for single plots, `(12, 10)` for multi-subplot
- Return the `Figure` object (don't call `plt.show()`)

### 2. Create `demos/run_demos.py`

A demo runner script that runs 5 scenarios and generates all plots.

**Structure:**
```python
#!/usr/bin/env python
"""SkySense GNC — Failure Recovery Demos"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import Simulation
from src.telemetry.plotter import (
    plot_altitude, plot_attitude, plot_control_output,
    plot_ekf_health, plot_sensor_status, plot_trajectory_3d,
    plot_position_error,
)

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def save_plots(name: str, log: dict, events: list = None):
    """Generate and save all standard plots for a demo."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    prefix = os.path.join(OUTPUT_DIR, name)

    plots = {
        "altitude": plot_altitude(log, events),
        "attitude": plot_attitude(log, events),
        "control": plot_control_output(log),
        "ekf_health": plot_ekf_health(log, events),
        "sensors": plot_sensor_status(log, events),
        "trajectory_3d": plot_trajectory_3d(log),
        "position_error": plot_position_error(log, events),
    }
    for suffix, fig in plots.items():
        fig.savefig(f"{prefix}_{suffix}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

    print(f"  Saved {len(plots)} plots to {OUTPUT_DIR}/{name}_*.png")
```

**Demo scenarios:**

```python
def demo_hover():
    """Baseline: hover at 2m for 10s. Shows convergence and stability."""
    print("\n[1/5] Hover Stability")
    sim = Simulation()
    sim.run(duration=10.0)
    log = sim.logger.get_log()

    final_pos = sim.drone.state.position
    target = sim.target_position
    error = np.linalg.norm(final_pos - target)
    print(f"  Final error: {error:.4f} m")

    save_plots("hover", log)


def demo_disturbance():
    """Hover, inject 5N wind gust at t=3s, show recovery."""
    print("\n[2/5] Disturbance Rejection")
    sim = Simulation()
    events = []

    # Run to t=3s
    sim.run(duration=3.0)

    # Inject disturbance
    sim.inject_disturbance(np.array([5.0, 0.0, 0.0]), duration=0.5)
    events.append((3.0, "5N Wind Gust"))

    # Run to t=10s
    sim.run(duration=7.0)

    log = sim.logger.get_log()
    save_plots("disturbance", log, events)


def demo_ultrasonic_dropout():
    """Hover, kill ultrasonic at t=3s, recover at t=8s."""
    print("\n[3/5] Ultrasonic Dropout & Recovery")
    sim = Simulation()
    events = []

    sim.run(duration=3.0)

    sim.inject_sensor_failure("ultrasonic", "off")
    events.append((3.0, "Ultrasonic OFF"))

    sim.run(duration=5.0)

    sim.inject_sensor_failure("ultrasonic", "recover")
    events.append((8.0, "Ultrasonic Recover"))

    sim.run(duration=4.0)

    log = sim.logger.get_log()
    save_plots("ultrasonic_dropout", log, events)


def demo_accel_degradation():
    """Hover, inject 10x accelerometer noise at t=3s, show EKF resilience."""
    print("\n[4/5] Accelerometer Degradation")
    sim = Simulation()
    events = []

    sim.run(duration=3.0)

    sim.inject_sensor_failure("accel", "noisy")
    events.append((3.0, "Accel 10x Noise"))

    sim.run(duration=7.0)

    log = sim.logger.get_log()
    save_plots("accel_degradation", log, events)


def demo_imu_failure():
    """Hover, full IMU failure at t=3s, trigger emergency descent."""
    print("\n[5/5] Full IMU Failure → Emergency Descent")
    sim = Simulation()
    events = []

    sim.run(duration=3.0)

    sim.inject_sensor_failure("imu", "off")
    events.append((3.0, "IMU FAILURE"))

    sim.run(duration=7.0)

    log = sim.logger.get_log()
    save_plots("imu_failure", log, events)


def main():
    print("=" * 50)
    print("SkySense GNC — Failure Recovery Demos")
    print("=" * 50)

    demo_hover()
    demo_disturbance()
    demo_ultrasonic_dropout()
    demo_accel_degradation()
    demo_imu_failure()

    print(f"\nAll demos complete. Plots saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
```

### 3. Create `demos/__init__.py` (empty) and ensure `demos/output/` directory is created at runtime.

## Verification
1. `python demos/run_demos.py` runs all 5 demos without crashing
2. 35 PNG files are generated in `demos/output/` (7 plots x 5 demos)
3. Plots are readable: labels, legends, grid all visible on dark background
4. Hover demo shows convergence to 2m target
5. Failure demos clearly show the injection point and system response
6. Existing tests still pass: `pytest tests/ -q`

## Files you own
- `src/telemetry/plotter.py` (CREATE)
- `demos/run_demos.py` (CREATE)
- `demos/__init__.py` (CREATE, empty)

## Files NOT to touch
- `main.py` (read only — use the Simulation class as-is)
- `src/telemetry/logger.py` (read only)
- All `src/` modules (read only)
- `tests/` (owned by Agent B)
- `showcase/` (complete, don't touch)

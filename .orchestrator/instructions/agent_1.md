# Agent 1 Instructions

## Assignment
- **Assigned Tool:** Claude
- **Wave:** 1
- **Role:** Project restructure, config system, directory scaffolding

## Objective
Transform the project layout from the legacy `Logic/` structure into the new `src/` GNC architecture. Create the full directory tree, move existing files to their new homes, set up the config system with all drone/sim parameters, and prepare requirements.txt.

## Context

This is a real drone project (Raspberry Pi 4, MPU-6050, Emax ECO II 2807 1300KV, quad-X). We're building a full GNC stack in simulation first. All conventions:
- **NED coordinate system** (Z-down, gravity = [0, 0, +9.81])
- **Quaternions** internally [qw, qx, qy, qz], Euler for display
- **200 Hz** fixed timestep (dt = 0.005)
- **Python 3.11+**, numpy/scipy/matplotlib

## Tasks

### Task 1.1a: Create Directory Tree
- **Description:** Create the entire `src/` directory structure with `__init__.py` files. Create `tests/`, `legacy/`, `demos/output/` directories.
- **Acceptance criteria:**
  - [ ] All directories exist: `src/config`, `src/simulation`, `src/sensors`, `src/navigation`, `src/control`, `src/telemetry`, `src/server`, `src/math_utils`
  - [ ] Every `src/` subdirectory has an `__init__.py` (can be empty or minimal exports)
  - [ ] `src/__init__.py` exists
  - [ ] `tests/` directory exists with `__init__.py`
  - [ ] `legacy/` directory exists
  - [ ] `demos/output/` directory exists

### Task 1.1b: Move Existing Files
- **Description:** Move legacy code to new locations. Preserve file content exactly — do not refactor yet.
  - `Logic/core/logic.py` -> `src/navigation/obstacle_detector.py`
  - `Logic/hardware/rplidar_reader.py` -> `src/sensors/lidar.py`
  - `Logic/hardware/hc_sr04_distance.py` -> `src/sensors/ultrasonic.py`
  - `Logic/rplidar_ros2/` -> `legacy/rplidar_ros2/` (entire directory)
  - Delete `Logic/rplidar_reader.py` (it's a duplicate of `Logic/hardware/rplidar_reader.py`)
  - Delete `nul` file at project root if it exists
  - Keep `Logic/server.py`, `Logic/main.py`, `Logic/visualization/` in place for now (legacy reference)
- **Acceptance criteria:**
  - [ ] Files exist at new paths with identical content
  - [ ] `legacy/rplidar_ros2/` contains the ROS2 node
  - [ ] Duplicate `Logic/rplidar_reader.py` is deleted
  - [ ] `Logic/core/`, `Logic/hardware/` original files may remain (git tracks the copies)

### Task 1.1c: Config System
- **Description:** Create the drone parameter and simulation parameter config files as Python dataclasses.
- **Acceptance criteria:**
  - [ ] `src/config/drone_params.py` contains `DroneParams` dataclass with all fields below
  - [ ] `src/config/sim_params.py` contains `SimParams` dataclass with all fields below
  - [ ] `src/config/__init__.py` exports both classes
  - [ ] All values have sensible defaults documented in comments

**`src/config/drone_params.py` — `DroneParams` dataclass fields:**

```python
from dataclasses import dataclass, field
import numpy as np

@dataclass
class DroneParams:
    mass: float = 1.8                    # Total mass in kg (frame + battery + electronics)
    arm_length: float = 0.18             # Center to motor distance in meters
    
    # Moment of inertia (kg*m^2) — approximate for 7-inch quad
    Ixx: float = 0.0123                  # Roll inertia
    Iyy: float = 0.0123                  # Pitch inertia (symmetric with roll for X-frame)
    Izz: float = 0.0224                  # Yaw inertia (higher due to arm geometry)
    
    # Motor parameters
    motor_tau: float = 0.03              # First-order lag time constant (seconds)
    motor_max_thrust: float = 12.0       # Max thrust per motor in Newtons
    motor_min_thrust: float = 0.0        # Min thrust per motor in Newtons
    
    # Aerodynamic coefficients
    drag_coefficient: float = 0.3        # Linear drag coefficient (N/(m/s))
    
    # Propeller coefficients (quad-X)
    thrust_coefficient: float = 1.0e-5   # ct: thrust = ct * omega^2
    torque_coefficient: float = 1.0e-7   # cq: torque = cq * omega^2
    
    # Physical constants
    gravity: float = 9.81                # m/s^2 (positive in NED Z-down)
    
    # Motor numbering for quad-X (viewed from above):
    #   4 (FL, CCW)   1 (FR, CW)
    #         \       /
    #          [BODY]
    #         /       \
    #   3 (BL, CW)    2 (BR, CCW)
    
    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz])
    
    @property
    def inertia_matrix_inv(self) -> np.ndarray:
        return np.diag([1.0/self.Ixx, 1.0/self.Iyy, 1.0/self.Izz])
    
    @property
    def hover_thrust_per_motor(self) -> float:
        return (self.mass * self.gravity) / 4.0
```

**`src/config/sim_params.py` — `SimParams` dataclass fields:**

```python
from dataclasses import dataclass

@dataclass
class SimParams:
    dt: float = 0.005                        # Timestep (200 Hz)
    duration: float = 30.0                   # Default sim duration in seconds
    
    # IMU noise parameters (MPU-6050 approximate)
    imu_accel_noise_std: float = 0.05        # m/s^2
    imu_gyro_noise_std: float = 0.01         # rad/s
    imu_gyro_bias_std: float = 0.001         # rad/s per sqrt(s) (random walk)
    
    # Ultrasonic noise parameters (HC-SR04)
    ultrasonic_noise_std: float = 0.02       # meters
    
    # Sensor rates
    imu_rate_hz: float = 200.0               # IMU runs every step
    ultrasonic_rate_hz: float = 20.0         # Ultrasonic at 20 Hz
    lidar_rate_hz: float = 10.0              # LiDAR at 10 Hz
    
    # EKF tuning
    ekf_process_noise_pos: float = 0.01      # Position process noise
    ekf_process_noise_vel: float = 0.1       # Velocity process noise
    ekf_process_noise_quat: float = 0.001    # Quaternion process noise
    ekf_process_noise_bias: float = 0.0001   # Gyro bias process noise
    ekf_accel_measurement_noise: float = 0.5 # Accelerometer measurement noise
    ekf_altitude_measurement_noise: float = 0.05  # Ultrasonic measurement noise
    ekf_innovation_threshold: float = 5.0    # Innovation gate (std devs)
    
    # Control defaults
    attitude_kp: float = 8.0
    attitude_ki: float = 0.5
    attitude_kd: float = 3.0
    altitude_kp: float = 5.0
    altitude_ki: float = 1.0
    altitude_kd: float = 3.0
    position_kp: float = 1.0
    position_ki: float = 0.1
    position_kd: float = 0.8
    max_tilt_angle: float = 0.524            # ~30 degrees in radians
```

### Task 1.1d: Requirements + Test Scaffold
- **Description:** Create requirements.txt and test infrastructure.
- **Acceptance criteria:**
  - [ ] `requirements.txt` at project root with: numpy, scipy, matplotlib, fastapi, uvicorn, pytest
  - [ ] `tests/__init__.py` exists (empty)
  - [ ] `tests/conftest.py` exists with fixtures for `DroneParams()` and `SimParams()` defaults

**`tests/conftest.py`:**
```python
import pytest
from src.config import DroneParams, SimParams

@pytest.fixture
def drone_params():
    return DroneParams()

@pytest.fixture
def sim_params():
    return SimParams()
```

## Scope

### Files you OWN (you may create, modify, delete):
- `src/__init__.py`
- `src/config/__init__.py`
- `src/config/drone_params.py`
- `src/config/sim_params.py`
- `src/simulation/__init__.py`
- `src/sensors/__init__.py`
- `src/navigation/__init__.py`
- `src/control/__init__.py`
- `src/telemetry/__init__.py`
- `src/server/__init__.py`
- `src/navigation/obstacle_detector.py` (moved file)
- `src/sensors/lidar.py` (moved file)
- `src/sensors/ultrasonic.py` (moved file)
- `legacy/` (entire directory)
- `tests/__init__.py`
- `tests/conftest.py`
- `requirements.txt`
- `demos/output/` (just the directory)
- `Logic/rplidar_reader.py` (delete this)

### Files you must AVOID (owned by other agents):
- `src/math_utils/quaternion.py` (Agent 2)
- `src/math_utils/rotations.py` (Agent 2)
- `src/math_utils/__init__.py` (Agent 2)
- `tests/test_quaternion.py` (Agent 2)
- `tests/test_rotations.py` (Agent 2)

### Files you may READ but not modify:
- `Logic/core/logic.py` (read to copy)
- `Logic/hardware/rplidar_reader.py` (read to copy)
- `Logic/hardware/hc_sr04_distance.py` (read to copy)
- `Logic/rplidar_ros2/` (read to copy)
- `Goal.md`

## Dependencies

### Before you start:
- No prerequisites. This is the first wave.

### What depends on YOUR output:
- **Agent 2 (Wave 1)** needs `src/math_utils/` directory to exist (you create the directory, they create the files)
- **All Wave 2 agents** need the config system (`DroneParams`, `SimParams`) and the full directory tree
- **All Wave 2+ agents** need `requirements.txt` to install dependencies

## When Blocked
If you encounter a blocker:
1. Document it in your status file at `.orchestrator/status/agent_1_status.md`
2. Continue with any unblocked tasks in your assignment
3. Do NOT modify files outside your scope to work around the blocker

## Status Updates
When you complete a task or hit a blocker, write to `.orchestrator/status/agent_1_status.md` using this format:

```
## Task 1.1a: Directory Tree
- status: DONE | IN_PROGRESS | BLOCKED
- blocker: <description, or "none">
- notes: <decisions made, things the next wave should know>
```

## Definition of Done
This agent's work is complete when:
- [ ] Full `src/` directory tree exists with all `__init__.py` files
- [ ] All three legacy files are copied to new locations
- [ ] ROS2 node is in `legacy/`
- [ ] Duplicate `Logic/rplidar_reader.py` is deleted
- [ ] `DroneParams` and `SimParams` dataclasses are importable: `from src.config import DroneParams, SimParams`
- [ ] `requirements.txt` exists with correct deps
- [ ] `tests/conftest.py` has working fixtures
- [ ] All tasks are marked DONE in the status file
- [ ] No files outside the owned scope were modified

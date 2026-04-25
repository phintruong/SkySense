# Agent 1 Instructions

## Assignment
- **Assigned Tool:** Claude
- **Wave:** 2
- **Role:** Drone dynamics model, motor model, and environment simulation

## Objective
Implement the physics engine that simulates the quad-X drone: rigid body dynamics, first-order motor lag, aerodynamic drag, and an environment with wind/disturbance injection. This is the "plant" in control theory terms — everything downstream (control, EKF) depends on this being physically correct.

## Context

**Conventions (CRITICAL — get these wrong and everything breaks):**
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81] m/s². Altitude of 2m above ground means z = -2.0.
- **Quaternion**: [qw, qx, qy, qz], scalar-first, Hamilton convention
- **Body frame**: X=forward, Y=right, Z=down (standard aerospace body frame)
- **Thrust direction**: Motors push DOWN in body frame, which is -Z body. So total thrust in body = [0, 0, -sum(thrusts)]
- **Motor numbering (quad-X, viewed from above)**:
  ```
  4 (FL, CCW)   1 (FR, CW)
        \       /
         [BODY]
        /       \
  3 (BL, CW)    2 (BR, CCW)
  ```
  - CW motors (1, 3) produce negative yaw torque reaction (positive yaw torque in mixing)
  - CCW motors (2, 4) produce positive yaw torque reaction (negative yaw torque in mixing)

**Available imports from Wave 1:**
```python
from src.config import DroneParams, SimParams
from src.math_utils import (
    quat_multiply, quat_normalize, quat_conjugate,
    quat_rotate_vector, quat_identity,
    body_to_ned, ned_to_body, quat_to_euler,
)
```

**DroneParams fields:** mass=1.8, arm_length=0.18, Ixx=0.0123, Iyy=0.0123, Izz=0.0224, motor_tau=0.03, motor_max_thrust=12.0, drag_coefficient=0.3, gravity=9.81, thrust_coefficient, torque_coefficient, inertia_matrix (property), inertia_matrix_inv (property), hover_thrust_per_motor (property ~4.41N).

## Tasks

### Task 2.1a: Drone Dynamics Model
- **Description:** Create `src/simulation/drone_model.py`
- **Implementation details:**

```python
import numpy as np
from dataclasses import dataclass, field
from src.config import DroneParams
from src.math_utils import (
    quat_multiply, quat_normalize, quat_identity,
    body_to_ned, ned_to_body,
)

@dataclass
class DroneState:
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))      # NED [x, y, z]
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))      # NED [vx, vy, vz]
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # [qw, qx, qy, qz]
    angular_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))  # body frame [wx, wy, wz]

class DroneModel:
    def __init__(self, params: DroneParams):
        self.params = params
        self.state = DroneState()
    
    def reset(self, initial_state: DroneState = None):
        """Reset to given state or default."""
        self.state = initial_state if initial_state else DroneState()
    
    def compute_torques(self, motor_thrusts: np.ndarray) -> np.ndarray:
        """
        Compute body-frame torques [tau_roll, tau_pitch, tau_yaw] from 4 motor thrusts.
        
        Quad-X geometry: motors at 45 degrees from body axes.
        L = arm_length, T1..T4 = motor thrusts, cq = torque_coefficient/thrust_coefficient ratio
        
        tau_roll  = (L / sqrt(2)) * (-T1 - T2 + T3 + T4)
        tau_pitch = (L / sqrt(2)) * (-T1 + T2 + T3 - T4)  
        tau_yaw   = cq_ratio * (-T1 + T2 - T3 + T4)
        
        Where cq_ratio = torque_coefficient / thrust_coefficient (ratio of reactive torque to thrust)
        """
    
    def step(self, motor_thrusts: np.ndarray, dt: float, 
             external_force: np.ndarray = None) -> DroneState:
        """
        Advance physics by one timestep.
        
        Args:
            motor_thrusts: actual thrust per motor [T1, T2, T3, T4] in Newtons
            dt: timestep in seconds
            external_force: optional external force in NED frame [Fx, Fy, Fz] in Newtons
        
        Update order:
        1. Total thrust in body frame: F_body = [0, 0, -sum(motor_thrusts)]
        2. Rotate to NED: F_ned = body_to_ned(q, F_body)
        3. Gravity: F_ned += [0, 0, mass * g]
        4. External force: F_ned += external_force
        5. Linear drag: F_ned += -drag_coeff * velocity
        6. Linear acceleration: a = F_ned / mass
        7. Update velocity: v += a * dt
        8. Update position: p += v * dt
        9. Ground constraint: if p[2] > 0: p[2] = 0, v[2] = min(v[2], 0)
        10. Compute body torques from motor thrusts
        11. Angular acceleration: alpha = I_inv @ (torques - cross(omega, I @ omega))
        12. Update angular velocity: omega += alpha * dt
        13. Quaternion integration: q += 0.5 * quat_multiply([0, omega_x, omega_y, omega_z], q) * dt
        14. Normalize quaternion
        
        Returns updated DroneState.
        """
    
    def get_body_acceleration(self) -> np.ndarray:
        """Return last computed acceleration in body frame (for IMU simulation)."""
    
    def get_angular_velocity(self) -> np.ndarray:
        """Return angular velocity in body frame (for IMU simulation)."""
```

**Ground constraint (step 9):** In NED, ground is z=0. The drone is above ground when z < 0. If z > 0, clamp z=0 and kill downward velocity (v_z = min(v_z, 0)). This prevents the drone from falling through the ground.

**Euler's rotation equation (step 11):** `I @ alpha = torques - omega x (I @ omega)`. The `omega x (I @ omega)` term is the gyroscopic effect. For a symmetric quad (Ixx ≈ Iyy), this mainly affects yaw coupling.

**Quaternion integration (step 13):** The derivative of a quaternion is `q_dot = 0.5 * omega_quat * q` where `omega_quat = [0, wx, wy, wz]`. First-order Euler integration: `q_new = q + q_dot * dt`, then normalize. This is sufficient at 200 Hz.

- **Acceptance criteria:**
  - [ ] `DroneState` dataclass with position, velocity, quaternion, angular_velocity
  - [ ] `DroneModel.step()` implements full rigid body dynamics
  - [ ] `compute_torques()` correct for quad-X geometry
  - [ ] Ground constraint prevents falling through z=0
  - [ ] Hover test: setting all motors to hover_thrust_per_motor keeps the drone stationary (velocity stays near zero)

### Task 2.1b: Motor Model
- **Description:** Create `src/simulation/motor_model.py`

```python
import numpy as np
from src.config import DroneParams

class MotorModel:
    def __init__(self, params: DroneParams):
        self.params = params
        self.thrusts = np.zeros(4)  # Current actual thrust per motor
    
    def reset(self):
        self.thrusts = np.zeros(4)
    
    def update(self, commanded: np.ndarray, dt: float) -> np.ndarray:
        """
        Apply first-order lag to each motor.
        thrust += (commanded - thrust) * dt / tau
        Clamp to [min_thrust, max_thrust].
        Returns actual thrusts.
        """
```

- **Acceptance criteria:**
  - [ ] First-order lag with configurable tau
  - [ ] Clamping to valid thrust range
  - [ ] Step response converges to commanded value

### Task 2.1c: Environment
- **Description:** Create `src/simulation/environment.py`

```python
import numpy as np

class Environment:
    def __init__(self):
        self.constant_wind = np.zeros(3)     # NED constant wind force
        self.gust_force = np.zeros(3)        # one-shot disturbance
        self._gust_remaining = 0.0           # duration remaining for gust
    
    def set_wind(self, wind_force: np.ndarray):
        """Set constant wind force in NED frame (Newtons)."""
    
    def inject_disturbance(self, force: np.ndarray, duration: float = 0.1):
        """Inject a temporary force disturbance (e.g., a push)."""
    
    def get_external_force(self, position: np.ndarray, time: float, dt: float) -> np.ndarray:
        """
        Get total external force at current position/time.
        Returns: constant_wind + active_gust (decaying over duration)
        Decrements gust timer by dt.
        """
    
    def reset(self):
        """Reset all disturbances."""
```

- **Acceptance criteria:**
  - [ ] Constant wind applies persistent force
  - [ ] Disturbance injection applies temporary force that expires
  - [ ] get_external_force returns combined forces
  - [ ] Reset clears all forces

### Task 2.1d: Module Init
- **Description:** Update `src/simulation/__init__.py` to export classes.
- **Acceptance criteria:**
  - [ ] `from src.simulation import DroneModel, DroneState, MotorModel, Environment` works

## Scope

### Files you OWN (you may create, modify, delete):
- `src/simulation/drone_model.py`
- `src/simulation/motor_model.py`
- `src/simulation/environment.py`
- `src/simulation/__init__.py`

### Files you must AVOID (owned by other agents):
- `src/control/` (Agent 2)
- `src/sensors/imu_sim.py`, `src/sensors/ultrasonic_sim.py` (Agent 3)
- `tests/test_pid.py`, `tests/test_motor_mixer.py` (Agent 2)

### Files you may READ but not modify:
- `src/config/drone_params.py`, `src/config/sim_params.py`
- `src/math_utils/quaternion.py`, `src/math_utils/rotations.py`
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- Wave 1 is complete. Verify:
  - `from src.config import DroneParams` works
  - `from src.math_utils import body_to_ned, quat_multiply, quat_normalize` works

### What depends on YOUR output:
- **Wave 3, Task 3.2 (Main Loop)**: imports DroneModel, MotorModel, Environment
- **Wave 2, Task 2.3 (Sensor Sim)** will need DroneState to read true state for sensor simulation — but they're in the same wave and don't import your code directly (main loop passes data between them)

## When Blocked
If you encounter a blocker:
1. Document it in your status file at `.orchestrator/status/agent_1_status.md`
2. Continue with any unblocked tasks in your assignment
3. Do NOT modify files outside your scope to work around the blocker

## Status Updates
When you complete a task or hit a blocker, write to `.orchestrator/status/agent_1_status.md` using this format:

```
## Task 2.1a: Drone Dynamics Model
- status: DONE | IN_PROGRESS | BLOCKED
- blocker: <description, or "none">
- notes: <decisions made, things the next wave should know>
```

## Definition of Done
This agent's work is complete when:
- [ ] `DroneModel` with full rigid body dynamics, ground constraint, drag
- [ ] `MotorModel` with first-order lag on 4 motors
- [ ] `Environment` with wind + disturbance injection
- [ ] Hover sanity check: 4 motors at hover_thrust -> drone stays still
- [ ] All classes importable from `src.simulation`
- [ ] All tasks are marked DONE in the status file
- [ ] No files outside the owned scope were modified

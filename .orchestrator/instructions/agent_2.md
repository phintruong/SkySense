# Agent 2 Instructions

## Assignment
- **Assigned Tool:** Codex
- **Wave:** 2
- **Role:** Control system — PID controller, attitude controller, position controller, motor mixer

## Objective
Implement the full cascaded control stack: a generic PID controller class, an inner-loop attitude controller, an outer-loop position controller, and a quad-X motor mixer. These are developed independently from the simulation — they consume state estimates and produce motor commands.

## Context

**Conventions:**
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81].
- **Euler angles**: [roll, pitch, yaw] in radians. 3-2-1 (ZYX) order.
- **Cascaded control flow:**
  ```
  Position PID (outer) -> desired_roll, desired_pitch, thrust
  Attitude PID (inner) -> torque_roll, torque_pitch, torque_yaw
  Motor Mixer -> 4 motor thrust commands
  ```
- **Motor numbering (quad-X, viewed from above):**
  ```
  4 (FL, CCW)   1 (FR, CW)
        \       /
         [BODY]
        /       \
  3 (BL, CW)    2 (BR, CCW)
  ```

**Available imports:**
```python
from src.config import DroneParams, SimParams
# SimParams has: attitude_kp=8.0, attitude_ki=0.5, attitude_kd=3.0
#                altitude_kp=5.0, altitude_ki=1.0, altitude_kd=3.0
#                position_kp=1.0, position_ki=0.1, position_kd=0.8
#                max_tilt_angle=0.524 (~30 degrees)
# DroneParams has: motor_max_thrust=12.0, motor_min_thrust=0.0, mass=1.8, gravity=9.81
```

## Tasks

### Task 2.2a: Generic PID Controller
- **Description:** Create `src/control/pid.py`

```python
import numpy as np

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -float('inf'),
                 output_max: float = float('inf'),
                 integral_max: float = float('inf')):
        """
        Generic PID controller with anti-windup.
        
        Args:
            kp, ki, kd: proportional, integral, derivative gains
            output_min, output_max: clamp final output
            integral_max: clamp integral term magnitude (anti-windup)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self._integral = 0.0
        self._prev_error = None
    
    def update(self, error: float, dt: float) -> float:
        """
        Compute PID output.
        
        - Proportional: kp * error
        - Integral: ki * cumulative_error (clamped by integral_max)
        - Derivative: kd * (error - prev_error) / dt
          - First call: derivative = 0 (no previous error)
        - Output clamped to [output_min, output_max]
        """
    
    def reset(self):
        """Zero integral and previous error."""
        self._integral = 0.0
        self._prev_error = None
```

- **Acceptance criteria:**
  - [ ] Proportional, integral, derivative terms computed correctly
  - [ ] Anti-windup: integral clamped to [-integral_max, integral_max]
  - [ ] Output clamped to [output_min, output_max]
  - [ ] First call handles missing prev_error gracefully (derivative = 0)
  - [ ] reset() zeros state

### Task 2.2b: Attitude Controller
- **Description:** Create `src/control/attitude_controller.py`

```python
import numpy as np
from .pid import PIDController

class AttitudeController:
    def __init__(self, kp: float = 8.0, ki: float = 0.5, kd: float = 3.0):
        """
        Inner-loop controller. Takes angle errors, outputs body torques.
        3 independent PIDs for roll, pitch, yaw.
        """
        self.roll_pid = PIDController(kp, ki, kd, output_min=-5.0, output_max=5.0, integral_max=1.0)
        self.pitch_pid = PIDController(kp, ki, kd, output_min=-5.0, output_max=5.0, integral_max=1.0)
        self.yaw_pid = PIDController(kp * 0.5, ki * 0.5, kd * 0.5, output_min=-2.0, output_max=2.0, integral_max=0.5)
        # Yaw gains are typically lower than roll/pitch
    
    def compute(self, current_euler: np.ndarray, desired_euler: np.ndarray, 
                dt: float) -> np.ndarray:
        """
        Compute body-frame torques.
        
        Args:
            current_euler: [roll, pitch, yaw] in radians
            desired_euler: [roll, pitch, yaw] in radians
            dt: timestep
        
        Returns:
            torques: [tau_roll, tau_pitch, tau_yaw] in N*m
        
        IMPORTANT: Yaw error must handle angle wrapping.
        error = desired - current, but for yaw:
        error = atan2(sin(desired - current), cos(desired - current))
        This gives shortest path error in [-pi, pi].
        """
    
    def reset(self):
        """Reset all PIDs."""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
```

- **Acceptance criteria:**
  - [ ] 3 PID controllers for roll, pitch, yaw
  - [ ] Yaw angle wrapping (atan2 method)
  - [ ] Yaw gains scaled down relative to roll/pitch
  - [ ] Output limits prevent torque saturation

### Task 2.2c: Position Controller
- **Description:** Create `src/control/position_controller.py`

```python
import numpy as np
from .pid import PIDController

class PositionController:
    def __init__(self, pos_kp=1.0, pos_ki=0.1, pos_kd=0.8,
                 alt_kp=5.0, alt_ki=1.0, alt_kd=3.0,
                 max_tilt: float = 0.524,
                 hover_thrust: float = 4.41 * 4):
        """
        Outer-loop controller. Takes position error, outputs desired attitude + thrust.
        
        Args:
            pos_kp, pos_ki, pos_kd: X/Y position gains
            alt_kp, alt_ki, alt_kd: altitude (Z) gains
            max_tilt: maximum roll/pitch angle in radians (~30 deg)
            hover_thrust: total thrust needed to hover (mass * g)
        """
        self.x_pid = PIDController(pos_kp, pos_ki, pos_kd, output_min=-max_tilt, output_max=max_tilt)
        self.y_pid = PIDController(pos_kp, pos_ki, pos_kd, output_min=-max_tilt, output_max=max_tilt)
        self.z_pid = PIDController(alt_kp, alt_ki, alt_kd, output_min=-10.0, output_max=10.0, integral_max=5.0)
        self.hover_thrust = hover_thrust
        self.max_tilt = max_tilt
    
    def compute(self, current_pos: np.ndarray, desired_pos: np.ndarray,
                current_vel: np.ndarray, current_yaw: float,
                dt: float) -> tuple:
        """
        Compute desired attitude and total thrust.
        
        Args:
            current_pos: [x, y, z] in NED
            desired_pos: [x, y, z] in NED (z negative = above ground)
            current_vel: [vx, vy, vz] in NED (for derivative term via error)
            current_yaw: current yaw in radians (to rotate commands to body frame)
            dt: timestep
        
        Returns:
            (desired_roll, desired_pitch, total_thrust)
        
        Algorithm:
        1. Z (altitude) error -> thrust adjustment
           - thrust = hover_thrust + z_pid.update(z_error, dt)
           - In NED: z_error = desired_z - current_z
           - Negative z_error (drone too high) -> reduce thrust
           - Clamp total thrust to [0, 4 * motor_max_thrust]
        
        2. X/Y position error -> desired tilt angles
           - error_x = desired_x - current_x (North error)
           - error_y = desired_y - current_y (East error)
           - These are in NED frame, need to rotate to body frame using yaw:
             error_body_x = cos(yaw) * error_x + sin(yaw) * error_y
             error_body_y = -sin(yaw) * error_x + cos(yaw) * error_y
           - desired_pitch = -x_pid.update(error_body_x, dt)  (pitch forward = negative in NED)
           - desired_roll = y_pid.update(error_body_y, dt)     (roll right = positive for east movement)
           - Clamp both to [-max_tilt, max_tilt]
        
        3. desired_yaw: just pass through current yaw (no yaw position control in v1,
           yaw setpoint can be added later)
        """
    
    def reset(self):
        self.x_pid.reset()
        self.y_pid.reset()
        self.z_pid.reset()
```

- **Acceptance criteria:**
  - [ ] Altitude PID: error -> thrust offset around hover_thrust
  - [ ] Position PID: NED error -> body-frame desired angles (rotated by yaw)
  - [ ] Tilt angle clamped to max_tilt
  - [ ] Thrust clamped to valid range
  - [ ] At hover (all errors zero): outputs hover_thrust and zero angles

### Task 2.2d: Motor Mixer
- **Description:** Create `src/control/motor_mixer.py`

```python
import numpy as np
from src.config import DroneParams

class MotorMixer:
    def __init__(self, params: DroneParams):
        """
        Quad-X motor mixer.
        Converts total thrust + body torques into 4 individual motor thrusts.
        
        Motor layout (viewed from above, NED body frame):
            4 (FL, CCW)   1 (FR, CW)
                  \       /
                   [BODY]  -> X (forward/North)
                  /       \
            3 (BL, CW)    2 (BR, CCW)
                     |
                     v Y (right/East)
        
        Mixing matrix (thrust, roll_torque, pitch_torque, yaw_torque -> motor thrusts):
        
        For quad-X with arms at 45 degrees:
        Each motor contributes to roll/pitch through its (x,y) position.
        
        M1 (FR, +x, +y, CW):  thrust/4 - roll/(4*d) - pitch/(4*d) - yaw/(4*cq)
        M2 (BR, -x, +y, CCW): thrust/4 - roll/(4*d) + pitch/(4*d) + yaw/(4*cq)
        M3 (BL, -x, -y, CW):  thrust/4 + roll/(4*d) + pitch/(4*d) - yaw/(4*cq)
        M4 (FL, +x, -y, CCW): thrust/4 + roll/(4*d) - pitch/(4*d) + yaw/(4*cq)
        
        Where d = arm_length / sqrt(2) (lever arm for 45-degree mounted motors)
        And cq = torque_coefficient / thrust_coefficient (yaw torque per unit thrust)
        """
        self.params = params
        self.d = params.arm_length / np.sqrt(2)
        self.cq = params.torque_coefficient / params.thrust_coefficient
    
    def mix(self, thrust: float, torque_roll: float, 
            torque_pitch: float, torque_yaw: float) -> np.ndarray:
        """
        Convert total thrust + torques to 4 motor thrusts.
        
        Args:
            thrust: total thrust in Newtons (sum of all motors)
            torque_roll: desired roll torque (N*m)
            torque_pitch: desired pitch torque (N*m)
            torque_yaw: desired yaw torque (N*m)
        
        Returns:
            np.array([T1, T2, T3, T4]) in Newtons
        
        After computing raw values:
        1. Clamp each to [min_thrust, max_thrust]
        2. If any motor saturates, scale all motors proportionally 
           to preserve torque ratios (priority: attitude > position)
        """
    
    def reset(self):
        """Nothing to reset, but included for interface consistency."""
        pass
```

- **Acceptance criteria:**
  - [ ] Pure thrust (zero torques) -> all 4 motors equal (thrust/4)
  - [ ] Positive roll torque -> motors 3,4 increase, motors 1,2 decrease
  - [ ] Positive pitch torque -> motors 2,3 increase, motors 1,4 decrease  
  - [ ] Positive yaw torque -> CCW motors (2,4) increase, CW motors (1,3) decrease
  - [ ] Clamping works
  - [ ] Zero thrust + zero torques -> all zeros

### Task 2.2e: Module Init + Tests
- **Description:** 
  1. Update `src/control/__init__.py` to export all classes
  2. Create `tests/test_pid.py`
  3. Create `tests/test_motor_mixer.py`

**tests/test_pid.py:**
```python
import pytest
from src.control import PIDController

def test_proportional_only():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(2.0)

def test_integral_accumulates():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    pid.update(error=1.0, dt=0.01)  # integral = 0.01
    output = pid.update(error=1.0, dt=0.01)  # integral = 0.02
    assert output == pytest.approx(0.02)

def test_integral_windup():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, integral_max=0.1)
    for _ in range(1000):
        pid.update(error=1.0, dt=0.01)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(0.1)  # clamped

def test_output_clamping():
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(1.0)  # clamped

def test_zero_error():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(error=0.0, dt=0.01)
    assert output == pytest.approx(0.0)

def test_reset():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    pid.update(error=1.0, dt=0.01)
    pid.reset()
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(0.01)  # fresh start

def test_derivative_first_call_zero():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
    output = pid.update(error=5.0, dt=0.01)
    assert output == pytest.approx(0.0)  # no prev error
```

**tests/test_motor_mixer.py:**
```python
import numpy as np
import pytest
from src.config import DroneParams
from src.control import MotorMixer

@pytest.fixture
def mixer():
    return MotorMixer(DroneParams())

def test_pure_thrust(mixer):
    """Equal thrust, no torques -> all motors equal."""
    result = mixer.mix(thrust=17.64, torque_roll=0, torque_pitch=0, torque_yaw=0)
    np.testing.assert_allclose(result, [4.41, 4.41, 4.41, 4.41], atol=0.01)

def test_zero_input(mixer):
    """Zero everything -> zero motors."""
    result = mixer.mix(thrust=0, torque_roll=0, torque_pitch=0, torque_yaw=0)
    np.testing.assert_allclose(result, [0, 0, 0, 0], atol=1e-10)

def test_positive_roll(mixer):
    """Positive roll torque: motors 3,4 up, motors 1,2 down."""
    result = mixer.mix(thrust=17.64, torque_roll=1.0, torque_pitch=0, torque_yaw=0)
    assert result[2] > result[0]  # M3 > M1
    assert result[3] > result[1]  # M4 > M2

def test_positive_pitch(mixer):
    """Positive pitch torque: motors 2,3 up, motors 1,4 down."""
    result = mixer.mix(thrust=17.64, torque_roll=0, torque_pitch=1.0, torque_yaw=0)
    assert result[1] > result[0]  # M2 > M1
    assert result[2] > result[3]  # M3 > M4

def test_positive_yaw(mixer):
    """Positive yaw: CCW motors (2,4) up, CW motors (1,3) down."""
    result = mixer.mix(thrust=17.64, torque_roll=0, torque_pitch=0, torque_yaw=0.1)
    assert result[1] > result[0]  # M2 (CCW) > M1 (CW)
    assert result[3] > result[2]  # M4 (CCW) > M3 (CW)

def test_clamping(mixer):
    """Motors clamp to [0, max_thrust]."""
    result = mixer.mix(thrust=100, torque_roll=0, torque_pitch=0, torque_yaw=0)
    assert np.all(result <= DroneParams().motor_max_thrust + 0.01)
    assert np.all(result >= 0)
```

- **Acceptance criteria:**
  - [ ] `from src.control import PIDController, AttitudeController, PositionController, MotorMixer` works
  - [ ] `pytest tests/test_pid.py -v` passes (7+ tests)
  - [ ] `pytest tests/test_motor_mixer.py -v` passes (6+ tests)

## Scope

### Files you OWN (you may create, modify, delete):
- `src/control/pid.py`
- `src/control/attitude_controller.py`
- `src/control/position_controller.py`
- `src/control/motor_mixer.py`
- `src/control/__init__.py`
- `tests/test_pid.py`
- `tests/test_motor_mixer.py`

### Files you must AVOID (owned by other agents):
- `src/simulation/` (Agent 1)
- `src/sensors/` (Agent 3)
- `src/navigation/` (nobody this wave)
- `src/math_utils/` (read only)
- `tests/test_quaternion.py`, `tests/test_rotations.py`

### Files you may READ but not modify:
- `src/config/drone_params.py`, `src/config/sim_params.py`
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- Wave 1 is complete. Verify: `from src.config import DroneParams, SimParams` works.
- numpy must be available.

### What depends on YOUR output:
- **Wave 3, Task 3.2 (Main Loop)**: imports AttitudeController, PositionController, MotorMixer
- Getting the motor mixer signs wrong will make the drone flip. Test thoroughly.

## When Blocked
If you encounter a blocker:
1. Document it in your status file at `.orchestrator/status/agent_2_status.md`
2. Continue with any unblocked tasks in your assignment
3. Do NOT modify files outside your scope to work around the blocker

## Status Updates
When you complete a task or hit a blocker, write to `.orchestrator/status/agent_2_status.md` using this format:

```
## Task 2.2a: PID Controller
- status: DONE | IN_PROGRESS | BLOCKED
- blocker: <description, or "none">
- notes: <decisions made, things the next wave should know>
```

## Definition of Done
This agent's work is complete when:
- [ ] `PIDController` with anti-windup and output clamping
- [ ] `AttitudeController` with yaw wrapping
- [ ] `PositionController` with yaw-rotated body frame errors
- [ ] `MotorMixer` with correct quad-X signs and clamping
- [ ] `pytest tests/test_pid.py -v` passes (7+ tests)
- [ ] `pytest tests/test_motor_mixer.py -v` passes (6+ tests)
- [ ] All classes importable from `src.control`
- [ ] All tasks are marked DONE in the status file
- [ ] No files outside the owned scope were modified

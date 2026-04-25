# Agent 3 Instructions

## Assignment
- **Assigned Tool:** Codex
- **Wave:** 2
- **Role:** Sensor simulation — IMU and ultrasonic with noise models and failure injection

## Objective
Implement simulated versions of the MPU-6050 IMU and HC-SR04 ultrasonic sensor. These take true drone state and produce noisy measurements that the EKF will consume. Each sensor supports failure injection (off, noisy, recover) for the sensor failure recovery demos.

## Context

**Conventions:**
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81].
- **Body frame**: X=forward, Y=right, Z=down.
- **IMU measures SPECIFIC FORCE, not acceleration.** Specific force = acceleration - gravity (in inertial frame). In hover: true acceleration is zero, but the accelerometer reads [0, 0, -9.81] in body frame because it measures the force that opposes gravity. Think of it as: if you hold an accelerometer still, it reads -g because the table pushes up against gravity.
  - In NED: gravity_ned = [0, 0, 9.81]
  - Specific force in NED = acceleration_ned - gravity_ned
  - In hover (a_ned = [0,0,0]): sf_ned = [0, 0, -9.81]
  - Rotate to body frame: sf_body = ned_to_body(q, sf_ned)
  - For level hover: sf_body = [0, 0, -9.81] (same as NED since no rotation)
- **Gyroscope** measures angular velocity in body frame, directly.
- **Ultrasonic** measures altitude (distance to ground) in meters. In NED, altitude = -z (drone at z=-2 is 2m above ground).

**Available imports:**
```python
import numpy as np
from src.config import SimParams
from src.math_utils import ned_to_body
```

**SimParams fields:**
- `imu_accel_noise_std`: 0.05 m/s^2
- `imu_gyro_noise_std`: 0.01 rad/s
- `imu_gyro_bias_std`: 0.001 rad/s per sqrt(s)
- `ultrasonic_noise_std`: 0.02 m

## Tasks

### Task 2.3a: IMU Simulator
- **Description:** Create `src/sensors/imu_sim.py`

```python
import numpy as np
from src.config import SimParams
from src.math_utils import ned_to_body

class IMUSim:
    def __init__(self, params: SimParams):
        """
        Simulated MPU-6050 IMU.
        
        Outputs:
        - accelerometer: specific force in body frame + noise
        - gyroscope: angular velocity in body frame + bias + noise
        
        Gyro bias random walk: bias evolves slowly over time, simulating
        real sensor drift. bias += N(0, bias_std) * sqrt(dt) each step.
        """
        self.params = params
        self.gyro_bias = np.zeros(3)      # current gyro bias (rad/s)
        self._healthy = True
        self._failure_mode = "normal"      # "normal", "off", "noisy"
    
    def read(self, true_acceleration_ned: np.ndarray, 
             true_angular_velocity_body: np.ndarray,
             quaternion: np.ndarray,
             dt: float) -> dict:
        """
        Generate noisy IMU reading.
        
        Args:
            true_acceleration_ned: true linear acceleration in NED frame (m/s^2)
            true_angular_velocity_body: true angular velocity in body frame (rad/s)
            quaternion: current orientation [qw, qx, qy, qz] (for rotating gravity to body)
            dt: timestep for bias random walk
        
        Returns:
            {
                "accel": np.array[3],  # specific force in body frame (m/s^2)
                "gyro": np.array[3],   # angular velocity in body frame (rad/s)
                "healthy": bool
            }
        
        Algorithm:
        1. Compute specific force:
           gravity_ned = [0, 0, 9.81]
           specific_force_ned = true_acceleration_ned - gravity_ned
           specific_force_body = ned_to_body(quaternion, specific_force_ned)
        
        2. Add accelerometer noise:
           accel = specific_force_body + N(0, accel_noise_std) per axis
        
        3. Update gyro bias random walk:
           gyro_bias += N(0, gyro_bias_std * sqrt(dt)) per axis
        
        4. Add gyro noise + bias:
           gyro = true_angular_velocity_body + gyro_bias + N(0, gyro_noise_std) per axis
        
        5. Apply failure mode:
           - "off": return zeros for both, healthy=False
           - "noisy": multiply noise std by 10
           - "normal": standard noise
        """
    
    def inject_failure(self, mode: str):
        """
        Set failure mode.
        Args:
            mode: "off" (sensor dead), "noisy" (10x noise), "recover" (back to normal)
        """
        if mode == "recover":
            self._failure_mode = "normal"
            self._healthy = True
        else:
            self._failure_mode = mode
            self._healthy = (mode == "normal")
    
    def is_healthy(self) -> bool:
        return self._healthy
    
    def reset(self):
        """Reset bias and failure state."""
        self.gyro_bias = np.zeros(3)
        self._healthy = True
        self._failure_mode = "normal"
```

- **Acceptance criteria:**
  - [ ] In hover (zero acceleration, identity quaternion): accel reads ~[0, 0, -9.81] + noise
  - [ ] Gyro reads true angular velocity + bias + noise
  - [ ] Gyro bias drifts over time (random walk)
  - [ ] "off" mode returns zeros and healthy=False
  - [ ] "noisy" mode has visibly larger noise (10x std)
  - [ ] "recover" restores normal operation

### Task 2.3b: Ultrasonic Simulator
- **Description:** Create `src/sensors/ultrasonic_sim.py`

```python
import numpy as np
from src.config import SimParams

class UltrasonicSim:
    # HC-SR04 specifications
    MIN_RANGE = 0.02    # 2 cm minimum
    MAX_RANGE = 4.0     # 4 m maximum
    
    def __init__(self, params: SimParams):
        """
        Simulated HC-SR04 downward-facing ultrasonic sensor.
        Measures altitude (distance to ground).
        """
        self.params = params
        self._healthy = True
        self._failure_mode = "normal"
    
    def read(self, true_position_ned: np.ndarray) -> dict:
        """
        Generate noisy altitude reading.
        
        Args:
            true_position_ned: [x, y, z] in NED. z < 0 means above ground.
        
        Returns:
            {
                "altitude": float,  # meters above ground (positive up)
                "valid": bool       # False if out of range or sensor failed
            }
        
        Algorithm:
        1. true_altitude = -true_position_ned[2]  (NED z is down, altitude is up)
        2. Check range: if altitude < MIN_RANGE or > MAX_RANGE: valid=False
        3. Add noise: altitude += N(0, ultrasonic_noise_std)
        4. Apply failure mode:
           - "off": return altitude=0, valid=False
           - "normal": return noisy altitude, valid=True (if in range)
        """
    
    def inject_failure(self, mode: str):
        """
        Set failure mode.
        Args:
            mode: "off" (sensor dead), "recover" (back to normal)
        """
        if mode == "recover":
            self._failure_mode = "normal"
            self._healthy = True
        else:
            self._failure_mode = mode
            self._healthy = (mode != "off")
    
    def is_healthy(self) -> bool:
        return self._healthy
    
    def reset(self):
        self._healthy = True
        self._failure_mode = "normal"
```

- **Acceptance criteria:**
  - [ ] At z=-2.0 (2m altitude): reads ~2.0 + noise, valid=True
  - [ ] At z=0.0 (on ground): reads ~0.0, may be invalid (below MIN_RANGE)
  - [ ] At z=-5.0 (above MAX_RANGE): valid=False
  - [ ] "off" mode: valid=False
  - [ ] "recover": back to normal

### Task 2.3c: Module Init
- **Description:** Update `src/sensors/__init__.py` to export the sim classes.

```python
from .imu_sim import IMUSim
from .ultrasonic_sim import UltrasonicSim
```

Keep any existing exports for the real hardware classes (lidar.py, ultrasonic.py) if they exist, but wrap in try/except since hardware deps (pyrplidar, gpiozero) won't be installed in dev.

- **Acceptance criteria:**
  - [ ] `from src.sensors import IMUSim, UltrasonicSim` works
  - [ ] Import doesn't crash even without pyrplidar/gpiozero installed

## Scope

### Files you OWN (you may create, modify, delete):
- `src/sensors/imu_sim.py`
- `src/sensors/ultrasonic_sim.py`
- `src/sensors/__init__.py`

### Files you must AVOID (owned by other agents):
- `src/sensors/lidar.py` (moved hardware file, don't modify)
- `src/sensors/ultrasonic.py` (moved hardware file, don't modify)
- `src/simulation/` (Agent 1)
- `src/control/` (Agent 2)
- `src/navigation/`
- `src/math_utils/` (read only)
- `src/config/` (read only)

### Files you may READ but not modify:
- `src/config/sim_params.py`
- `src/math_utils/rotations.py` (for `ned_to_body`)
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- Wave 1 is complete. Verify:
  - `from src.config import SimParams` works
  - `from src.math_utils import ned_to_body` works

### What depends on YOUR output:
- **Wave 3, Task 3.2 (Main Loop)**: imports IMUSim, UltrasonicSim to generate sensor data
- **Wave 3, Task 3.1 (EKF)**: designed to consume the dict format you return
- The `"accel"` and `"gyro"` keys in your IMU return dict, and `"altitude"` and `"valid"` in ultrasonic, become the EKF's measurement inputs. Don't change these key names.

## When Blocked
If you encounter a blocker:
1. Document it in your status file at `.orchestrator/status/agent_3_status.md`
2. Continue with any unblocked tasks in your assignment
3. Do NOT modify files outside your scope to work around the blocker

## Status Updates
When you complete a task or hit a blocker, write to `.orchestrator/status/agent_3_status.md` using this format:

```
## Task 2.3a: IMU Simulator
- status: DONE | IN_PROGRESS | BLOCKED
- blocker: <description, or "none">
- notes: <decisions made, things the next wave should know>
```

## Definition of Done
This agent's work is complete when:
- [ ] `IMUSim` produces correct specific force in hover (~[0, 0, -9.81] in body frame)
- [ ] `IMUSim` gyro bias drifts over time via random walk
- [ ] `IMUSim` failure injection works (off, noisy, recover)
- [ ] `UltrasonicSim` converts NED z-position to altitude correctly
- [ ] `UltrasonicSim` respects range limits (0.02m - 4.0m)
- [ ] `UltrasonicSim` failure injection works (off, recover)
- [ ] `from src.sensors import IMUSim, UltrasonicSim` works
- [ ] All tasks are marked DONE in the status file
- [ ] No files outside the owned scope were modified

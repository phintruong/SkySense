# Agent 1 Instructions — Wave 3

## Assignment
- **Assigned Tool:** Claude
- **Wave:** 3
- **Role:** 13-state Extended Kalman Filter + State Estimator

## Objective
Implement the EKF — the most mathematically complex module in the stack. It fuses IMU and ultrasonic measurements to estimate the drone's full state (position, velocity, orientation, gyro bias). This is the brain of the navigation system.

## Context

**Conventions (CRITICAL):**
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81].
- **Quaternion**: [qw, qx, qy, qz], scalar-first, Hamilton convention
- **Body frame**: X=forward, Y=right, Z=down
- **IMU measures SPECIFIC FORCE**: accel_measured = specific_force_body = R_ned_to_body @ (a_ned - g_ned). In hover: reads [0, 0, -9.81] in body.

**Available imports:**
```python
from src.config import DroneParams, SimParams
from src.math_utils import (
    quat_multiply, quat_normalize, quat_conjugate, quat_inverse,
    quat_rotate_vector, quat_identity, quat_from_axis_angle,
    quat_to_euler, euler_to_quat, quat_to_rotation_matrix,
    body_to_ned, ned_to_body,
)
```

**SimParams EKF fields:**
- `ekf_process_noise_pos`: 0.01
- `ekf_process_noise_vel`: 0.1
- `ekf_process_noise_quat`: 0.001
- `ekf_process_noise_bias`: 0.0001
- `ekf_accel_measurement_noise`: 0.5
- `ekf_altitude_measurement_noise`: 0.05
- `ekf_innovation_threshold`: 5.0

**Sensor data formats (from Wave 2 sensors):**
```python
# IMUSim.read() returns:
{"accel": np.array[3], "gyro": np.array[3], "healthy": bool}

# UltrasonicSim.read() returns:
{"altitude": float, "valid": bool}
```

## Tasks

### Task 3.1a: Extended Kalman Filter
- **Description:** Create `src/navigation/ekf.py`

```python
import numpy as np
from src.config import SimParams
from src.math_utils import (
    quat_multiply, quat_normalize, quat_to_rotation_matrix,
    body_to_ned, ned_to_body,
)

class EKF:
    """
    13-state Extended Kalman Filter for drone state estimation.
    
    State vector x (13,):
        [0:3]   position (NED)
        [3:6]   velocity (NED)
        [6:10]  quaternion [qw, qx, qy, qz]
        [10:13] gyro bias (body frame, rad/s)
    
    Covariance P (13x13)
    """
    
    def __init__(self, params: SimParams):
        self.params = params
        self.x = np.zeros(13)
        self.x[6] = 1.0  # identity quaternion qw=1
        self.P = np.eye(13) * 0.1
        # Larger initial uncertainty for bias
        self.P[10:13, 10:13] = np.eye(3) * 0.01
        self._last_innovation = np.zeros(3)
    
    def predict(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """
        EKF prediction step using IMU data.
        
        Args:
            gyro: measured angular velocity in body frame (rad/s) — includes bias
            accel: measured specific force in body frame (m/s^2)
            dt: timestep
        
        Algorithm:
        1. Subtract estimated gyro bias: omega = gyro - x[10:13]
        2. Quaternion integration:
           omega_quat = [0, omega_x, omega_y, omega_z]
           q_dot = 0.5 * quat_multiply(omega_quat, q)
           q_new = q + q_dot * dt
           q_new = normalize(q_new)
        3. Rotate accel to NED and add gravity:
           a_ned = body_to_ned(q, accel) + [0, 0, g]
           (accel is specific force, so true_accel = specific_force_rotated + gravity)
        4. Velocity integration: v_new = v + a_ned * dt
        5. Position integration: p_new = p + v * dt
        6. Gyro bias: unchanged (random walk in Q)
        7. Compute Jacobian F (13x13) — linearized state transition
        8. Process noise Q (13x13) — diagonal blocks
        9. Covariance update: P = F @ P @ F.T + Q
        """
    
    def update_accel(self, accel_measurement: np.ndarray):
        """
        Update using accelerometer measurement (body-frame specific force).
        
        Measurement model: In the absence of linear acceleration,
        the accelerometer measures gravity rotated to body frame:
            h(x) = ned_to_body(q, [0, 0, -g])
        
        This gives roll/pitch observability (not yaw — gravity is symmetric about Z).
        
        Steps:
        1. Predicted measurement: h = ned_to_body(q, [0, 0, -9.81])
        2. Innovation: y = accel_measurement - h
        3. Innovation gating: if ||y|| > threshold * sqrt(trace(S)), reduce gain or skip
        4. Jacobian H (3x13): derivatives of h w.r.t. state
           - Only quaternion components (columns 6:10) are nonzero
           - H_q = d(R(q).T @ g_ned) / dq — compute numerically or analytically
        5. Innovation covariance: S = H @ P @ H.T + R_accel
        6. Kalman gain: K = P @ H.T @ inv(S)
        7. State update: x += K @ y
        8. Normalize quaternion
        9. Covariance: P = (I - K @ H) @ P
        
        Use numerical Jacobian if analytical is too complex:
            H[:, i] = (h(x + eps_i) - h(x - eps_i)) / (2 * eps)
        """
    
    def update_altitude(self, altitude: float):
        """
        Update using ultrasonic altitude measurement.
        
        Measurement model: altitude = -z (NED: z is down, altitude is up)
            h(x) = -x[2]
            H = [0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  (1x13)
        
        Standard linear Kalman update:
        1. Innovation: y = altitude - (-x[2])
        2. Innovation gating (same threshold logic)
        3. S = H @ P @ H.T + R_alt (scalar)
        4. K = P @ H.T / S (13x1)
        5. x += K * y
        6. Normalize quaternion
        7. P = (I - K @ H) @ P
        """
    
    def get_state(self) -> dict:
        """
        Returns current state estimate as a dict:
        {
            "position": np.array[3],      # NED
            "velocity": np.array[3],      # NED
            "quaternion": np.array[4],    # [qw, qx, qy, qz]
            "euler": np.array[3],         # [roll, pitch, yaw] radians
            "gyro_bias": np.array[3],     # body frame rad/s
        }
        """
    
    def get_covariance_trace(self) -> float:
        """trace(P) — overall uncertainty level."""
        return np.trace(self.P)
    
    def get_innovation_norm(self) -> float:
        """||last innovation|| — for frontend health display."""
        return np.linalg.norm(self._last_innovation)
    
    def reset(self):
        """Reset state and covariance to initial values."""
```

**Jacobian F (predict step) key structure:**
```
F = I(13x13) + dt * [
    d(pos_dot)/d(state)    -> row 0:3: velocity affects position
    d(vel_dot)/d(state)    -> row 3:6: quaternion affects accel rotation, vel for drag
    d(quat_dot)/d(state)   -> row 6:10: quaternion, gyro bias affect quat integration
    d(bias_dot)/d(state)   -> row 10:13: all zeros (bias is random walk)
]
```

The Jacobian is the hardest part. Numerical Jacobian is acceptable for v1 — it's slower but correct. If you implement analytical, verify against numerical.

- **Acceptance criteria:**
  - [ ] State vector initialized correctly (identity quaternion)
  - [ ] predict() integrates quaternion, velocity, position correctly
  - [ ] Quaternion stays normalized through predict+update cycles
  - [ ] update_altitude() pulls z-estimate toward measurement
  - [ ] update_accel() provides roll/pitch observability
  - [ ] Innovation gating rejects outlier measurements
  - [ ] get_state() returns correct dict format
  - [ ] Covariance grows during predict, shrinks during update

### Task 3.1b: State Estimator Wrapper
- **Description:** Create `src/navigation/state_estimator.py`

```python
import numpy as np
from .ekf import EKF
from src.config import SimParams

class StateEstimator:
    """
    Wraps EKF with sensor scheduling and health management.
    """
    
    def __init__(self, params: SimParams):
        self.ekf = EKF(params)
        self._imu_healthy = True
        self._ultrasonic_healthy = True
        self._emergency = False
    
    def predict(self, imu_data: dict, dt: float):
        """
        Called every simulation step (200 Hz).
        
        Args:
            imu_data: {"accel": np.array[3], "gyro": np.array[3], "healthy": bool}
            dt: timestep
        
        If IMU is unhealthy:
        - Use last known gyro bias as gyro estimate
        - Use [0, 0, -9.81] as default accel (hover assumption)
        - Set emergency flag
        """
    
    def update_accel(self, imu_data: dict):
        """
        Accelerometer measurement update.
        Called at IMU rate (every step or sub-sampled).
        Skip if IMU unhealthy.
        """
    
    def update_altitude(self, ultrasonic_data: dict):
        """
        Ultrasonic altitude update.
        Called at ultrasonic rate (20 Hz).
        Skip if valid=False.
        """
    
    def get_state(self) -> dict:
        """Delegate to EKF."""
        return self.ekf.get_state()
    
    def is_emergency(self) -> bool:
        return self._emergency
    
    def get_health(self) -> dict:
        return {
            "imu_healthy": self._imu_healthy,
            "ultrasonic_healthy": self._ultrasonic_healthy,
            "emergency": self._emergency,
            "covariance_trace": self.ekf.get_covariance_trace(),
            "innovation_norm": self.ekf.get_innovation_norm(),
        }
    
    def reset(self):
        self.ekf.reset()
        self._imu_healthy = True
        self._ultrasonic_healthy = True
        self._emergency = False
```

- **Acceptance criteria:**
  - [ ] Wraps EKF correctly
  - [ ] Handles IMU unhealthy → fallback values + emergency flag
  - [ ] Skips altitude update when ultrasonic invalid
  - [ ] get_health() returns all status fields

### Task 3.1c: Module Init + Tests
- **Description:**
  1. Update `src/navigation/__init__.py` to export EKF, StateEstimator
  2. Create `tests/test_ekf.py`

**tests/test_ekf.py:**
```python
import numpy as np
import pytest
from src.navigation import EKF
from src.config import SimParams

@pytest.fixture
def ekf():
    return EKF(SimParams())

def test_initial_state(ekf):
    """State starts at origin with identity quaternion."""
    state = ekf.get_state()
    np.testing.assert_allclose(state["position"], [0, 0, 0])
    np.testing.assert_allclose(state["quaternion"], [1, 0, 0, 0])

def test_predict_gravity_only(ekf):
    """With hover accel reading and zero gyro, velocity should grow in +Z (NED gravity)."""
    # Hover specific force in body frame
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.zeros(3)
    for _ in range(200):  # 1 second
        ekf.predict(gyro, accel, 0.005)
    state = ekf.get_state()
    # Position and velocity should stay ~zero (accel + gravity cancel)
    assert abs(state["velocity"][2]) < 0.1

def test_altitude_update_pulls_state(ekf):
    """Altitude update should pull z-estimate toward measurement."""
    # Start at z=0, feed measurement saying altitude=2 (z=-2)
    ekf.update_altitude(2.0)
    state = ekf.get_state()
    assert state["position"][2] < 0  # should have moved toward z=-2

def test_quaternion_stays_normalized(ekf):
    """Quaternion norm should stay ~1 through many predict+update cycles."""
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.array([0.1, 0.05, 0.02])
    for _ in range(1000):
        ekf.predict(gyro, accel, 0.005)
        if _ % 10 == 0:
            ekf.update_altitude(2.0)
            ekf.update_accel(accel)
    q = ekf.x[6:10]
    assert abs(np.linalg.norm(q) - 1.0) < 1e-6

def test_covariance_grows_without_updates(ekf):
    """Covariance should grow during prediction without measurement updates."""
    trace_before = ekf.get_covariance_trace()
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.zeros(3)
    for _ in range(100):
        ekf.predict(gyro, accel, 0.005)
    trace_after = ekf.get_covariance_trace()
    assert trace_after > trace_before

def test_innovation_gating(ekf):
    """Large outlier measurement should be rejected or reduced."""
    # Give the EKF some time to settle
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.zeros(3)
    for _ in range(100):
        ekf.predict(gyro, accel, 0.005)
        ekf.update_altitude(2.0)
    state_before = ekf.get_state()
    # Now inject a wildly wrong altitude
    ekf.update_altitude(100.0)
    state_after = ekf.get_state()
    # State should NOT jump to z=-100
    assert abs(state_after["position"][2]) < 50
```

- **Acceptance criteria:**
  - [ ] `from src.navigation import EKF, StateEstimator` works
  - [ ] `pytest tests/test_ekf.py -v` passes (6+ tests)

## Scope

### Files you OWN:
- `src/navigation/ekf.py`
- `src/navigation/state_estimator.py`
- `src/navigation/__init__.py`
- `tests/test_ekf.py`

### Files to AVOID:
- `src/navigation/obstacle_detector.py` (legacy, don't touch)
- `src/simulation/` (Wave 2, don't modify)
- `src/control/` (Wave 2, don't modify)
- `src/sensors/` (Wave 2, don't modify)

### Files you may READ:
- `src/config/drone_params.py`, `src/config/sim_params.py`
- `src/math_utils/quaternion.py`, `src/math_utils/rotations.py`
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- Wave 1+2 complete. Verify:
  - `from src.math_utils import body_to_ned, ned_to_body, quat_multiply, quat_normalize` works
  - `from src.config import SimParams` works

### What depends on YOUR output:
- **Task 3.2 (Main Loop)**: imports EKF, StateEstimator
- **Task 4.1 (Demos)**: uses EKF health metrics for failure demo plots

## Status Updates
Write to `.orchestrator/status/agent_1_wave3_status.md`.

## Definition of Done
- [ ] EKF predict integrates state correctly from IMU
- [ ] EKF update_altitude provides altitude observability
- [ ] EKF update_accel provides roll/pitch observability
- [ ] Innovation gating rejects outlier measurements
- [ ] Quaternion stays normalized through all operations
- [ ] StateEstimator handles sensor health and emergency detection
- [ ] `pytest tests/test_ekf.py -v` passes
- [ ] No files outside scope were modified

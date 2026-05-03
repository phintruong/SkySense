# Agent B Instructions — Wave 5 (Expanded Tests)

## Assignment
- **Role:** Expanded unit test suite for drone dynamics and motor model
- **Wave:** 5 (runs in parallel with Agents A and C)

## Objective
Add missing test coverage for `DroneModel` and `MotorModel`, and expand EKF test coverage. The project currently has 52 tests but is missing physics-level tests for the rigid body dynamics and motor lag model.

## Context

**Working directory:** `C:\Users\phine\OneDrive\Laptop\CODESTUFF\DRONE\SkySense`
**Test framework:** pytest. Run with `pytest tests/ -v`.
**Existing test files:**
- `test_quaternion.py` (13 tests)
- `test_rotations.py` (12 tests)
- `test_pid.py` (8 tests)
- `test_motor_mixer.py` (6 tests)
- `test_ekf.py` (8 tests)
- `test_closed_loop.py` (3 tests)
- `test_control_config.py` (2 tests)
- `conftest.py` — shared fixtures providing `DroneParams()` and `SimParams()`

**Conventions:**
- NED: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81]. Drone at 2m altitude = z=-2.0.
- Quaternion: [qw, qx, qy, qz] scalar-first.
- Thrust in body frame: [0, 0, -sum(thrusts)] (upward is -Z in body NED).
- Hover thrust per motor: mass * g / 4 = 1.8 * 9.81 / 4 = 4.4145 N.

## Available imports

```python
import numpy as np
from src.config import DroneParams, SimParams
from src.simulation import DroneModel, MotorModel, Environment
from src.simulation.drone_model import DroneState
```

**conftest.py fixtures:**
```python
@pytest.fixture
def drone_params():
    return DroneParams()

@pytest.fixture
def sim_params():
    return SimParams()
```

## Key class APIs

**DroneModel:**
```python
class DroneModel:
    def __init__(self, params: DroneParams):
        self.state = DroneState()  # pos=[0,0,0], vel=[0,0,0], quat=[1,0,0,0], omega=[0,0,0]

    def reset(self, initial_state=None):  # Reset to given state or default
    def compute_torques(self, motor_thrusts: np.ndarray) -> np.ndarray:  # [roll, pitch, yaw] Nm
    def step(self, motor_thrusts, dt, external_force=None) -> DroneState:
    def get_body_acceleration(self) -> np.ndarray:  # Last accel in body frame
    def get_angular_velocity(self) -> np.ndarray:  # Body-frame angular velocity

# DroneState fields: position(3), velocity(3), quaternion(4), angular_velocity(3)
```

**DroneParams defaults:**
```python
mass = 1.8
arm_length = 0.18
gravity = 9.81
drag_coefficient = 0.3
motor_max_thrust = 12.0
motor_min_thrust = 0.0
motor_tau = 0.03
thrust_coefficient = 1.0e-5
torque_coefficient = 1.0e-7
hover_thrust_per_motor = 4.4145  # mass * g / 4
```

**MotorModel:**
```python
class MotorModel:
    def __init__(self, params: DroneParams):
        self.thrusts = np.zeros(4)

    def reset(self):
    def update(self, commanded: np.ndarray, dt: float) -> np.ndarray:
        # First-order lag: thrust += (commanded - thrust) * dt / tau
        # Clamp to [motor_min_thrust, motor_max_thrust]
```

**Ground collision in DroneModel.step():**
```python
if self.state.position[2] > 0.0:  # Below ground in NED
    self.state.position[2] = 0.0
    self.state.velocity[2] = min(self.state.velocity[2], 0.0)
```

## Tasks

### 1. Create `tests/test_drone_model.py`

```python
"""Tests for DroneModel rigid body dynamics."""
import numpy as np
import pytest
from src.config import DroneParams
from src.simulation import DroneModel
from src.simulation.drone_model import DroneState


class TestFreefall:
    """No thrust — drone should fall under gravity."""

    def test_freefall_z_increases(self, drone_params):
        """With zero thrust, position z increases (falls in NED)."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.position[2] > -2.0  # Fell toward ground

    def test_freefall_velocity_grows(self, drone_params):
        """Velocity z should increase (accelerate downward in NED)."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -5.0])))
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.velocity[2] > 0.0  # Falling down in NED

    def test_freefall_xy_unchanged(self, drone_params):
        """With no thrust or external force, x/y position should stay ~zero."""
        model = DroneModel(drone_params)
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert abs(model.state.position[0]) < 1e-10
        assert abs(model.state.position[1]) < 1e-10


class TestHover:
    """Hover thrust = mass * g / 4 per motor should hold position."""

    def test_hover_holds_altitude(self, drone_params):
        """At hover thrust, vertical velocity stays near zero."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005)
        assert abs(model.state.velocity[2]) < 0.1  # Near-zero vertical velocity

    def test_hover_altitude_stable(self, drone_params):
        """At hover thrust, altitude stays within 0.1m of initial over 1s."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005)
        assert abs(model.state.position[2] - (-2.0)) < 0.1


class TestDrag:
    """Verify drag model slows the drone."""

    def test_drag_limits_velocity(self, drone_params):
        """With constant excess thrust, velocity converges (doesn't grow forever)."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -10.0])))
        excess_thrust = drone_params.hover_thrust_per_motor * 1.5
        # Run long enough for drag to kick in
        for _ in range(2000):
            model.step(np.full(4, excess_thrust), dt=0.005)
        vel1 = model.state.velocity[2]
        for _ in range(2000):
            model.step(np.full(4, excess_thrust), dt=0.005)
        vel2 = model.state.velocity[2]
        # Velocity should converge: second half changes much less
        assert abs(vel2 - vel1) < abs(vel1) * 0.5


class TestTorques:
    """Verify asymmetric thrust produces angular acceleration."""

    def test_roll_torque_from_asymmetric_thrust(self, drone_params):
        """Motors 3,4 higher than 1,2 should produce positive roll angular velocity."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
        thrusts = np.array([hover - 1, hover - 1, hover + 1, hover + 1])
        for _ in range(100):
            model.step(thrusts, dt=0.005)
        # Roll is angular_velocity[0] — should be non-zero
        assert abs(model.state.angular_velocity[0]) > 0.01

    def test_equal_thrust_no_rotation(self, drone_params):
        """Equal thrust on all motors should produce no angular velocity."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
        for _ in range(100):
            model.step(np.full(4, hover), dt=0.005)
        assert np.linalg.norm(model.state.angular_velocity) < 1e-10

    def test_yaw_torque_from_diagonal_motors(self, drone_params):
        """Diagonal motor pairs produce yaw torque."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
        # Motors 1,3 (CW) vs 2,4 (CCW) — increase 2,4
        thrusts = np.array([hover - 0.5, hover + 0.5, hover - 0.5, hover + 0.5])
        for _ in range(200):
            model.step(thrusts, dt=0.005)
        assert abs(model.state.angular_velocity[2]) > 0.001


class TestGroundCollision:
    """Verify ground collision prevents z > 0."""

    def test_ground_clamp(self, drone_params):
        """Drone starting at ground with no thrust stays at z=0."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, 0.0])))
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.position[2] <= 0.0

    def test_downward_velocity_clamped_at_ground(self, drone_params):
        """Velocity z should be clamped to <= 0 when at ground."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -0.01])))
        # Let it fall to ground
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.position[2] <= 0.0
        assert model.state.velocity[2] <= 0.0


class TestExternalForce:
    """Verify external forces affect dynamics."""

    def test_external_force_moves_drone(self, drone_params):
        """Applying an external force in x-direction should move the drone in x."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        ext_force = np.array([5.0, 0.0, 0.0])
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005, external_force=ext_force)
        assert model.state.position[0] > 0.1  # Moved north


class TestComputeTorques:
    """Direct tests on the torque computation."""

    def test_equal_thrust_zero_torque(self, drone_params):
        model = DroneModel(drone_params)
        torques = model.compute_torques(np.full(4, 5.0))
        np.testing.assert_allclose(torques, 0.0, atol=1e-12)

    def test_torque_signs(self, drone_params):
        """Verify torque signs follow quad-X convention."""
        model = DroneModel(drone_params)
        hover = 5.0
        # Increase motors 3,4 (left side) → positive roll
        t_roll = model.compute_torques(np.array([hover-1, hover-1, hover+1, hover+1]))
        assert t_roll[0] > 0  # Positive roll torque
```

### 2. Create `tests/test_motor_model.py`

```python
"""Tests for MotorModel first-order lag."""
import numpy as np
import pytest
from src.config import DroneParams
from src.simulation import MotorModel


class TestStepResponse:
    """Step response behavior of the first-order lag."""

    def test_step_response_converges(self, drone_params):
        """Commanded thrust, actual thrust should converge to commanded."""
        motor = MotorModel(drone_params)
        commanded = np.full(4, 8.0)
        for _ in range(200):
            actual = motor.update(commanded, dt=0.005)
        np.testing.assert_allclose(actual, 8.0, atol=0.01)

    def test_first_step_not_instant(self, drone_params):
        """First step should not immediately reach commanded value."""
        motor = MotorModel(drone_params)
        actual = motor.update(np.full(4, 10.0), dt=0.005)
        assert all(a < 10.0 for a in actual)

    def test_time_constant(self, drone_params):
        """After one tau, thrust should reach ~63% of commanded."""
        motor = MotorModel(drone_params)
        target = 10.0
        steps_for_tau = int(drone_params.motor_tau / 0.005)
        for _ in range(steps_for_tau):
            actual = motor.update(np.full(4, target), dt=0.005)
        expected = target * (1 - np.exp(-1))  # ~63.2%
        np.testing.assert_allclose(actual[0], expected, atol=0.5)


class TestClamping:
    """Motor thrust clamping."""

    def test_max_clamp(self, drone_params):
        """Commanded above max should be clamped."""
        motor = MotorModel(drone_params)
        for _ in range(500):
            actual = motor.update(np.full(4, 100.0), dt=0.005)
        assert all(a <= drone_params.motor_max_thrust for a in actual)

    def test_min_clamp(self, drone_params):
        """Commanded below min should be clamped."""
        motor = MotorModel(drone_params)
        for _ in range(500):
            actual = motor.update(np.full(4, -5.0), dt=0.005)
        assert all(a >= drone_params.motor_min_thrust for a in actual)

    def test_individual_motors_independent(self, drone_params):
        """Each motor tracks its own commanded value."""
        motor = MotorModel(drone_params)
        commanded = np.array([2.0, 4.0, 6.0, 8.0])
        for _ in range(500):
            actual = motor.update(commanded, dt=0.005)
        np.testing.assert_allclose(actual, commanded, atol=0.01)


class TestReset:

    def test_reset_zeros_thrusts(self, drone_params):
        motor = MotorModel(drone_params)
        motor.update(np.full(4, 10.0), dt=0.005)
        motor.reset()
        np.testing.assert_allclose(motor.thrusts, 0.0)
```

### 3. Expand `tests/test_ekf.py` — add these tests

Append to the existing file (don't delete existing tests). The existing tests cover:
- `test_initial_state`, `test_predict_gravity_only`, `test_altitude_update_pulls_state`
- `test_quaternion_stays_normalized`, `test_covariance_grows_without_updates`
- `test_innovation_gating`, `test_state_estimator_unhealthy_imu_sets_emergency`
- `test_state_estimator_skips_invalid_altitude`

**Add:**
```python
def test_bias_estimation_convergence(sim_params):
    """EKF should estimate a known gyro bias over time."""
    ekf = EKF(sim_params)
    true_bias = np.array([0.05, -0.03, 0.02])
    dt = 0.005
    # Simulate hover IMU readings with known bias
    for _ in range(2000):
        gyro = true_bias + np.random.normal(0, 0.01, 3)
        accel = np.array([0.0, 0.0, -9.81])  # Level hover specific force
        ekf.predict(gyro, accel, dt)
        ekf.update_accel(accel)
        if _ % 10 == 0:
            ekf.update_altitude(2.0)
    est_bias = ekf.x[10:13]
    # Should converge to within 50% of true bias
    assert np.linalg.norm(est_bias - true_bias) < np.linalg.norm(true_bias)


def test_altitude_covariance_grows_without_updates(sim_params):
    """Z position covariance should grow when altitude updates stop."""
    ekf = EKF(sim_params)
    dt = 0.005
    # First: run with altitude updates to establish baseline
    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
        ekf.update_altitude(2.0)
    cov_with_updates = ekf.P[2, 2]
    # Then: run without altitude updates
    for _ in range(200):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
    cov_without_updates = ekf.P[2, 2]
    assert cov_without_updates > cov_with_updates * 2


def test_velocity_estimation_with_accel(sim_params):
    """EKF velocity should roughly track when acceleration is applied."""
    ekf = EKF(sim_params)
    dt = 0.005
    # Initialize at hover
    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
        ekf.update_accel(np.array([0.0, 0.0, -9.81]))
        ekf.update_altitude(2.0)
    initial_vz = ekf.x[5]
    # Now predict with slightly different accel (simulating reduced thrust)
    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -8.0]), dt)
    # Velocity should change from the accel difference
    final_vz = ekf.x[5]
    assert abs(final_vz - initial_vz) > 0.01
```

**Import needed at top of test_ekf.py** (check if EKF is already imported):
```python
from src.navigation.ekf import EKF
```

## Verification
1. `pytest tests/ -v` — all tests pass (52 existing + ~20 new)
2. No test modifies global state or interferes with other tests
3. Each test runs in under 2 seconds (no long simulations in unit tests)

## Files you own
- `tests/test_drone_model.py` (CREATE)
- `tests/test_motor_model.py` (CREATE)
- `tests/test_ekf.py` (APPEND new tests only — do NOT delete existing tests)

## Files NOT to touch
- All `src/` files (read only)
- `main.py` (read only)
- `demos/` (owned by Agent A)
- `showcase/` (complete, don't touch)
- `tests/conftest.py` (already has the fixtures you need)
- Other test files (don't modify)

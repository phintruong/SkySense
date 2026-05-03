import numpy as np
import pytest

from src.config import SimParams
from src.navigation import EKF, StateEstimator


@pytest.fixture
def ekf():
    return EKF(SimParams())


def test_initial_state(ekf):
    """State starts at origin with identity quaternion."""
    state = ekf.get_state()
    np.testing.assert_allclose(state["position"], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(state["quaternion"], [1.0, 0.0, 0.0, 0.0])


def test_predict_gravity_only(ekf):
    """Hover specific force should cancel gravity in NED."""
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.zeros(3)
    for _ in range(200):
        ekf.predict(gyro, accel, 0.005)
    state = ekf.get_state()
    assert abs(state["velocity"][2]) < 0.1


def test_altitude_update_pulls_state(ekf):
    """Altitude update should pull z-estimate toward measurement."""
    ekf.update_altitude(2.0)
    state = ekf.get_state()
    assert state["position"][2] < 0.0


def test_quaternion_stays_normalized(ekf):
    """Quaternion norm should stay near 1 through predict and update cycles."""
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.array([0.1, 0.05, 0.02])
    for i in range(1000):
        ekf.predict(gyro, accel, 0.005)
        if i % 10 == 0:
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
    """Large outlier altitude measurement should be rejected."""
    accel = np.array([0.0, 0.0, -9.81])
    gyro = np.zeros(3)
    for _ in range(100):
        ekf.predict(gyro, accel, 0.005)
        ekf.update_altitude(2.0)
    ekf.update_altitude(100.0)
    state_after = ekf.get_state()
    assert abs(state_after["position"][2]) < 50.0


def test_state_estimator_unhealthy_imu_sets_emergency():
    estimator = StateEstimator(SimParams())
    estimator.predict(
        {"accel": np.zeros(3), "gyro": np.zeros(3), "healthy": False},
        0.005,
    )
    health = estimator.get_health()
    assert health["imu_healthy"] is False
    assert health["emergency"] is True


def test_state_estimator_skips_invalid_altitude():
    estimator = StateEstimator(SimParams())
    estimator.update_altitude({"altitude": 2.0, "valid": False})
    np.testing.assert_allclose(estimator.get_state()["position"], [0.0, 0.0, 0.0])


def test_bias_estimation_convergence(sim_params):
    """EKF should estimate a known gyro bias over time."""
    rng = np.random.default_rng(7)
    ekf = EKF(sim_params)
    true_bias = np.array([0.05, -0.03, 0.02])
    dt = 0.005
    for i in range(2000):
        gyro = true_bias + rng.normal(0.0, 0.01, 3)
        accel = np.array([0.0, 0.0, -9.81])
        ekf.predict(gyro, accel, dt)
        ekf.update_accel(accel)
        if i % 10 == 0:
            ekf.update_altitude(2.0)
    estimated_bias = ekf.x[10:13]
    assert np.linalg.norm(estimated_bias - true_bias) < np.linalg.norm(true_bias)


def test_altitude_covariance_grows_without_updates(sim_params):
    """Z position covariance should grow when altitude updates stop."""
    ekf = EKF(sim_params)
    dt = 0.005
    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
        ekf.update_altitude(2.0)
    cov_with_updates = ekf.P[2, 2]

    for _ in range(200):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
    cov_without_updates = ekf.P[2, 2]

    assert cov_without_updates > cov_with_updates * 2.0


def test_velocity_estimation_with_accel(sim_params):
    """EKF velocity should roughly track when acceleration is applied."""
    ekf = EKF(sim_params)
    dt = 0.005
    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.81]), dt)
        ekf.update_accel(np.array([0.0, 0.0, -9.81]))
        ekf.update_altitude(2.0)
    initial_vz = ekf.x[5]

    for _ in range(100):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -8.0]), dt)
    final_vz = ekf.x[5]

    assert abs(final_vz - initial_vz) > 0.01

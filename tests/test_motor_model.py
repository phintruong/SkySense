"""Tests for MotorModel first-order lag."""

import numpy as np

from src.simulation import MotorModel


class TestStepResponse:
    """Step response behavior of the first-order lag."""

    def test_step_response_converges(self, drone_params):
        """Commanded thrust should converge to commanded value."""
        motor = MotorModel(drone_params)
        commanded = np.full(4, 8.0)
        for _ in range(200):
            actual = motor.update(commanded, dt=0.005)
        np.testing.assert_allclose(actual, 8.0, atol=0.01)

    def test_first_step_not_instant(self, drone_params):
        """First step should not immediately reach commanded value."""
        motor = MotorModel(drone_params)
        actual = motor.update(np.full(4, 10.0), dt=0.005)
        assert all(value < 10.0 for value in actual)

    def test_time_constant(self, drone_params):
        """After one tau, thrust should reach about 63% of commanded."""
        motor = MotorModel(drone_params)
        target = 10.0
        steps_for_tau = int(drone_params.motor_tau / 0.005)
        for _ in range(steps_for_tau):
            actual = motor.update(np.full(4, target), dt=0.005)
        expected = target * (1.0 - np.exp(-1.0))
        np.testing.assert_allclose(actual[0], expected, atol=0.5)


class TestClamping:
    """Motor thrust clamping."""

    def test_max_clamp(self, drone_params):
        """Commanded above max should be clamped."""
        motor = MotorModel(drone_params)
        for _ in range(500):
            actual = motor.update(np.full(4, 100.0), dt=0.005)
        assert all(value <= drone_params.motor_max_thrust for value in actual)

    def test_min_clamp(self, drone_params):
        """Commanded below min should be clamped."""
        motor = MotorModel(drone_params)
        for _ in range(500):
            actual = motor.update(np.full(4, -5.0), dt=0.005)
        assert all(value >= drone_params.motor_min_thrust for value in actual)

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

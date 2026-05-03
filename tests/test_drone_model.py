"""Tests for DroneModel rigid body dynamics."""

import numpy as np

from src.simulation import DroneModel
from src.simulation.drone_model import DroneState


class TestFreefall:
    """No thrust: drone should fall under gravity."""

    def test_freefall_z_increases(self, drone_params):
        """With zero thrust, position z increases (falls in NED)."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.position[2] > -2.0

    def test_freefall_velocity_grows(self, drone_params):
        """Velocity z should increase (accelerate downward in NED)."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -5.0])))
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.velocity[2] > 0.0

    def test_freefall_xy_unchanged(self, drone_params):
        """With no thrust or external force, x/y position should stay zero."""
        model = DroneModel(drone_params)
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert abs(model.state.position[0]) < 1e-10
        assert abs(model.state.position[1]) < 1e-10


class TestHover:
    """Hover thrust should hold position."""

    def test_hover_holds_altitude(self, drone_params):
        """At hover thrust, vertical velocity stays near zero."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005)
        assert abs(model.state.velocity[2]) < 0.1

    def test_hover_altitude_stable(self, drone_params):
        """At hover thrust, altitude stays within 0.1 m of initial over 1 s."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005)
        assert abs(model.state.position[2] - (-2.0)) < 0.1


class TestDrag:
    """Verify drag model slows acceleration."""

    def test_drag_limits_velocity(self, drone_params):
        """With constant excess thrust, velocity trends toward convergence."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -10.0])))
        excess_thrust = drone_params.hover_thrust_per_motor * 1.5
        for _ in range(2000):
            model.step(np.full(4, excess_thrust), dt=0.005)
        vel1 = model.state.velocity[2]
        for _ in range(2000):
            model.step(np.full(4, excess_thrust), dt=0.005)
        vel2 = model.state.velocity[2]
        assert abs(vel2 - vel1) < abs(vel1) * 0.5


class TestTorques:
    """Verify asymmetric thrust produces angular acceleration."""

    def test_roll_torque_from_asymmetric_thrust(self, drone_params):
        """Motors 3 and 4 higher than 1 and 2 should produce roll motion."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
        thrusts = np.array([hover - 1.0, hover - 1.0, hover + 1.0, hover + 1.0])
        for _ in range(100):
            model.step(thrusts, dt=0.005)
        assert abs(model.state.angular_velocity[0]) > 0.01

    def test_equal_thrust_no_rotation(self, drone_params):
        """Equal thrust on all motors should produce no angular velocity."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
        for _ in range(100):
            model.step(np.full(4, hover), dt=0.005)
        assert np.linalg.norm(model.state.angular_velocity) < 1e-10

    def test_yaw_torque_from_diagonal_motors(self, drone_params):
        """Differential diagonal motor pairs produce yaw torque."""
        model = DroneModel(drone_params)
        hover = drone_params.hover_thrust_per_motor
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
        for _ in range(100):
            model.step(np.zeros(4), dt=0.005)
        assert model.state.position[2] <= 0.0
        assert model.state.velocity[2] <= 0.0


class TestExternalForce:
    """Verify external forces affect dynamics."""

    def test_external_force_moves_drone(self, drone_params):
        """External north force should move the drone in x."""
        model = DroneModel(drone_params)
        model.reset(DroneState(position=np.array([0.0, 0.0, -2.0])))
        hover = drone_params.hover_thrust_per_motor
        external_force = np.array([5.0, 0.0, 0.0])
        for _ in range(200):
            model.step(np.full(4, hover), dt=0.005, external_force=external_force)
        assert model.state.position[0] > 0.1


class TestComputeTorques:
    """Direct tests on torque computation."""

    def test_equal_thrust_zero_torque(self, drone_params):
        model = DroneModel(drone_params)
        torques = model.compute_torques(np.full(4, 5.0))
        np.testing.assert_allclose(torques, 0.0, atol=1e-12)

    def test_torque_signs(self, drone_params):
        """Verify torque signs follow the quad-X convention."""
        model = DroneModel(drone_params)
        hover = 5.0
        torques = model.compute_torques(
            np.array([hover - 1.0, hover - 1.0, hover + 1.0, hover + 1.0])
        )
        assert torques[0] > 0.0

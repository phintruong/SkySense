from dataclasses import dataclass, field

import numpy as np

from src.config import DroneParams
from src.math_utils import body_to_ned, ned_to_body, quat_identity, quat_multiply, quat_normalize


@dataclass
class DroneState:
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    quaternion: np.ndarray = field(default_factory=quat_identity)
    angular_velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))


class DroneModel:
    def __init__(self, params: DroneParams):
        self.params = params
        self.state = DroneState()
        self._last_acceleration_ned = np.zeros(3, dtype=float)

    def reset(self, initial_state: DroneState | None = None) -> None:
        """Reset to given state or default."""
        self.state = initial_state if initial_state is not None else DroneState()
        self.state.quaternion = quat_normalize(self.state.quaternion)
        self._last_acceleration_ned = np.zeros(3, dtype=float)

    def compute_torques(self, motor_thrusts: np.ndarray) -> np.ndarray:
        """Compute body-frame torques [roll, pitch, yaw] from quad-X motor thrusts."""
        thrusts = np.asarray(motor_thrusts, dtype=float)
        t1, t2, t3, t4 = thrusts
        d = self.params.arm_length / np.sqrt(2.0)
        cq_ratio = self.params.torque_coefficient / self.params.thrust_coefficient

        return np.array(
            [
                d * (-t1 - t2 + t3 + t4),
                d * (-t1 + t2 + t3 - t4),
                cq_ratio * (-t1 + t2 - t3 + t4),
            ],
            dtype=float,
        )

    def step(
        self,
        motor_thrusts: np.ndarray,
        dt: float,
        external_force: np.ndarray | None = None,
    ) -> DroneState:
        """Advance rigid body dynamics by one timestep."""
        thrusts = np.asarray(motor_thrusts, dtype=float)
        ext_force = np.zeros(3, dtype=float) if external_force is None else np.asarray(external_force, dtype=float)

        q = quat_normalize(self.state.quaternion)
        thrust_body = np.array([0.0, 0.0, -float(np.sum(thrusts))], dtype=float)
        force_ned = body_to_ned(q, thrust_body)
        force_ned += np.array([0.0, 0.0, self.params.mass * self.params.gravity], dtype=float)
        force_ned += ext_force
        force_ned += -self.params.drag_coefficient * self.state.velocity

        acceleration = force_ned / self.params.mass
        self._last_acceleration_ned = acceleration

        self.state.velocity = self.state.velocity + acceleration * dt
        self.state.position = self.state.position + self.state.velocity * dt

        if self.state.position[2] > 0.0:
            self.state.position[2] = 0.0
            self.state.velocity[2] = min(self.state.velocity[2], 0.0)

        torques = self.compute_torques(thrusts)
        omega = self.state.angular_velocity
        inertia = self.params.inertia_matrix
        angular_acceleration = self.params.inertia_matrix_inv @ (
            torques - np.cross(omega, inertia @ omega)
        )

        self.state.angular_velocity = omega + angular_acceleration * dt
        omega_quat = np.array([0.0, *self.state.angular_velocity], dtype=float)
        q_dot = 0.5 * quat_multiply(q, omega_quat)
        self.state.quaternion = quat_normalize(q + q_dot * dt)

        return self.state

    def get_body_acceleration(self) -> np.ndarray:
        """Return last computed acceleration in body frame."""
        return ned_to_body(self.state.quaternion, self._last_acceleration_ned)

    def get_angular_velocity(self) -> np.ndarray:
        """Return angular velocity in body frame."""
        return self.state.angular_velocity.copy()

import numpy as np

from src.config import DroneParams


class MotorMixer:
    def __init__(self, params: DroneParams):
        self.params = params
        self.d = params.arm_length / np.sqrt(2.0)
        self.cq = params.torque_coefficient / params.thrust_coefficient

    def mix(
        self,
        thrust: float,
        torque_roll: float,
        torque_pitch: float,
        torque_yaw: float,
    ) -> np.ndarray:
        """Convert total thrust and body torques to quad-X motor thrusts."""
        base = thrust / 4.0
        motors = np.array(
            [
                base - torque_roll / (4.0 * self.d) - torque_pitch / (4.0 * self.d) - torque_yaw / (4.0 * self.cq),
                base - torque_roll / (4.0 * self.d) + torque_pitch / (4.0 * self.d) + torque_yaw / (4.0 * self.cq),
                base + torque_roll / (4.0 * self.d) + torque_pitch / (4.0 * self.d) - torque_yaw / (4.0 * self.cq),
                base + torque_roll / (4.0 * self.d) - torque_pitch / (4.0 * self.d) + torque_yaw / (4.0 * self.cq),
            ],
            dtype=float,
        )

        return np.clip(motors, self.params.motor_min_thrust, self.params.motor_max_thrust)

    def reset(self) -> None:
        """No internal state to reset."""
        pass

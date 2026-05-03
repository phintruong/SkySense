import numpy as np

from .pid import PIDController


class AttitudeController:
    def __init__(self, kp: float = 8.0, ki: float = 0.5, kd: float = 3.0):
        self.roll_pid = PIDController(kp, ki, kd, output_min=-5.0, output_max=5.0, integral_max=1.0)
        self.pitch_pid = PIDController(kp, ki, kd, output_min=-5.0, output_max=5.0, integral_max=1.0)
        self.yaw_pid = PIDController(
            kp * 0.5,
            ki * 0.5,
            kd * 0.5,
            output_min=-2.0,
            output_max=2.0,
            integral_max=0.5,
        )

    @classmethod
    def from_sim_params(cls, params) -> "AttitudeController":
        return cls(
            kp=params.attitude_kp,
            ki=params.attitude_ki,
            kd=params.attitude_kd,
        )

    def compute(
        self,
        current_euler: np.ndarray,
        desired_euler: np.ndarray,
        dt: float,
        current_rates_body: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute body-frame torque commands from Euler angle errors."""
        current = np.asarray(current_euler, dtype=float)
        desired = np.asarray(desired_euler, dtype=float)
        error = desired - current
        yaw_error = np.arctan2(np.sin(error[2]), np.cos(error[2]))

        if current_rates_body is not None:
            rates = np.asarray(current_rates_body, dtype=float)
            return np.array(
                [
                    self.roll_pid.update_with_measured_rate(float(error[0]), float(rates[0]), dt),
                    self.pitch_pid.update_with_measured_rate(float(error[1]), float(rates[1]), dt),
                    self.yaw_pid.update_with_measured_rate(float(yaw_error), float(rates[2]), dt),
                ],
                dtype=float,
            )

        return np.array(
            [
                self.roll_pid.update(float(error[0]), dt),
                self.pitch_pid.update(float(error[1]), dt),
                self.yaw_pid.update(float(yaw_error), dt),
            ],
            dtype=float,
        )

    def reset(self) -> None:
        """Reset all PIDs."""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

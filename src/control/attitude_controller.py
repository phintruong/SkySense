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
                    self._update_with_rate_damping(self.roll_pid, float(error[0]), float(rates[0]), dt),
                    self._update_with_rate_damping(self.pitch_pid, float(error[1]), float(rates[1]), dt),
                    self._update_with_rate_damping(self.yaw_pid, float(yaw_error), float(rates[2]), dt),
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

    def _update_with_rate_damping(
        self,
        pid: PIDController,
        error: float,
        body_rate: float,
        dt: float,
    ) -> float:
        pid._integral += error * dt
        pid._integral = float(np.clip(pid._integral, -pid.integral_max, pid.integral_max))
        pid._prev_error = error

        output = pid.kp * error + pid.ki * pid._integral - pid.kd * body_rate
        return float(np.clip(output, pid.output_min, pid.output_max))

    def reset(self) -> None:
        """Reset all PIDs."""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

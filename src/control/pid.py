import numpy as np


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -float("inf"),
        output_max: float = float("inf"),
        integral_max: float = float("inf"),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max
        self._integral = 0.0
        self._prev_error: float | None = None

    def update(self, error: float, dt: float) -> float:
        """Compute PID output with integral and output clamping."""
        derivative = 0.0
        if self._prev_error is not None and dt > 0.0:
            derivative = (error - self._prev_error) / dt

        self._integral += error * dt
        self._integral = float(np.clip(self._integral, -self.integral_max, self.integral_max))
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return float(np.clip(output, self.output_min, self.output_max))

    def reset(self) -> None:
        """Zero integral and previous error."""
        self._integral = 0.0
        self._prev_error = None

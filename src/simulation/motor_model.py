import numpy as np

from src.config import DroneParams


class MotorModel:
    def __init__(self, params: DroneParams):
        self.params = params
        self.thrusts = np.zeros(4, dtype=float)

    def reset(self) -> None:
        self.thrusts = np.zeros(4, dtype=float)

    def update(self, commanded: np.ndarray, dt: float) -> np.ndarray:
        """Apply first-order motor lag and clamp thrusts to valid limits."""
        commanded = np.asarray(commanded, dtype=float)
        commanded = np.clip(commanded, self.params.motor_min_thrust, self.params.motor_max_thrust)

        if self.params.motor_tau <= 0.0:
            self.thrusts = commanded
        else:
            alpha = max(0.0, min(1.0, dt / self.params.motor_tau))
            self.thrusts = self.thrusts + (commanded - self.thrusts) * alpha

        self.thrusts = np.clip(self.thrusts, self.params.motor_min_thrust, self.params.motor_max_thrust)
        return self.thrusts.copy()

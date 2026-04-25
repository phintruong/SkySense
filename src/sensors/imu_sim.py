import numpy as np

from src.config import SimParams
from src.math_utils import ned_to_body


class IMUSim:
    def __init__(self, params: SimParams):
        """
        Simulated MPU-6050 IMU.

        Outputs:
        - accelerometer: specific force in body frame + noise
        - gyroscope: angular velocity in body frame + bias + noise
        """
        self.params = params
        self.gyro_bias = np.zeros(3, dtype=float)
        self._healthy = True
        self._failure_mode = "normal"  # "normal", "off", "noisy"

    def read(
        self,
        true_acceleration_ned: np.ndarray,
        true_angular_velocity_body: np.ndarray,
        quaternion: np.ndarray,
        dt: float,
    ) -> dict:
        """Generate noisy IMU reading from true kinematics."""
        if self._failure_mode == "off":
            return {
                "accel": np.zeros(3, dtype=float),
                "gyro": np.zeros(3, dtype=float),
                "healthy": False,
            }

        noise_scale = 10.0 if self._failure_mode == "noisy" else 1.0

        gravity_ned = np.array([0.0, 0.0, 9.81], dtype=float)
        specific_force_ned = np.asarray(true_acceleration_ned, dtype=float) - gravity_ned
        specific_force_body = ned_to_body(np.asarray(quaternion, dtype=float), specific_force_ned)

        accel_noise_std = self.params.imu_accel_noise_std * noise_scale
        accel_noise = np.random.normal(0.0, accel_noise_std, size=3)
        accel = specific_force_body + accel_noise

        bias_step_std = self.params.imu_gyro_bias_std * np.sqrt(max(dt, 0.0))
        self.gyro_bias += np.random.normal(0.0, bias_step_std, size=3)

        gyro_noise_std = self.params.imu_gyro_noise_std * noise_scale
        gyro_noise = np.random.normal(0.0, gyro_noise_std, size=3)
        gyro = np.asarray(true_angular_velocity_body, dtype=float) + self.gyro_bias + gyro_noise

        return {
            "accel": accel,
            "gyro": gyro,
            "healthy": self._healthy,
        }

    def inject_failure(self, mode: str):
        """Set failure mode: off, noisy, or recover."""
        if mode == "recover":
            self._failure_mode = "normal"
            self._healthy = True
            return

        if mode not in {"normal", "off", "noisy"}:
            raise ValueError(f"Unsupported IMU failure mode: {mode}")

        self._failure_mode = mode
        self._healthy = mode == "normal"

    def is_healthy(self) -> bool:
        return self._healthy

    def reset(self):
        """Reset bias and failure state."""
        self.gyro_bias = np.zeros(3, dtype=float)
        self._healthy = True
        self._failure_mode = "normal"

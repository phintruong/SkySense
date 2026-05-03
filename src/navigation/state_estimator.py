import numpy as np

from src.config import SimParams

from .ekf import EKF


class StateEstimator:
    """Wrap EKF prediction and updates with basic sensor health handling."""

    def __init__(self, params: SimParams):
        self.ekf = EKF(params)
        self._imu_healthy = True
        self._ultrasonic_healthy = True
        self._emergency = False

    def predict(self, imu_data: dict, dt: float):
        """Run the IMU prediction step, falling back to hover assumptions on IMU failure."""
        self._imu_healthy = bool(imu_data.get("healthy", False))
        if self._imu_healthy:
            gyro = np.asarray(imu_data["gyro"], dtype=float)
            accel = np.asarray(imu_data["accel"], dtype=float)
        else:
            gyro = self.ekf.x[10:13].copy()
            accel = np.array([0.0, 0.0, -9.81], dtype=float)
            self._emergency = True

        self.ekf.predict(gyro, accel, dt)

    def update_accel(self, imu_data: dict):
        """Apply accelerometer update when the IMU is healthy."""
        self._imu_healthy = bool(imu_data.get("healthy", False))
        if not self._imu_healthy:
            self._emergency = True
            return
        self.ekf.update_accel(np.asarray(imu_data["accel"], dtype=float))

    def update_altitude(self, ultrasonic_data: dict):
        """Apply ultrasonic altitude update when the reading is valid."""
        self._ultrasonic_healthy = bool(ultrasonic_data.get("valid", False))
        if not self._ultrasonic_healthy:
            return
        self.ekf.update_altitude(float(ultrasonic_data["altitude"]))

    def get_state(self) -> dict:
        """Return EKF state estimate."""
        return self.ekf.get_state()

    def is_emergency(self) -> bool:
        return self._emergency

    def get_health(self) -> dict:
        return {
            "imu_healthy": self._imu_healthy,
            "ultrasonic_healthy": self._ultrasonic_healthy,
            "emergency": self._emergency,
            "covariance_trace": self.ekf.get_covariance_trace(),
            "innovation_norm": self.ekf.get_innovation_norm(),
        }

    def reset(self):
        self.ekf.reset()
        self._imu_healthy = True
        self._ultrasonic_healthy = True
        self._emergency = False

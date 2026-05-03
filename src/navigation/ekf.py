import numpy as np

from src.config import SimParams
from src.math_utils import (
    body_to_ned,
    ned_to_body,
    quat_multiply,
    quat_normalize,
    quat_to_euler,
)


class EKF:
    """
    13-state Extended Kalman Filter for drone state estimation.

    State vector:
        [0:3]   position in NED
        [3:6]   velocity in NED
        [6:10]  quaternion [qw, qx, qy, qz]
        [10:13] gyro bias in body frame
    """

    def __init__(self, params: SimParams):
        self.params = params
        self.x = np.zeros(13, dtype=float)
        self.x[6] = 1.0
        self.P = np.eye(13, dtype=float) * 0.1
        self.P[10:13, 10:13] = np.eye(3, dtype=float) * 0.01
        self._last_innovation = np.zeros(3, dtype=float)

    def predict(self, gyro: np.ndarray, accel: np.ndarray, dt: float):
        """Predict state and covariance using body-frame IMU readings."""
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        dt = float(dt)

        F = self._numerical_transition_jacobian(self.x, gyro, accel, dt)
        self.x = self._transition(self.x, gyro, accel, dt)
        self._normalize_state_quaternion()

        Q = self._process_noise(dt)
        self.P = F @ self.P @ F.T + Q
        self._symmetrize_covariance()

    def update_accel(self, accel_measurement: np.ndarray):
        """
        Update roll/pitch from accelerometer specific force.

        This assumes low translational acceleration, so the accelerometer mostly
        observes gravity in the body frame.
        """
        z = np.asarray(accel_measurement, dtype=float)
        h = self._accel_measurement_model(self.x)
        H = self._numerical_accel_jacobian()
        R = np.eye(3, dtype=float) * (self.params.ekf_accel_measurement_noise**2)

        y = z - h
        S = H @ self.P @ H.T + R
        self._last_innovation = y.copy()
        if self._is_gated(y, S):
            return

        K = self._kalman_gain(H, S)
        self.x = self.x + K @ y
        self._normalize_state_quaternion()
        I = np.eye(13, dtype=float)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self._symmetrize_covariance()

    def update_altitude(self, altitude: float):
        """Update NED z-position from ultrasonic altitude measurement."""
        H = np.zeros((1, 13), dtype=float)
        H[0, 2] = -1.0
        R = np.array([[self.params.ekf_altitude_measurement_noise**2]], dtype=float)

        y = np.array([float(altitude) - (-self.x[2])], dtype=float)
        S = H @ self.P @ H.T + R
        self._last_innovation = y.copy()
        if self._is_gated(y, S):
            return

        K = self._kalman_gain(H, S)
        self.x = self.x + (K @ y).reshape(13)
        self._normalize_state_quaternion()
        I = np.eye(13, dtype=float)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        self._symmetrize_covariance()

    def get_state(self) -> dict:
        """Return a copy of the current state estimate."""
        q = quat_normalize(self.x[6:10])
        return {
            "position": self.x[0:3].copy(),
            "velocity": self.x[3:6].copy(),
            "quaternion": q.copy(),
            "euler": quat_to_euler(q),
            "gyro_bias": self.x[10:13].copy(),
        }

    def get_covariance_trace(self) -> float:
        """Return trace(P), an aggregate uncertainty metric."""
        return float(np.trace(self.P))

    def get_innovation_norm(self) -> float:
        """Return norm of the latest accepted or rejected innovation."""
        return float(np.linalg.norm(self._last_innovation))

    def reset(self):
        """Reset state and covariance to initial values."""
        self.__init__(self.params)

    def _transition(self, x: np.ndarray, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        x_next = np.asarray(x, dtype=float).copy()
        q = quat_normalize(x_next[6:10])
        bias = x_next[10:13]
        omega = gyro - bias

        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]], dtype=float)
        q_dot = 0.5 * quat_multiply(q, omega_quat)

        gravity_ned = np.array([0.0, 0.0, 9.81], dtype=float)
        acceleration_ned = body_to_ned(q, accel) + gravity_ned

        x_next[0:3] = x_next[0:3] + x_next[3:6] * dt
        x_next[3:6] = x_next[3:6] + acceleration_ned * dt
        x_next[6:10] = quat_normalize(q + q_dot * dt)
        return x_next

    def _numerical_transition_jacobian(
        self,
        x: np.ndarray,
        gyro: np.ndarray,
        accel: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        eps = 1.0e-6
        F = np.eye(13, dtype=float)
        F[0:3, 3:6] = np.eye(3, dtype=float) * dt

        for i in range(6, 13):
            dx = np.zeros(13, dtype=float)
            dx[i] = eps
            x_plus = x + dx
            x_minus = x - dx
            x_plus[6:10] = quat_normalize(x_plus[6:10])
            x_minus[6:10] = quat_normalize(x_minus[6:10])
            f_plus = self._transition(x_plus, gyro, accel, dt)
            f_minus = self._transition(x_minus, gyro, accel, dt)
            F[:, i] = (f_plus - f_minus) / (2.0 * eps)
        return F

    def _process_noise(self, dt: float) -> np.ndarray:
        dt = max(float(dt), 0.0)
        Q = np.zeros((13, 13), dtype=float)
        Q[0:3, 0:3] = np.eye(3) * self.params.ekf_process_noise_pos * dt
        Q[3:6, 3:6] = np.eye(3) * self.params.ekf_process_noise_vel * dt
        Q[6:10, 6:10] = np.eye(4) * self.params.ekf_process_noise_quat * dt
        Q[10:13, 10:13] = np.eye(3) * self.params.ekf_process_noise_bias * dt
        return Q

    def _accel_measurement_model(self, x: np.ndarray) -> np.ndarray:
        q = quat_normalize(x[6:10])
        hover_specific_force_ned = np.array([0.0, 0.0, -9.81], dtype=float)
        return ned_to_body(q, hover_specific_force_ned)

    def _numerical_accel_jacobian(self) -> np.ndarray:
        eps = 1.0e-6
        H = np.zeros((3, 13), dtype=float)
        for i in range(6, 10):
            dx = np.zeros(13, dtype=float)
            dx[i] = eps
            x_plus = self.x + dx
            x_minus = self.x - dx
            x_plus[6:10] = quat_normalize(x_plus[6:10])
            x_minus[6:10] = quat_normalize(x_minus[6:10])
            H[:, i] = (
                self._accel_measurement_model(x_plus)
                - self._accel_measurement_model(x_minus)
            ) / (2.0 * eps)
        return H

    def _is_gated(self, innovation: np.ndarray, S: np.ndarray) -> bool:
        statistical_gate = self.params.ekf_innovation_threshold * np.sqrt(float(np.trace(S)))
        gate = max(statistical_gate, self.params.ekf_innovation_threshold)
        return float(np.linalg.norm(innovation)) > gate

    def _kalman_gain(self, H: np.ndarray, S: np.ndarray) -> np.ndarray:
        return np.linalg.solve(S, H @ self.P.T).T

    def _normalize_state_quaternion(self):
        self.x[6:10] = quat_normalize(self.x[6:10])

    def _symmetrize_covariance(self):
        self.P = 0.5 * (self.P + self.P.T)

import numpy as np

from .pid import PIDController


class PositionController:
    def __init__(
        self,
        pos_kp: float = 1.0,
        pos_ki: float = 0.1,
        pos_kd: float = 0.8,
        alt_kp: float = 5.0,
        alt_ki: float = 1.0,
        alt_kd: float = 3.0,
        max_tilt: float = 0.524,
        hover_thrust: float = 4.41 * 4,
    ):
        self.x_pid = PIDController(pos_kp, pos_ki, pos_kd, output_min=-max_tilt, output_max=max_tilt)
        self.y_pid = PIDController(pos_kp, pos_ki, pos_kd, output_min=-max_tilt, output_max=max_tilt)
        self.z_pid = PIDController(alt_kp, alt_ki, alt_kd, output_min=-10.0, output_max=10.0, integral_max=5.0)
        self.hover_thrust = hover_thrust
        self.max_tilt = max_tilt
        self.max_total_thrust = 48.0

    def compute(
        self,
        current_pos: np.ndarray,
        desired_pos: np.ndarray,
        current_vel: np.ndarray,
        current_yaw: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """Compute desired roll, desired pitch, and total thrust."""
        del current_vel
        current = np.asarray(current_pos, dtype=float)
        desired = np.asarray(desired_pos, dtype=float)
        error_ned = desired - current

        thrust = self.hover_thrust + self.z_pid.update(float(error_ned[2]), dt)
        thrust = float(np.clip(thrust, 0.0, self.max_total_thrust))

        cos_yaw = np.cos(current_yaw)
        sin_yaw = np.sin(current_yaw)
        error_body_x = cos_yaw * error_ned[0] + sin_yaw * error_ned[1]
        error_body_y = -sin_yaw * error_ned[0] + cos_yaw * error_ned[1]

        desired_pitch = -self.x_pid.update(float(error_body_x), dt)
        desired_roll = self.y_pid.update(float(error_body_y), dt)

        desired_roll = float(np.clip(desired_roll, -self.max_tilt, self.max_tilt))
        desired_pitch = float(np.clip(desired_pitch, -self.max_tilt, self.max_tilt))
        return desired_roll, desired_pitch, thrust

    def reset(self) -> None:
        self.x_pid.reset()
        self.y_pid.reset()
        self.z_pid.reset()

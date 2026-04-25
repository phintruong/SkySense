from dataclasses import dataclass

import numpy as np


@dataclass
class DroneParams:
    mass: float = 1.8  # Total mass in kg (frame + battery + electronics)
    arm_length: float = 0.18  # Center to motor distance in meters

    # Moment of inertia (kg*m^2) — approximate for 7-inch quad
    Ixx: float = 0.0123  # Roll inertia
    Iyy: float = 0.0123  # Pitch inertia (symmetric with roll for X-frame)
    Izz: float = 0.0224  # Yaw inertia (higher due to arm geometry)

    # Motor parameters
    motor_tau: float = 0.03  # First-order lag time constant (seconds)
    motor_max_thrust: float = 12.0  # Max thrust per motor in Newtons
    motor_min_thrust: float = 0.0  # Min thrust per motor in Newtons

    # Aerodynamic coefficients
    drag_coefficient: float = 0.3  # Linear drag coefficient (N/(m/s))

    # Propeller coefficients (quad-X)
    thrust_coefficient: float = 1.0e-5  # ct: thrust = ct * omega^2
    torque_coefficient: float = 1.0e-7  # cq: torque = cq * omega^2

    # Physical constants
    gravity: float = 9.81  # m/s^2 (positive in NED Z-down)

    # Motor numbering for quad-X (viewed from above):
    #   4 (FL, CCW)   1 (FR, CW)
    #         \       /
    #          [BODY]
    #         /       \
    #   3 (BL, CW)    2 (BR, CCW)

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz])

    @property
    def inertia_matrix_inv(self) -> np.ndarray:
        return np.diag([1.0 / self.Ixx, 1.0 / self.Iyy, 1.0 / self.Izz])

    @property
    def hover_thrust_per_motor(self) -> float:
        return (self.mass * self.gravity) / 4.0

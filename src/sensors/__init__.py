"""Sensor interfaces for simulation and optional hardware drivers."""

from .imu_sim import IMUSim
from .ultrasonic_sim import UltrasonicSim

try:
    from .lidar import RPLidarReader
except ImportError:  # Optional hardware dependency (pyrplidar)
    RPLidarReader = None

try:
    from .ultrasonic import DistanceSensor
except ImportError:  # Optional hardware dependency (gpiozero)
    DistanceSensor = None

__all__ = ["IMUSim", "UltrasonicSim", "RPLidarReader", "DistanceSensor"]

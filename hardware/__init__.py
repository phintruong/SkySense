"""
Hardware Interface Module

Provides interfaces for physical sensors:
- RPLIDAR A1 LiDAR sensor
- HC-SR04 ultrasonic distance sensor
"""

from .rplidar_reader import RPLidarReader

__all__ = ['RPLidarReader']


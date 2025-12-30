"""
Simulation Module

Provides simulated LiDAR scan data for testing obstacle detection logic
without requiring physical hardware.
"""

from .lidar_sim import generate_scan, generate_scan_with_pattern

__all__ = ['generate_scan', 'generate_scan_with_pattern']


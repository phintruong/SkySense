"""
Core Logic Module

Contains obstacle detection algorithms and navigation decision logic.
"""

from .logic import (
    process_scan,
    DANGER_RADIUS,
    FORWARD_CONE_HALF_ANGLE,
    get_forward_cone_obstacles,
    determine_action
)

__all__ = [
    'process_scan',
    'DANGER_RADIUS',
    'FORWARD_CONE_HALF_ANGLE',
    'get_forward_cone_obstacles',
    'determine_action'
]


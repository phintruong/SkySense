"""
Utility functions for LiDAR scan processing and obstacle avoidance.

This module provides functions for cleaning LiDAR scans, applying filters,
segmenting scans into sectors, and implementing VFH-style avoidance logic.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import math


def clean_scan(ranges: List[float], range_min: float, range_max: float) -> np.ndarray:
    """
    Clean LiDAR scan data by handling invalid values and clipping to valid range.
    
    Args:
        ranges: List of range measurements
        range_min: Minimum valid range
        range_max: Maximum valid range
        
    Returns:
        Cleaned numpy array of ranges
    """
    # Convert to numpy array
    ranges_array = np.array(ranges, dtype=np.float32)
    
    # Replace NaN and inf with range_max
    ranges_array = np.where(
        np.isnan(ranges_array) | np.isinf(ranges_array),
        range_max,
        ranges_array
    )
    
    # Clip to valid range
    ranges_array = np.clip(ranges_array, range_min, range_max)
    
    return ranges_array


def moving_median_filter(ranges: np.ndarray, window_size: int = 5) -> np.ndarray:
    """
    Apply moving median filter to reduce noise in LiDAR scan.
    
    Args:
        ranges: Array of range measurements
        window_size: Size of the median filter window
        
    Returns:
        Filtered array of ranges
    """
    if window_size <= 1:
        return ranges
    
    # Pad the array to handle edge cases
    pad_width = window_size // 2
    padded_ranges = np.pad(ranges, pad_width, mode='edge')
    
    # Apply moving median filter
    filtered_ranges = np.zeros_like(ranges)
    for i in range(len(ranges)):
        start_idx = i
        end_idx = i + window_size
        filtered_ranges[i] = np.median(padded_ranges[start_idx:end_idx])
    
    return filtered_ranges


def segment_scan(ranges: np.ndarray, angle_min: float, angle_max: float, 
                angle_increment: float, sector_count: int) -> List[float]:
    """
    Segment LiDAR scan into sectors and compute minimum distance in each sector.
    
    Args:
        ranges: Array of range measurements
        angle_min: Minimum scan angle (radians)
        angle_max: Maximum scan angle (radians)
        angle_increment: Angle increment between measurements (radians)
        sector_count: Number of sectors to divide the scan into
        
    Returns:
        List of minimum distances for each sector
    """
    if sector_count <= 0:
        raise ValueError("sector_count must be positive")
    
    # Calculate sector angle width
    total_angle = angle_max - angle_min
    sector_width = total_angle / sector_count
    
    # Initialize sector minimum distances
    sector_mins = [float('inf')] * sector_count
    
    # Process each range measurement
    for i, range_val in enumerate(ranges):
        # Calculate angle for this measurement
        angle = angle_min + i * angle_increment
        
        # Determine which sector this angle belongs to
        sector_idx = int((angle - angle_min) / sector_width)
        sector_idx = max(0, min(sector_count - 1, sector_idx))
        
        # Update minimum distance for this sector
        sector_mins[sector_idx] = min(sector_mins[sector_idx], range_val)
    
    # Replace inf with a large value (no obstacles detected)
    sector_mins = [val if val != float('inf') else 1000.0 for val in sector_mins]
    
    return sector_mins


def find_free_sectors(sector_distances: List[float], safe_distance: float) -> List[bool]:
    """
    Identify which sectors are free of obstacles.
    
    Args:
        sector_distances: List of minimum distances for each sector
        safe_distance: Minimum safe distance threshold
        
    Returns:
        List of booleans indicating free sectors
    """
    return [dist >= safe_distance for dist in sector_distances]


def calculate_sector_angles(sector_count: int, angle_min: float, angle_max: float) -> List[float]:
    """
    Calculate the center angle for each sector.
    
    Args:
        sector_count: Number of sectors
        angle_min: Minimum scan angle (radians)
        angle_max: Maximum scan angle (radians)
        
    Returns:
        List of center angles for each sector
    """
    total_angle = angle_max - angle_min
    sector_width = total_angle / sector_count
    
    sector_angles = []
    for i in range(sector_count):
        center_angle = angle_min + (i + 0.5) * sector_width
        sector_angles.append(center_angle)
    
    return sector_angles


def vfh_avoidance(sector_distances: List[float], sector_angles: List[float],
                 safe_distance: float, current_heading: float = 0.0) -> Tuple[float, int]:
    """
    Implement VFH-style avoidance algorithm to choose steering direction.
    
    Args:
        sector_distances: List of minimum distances for each sector
        sector_angles: List of center angles for each sector
        safe_distance: Minimum safe distance threshold
        current_heading: Current heading direction (radians)
        
    Returns:
        Tuple of (chosen_angle, chosen_sector_index)
    """
    if not sector_distances or not sector_angles:
        return current_heading, 0
    
    # Find free sectors
    free_sectors = find_free_sectors(sector_distances, safe_distance)
    
    if not any(free_sectors):
        # No free sectors, choose the sector with maximum distance
        best_sector = np.argmax(sector_distances)
        return sector_angles[best_sector], best_sector
    
    # Calculate clearance for each free sector
    sector_clearances = []
    for i, is_free in enumerate(free_sectors):
        if is_free:
            # Clearance is the distance minus safe distance
            clearance = sector_distances[i] - safe_distance
            sector_clearances.append((clearance, i))
        else:
            sector_clearances.append((0.0, i))
    
    # Sort by clearance (descending) and then by proximity to current heading
    def sort_key(item):
        clearance, sector_idx = item
        angle_diff = abs(sector_angles[sector_idx] - current_heading)
        # Normalize angle difference to [0, 2π]
        angle_diff = min(angle_diff, 2 * math.pi - angle_diff)
        # Prioritize clearance, then minimize angle difference
        return (-clearance, angle_diff)
    
    sector_clearances.sort(key=sort_key)
    
    # Choose the best sector
    best_clearance, best_sector_idx = sector_clearances[0]
    chosen_angle = sector_angles[best_sector_idx]
    
    return chosen_angle, best_sector_idx


def calculate_velocity_commands(chosen_angle: float, current_heading: float,
                              front_clearance: float, base_speed: float,
                              max_yaw_rate: float, stop_distance: float,
                              min_clearance: float) -> Tuple[float, float, float]:
    """
    Calculate velocity commands based on chosen direction and clearance.
    
    Args:
        chosen_angle: Chosen steering angle (radians)
        current_heading: Current heading (radians)
        front_clearance: Clearance in front direction
        base_speed: Base forward speed
        max_yaw_rate: Maximum yaw rate
        stop_distance: Distance threshold for stopping
        min_clearance: Minimum clearance in any direction
        
    Returns:
        Tuple of (vx, vy, yaw_rate)
    """
    # Check for emergency stop
    if min_clearance < stop_distance:
        return 0.0, 0.0, 0.0
    
    # Calculate angle difference for steering
    angle_diff = chosen_angle - current_heading
    
    # Normalize angle difference to [-π, π]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    # Calculate yaw rate (proportional to angle difference)
    yaw_rate = np.clip(angle_diff * 2.0, -max_yaw_rate, max_yaw_rate)
    
    # Scale forward speed based on front clearance
    if front_clearance > stop_distance:
        # Scale speed based on clearance (more clearance = faster)
        speed_scale = min(1.0, (front_clearance - stop_distance) / (2.0 * stop_distance))
        vx = base_speed * speed_scale
    else:
        vx = 0.0
    
    # No lateral movement for now (vy = 0)
    vy = 0.0
    
    return vx, vy, yaw_rate


def get_front_sector_index(sector_count: int) -> int:
    """
    Get the index of the front sector (assuming 0° is front).
    
    Args:
        sector_count: Number of sectors
        
    Returns:
        Index of the front sector
    """
    return sector_count // 2


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-π, π] range.
    
    Args:
        angle: Input angle in radians
        
    Returns:
        Normalized angle in [-π, π] range
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_min_distance_in_sector(ranges: np.ndarray, angle_min: float,
                                   angle_increment: float, sector_start_angle: float,
                                   sector_end_angle: float) -> float:
    """
    Calculate minimum distance within a specific angular sector.
    
    Args:
        ranges: Array of range measurements
        angle_min: Minimum scan angle (radians)
        angle_increment: Angle increment between measurements (radians)
        sector_start_angle: Start angle of the sector (radians)
        sector_end_angle: End angle of the sector (radians)
        
    Returns:
        Minimum distance in the sector
    """
    min_dist = float('inf')
    
    for i, range_val in enumerate(ranges):
        angle = angle_min + i * angle_increment
        
        # Check if this measurement is within the sector
        if sector_start_angle <= angle <= sector_end_angle:
            min_dist = min(min_dist, range_val)
    
    return min_dist if min_dist != float('inf') else 1000.0


def create_histogram_data(sector_distances: List[float], sector_angles: List[float],
                         safe_distance: float) -> Dict[str, Any]:
    """
    Create histogram data for visualization and debugging.
    
    Args:
        sector_distances: List of minimum distances for each sector
        sector_angles: List of center angles for each sector
        safe_distance: Safe distance threshold
        
    Returns:
        Dictionary containing histogram data
    """
    free_sectors = find_free_sectors(sector_distances, safe_distance)
    
    return {
        'sector_angles': sector_angles,
        'sector_distances': sector_distances,
        'free_sectors': free_sectors,
        'safe_distance': safe_distance,
        'min_distance': min(sector_distances),
        'max_distance': max(sector_distances),
        'free_sector_count': sum(free_sectors),
        'total_sectors': len(sector_distances)
    }


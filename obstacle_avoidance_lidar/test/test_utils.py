"""
Unit tests for obstacle avoidance utility functions.

This module contains comprehensive tests for all utility functions
including filtering, sectoring, and VFH avoidance logic.
"""

import pytest
import numpy as np
import math
from typing import List, Tuple

# Import the functions to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from utils import (
    clean_scan, moving_median_filter, segment_scan, find_free_sectors,
    calculate_sector_angles, vfh_avoidance, calculate_velocity_commands,
    get_front_sector_index, normalize_angle, calculate_min_distance_in_sector,
    create_histogram_data
)


class TestCleanScan:
    """Test cases for scan cleaning function."""
    
    def test_clean_scan_normal_data(self):
        """Test cleaning with normal range data."""
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        range_min, range_max = 0.1, 10.0
        
        result = clean_scan(ranges, range_min, range_max)
        
        assert len(result) == len(ranges)
        assert np.array_equal(result, np.array(ranges))
    
    def test_clean_scan_with_nan(self):
        """Test cleaning with NaN values."""
        ranges = [1.0, float('nan'), 3.0, float('inf'), 5.0]
        range_min, range_max = 0.1, 10.0
        
        result = clean_scan(ranges, range_min, range_max)
        
        assert len(result) == len(ranges)
        assert result[0] == 1.0
        assert result[1] == range_max  # NaN replaced with range_max
        assert result[2] == 3.0
        assert result[3] == range_max  # inf replaced with range_max
        assert result[4] == 5.0
    
    def test_clean_scan_clipping(self):
        """Test clipping to valid range."""
        ranges = [0.05, 1.0, 5.0, 15.0, 25.0]
        range_min, range_max = 0.1, 10.0
        
        result = clean_scan(ranges, range_min, range_max)
        
        assert result[0] == range_min  # Clipped to range_min
        assert result[1] == 1.0
        assert result[2] == 5.0
        assert result[3] == 10.0  # Clipped to range_max
        assert result[4] == range_max  # Clipped to range_max


class TestMovingMedianFilter:
    """Test cases for moving median filter."""
    
    def test_moving_median_filter_basic(self):
        """Test basic median filtering."""
        ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        window_size = 3
        
        result = moving_median_filter(ranges, window_size)
        
        assert len(result) == len(ranges)
        # Check that filtering was applied (values should be different from input)
        assert not np.array_equal(result, ranges)
    
    def test_moving_median_filter_window_size_one(self):
        """Test median filter with window size 1 (no filtering)."""
        ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        window_size = 1
        
        result = moving_median_filter(ranges, window_size)
        
        assert np.array_equal(result, ranges)
    
    def test_moving_median_filter_noise_reduction(self):
        """Test that median filter reduces noise."""
        # Create signal with noise
        clean_signal = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0])
        noisy_signal = clean_signal + np.array([0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0])
        window_size = 3
        
        result = moving_median_filter(noisy_signal, window_size)
        
        # Result should be closer to clean signal
        assert np.allclose(result, clean_signal, atol=0.1)


class TestSegmentScan:
    """Test cases for scan segmentation."""
    
    def test_segment_scan_basic(self):
        """Test basic scan segmentation."""
        ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = math.pi / 4
        sector_count = 4
        
        result = segment_scan(ranges, angle_min, angle_max, angle_increment, sector_count)
        
        assert len(result) == sector_count
        assert all(isinstance(d, float) for d in result)
        assert all(d > 0 for d in result)
    
    def test_segment_scan_single_sector(self):
        """Test segmentation with single sector."""
        ranges = np.array([1.0, 2.0, 3.0])
        angle_min = 0.0
        angle_max = math.pi
        angle_increment = math.pi / 3
        sector_count = 1
        
        result = segment_scan(ranges, angle_min, angle_max, angle_increment, sector_count)
        
        assert len(result) == 1
        assert result[0] == 1.0  # Minimum distance
    
    def test_segment_scan_invalid_sector_count(self):
        """Test segmentation with invalid sector count."""
        ranges = np.array([1.0, 2.0, 3.0])
        angle_min = 0.0
        angle_max = math.pi
        angle_increment = math.pi / 3
        sector_count = 0
        
        with pytest.raises(ValueError):
            segment_scan(ranges, angle_min, angle_max, angle_increment, sector_count)


class TestFindFreeSectors:
    """Test cases for finding free sectors."""
    
    def test_find_free_sectors_basic(self):
        """Test basic free sector detection."""
        sector_distances = [1.0, 2.5, 1.5, 3.0, 0.8]
        safe_distance = 2.0
        
        result = find_free_sectors(sector_distances, safe_distance)
        
        expected = [False, True, False, True, False]
        assert result == expected
    
    def test_find_free_sectors_all_free(self):
        """Test when all sectors are free."""
        sector_distances = [3.0, 4.0, 5.0, 6.0]
        safe_distance = 2.0
        
        result = find_free_sectors(sector_distances, safe_distance)
        
        assert all(result)
    
    def test_find_free_sectors_all_blocked(self):
        """Test when all sectors are blocked."""
        sector_distances = [0.5, 1.0, 1.5, 0.8]
        safe_distance = 2.0
        
        result = find_free_sectors(sector_distances, safe_distance)
        
        assert not any(result)


class TestCalculateSectorAngles:
    """Test cases for sector angle calculation."""
    
    def test_calculate_sector_angles_basic(self):
        """Test basic sector angle calculation."""
        sector_count = 4
        angle_min = -math.pi
        angle_max = math.pi
        
        result = calculate_sector_angles(sector_count, angle_min, angle_max)
        
        assert len(result) == sector_count
        assert all(-math.pi <= angle <= math.pi for angle in result)
    
    def test_calculate_sector_angles_single_sector(self):
        """Test angle calculation with single sector."""
        sector_count = 1
        angle_min = 0.0
        angle_max = math.pi
        
        result = calculate_sector_angles(sector_count, angle_min, angle_max)
        
        assert len(result) == 1
        assert abs(result[0] - math.pi/2) < 0.01  # Center of [0, π]


class TestVfhAvoidance:
    """Test cases for VFH avoidance algorithm."""
    
    def test_vfh_avoidance_basic(self):
        """Test basic VFH avoidance."""
        sector_distances = [1.0, 2.5, 1.5, 3.0, 0.8]
        sector_angles = [-math.pi, -math.pi/2, 0.0, math.pi/2, math.pi]
        safe_distance = 2.0
        current_heading = 0.0
        
        chosen_angle, chosen_sector = vfh_avoidance(
            sector_distances, sector_angles, safe_distance, current_heading
        )
        
        assert isinstance(chosen_angle, float)
        assert isinstance(chosen_sector, int)
        assert 0 <= chosen_sector < len(sector_distances)
    
    def test_vfh_avoidance_no_free_sectors(self):
        """Test VFH when no sectors are free."""
        sector_distances = [0.5, 1.0, 1.5, 0.8, 1.2]
        sector_angles = [-math.pi, -math.pi/2, 0.0, math.pi/2, math.pi]
        safe_distance = 2.0
        current_heading = 0.0
        
        chosen_angle, chosen_sector = vfh_avoidance(
            sector_distances, sector_angles, safe_distance, current_heading
        )
        
        # Should choose sector with maximum distance
        expected_sector = sector_distances.index(max(sector_distances))
        assert chosen_sector == expected_sector
    
    def test_vfh_avoidance_empty_input(self):
        """Test VFH with empty input."""
        sector_distances = []
        sector_angles = []
        safe_distance = 2.0
        current_heading = 0.0
        
        chosen_angle, chosen_sector = vfh_avoidance(
            sector_distances, sector_angles, safe_distance, current_heading
        )
        
        assert chosen_angle == current_heading
        assert chosen_sector == 0


class TestCalculateVelocityCommands:
    """Test cases for velocity command calculation."""
    
    def test_calculate_velocity_commands_clear_path(self):
        """Test velocity calculation with clear path."""
        chosen_angle = 0.0
        current_heading = 0.0
        front_clearance = 5.0
        base_speed = 1.0
        max_yaw_rate = 1.0
        stop_distance = 0.5
        min_clearance = 3.0
        
        vx, vy, yaw_rate = calculate_velocity_commands(
            chosen_angle, current_heading, front_clearance,
            base_speed, max_yaw_rate, stop_distance, min_clearance
        )
        
        assert vx > 0  # Should move forward
        assert vy == 0.0  # No lateral movement
        assert yaw_rate == 0.0  # No turning needed
    
    def test_calculate_velocity_commands_emergency_stop(self):
        """Test velocity calculation with emergency stop."""
        chosen_angle = 0.0
        current_heading = 0.0
        front_clearance = 5.0
        base_speed = 1.0
        max_yaw_rate = 1.0
        stop_distance = 0.5
        min_clearance = 0.3  # Below stop distance
        
        vx, vy, yaw_rate = calculate_velocity_commands(
            chosen_angle, current_heading, front_clearance,
            base_speed, max_yaw_rate, stop_distance, min_clearance
        )
        
        assert vx == 0.0  # Emergency stop
        assert vy == 0.0
        assert yaw_rate == 0.0
    
    def test_calculate_velocity_commands_turning(self):
        """Test velocity calculation with turning."""
        chosen_angle = math.pi/4
        current_heading = 0.0
        front_clearance = 5.0
        base_speed = 1.0
        max_yaw_rate = 1.0
        stop_distance = 0.5
        min_clearance = 3.0
        
        vx, vy, yaw_rate = calculate_velocity_commands(
            chosen_angle, current_heading, front_clearance,
            base_speed, max_yaw_rate, stop_distance, min_clearance
        )
        
        assert vx > 0  # Should move forward
        assert vy == 0.0  # No lateral movement
        assert yaw_rate > 0  # Should turn right


class TestGetFrontSectorIndex:
    """Test cases for front sector index calculation."""
    
    def test_get_front_sector_index_even(self):
        """Test front sector index with even sector count."""
        sector_count = 8
        result = get_front_sector_index(sector_count)
        assert result == 4  # Middle of 8 sectors (0-7)
    
    def test_get_front_sector_index_odd(self):
        """Test front sector index with odd sector count."""
        sector_count = 7
        result = get_front_sector_index(sector_count)
        assert result == 3  # Middle of 7 sectors (0-6)


class TestNormalizeAngle:
    """Test cases for angle normalization."""
    
    def test_normalize_angle_in_range(self):
        """Test angle already in range."""
        angle = math.pi/4
        result = normalize_angle(angle)
        assert result == math.pi/4
    
    def test_normalize_angle_positive_out_of_range(self):
        """Test positive angle out of range."""
        angle = 3*math.pi/2
        result = normalize_angle(angle)
        assert abs(result - (-math.pi/2)) < 0.01
    
    def test_normalize_angle_negative_out_of_range(self):
        """Test negative angle out of range."""
        angle = -3*math.pi/2
        result = normalize_angle(angle)
        assert abs(result - math.pi/2) < 0.01


class TestCalculateMinDistanceInSector:
    """Test cases for minimum distance calculation in sector."""
    
    def test_calculate_min_distance_in_sector_basic(self):
        """Test basic minimum distance calculation."""
        ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0])
        angle_min = 0.0
        angle_increment = math.pi/4
        sector_start_angle = 0.0
        sector_end_angle = math.pi/2
        
        result = calculate_min_distance_in_sector(
            ranges, angle_min, angle_increment, sector_start_angle, sector_end_angle
        )
        
        assert result == 1.0  # Minimum in the sector
    
    def test_calculate_min_distance_in_sector_no_measurements(self):
        """Test when no measurements are in sector."""
        ranges = np.array([1.0, 2.0, 3.0])
        angle_min = 0.0
        angle_increment = math.pi/4
        sector_start_angle = math.pi
        sector_end_angle = 2*math.pi
        
        result = calculate_min_distance_in_sector(
            ranges, angle_min, angle_increment, sector_start_angle, sector_end_angle
        )
        
        assert result == 1000.0  # Default large value


class TestCreateHistogramData:
    """Test cases for histogram data creation."""
    
    def test_create_histogram_data_basic(self):
        """Test basic histogram data creation."""
        sector_distances = [1.0, 2.5, 1.5, 3.0, 0.8]
        sector_angles = [-math.pi, -math.pi/2, 0.0, math.pi/2, math.pi]
        safe_distance = 2.0
        
        result = create_histogram_data(sector_distances, sector_angles, safe_distance)
        
        assert 'sector_angles' in result
        assert 'sector_distances' in result
        assert 'free_sectors' in result
        assert 'safe_distance' in result
        assert 'min_distance' in result
        assert 'max_distance' in result
        assert 'free_sector_count' in result
        assert 'total_sectors' in result
        
        assert result['sector_angles'] == sector_angles
        assert result['sector_distances'] == sector_distances
        assert result['safe_distance'] == safe_distance
        assert result['min_distance'] == min(sector_distances)
        assert result['max_distance'] == max(sector_distances)
        assert result['total_sectors'] == len(sector_distances)


if __name__ == '__main__':
    pytest.main([__file__])


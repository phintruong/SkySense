"""
LiDAR Simulation Module
Generates fake 360-degree LiDAR scan data for testing obstacle detection logic.
"""

import random
import math


def generate_scan(obstacles=None):
    """
    Generate a simulated 360-degree LiDAR scan.
    
    Args:
        obstacles: Optional list of tuples (angle, distance) for specific obstacles.
                  If None, generates random obstacles.
    
    Returns:
        List of tuples: [(angle_in_degrees, distance_in_meters), ...]
        Angles range from 0 to 359 degrees.
        Distances range from 0.2m to 3.0m (realistic LiDAR range).
    """
    scan_data = []
    
    # Generate scan points for all 360 degrees
    for angle in range(360):
        # Base distance with some variation
        base_distance = random.uniform(1.5, 3.0)
        
        # Add some closer obstacles randomly
        if random.random() < 0.05:  # 5% chance of close obstacle
            distance = random.uniform(0.2, 0.8)
        else:
            distance = base_distance
        
        scan_data.append((angle, round(distance, 2)))
    
    # If specific obstacles are provided, override those angles
    if obstacles:
        scan_dict = dict(scan_data)
        for angle, distance in obstacles:
            scan_dict[angle % 360] = distance
        scan_data = [(angle, scan_dict[angle]) for angle in range(360)]
    
    return scan_data


def generate_scan_with_pattern():
    """
    Generate a scan with a predictable pattern for testing.
    Includes obstacles in front, left, and right regions.
    """
    obstacles = [
        (0, 0.4),      # Front obstacle
        (45, 0.6),     # Front-left obstacle
        (90, 0.5),     # Left obstacle
        (270, 0.7),    # Right obstacle
        (180, 2.0),    # Back (far, should be safe)
    ]
    return generate_scan(obstacles)


if __name__ == "__main__":
    # Test the simulator
    print("Testing LiDAR Simulator")
    print("=" * 40)
    
    scan = generate_scan()
    print(f"Generated {len(scan)} scan points")
    print("\nFirst 10 points:")
    for angle, distance in scan[:10]:
        print(f"  Angle: {angle:3d}°, Distance: {distance:.2f}m")
    
    print("\nScan with pattern:")
    pattern_scan = generate_scan_with_pattern()
    for angle, distance in pattern_scan:
        if distance < 1.0:  # Show only close obstacles
            print(f"  Angle: {angle:3d}°, Distance: {distance:.2f}m")


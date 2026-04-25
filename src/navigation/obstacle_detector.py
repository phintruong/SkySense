"""
Obstacle Detection Logic Module
Processes LiDAR scan data and determines navigation actions.
Focuses on forward-facing cone (±80 degrees) for directional navigation.
Accounts for drone tilt by rotating the forward cone based on roll angle.
"""

import math


# Danger radius in meters
DANGER_RADIUS = 0.5

# Forward cone angle range (±80 degrees from 0°)
FORWARD_CONE_HALF_ANGLE = 80


def polar_to_cartesian(angle_degrees, distance):
    """
    Convert polar coordinates to Cartesian coordinates.
    
    Args:
        angle_degrees: Angle in degrees (0-359)
        distance: Distance in meters
    
    Returns:
        Tuple (x, y) in meters
    """
    angle_radians = math.radians(angle_degrees)
    x = distance * math.cos(angle_radians)
    y = distance * math.sin(angle_radians)
    return (x, y)


def is_in_forward_cone(angle_degrees, tilt_angle=0):
    """
    Check if an angle is within the forward-facing cone (±80 degrees from forward direction).
    Accounts for drone tilt by rotating the forward cone.
    
    Forward cone: ±80 degrees from the forward direction (adjusted by tilt_angle)
    
    Args:
        angle_degrees: Angle in degrees (0-359)
        tilt_angle: Tilt/roll angle in degrees (positive = left roll, negative = right roll)
                   When drone tilts left, forward cone rotates right relative to horizontal plane
    
    Returns:
        Boolean: True if angle is in forward cone
    """
    angle = angle_degrees % 360
    
    # Adjust angle relative to tilted forward direction
    # When drone tilts left (positive tilt), the forward direction shifts right
    adjusted_angle = (angle - tilt_angle) % 360
    
    # Forward cone spans: ±80 degrees from adjusted forward direction
    # Check if adjusted_angle is within ±80 degrees of 0°
    if adjusted_angle <= FORWARD_CONE_HALF_ANGLE:
        return True
    elif adjusted_angle >= (360 - FORWARD_CONE_HALF_ANGLE):
        return True
    return False


def get_forward_region(angle_degrees, tilt_angle=0):
    """
    Classify angle within the forward cone as center, left-front, or right-front.
    Accounts for drone tilt.
    
    Args:
        angle_degrees: Angle in degrees (0-359), assumed to be in forward cone
        tilt_angle: Tilt/roll angle in degrees (positive = left roll)
    
    Returns:
        String: "center", "left-front", or "right-front"
    """
    angle = angle_degrees % 360
    
    # Adjust angle relative to tilted forward direction
    adjusted_angle = (angle - tilt_angle) % 360
    
    # Center region: very close to forward direction (within ±20 degrees)
    if adjusted_angle <= 20 or adjusted_angle >= 340:
        return "center"
    # Left-front: 20-80 degrees (positive angles from forward)
    elif 20 < adjusted_angle <= 80:
        return "left-front"
    # Right-front: 280-340 degrees (wraps around, negative angles from forward)
    else:  # 280 <= adjusted_angle < 340
        return "right-front"


def get_forward_cone_obstacles(scan_data, danger_radius=DANGER_RADIUS, tilt_angle=0):
    """
    Extract obstacles from the forward-facing cone (±80 degrees).
    Accounts for drone tilt by rotating the forward cone.
    
    Args:
        scan_data: List of tuples [(angle, distance), ...]
        danger_radius: Danger radius in meters (default: 0.5m)
        tilt_angle: Tilt/roll angle in degrees (positive = left roll, negative = right roll)
    
    Returns:
        List of tuples: [(angle, distance, region), ...] for obstacles in forward cone within danger zone
    """
    obstacles = []
    
    for angle, distance in scan_data:
        # Only consider obstacles in forward cone (accounting for tilt)
        if is_in_forward_cone(angle, tilt_angle) and distance < danger_radius:
            region = get_forward_region(angle, tilt_angle)
            obstacles.append((angle, distance, region))
    
    return obstacles


def determine_action(forward_obstacles):
    """
    Determine navigation action based on obstacles in forward cone.
    
    Decision logic:
    - No obstacles -> MOVE_FORWARD
    - Center blocked -> Compare left vs right, choose clearer side (LEFT or RIGHT)
    - Both sides have obstacles -> Compare distances, choose safer side
    - Very close obstacles everywhere -> HOVER
    - Only one side blocked -> Turn to clear side
    
    Args:
        forward_obstacles: List of tuples [(angle, distance, region), ...]
    
    Returns:
        Tuple: (action_string, obstacle_info)
               action_string: "MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "MOVE_BACKWARD", or "HOVER"
               obstacle_info: List of obstacle details for logging
    """
    if not forward_obstacles:
        return ("MOVE_FORWARD", [])
    
    obstacle_info = []
    center_obstacles = []
    left_front_obstacles = []
    right_front_obstacles = []
    
    for angle, distance, region in forward_obstacles:
        obstacle_info.append({
            'angle': angle,
            'distance': distance,
            'region': region
        })
        
        if region == "center":
            center_obstacles.append((angle, distance))
        elif region == "left-front":
            left_front_obstacles.append((angle, distance))
        elif region == "right-front":
            right_front_obstacles.append((angle, distance))
    
    # If center is blocked, we need to decide left or right
    if center_obstacles:
        closest_center = min(center_obstacles, key=lambda x: x[1])
        
        # If center obstacle is very close, check if we should hover
        if closest_center[1] < 0.3:
            # Check if both sides also have very close obstacles
            left_closest = min(left_front_obstacles, key=lambda x: x[1]) if left_front_obstacles else None
            right_closest = min(right_front_obstacles, key=lambda x: x[1]) if right_front_obstacles else None
            
            if left_closest and left_closest[1] < 0.3 and right_closest and right_closest[1] < 0.3:
                return ("HOVER", obstacle_info)
        
        # Compare left vs right to decide which direction
        if not left_front_obstacles:
            return ("TURN_LEFT", obstacle_info)
        elif not right_front_obstacles:
            return ("TURN_RIGHT", obstacle_info)
        else:
            # Both sides have obstacles, choose the side with farther obstacles
            left_min_dist = min(left_front_obstacles, key=lambda x: x[1])[1]
            right_min_dist = min(right_front_obstacles, key=lambda x: x[1])[1]
            
            if left_min_dist > right_min_dist:
                return ("TURN_LEFT", obstacle_info)
            elif right_min_dist > left_min_dist:
                return ("TURN_RIGHT", obstacle_info)
            else:
                # Equal distances, default to left
                return ("TURN_LEFT", obstacle_info)
    
    # No center obstacles, check sides
    if left_front_obstacles and not right_front_obstacles:
        # Left side blocked, go right
        return ("TURN_RIGHT", obstacle_info)
    elif right_front_obstacles and not left_front_obstacles:
        # Right side blocked, go left
        return ("TURN_LEFT", obstacle_info)
    elif left_front_obstacles and right_front_obstacles:
        # Both sides have obstacles but center is clear
        # Compare distances to choose safer path
        left_min_dist = min(left_front_obstacles, key=lambda x: x[1])[1]
        right_min_dist = min(right_front_obstacles, key=lambda x: x[1])[1]
        
        # If both are very close, go backward
        if left_min_dist < 0.3 and right_min_dist < 0.3:
            return ("MOVE_BACKWARD", obstacle_info)
        
        # Choose the side with farther obstacles
        if left_min_dist > right_min_dist:
            return ("TURN_LEFT", obstacle_info)
        else:
            return ("TURN_RIGHT", obstacle_info)
    
    # Shouldn't reach here, but default to forward
    return ("MOVE_FORWARD", obstacle_info)


def process_scan(scan_data, danger_radius=DANGER_RADIUS, tilt_angle=0):
    """
    Main function to process a LiDAR scan and determine action.
    Only analyzes obstacles in the forward-facing cone (±80 degrees).
    Accounts for drone tilt by rotating the forward cone.
    
    Args:
        scan_data: List of tuples [(angle, distance), ...]
        danger_radius: Danger radius in meters
        tilt_angle: Tilt/roll angle in degrees (positive = left roll, negative = right roll)
                   This simulates MPU/IMU data. Use 0 for no tilt.
    
    Returns:
        Tuple: (action, obstacles_info)
    """
    forward_obstacles = get_forward_cone_obstacles(scan_data, danger_radius, tilt_angle)
    action, obstacle_info = determine_action(forward_obstacles)
    return action, obstacle_info


if __name__ == "__main__":
    # Test the logic module
    print("Testing Obstacle Detection Logic (Forward Cone)")
    print("=" * 50)
    print(f"Forward cone: ±{FORWARD_CONE_HALF_ANGLE}° from 0°")
    print(f"Danger radius: {DANGER_RADIUS}m\n")
    
    # Test scan with obstacles
    test_scan = [
        (0, 0.4),      # Center obstacle
        (45, 0.6),     # Left-front (in cone)
        (90, 0.5),     # Left (out of cone, should be ignored)
        (270, 0.4),    # Right-front (in cone)
        (180, 2.0),    # Back (out of cone, should be ignored)
        (60, 0.3),     # Left-front (in cone, close)
        (300, 0.8),    # Right-front (in cone, but > 0.5m, should be ignored)
        (350, 0.35),   # Center-right (in cone)
    ]
    
    # Test without tilt
    forward_obstacles = get_forward_cone_obstacles(test_scan)
    print(f"Detected {len(forward_obstacles)} obstacles in forward cone (within {DANGER_RADIUS}m, no tilt):")
    for angle, distance, region in forward_obstacles:
        print(f"  Angle: {angle:3d}°, Distance: {distance:.2f}m, Region: {region}")
    
    action, info = determine_action(forward_obstacles)
    print(f"\nDecision: {action}")
    
    # Test with tilt
    print("\n" + "=" * 50)
    tilt_angle = 20  # 20° left roll
    print(f"Testing with {tilt_angle}° left roll:")
    forward_obstacles_tilted = get_forward_cone_obstacles(test_scan, tilt_angle=tilt_angle)
    print(f"Detected {len(forward_obstacles_tilted)} obstacles in forward cone (within {DANGER_RADIUS}m, with tilt):")
    for angle, distance, region in forward_obstacles_tilted:
        print(f"  Angle: {angle:3d}°, Distance: {distance:.2f}m, Region: {region}")
    
    action_tilted, info_tilted = determine_action(forward_obstacles_tilted)
    print(f"\nDecision: {action_tilted}")


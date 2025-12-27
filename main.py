"""
Main Simulation Program
Runs the LiDAR obstacle detection simulation loop.
"""

import time
from lidar_sim import generate_scan, generate_scan_with_pattern
from logic import process_scan, DANGER_RADIUS, FORWARD_CONE_HALF_ANGLE


def format_action_message(action):
    """Convert action code to readable message."""
    action_messages = {
        "TURN_LEFT": "Turn left",
        "TURN_RIGHT": "Turn right",
        "MOVE_FORWARD": "Move forward",
        "MOVE_BACKWARD": "Move backward",
        "HOVER": "Hover in place"
    }
    return action_messages.get(action, action)


def print_scan_summary(scan_data, action, obstacle_info):
    """Print a readable summary of the scan and decision."""
    print(f"[SCAN] {len(scan_data)} points received (analyzing forward cone: ±{FORWARD_CONE_HALF_ANGLE}°)")
    
    if obstacle_info:
        print(f"[INFO] {len(obstacle_info)} obstacle(s) in forward cone within danger zone ({DANGER_RADIUS}m):")
        for obs in obstacle_info:
            angle = obs['angle']
            distance = obs['distance']
            region = obs['region']
            print(f"       - {distance:.2f}m at {angle}° ({region})")
        
        # Find closest obstacle
        closest = min(obstacle_info, key=lambda x: x['distance'])
        print(f"[INFO] Closest obstacle: {closest['distance']:.2f}m at {closest['angle']}° ({closest['region']})")
    else:
        print(f"[INFO] Forward path clear - no obstacles within {DANGER_RADIUS}m in forward cone")
    
    print(f"[ACTION] {format_action_message(action)}")
    print("-" * 60)


def main():
    """Main simulation loop."""
    print("LiDAR Obstacle Detection Simulation")
    print("=" * 60)
    print(f"Danger radius: {DANGER_RADIUS}m")
    print(f"Forward cone: ±{FORWARD_CONE_HALF_ANGLE}° (analyzing direction of movement)")
    print("Press Ctrl+C to stop\n")
    
    cycle = 0
    
    try:
        while True:
            cycle += 1
            print(f"\n--- Cycle {cycle} ---")
            
            # Generate LiDAR scan
            # Use generate_scan() for random obstacles
            # Use generate_scan_with_pattern() for predictable test pattern
            scan_data = generate_scan()
            # scan_data = generate_scan_with_pattern()  # Uncomment for test pattern
            
            # Process scan and determine action
            action, obstacle_info = process_scan(scan_data)
            
            # Print results
            print_scan_summary(scan_data, action, obstacle_info)
            
            # Small delay between cycles
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()


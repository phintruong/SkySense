"""
Main Program - Real Hardware Mode
Runs obstacle detection using actual RPLIDAR sensor data.
"""

import time
import signal
import sys
from lidar import RPLidarReader
from core import process_scan, DANGER_RADIUS, FORWARD_CONE_HALF_ANGLE


# Global reference for signal handler
_lidar = None


def signal_handler(sig, frame):
    """Handle interrupt signal for graceful shutdown."""
    print("\nShutting down...")
    if _lidar:
        _lidar.disconnect()
    sys.exit(0)


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
    print(f"[SCAN] {len(scan_data)} points received")
    
    if obstacle_info:
        print(f"[INFO] {len(obstacle_info)} obstacle(s) in forward cone within {DANGER_RADIUS}m:")
        for obs in obstacle_info:
            angle = obs['angle']
            distance = obs['distance']
            region = obs['region']
            print(f"       - {distance:.2f}m at {angle:.1f}deg ({region})")
        
        closest = min(obstacle_info, key=lambda x: x['distance'])
        print(f"[INFO] Closest: {closest['distance']:.2f}m at {closest['angle']:.1f}deg")
    else:
        print(f"[INFO] Forward path clear")
    
    print(f"[ACTION] {format_action_message(action)}")
    print("-" * 50)


def main():
    """Main loop with real RPLIDAR hardware."""
    global _lidar
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("SkySense Obstacle Detection")
    print("=" * 50)
    print(f"Danger radius: {DANGER_RADIUS}m")
    print(f"Forward cone: +/-{FORWARD_CONE_HALF_ANGLE}deg")
    print("Using: RPLIDAR A1 Hardware")
    print("Press Ctrl+C to stop\n")
    
    # Initialize LiDAR
    lidar = RPLidarReader()
    _lidar = lidar
    
    if not lidar.connect():
        print("ERROR: Failed to connect to RPLIDAR")
        print("Check USB connection and try again")
        sys.exit(1)
    
    if not lidar.start():
        print("ERROR: Failed to start RPLIDAR motor")
        lidar.disconnect()
        sys.exit(1)
    
    print("RPLIDAR connected and scanning...\n")
    
    try:
        cycle = 0
        for scan_data in lidar.iter_scans():
            cycle += 1
            print(f"\n--- Cycle {cycle} ---")
            
            # Process scan (tilt_angle=0 for now, integrate IMU later)
            action, obstacle_info = process_scan(scan_data, tilt_angle=0)
            
            print_scan_summary(scan_data, action, obstacle_info)
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        lidar.disconnect()
        print("Shutdown complete")


if __name__ == "__main__":
    main()

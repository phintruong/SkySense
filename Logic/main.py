"""
Main Program
Runs the LiDAR obstacle detection loop with real hardware.
"""

import time
from hardware import RPLidarReader
from core import process_scan, DANGER_RADIUS, FORWARD_CONE_HALF_ANGLE


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


def print_scan_summary(scan_data, action, obstacle_info, tilt_angle=0):
    """Print a readable summary of the scan and decision."""
    print(f"[SCAN] {len(scan_data)} points received (analyzing forward cone: ±{FORWARD_CONE_HALF_ANGLE}°)")
    if tilt_angle != 0:
        tilt_direction = "left" if tilt_angle > 0 else "right"
        print(f"[TILT] Drone tilted {abs(tilt_angle):.1f}° {tilt_direction} (forward cone adjusted)")

    if obstacle_info:
        print(f"[INFO] {len(obstacle_info)} obstacle(s) in forward cone within danger zone ({DANGER_RADIUS}m):")
        for obs in obstacle_info:
            angle = obs['angle']
            distance = obs['distance']
            region = obs['region']
            print(f"       - {distance:.2f}m at {angle}° ({region})")

        closest = min(obstacle_info, key=lambda x: x['distance'])
        print(f"[INFO] Closest obstacle: {closest['distance']:.2f}m at {closest['angle']}° ({closest['region']})")
    else:
        print(f"[INFO] Forward path clear - no obstacles within {DANGER_RADIUS}m in forward cone")

    print(f"[ACTION] {format_action_message(action)}")
    print("-" * 60)


def main():
    """Main loop using real RPLIDAR hardware."""
    print("SkySense — LiDAR Obstacle Detection")
    print("=" * 60)
    print(f"Danger radius: {DANGER_RADIUS}m")
    print(f"Forward cone: ±{FORWARD_CONE_HALF_ANGLE}° (analyzing direction of movement)")
    print("Press Ctrl+C to stop\n")

    lidar = RPLidarReader()
    lidar.connect()

    cycle = 0

    try:
        for scan_data in lidar.iter_scans():
            cycle += 1
            print(f"\n--- Cycle {cycle} ---")

            # TODO: Replace with real MPU-6050 IMU tilt angle
            tilt_angle = 0

            action, obstacle_info = process_scan(scan_data, tilt_angle=tilt_angle)
            print_scan_summary(scan_data, action, obstacle_info, tilt_angle)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        lidar.disconnect()


if __name__ == "__main__":
    main()


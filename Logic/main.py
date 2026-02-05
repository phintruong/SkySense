"""
Main Program - Real Hardware Mode
Runs obstacle detection using actual RPLIDAR sensor data.

Usage:
    python main.py                    # Use USB mode (default)
    python main.py --gpio             # Use GPIO mode (direct pins)
    python main.py --gpio --port /dev/serial0  # Custom port
"""

import time
import signal
import sys
import argparse
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

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='SkySense Obstacle Detection - RPLIDAR A1',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--gpio',
        action='store_true',
        help='Use GPIO motor control (direct pin connections) instead of USB adapter'
    )
    parser.add_argument(
        '--port',
        type=str,
        default=None,
        help='Serial port (default: /dev/ttyUSB0 for USB, /dev/serial0 for GPIO)'
    )
    parser.add_argument(
        '--motor-duty',
        type=int,
        default=50,
        help='Motor duty cycle for GPIO mode (0-100, default: 50)'
    )
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    print("SkySense Obstacle Detection")
    print("=" * 50)
    print(f"Danger radius: {DANGER_RADIUS}m")
    print(f"Forward cone: +/-{FORWARD_CONE_HALF_ANGLE}deg")
    print("Using: RPLIDAR A1 Hardware")

    # Determine port and mode
    if args.gpio:
        port = args.port or '/dev/serial0'
        mode = "GPIO (Direct Pins)"
        print(f"Mode: {mode}")
        print(f"Port: {port}")
        print(f"Motor: GPIO PWM @ {args.motor_duty}% duty")
    else:
        port = args.port or '/dev/ttyUSB0'  # Will auto-detect COM3 on Windows
        mode = "USB Adapter"
        print(f"Mode: {mode}")
        print(f"Port: {port}")
        print("Motor: Serial Control")

    print("Press Ctrl+C to stop\n")

    # Initialize LiDAR with appropriate settings
    lidar = RPLidarReader(
        port=port,
        use_gpio_motor=args.gpio,
        motor_duty_cycle=args.motor_duty
    )
    _lidar = lidar
    
    if not lidar.connect():
        print("ERROR: Failed to connect to RPLIDAR")
        if args.gpio:
            print("Check:")
            print("  - UART enabled in /boot/config.txt")
            print("  - TX/RX wires connected (not swapped)")
            print("  - /dev/serial0 exists and is accessible")
            print("  - Permissions: sudo usermod -a -G dialout,gpio $USER")
        else:
            print("Check:")
            print("  - USB cable is connected")
            print("  - Device is detected (ls /dev/ttyUSB* or check Device Manager)")
            print("  - Permissions: sudo usermod -a -G dialout $USER")
        sys.exit(1)

    if not lidar.start():
        print("ERROR: Failed to start RPLIDAR motor")
        if args.gpio:
            print("Check:")
            print("  - MOTOCTL wire connected to GPIO12 (Pin 32)")
            print("  - 5V and GND properly connected")
            print("  - Try adjusting duty cycle: --motor-duty 70")
        else:
            print("Check:")
            print("  - Motor power is supplied via USB")
            print("  - LiDAR is receiving sufficient power")
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

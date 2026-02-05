#!/usr/bin/env python3
"""
Example: RPLIDAR A1 with GPIO Motor Control

This script demonstrates how to use the RPLIDAR A1 with direct GPIO pin connections
instead of the USB adapter. The motor control pin (MOTOCTL) is driven by GPIO PWM.

Usage:
    # For USB mode (current setup):
    python example_gpio_mode.py --mode usb

    # For GPIO mode (direct pins):
    python example_gpio_mode.py --mode gpio

Hardware Requirements for GPIO Mode:
    - RPLIDAR A1 TX → Pi RXD (GPIO15, Pin 10)
    - RPLIDAR A1 RX → Pi TXD (GPIO14, Pin 8)
    - RPLIDAR A1 MOTOCTL → Pi GPIO12 (Pin 32)
    - RPLIDAR A1 5V → Pi 5V (Pin 2/4)
    - RPLIDAR A1 GND → Pi GND (Pin 6)

See RPLIDAR_GPIO_WIRING.md for detailed wiring instructions.
"""

import argparse
import sys
import signal
import time
from rplidar_reader import RPLidarReader


# Global for cleanup
_lidar = None


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\n\nShutting down...")
    if _lidar:
        _lidar.disconnect()
    sys.exit(0)


def run_usb_mode():
    """Run LiDAR using USB adapter (motor controlled via serial)."""
    global _lidar

    print("=" * 60)
    print("RPLIDAR A1 - USB Mode")
    print("=" * 60)
    print("Using USB adapter for both data and motor control")
    print("Port: /dev/ttyUSB0 (Linux) or COM3 (Windows)")
    print()

    # Initialize with USB/serial motor control
    lidar = RPLidarReader(
        port='/dev/ttyUSB0',  # Will auto-detect COM3 on Windows
        use_gpio_motor=False  # Use serial motor control
    )
    _lidar = lidar

    if not lidar.connect():
        print("❌ Failed to connect to RPLIDAR")
        print("Check:")
        print("  - USB cable is connected")
        print("  - Device shows as /dev/ttyUSB0 (or COM3 on Windows)")
        print("  - You have permissions (try: sudo usermod -a -G dialout $USER)")
        return False

    print("✓ Connected to RPLIDAR\n")

    if not lidar.start():
        print("❌ Failed to start motor")
        lidar.disconnect()
        return False

    print("✓ Motor started, scanning...\n")

    try:
        scan_count = 0
        for scan_data in lidar.iter_scans():
            scan_count += 1
            print(f"Scan #{scan_count}: {len(scan_data)} points", end='\r')

            if scan_count >= 100:  # Stop after 100 scans
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        lidar.disconnect()
        print("\n\n✓ LiDAR stopped")

    return True


def run_gpio_mode():
    """Run LiDAR using direct GPIO pins (motor controlled via GPIO PWM)."""
    global _lidar

    print("=" * 60)
    print("RPLIDAR A1 - GPIO Mode")
    print("=" * 60)
    print("Using direct GPIO pin connections")
    print("Data: /dev/serial0 (Pi UART)")
    print("Motor: GPIO12 (Pin 32, Hardware PWM)")
    print()

    # Initialize with GPIO motor control
    lidar = RPLidarReader(
        port='/dev/serial0',       # Pi UART (not USB)
        use_gpio_motor=True,       # Use GPIO for motor control
        motor_pin=12,              # GPIO12 (Hardware PWM0)
        motor_duty_cycle=50        # 50% duty cycle
    )
    _lidar = lidar

    if not lidar.connect():
        print("❌ Failed to connect to RPLIDAR")
        print("Check:")
        print("  - UART is enabled in /boot/config.txt (enable_uart=1)")
        print("  - TX/RX wires are connected correctly (not swapped)")
        print("  - /dev/serial0 exists (ls -l /dev/serial*)")
        print("  - Serial console is disabled (raspi-config → Interface → Serial)")
        print("  - You have permissions (sudo usermod -a -G dialout,gpio $USER)")
        return False

    print("✓ Connected to RPLIDAR\n")

    if not lidar.start():
        print("❌ Failed to start motor")
        print("Check:")
        print("  - MOTOCTL wire connected to GPIO12 (Pin 32)")
        print("  - 5V and GND are properly connected")
        print("  - Sufficient power supply (~200mA @ 5V)")
        print("\nTry adjusting motor duty cycle:")
        print("  lidar = RPLidarReader(..., motor_duty_cycle=70)")
        lidar.disconnect()
        return False

    print("✓ Motor started (GPIO PWM), scanning...\n")

    try:
        scan_count = 0
        for scan_data in lidar.iter_scans():
            scan_count += 1
            print(f"Scan #{scan_count}: {len(scan_data)} points", end='\r')

            if scan_count >= 100:  # Stop after 100 scans
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        lidar.disconnect()
        print("\n\n✓ LiDAR stopped, GPIO cleaned up")

    return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='RPLIDAR A1 Example - USB vs GPIO Mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument(
        '--mode',
        choices=['usb', 'gpio'],
        default='usb',
        help='Connection mode: usb (via adapter) or gpio (direct pins)'
    )
    parser.add_argument(
        '--duty-cycle',
        type=int,
        default=50,
        help='Motor duty cycle for GPIO mode (0-100, default: 50)'
    )

    args = parser.parse_args()

    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Run selected mode
    if args.mode == 'usb':
        success = run_usb_mode()
    else:  # gpio
        success = run_gpio_mode()

    if success:
        print("\n✓ Test completed successfully!")
    else:
        print("\n❌ Test failed - see error messages above")
        sys.exit(1)


if __name__ == '__main__':
    main()

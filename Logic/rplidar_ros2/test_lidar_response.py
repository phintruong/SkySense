#!/usr/bin/env python3
"""
Simple test: see if the LIDAR sends back anything at all.

Usage:
  python3 test_lidar_response.py [port] [baud]
  Default port: /dev/ttyUSB0, baud: 115200

  Examples:
    python3 test_lidar_response.py /dev/ttyAMA0
    python3 test_lidar_response.py /dev/ttyAMA0 256000

Requires: pyserial (pip install pyserial, or use your ROS/venv).
"""

import os
import sys
import time

# Allow running without installing the package
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from rplidar_ros2.rplidar_protocol import RPLidarProtocol


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    print(f"Port: {port}, baud: {baud}")
    print("Connecting...")
    lidar = RPLidarProtocol(port=port, baudrate=baud, timeout=0.5)
    if not lidar.connect():
        print("FAIL: Could not open serial port.")
        return 1
    try:
        # 1) Raw read: see if ANY bytes appear (no command sent)
        print("\n1) Raw read (2 s, no command)...")
        lidar.serial.reset_input_buffer()
        time.sleep(2.0)
        n = lidar.serial.in_waiting
        if n > 0:
            raw = lidar.serial.read(min(64, n))
            print(f"   Received {n} byte(s). First bytes (hex): {raw[:32].hex()}")
        else:
            print("   No bytes (device may only respond to commands).")

        # 2) GET_INFO: does the device respond to a command?
        print("\n2) GET_INFO command...")
        lidar.serial.reset_input_buffer()
        # Send GET_INFO then peek at raw response (see if *any* bytes come back)
        lidar._send_command(0x50)  # GET_INFO
        time.sleep(0.4)
        raw_after_cmd = b""
        while lidar.serial.in_waiting:
            raw_after_cmd += lidar.serial.read(lidar.serial.in_waiting)
            time.sleep(0.05)
        if raw_after_cmd:
            print(f"   Raw bytes after command (hex): {raw_after_cmd[:64].hex()}")
        else:
            print("   No bytes after command → likely TX/RX swapped or LIDAR off.")
        lidar.serial.reset_input_buffer()
        info = lidar.get_info()
        if info:
            print(f"   OK - Model={info.get('model')}, Serial={info.get('serial')}, Firmware={info.get('firmware')}")
        else:
            print("   No valid response (timeout or wrong protocol/baud).")

        # 3) GET_HEALTH
        print("\n3) GET_HEALTH command...")
        health = lidar.get_health()
        if health:
            print(f"   OK - Status={health.get('status')}, ErrorCode={health.get('error_code')}")
        else:
            print("   No valid response.")
    finally:
        lidar.disconnect()
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Raw serial test: open UART, turn motor on, print any bytes received.
Run from Logic: sudo venv/bin/python scripts/raw_serial_test.py [port] [motor_gpio] [baud]
Example: sudo venv/bin/python scripts/raw_serial_test.py /dev/ttyAMA0 12 115200
If you see any hex bytes, the LiDAR is sending; if nothing, UART may be in use or wiring wrong.
"""
import sys
import time
import serial

# Add Logic to path for gpiozero
import os
_logic = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _logic not in sys.path:
    sys.path.insert(0, _logic)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
    motor_gpio = int(sys.argv[2]) if len(sys.argv) > 2 else 12
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200

    print(f"Port={port} motor_gpio={motor_gpio} baud={baud}")
    print("Turning motor ON (GPIO", motor_gpio, ")...")

    try:
        from gpiozero import OutputDevice
        motor = OutputDevice(motor_gpio)
        motor.on()
    except Exception as e:
        print("GPIO failed:", e)
        motor = None

    time.sleep(1)
    print("Opening serial, sending GET_INFO (0xA5 0x50), then reading for 10s...")

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # RPLIDAR GET_INFO: sync 0xA5, cmd 0x50, len 0, checksum = 0x50^0x00 = 0x50
        get_info = bytes([0xA5, 0x50, 0x00, 0x50])
        ser.write(get_info)
        ser.flush()
        print("  Sent GET_INFO (4 bytes). Waiting for response...")
        start = time.time()
        total = 0
        while time.time() - start < 10:
            n = ser.in_waiting
            if n > 0:
                data = ser.read(n)
                total += len(data)
                hex_str = " ".join(f"{b:02x}" for b in data[:20])
                print(f"  got {len(data)} bytes: {hex_str}" + (" ..." if len(data) > 20 else ""))
            time.sleep(0.05)
        ser.close()
        print(f"Total bytes in 10s: {total}")
        if total == 0:
            print("No response. Pi TX (pin8)->LiDAR RX, Pi RX (pin10)->LiDAR TX. Try baud 256000 or 460800.")
    except Exception as e:
        print("Serial error:", e)
    finally:
        if motor is not None:
            motor.off()
            motor.close()
        print("Done.")

if __name__ == "__main__":
    main()

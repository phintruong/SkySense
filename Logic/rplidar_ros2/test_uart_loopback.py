#!/usr/bin/env python3
"""
UART loopback test: check if the serial port can receive what it sends.
- Short GPIO TX (pin 8) to RX (pin 10) with a jumper wire.
- Run: python3 test_uart_loopback.py /dev/ttyAMA0
- If you see "OK: received our bytes" then the UART and wiring are working.
"""

import sys
import serial

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyAMA0'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    print(f"Loopback test on {port} at {baud}. TX and RX must be shorted (pin 8 to pin 10).")
    try:
        s = serial.Serial(port, baud, timeout=0.5)
        s.reset_input_buffer()
        s.reset_output_buffer()
        test = b'\xa5\x50'  # two bytes
        s.write(test)
        s.flush()
        import time
        time.sleep(0.1)
        r = s.read(2)
        s.close()
        if r == test:
            print("OK: received our bytes. UART and wiring are good.")
        else:
            print(f"Got back: {r.hex() if r else 'nothing'}. Expected {test.hex()}. Check short between TX and RX.")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())

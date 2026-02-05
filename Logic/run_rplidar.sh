#!/bin/bash
# Run RPLIDAR reader with serial port access (use when not in dialout group).
# Usage: sudo ./run_rplidar.sh [port] [motor_gpio]
# 
# Pinout: TX=pin23(GPIO11), RX=pin24(GPIO8), Motor=pin32(GPIO12)
# UART 4 (pins 23/24) needs: echo "dtoverlay=uart4" | sudo tee -a /boot/firmware/config.txt && sudo reboot
# Then use: sudo ./run_rplidar.sh /dev/ttyAMA4 12
#
# UART 0 (GPIO 14/15, pins 8=TX 10=RX): sudo ./run_rplidar.sh /dev/ttyAMA0 12
# If no response: 1) Disable serial console (raspi-config -> Interface -> Serial -> No). 2) Try baud: ... 12 256000 or ... 12 460800
# Raw test (see if any bytes arrive): sudo venv/bin/python scripts/raw_serial_test.py /dev/ttyAMA0 12 115200
cd "$(dirname "$0")"
exec "$(pwd)/venv/bin/python" -m hardware.rplidar_reader "$@"

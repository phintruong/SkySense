# RPLIDAR A1 GPIO Wiring Guide for Raspberry Pi 4

## Overview
This guide shows how to connect the RPLIDAR A1 directly to Raspberry Pi GPIO pins instead of using the USB adapter.

## Pin Connections

### RPLIDAR A1 to Raspberry Pi 4

| RPLIDAR A1 Pin | Wire Color | Pi4 GPIO Pin | Physical Pin | Description |
|----------------|------------|--------------|--------------|-------------|
| **5V**         | Red        | 5V           | Pin 2 or 4   | Power supply |
| **GND**        | Black      | GND          | Pin 6        | Ground |
| **TX**         | Green      | RXD (GPIO15) | Pin 10       | UART Transmit → Pi Receive |
| **RX**         | White      | TXD (GPIO14) | Pin 8        | UART Receive ← Pi Transmit |
| **MOTOCTL**    | Yellow     | GPIO12 (PWM0)| Pin 32       | Motor control (PWM signal) |

### Pinout Diagram
```
        RPLIDAR A1 Connector
     ┌─────────────────────┐
     │  ●  ●  ●  ●  ●  ●  │
     │ GND RX TX 5V NC MOTOCTL
     └─────────────────────┘
        │  │  │  │     │
        │  │  │  │     └─── Yellow → GPIO12 (Pin 32)
        │  │  │  └───────── Red → 5V (Pin 2)
        │  │  └──────────── Green → GPIO15/RXD (Pin 10)
        │  └─────────────── White → GPIO14/TXD (Pin 8)
        └────────────────── Black → GND (Pin 6)
```

## Raspberry Pi Configuration

### 1. Enable UART on Pi
Edit `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```

Add these lines:
```ini
# Enable UART for LiDAR communication
enable_uart=1
dtoverlay=disable-bt  # Disable Bluetooth to free up UART

# Enable hardware PWM on GPIO12 for motor control
dtoverlay=pwm,pin=12,func=4
```

Reboot:
```bash
sudo reboot
```

### 2. Disable Serial Console
The serial port needs to be freed from console use:
```bash
sudo raspi-config
```
Navigate to: **Interface Options → Serial Port**
- "Login shell over serial?" → **No**
- "Serial port hardware enabled?" → **Yes**

### 3. Verify UART is Available
After reboot, check:
```bash
ls -l /dev/serial*
# Should show /dev/serial0 -> ttyAMA0 or similar
```

## Code Usage

### For USB Connection (Current Setup)
```python
from Logic.lidar.rplidar_reader import RPLidarReader

# USB connection (default)
lidar = RPLidarReader(port='/dev/ttyUSB0')  # or 'COM3' on Windows
```

### For GPIO Pin Connection (Direct Wiring)
```python
from Logic.lidar.rplidar_reader import RPLidarReader

# Direct GPIO connection with motor control
lidar = RPLidarReader(
    port='/dev/serial0',      # Use Pi UART instead of USB
    use_gpio_motor=True,      # Enable GPIO motor control
    motor_pin=12,             # GPIO12 (default)
    motor_duty_cycle=50       # 50% duty cycle (adjust for speed)
)
```

## Pin Explanation

### Why GPIO12 for Motor Control?
- **GPIO12** is hardware PWM0 (supports high-frequency PWM)
- RPLIDAR motor needs 25kHz PWM signal
- Software PWM is too slow/jittery for motor control
- Hardware PWM provides smooth, stable motor operation

### Alternative PWM Pins
If GPIO12 is unavailable, you can use:
- **GPIO13** (Pin 33) - Hardware PWM1
- **GPIO18** (Pin 12) - Hardware PWM0 (alternative)
- **GPIO19** (Pin 35) - Hardware PWM1 (alternative)

Update the code:
```python
lidar = RPLidarReader(
    port='/dev/serial0',
    use_gpio_motor=True,
    motor_pin=13,  # Use GPIO13 instead
    motor_duty_cycle=50
)
```

## Troubleshooting

### Motor Not Spinning
1. **Check wiring**: Verify MOTOCTL (yellow) is connected to GPIO12
2. **Check power**: Ensure 5V and GND are properly connected
3. **Check PWM**: Run this test:
   ```python
   import RPi.GPIO as GPIO
   import time

   GPIO.setmode(GPIO.BCM)
   GPIO.setup(12, GPIO.OUT)
   pwm = GPIO.PWM(12, 25000)  # 25kHz
   pwm.start(50)  # 50% duty
   time.sleep(5)  # Motor should spin
   pwm.stop()
   GPIO.cleanup()
   ```
4. **Adjust duty cycle**: Try values 30-70%
   ```python
   lidar = RPLidarReader(
       port='/dev/serial0',
       use_gpio_motor=True,
       motor_duty_cycle=70  # Increase to 70%
   )
   ```

### No Data Received
1. **Check UART**: Verify TX/RX are not swapped
2. **Check baud rate**: Should be 460800 (A1 default)
3. **Test serial port**:
   ```bash
   sudo apt-get install minicom
   minicom -b 460800 -D /dev/serial0
   ```

### Permission Denied
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Add user to gpio group for GPIO access
sudo usermod -a -G gpio $USER
# Logout and login again
```

## Power Considerations

### Current Draw
- RPLIDAR A1 motor: ~150mA @ 5V
- Logic circuits: ~50mA @ 5V
- **Total: ~200mA**

### Power Source Options
1. **Pi 5V pin** (Pins 2/4): OK for testing, but shares Pi's supply
2. **External 5V supply**: Recommended for production
   - Connect external 5V to RPLIDAR 5V pin
   - **Must share GND** with Pi (connect both grounds together)

## Safety Notes

⚠️ **Important:**
- RPLIDAR uses 3.3V logic levels (compatible with Pi)
- Do NOT connect to 5V logic devices directly
- Always connect GND before power
- Disconnect power before changing wiring

## Testing Checklist

Before running the code:
- [ ] UART enabled in `/boot/config.txt`
- [ ] Bluetooth disabled (optional but recommended)
- [ ] Serial console disabled
- [ ] `/dev/serial0` exists
- [ ] User in `dialout` and `gpio` groups
- [ ] All 5 wires connected (5V, GND, TX, RX, MOTOCTL)
- [ ] Correct GPIO pin (12 default)
- [ ] Sufficient 5V power supply

## Example Full Setup

```python
#!/usr/bin/env python3
from Logic.lidar.rplidar_reader import RPLidarReader
import time

# Initialize with GPIO motor control
lidar = RPLidarReader(
    port='/dev/serial0',       # Pi UART (not USB)
    baudrate=460800,           # A1 default
    use_gpio_motor=True,       # Enable GPIO motor
    motor_pin=12,              # Hardware PWM pin
    motor_duty_cycle=50        # 50% speed
)

# Connect and start
if lidar.connect():
    print("LiDAR connected!")

    if lidar.start():
        print("Motor spinning, scanning started!")

        # Get some scans
        for i in range(10):
            scan = lidar.get_scan()
            if scan:
                print(f"Scan {i+1}: {len(scan)} points")
            time.sleep(0.1)

        lidar.stop()

    lidar.disconnect()
else:
    print("Failed to connect to LiDAR")
```

## References
- [RPLIDAR A1 Datasheet](https://www.slamtec.com/en/Lidar/A1Spec)
- [Raspberry Pi GPIO Pinout](https://pinout.xyz/)
- [RPi.GPIO Documentation](https://sourceforge.net/p/raspberry-gpio-python/wiki/Home/)

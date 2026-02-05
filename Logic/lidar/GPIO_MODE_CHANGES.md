# GPIO Motor Control - Changes Summary

## What Changed

Your RPLIDAR code now supports **two connection modes**:

### 1. **USB Mode** (Current - Working)
- Uses USB adapter for both data and motor control
- Motor controlled via serial commands: `lidar.set_motor_pwm(500)`
- Port: `/dev/ttyUSB0` (Linux) or `COM3` (Windows)
- **This is your current setup** - keep testing with this

### 2. **GPIO Mode** (For Direct Pin Connections)
- Uses direct GPIO pin connections to Raspberry Pi
- Motor controlled via GPIO PWM pin (MOTOCTL)
- Port: `/dev/serial0` (Pi UART)
- Data: TX/RX via UART pins
- Motor: PWM signal via GPIO12 (Pin 32)

## Files Modified

### 1. `rplidar_reader.py` - Added GPIO Motor Support
**New parameters:**
```python
lidar = RPLidarReader(
    port='/dev/serial0',      # UART port for GPIO mode
    use_gpio_motor=True,      # Enable GPIO motor control
    motor_pin=12,             # GPIO pin for MOTOCTL
    motor_duty_cycle=50       # PWM duty cycle (0-100%)
)
```

**Key changes:**
- Imports `RPi.GPIO` library (auto-detects if available)
- New `use_gpio_motor` flag to switch modes
- `start()` method now initializes GPIO PWM when `use_gpio_motor=True`
- `stop()` method cleans up GPIO properly
- Backwards compatible - USB mode still works exactly as before

### 2. `requirements.txt` - Added GPIO Library
```
RPi.GPIO>=0.7.0  # For direct GPIO motor control
```

### 3. `main.py` - Added Command-Line Arguments
**New usage:**
```bash
# USB mode (default - your current setup)
python main.py

# GPIO mode (when you switch to direct pins)
python main.py --gpio

# Custom settings
python main.py --gpio --port /dev/serial0 --motor-duty 70
```

## Files Created

### 1. `RPLIDAR_GPIO_WIRING.md` - Complete Wiring Guide
- Pin connection diagram
- Pi configuration steps (enable UART, disable Bluetooth)
- Troubleshooting tips
- Safety notes
- Testing checklist

### 2. `example_gpio_mode.py` - Test Script
Quick way to test both modes:
```bash
# Test USB mode
python Logic/lidar/example_gpio_mode.py --mode usb

# Test GPIO mode
python Logic/lidar/example_gpio_mode.py --mode gpio
```

### 3. `GPIO_MODE_CHANGES.md` - This file
Summary of all changes

## How to Use

### Now (Testing with USB):
**Nothing changes!** Your code works exactly as before:
```python
from Logic.lidar.rplidar_reader import RPLidarReader

lidar = RPLidarReader()  # Uses USB by default
lidar.connect()
lidar.start()
```

Or run main program:
```bash
python Logic/main.py  # USB mode (default)
```

### Later (When Using GPIO Pins):

**Step 1: Hardware Setup**
- Wire 5 connections (see `RPLIDAR_GPIO_WIRING.md`)
- Enable UART on Pi (edit `/boot/config.txt`)
- Disable serial console (`raspi-config`)

**Step 2: Run with GPIO Mode**
```bash
# Option A: Use main program
python Logic/main.py --gpio

# Option B: Use in your code
from Logic.lidar.rplidar_reader import RPLidarReader

lidar = RPLidarReader(
    port='/dev/serial0',
    use_gpio_motor=True
)
lidar.connect()
lidar.start()
```

## Why GPIO Motor Control is Needed

When using **direct pin connections** (not USB):

1. **Data (TX/RX)** → Handled by UART serial communication
2. **Motor (MOTOCTL)** → Needs separate GPIO PWM signal ⚠️

The RPLIDAR A1 motor won't spin without the MOTOCTL pin being driven with PWM signal. The USB adapter handles this automatically, but with direct pins, **you need to control it via GPIO**.

## Technical Details

### Motor Control Signal
- **Frequency**: 25 kHz (hardware PWM required)
- **Duty Cycle**: 30-70% (default: 50%)
- **Pin**: GPIO12 (Hardware PWM0, Physical Pin 32)

### Why Hardware PWM?
- Software PWM is too slow/jittery for 25 kHz
- GPIO12/13/18/19 have dedicated PWM hardware
- Provides smooth, stable motor operation

### Alternative PWM Pins
If GPIO12 is busy:
- GPIO13 (Pin 33)
- GPIO18 (Pin 12)
- GPIO19 (Pin 35)

Change in code:
```python
lidar = RPLidarReader(
    port='/dev/serial0',
    use_gpio_motor=True,
    motor_pin=13  # Use GPIO13 instead
)
```

## Quick Reference

| Mode | Port | Motor Control | When to Use |
|------|------|---------------|-------------|
| USB | `/dev/ttyUSB0` or `COM3` | Serial commands | Current setup, easiest |
| GPIO | `/dev/serial0` | GPIO PWM (Pin 32) | Direct wiring, embedded |

## Testing Checklist

Before switching to GPIO mode:

Hardware:
- [ ] RPLIDAR wired to Pi (5 wires: 5V, GND, TX, RX, MOTOCTL)
- [ ] MOTOCTL connected to GPIO12 (Pin 32)
- [ ] Sufficient 5V power supply (~200mA)

Software:
- [ ] UART enabled: `enable_uart=1` in `/boot/config.txt`
- [ ] Bluetooth disabled: `dtoverlay=disable-bt` in `/boot/config.txt`
- [ ] Serial console disabled via `raspi-config`
- [ ] `/dev/serial0` exists
- [ ] User in `dialout` and `gpio` groups
- [ ] `RPi.GPIO` installed: `pip install RPi.GPIO`

Test:
```bash
# Test GPIO mode
python Logic/lidar/example_gpio_mode.py --mode gpio

# If motor spins and data received → Success!
# If not, see troubleshooting in RPLIDAR_GPIO_WIRING.md
```

## Backwards Compatibility

✅ **All existing code still works!**
- Default mode is USB (no code changes needed)
- GPIO mode is opt-in via `use_gpio_motor=True`
- Your current USB setup is unaffected

## Need Help?

1. **Motor not spinning?** → See troubleshooting in `RPLIDAR_GPIO_WIRING.md`
2. **No data received?** → Check TX/RX wiring and UART config
3. **Permission errors?** → Run `sudo usermod -a -G dialout,gpio $USER` and re-login
4. **Test each mode** → Use `example_gpio_mode.py` to verify both work

## Summary

**Answer to your question:**
> "is this a code issue"

**It's BOTH hardware AND code:**
- **Hardware**: MOTOCTL pin needs to be wired to GPIO12
- **Code**: GPIO PWM needs to be initialized to drive MOTOCTL

**Your code NOW supports both modes** - USB (working now) and GPIO (ready when you switch).

When you wire it to pins, just add `--gpio` flag:
```bash
python Logic/main.py --gpio
```

That's it! 🚀

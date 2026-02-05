#!/usr/bin/env python3
"""
RPLIDAR A1 Hardware Interface Module

Provides a professional interface for reading scan data from RPLIDAR A1 sensors
using the PyRPlidar library (USB) or the project's RPLidarProtocol (UART/pins).
Integrates with the obstacle detection logic module.

References:
- PyLidar: https://www.pylidar.org/en/latest/ (for post-processing LiDAR data files)
- PyRPlidar: https://github.com/SkoltechRobotics/pyrplidar
"""

import os
import sys
import signal
import logging
import time
from typing import List, Tuple, Optional, Generator

# Allow importing rplidar_ros2 from Logic when run as -m hardware.rplidar_reader
_LOGIC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _LOGIC_DIR not in sys.path:
    sys.path.insert(0, _LOGIC_DIR)

RPLidarProtocol = None
try:
    from rplidar_ros2.rplidar_ros2.rplidar_protocol import RPLidarProtocol as _RPLidarProtocol
    RPLidarProtocol = _RPLidarProtocol
except ImportError:
    pass

from pyrplidar import PyRPlidar

try:
    from gpiozero import OutputDevice
except ImportError:
    OutputDevice = None  # type: ignore

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration constants
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 460800  # Higher baudrate for better performance (USB)
UART_BAUDRATE = 115200  # RPLIDAR A1 default for UART
DEFAULT_TIMEOUT = 3.0
DEFAULT_MOTOR_PWM = 500  # Motor PWM value (0-1023)
MIN_DISTANCE_MM = 50  # Minimum valid distance in millimeters (5 cm)
MAX_DISTANCE_MM = 3000  # Maximum valid distance in millimeters (300 cm)
# Ports that use UART (no DTR); use pyserial-based RPLidarProtocol instead of pyrplidar
# UART 0 (GPIO 14/15): /dev/ttyAMA0 or /dev/serial0
# UART 4 (GPIO 8/11, pins 24/23): /dev/ttyAMA4 (needs dtoverlay=uart4)
UART_PORTS = ('/dev/ttyAMA0', '/dev/ttyAMA4', '/dev/serial0', '/dev/ttyS0')


class RPLidarReader:
    """
    Professional interface for RPLIDAR A1 sensor communication using PyRPlidar.
    
    Handles sensor initialization, data acquisition, and cleanup operations.
    Provides scan data in format compatible with obstacle detection logic.
    """
    
    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 motor_pwm: int = DEFAULT_MOTOR_PWM, motor_gpio_pin: Optional[int] = None,
                 uart_baudrate: Optional[int] = None):
        """
        Initialize RPLIDAR reader.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)
            baudrate: Serial communication baud rate (default: 460800)
            motor_pwm: Motor PWM value 0-1023 (default: 500)
            motor_gpio_pin: For UART/pins only: GPIO BCM pin number to drive motor
                           (e.g. 12 = physical pin 32 on Pi). Wire to RPLIDAR MTR. If None, use DTR
                           (often does not work on Pi UART).
            uart_baudrate: For UART ports only: baud rate (default 115200). Try 256000 if no response.
        """
        self.port = port
        self.baudrate = baudrate
        self.motor_pwm = motor_pwm
        self.motor_gpio_pin = motor_gpio_pin
        self.uart_baudrate = uart_baudrate if uart_baudrate is not None else UART_BAUDRATE
        self.lidar: Optional[PyRPlidar] = None
        self._protocol = None  # RPLidarProtocol when using UART
        self._motor_gpio = None  # gpiozero OutputDevice when motor_gpio_pin set
        self._use_protocol = port in UART_PORTS and RPLidarProtocol is not None
        self._running = False
        self._scan_generator_func = None
        self._scan_generator: Optional[Generator] = None

    def connect(self) -> bool:
        """
        Establish connection to RPLIDAR sensor.
        Uses RPLidarProtocol (pyserial) for UART ports; PyRPlidar for USB.
        """
        if self._use_protocol:
            if RPLidarProtocol is None:
                logger.error(
                    f"UART port {self.port} requires rplidar_ros2 protocol. "
                    "Run from Logic directory so rplidar_ros2 can be imported."
                )
                return False
            try:
                logger.info(f"Connecting to RPLIDAR on {self.port} at {self.uart_baudrate} baud (UART)...")
                self._protocol = RPLidarProtocol(
                    port=self.port, baudrate=self.uart_baudrate, timeout=DEFAULT_TIMEOUT
                )
                if self._protocol.connect():
                    logger.info("RPLIDAR connection established (UART)")
                    return True
                logger.error(
                    "If permission denied: (1) run 'sudo usermod -aG dialout $USER' "
                    "then log out and back in. (2) Or in this terminal run 'newgrp dialout' "
                    "then run the command again. (3) Or run: sudo ./run_rplidar.sh /dev/ttyAMA0"
                )
                return False
            except Exception as e:
                logger.error(f"Connection failed: {type(e).__name__}: {e}")
                return False
        try:
            logger.info(f"Connecting to RPLIDAR on {self.port} at {self.baudrate} baud...")
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=DEFAULT_TIMEOUT)
            logger.info("RPLIDAR connection established")
            return True
        except FileNotFoundError:
            logger.error(f"RPLIDAR device not found on {self.port}")
            logger.error("Please verify USB connection and device mounting")
            return False
        except PermissionError:
            logger.error(f"Permission denied accessing {self.port}")
            logger.error("Try running with elevated privileges (sudo on Linux)")
            return False
        except Exception as e:
            logger.error(f"Connection failed: {type(e).__name__}: {e}")
            return False
    
    def start(self) -> bool:
        """
        Start the RPLIDAR motor and begin scanning.
        
        Returns:
            True if motor started successfully, False otherwise
        """
        if self._use_protocol:
            if not self._protocol:
                logger.error("Cannot start: RPLIDAR not connected")
                return False
            # Log config: PWM value and which path we use (motor_gpio_pin is BCM; e.g. 12 = physical pin 32 on Pi)
            print(f"[MOTOR] motor_pwm={self.motor_pwm}, motor_gpio_pin={self.motor_gpio_pin} (BCM; 12=physical pin 32 on Pi)")
            # Prefer GPIO motor control on UART (Pi UART DTR often doesn't drive motor)
            if self.motor_gpio_pin is not None and OutputDevice is not None:
                try:
                    self._motor_gpio = OutputDevice(self.motor_gpio_pin)
                    self._motor_gpio.on()
                    time.sleep(1.0)  # Let motor spin up
                    print(f"[MOTOR] Using GPIO: pin {self.motor_gpio_pin} ON (PWM not sent to device)")
                    logger.info(f"RPLIDAR motor started via GPIO {self.motor_gpio_pin}, scanning initiated")
                    # Set motor_running flag in protocol so iter_scans knows motor is on
                    self._protocol.motor_running = True
                except Exception as e:
                    print(f"[MOTOR] GPIO failed: {e}, falling back to DTR")
                    logger.warning(f"GPIO motor control failed: {e}. Trying DTR.")
                    self._motor_gpio = None
            if self._motor_gpio is None:
                try:
                    self._protocol.start_motor(pwm=self.motor_pwm)
                    time.sleep(0.5)
                    print(f"[MOTOR] Using DTR: pwm={self.motor_pwm} sent to device")
                    logger.info(f"RPLIDAR motor started (PWM: {self.motor_pwm}), scanning initiated")
                except Exception as e:
                    print(f"[MOTOR] DTR failed: {e}")
                    logger.warning(
                        "Motor control not available (e.g. UART has no DTR). "
                        "Wire motor to a GPIO and run with: ... /dev/ttyAMA0 <GPIO>"
                    )
            self._running = True
            return True

        if not self.lidar:
            logger.error("Cannot start: RPLIDAR not connected")
            return False

        print(f"[MOTOR] USB/PyRPlidar: motor_pwm={self.motor_pwm} sent to device")
        try:
            # Set motor PWM and wait for stabilization
            self.lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(2)  # Allow motor to stabilize
            
            # Start force scan (returns a generator function)
            self._scan_generator_func = self.lidar.force_scan()
            self._scan_generator = None
            self._running = True
            logger.info(f"RPLIDAR motor started (PWM: {self.motor_pwm}), scanning initiated")
            return True
        except (AttributeError, TypeError) as e:
            # UART (e.g. /dev/ttyAMA0) has no DTR; motor control fails. Try scanning anyway.
            if 'dtr' in str(e).lower():
                logger.warning(
                    "Motor control not available (e.g. UART has no DTR). "
                    "Trying to scan anyway—ensure motor is powered separately if needed."
                )
            try:
                self._scan_generator_func = self.lidar.force_scan()
                self._scan_generator = None
                self._running = True
                logger.info("Scanning initiated without motor control")
                return True
            except Exception as e2:
                logger.error(f"Failed to start scan: {type(e2).__name__}: {e2}")
                return False
        except Exception as e:
            logger.error(f"Failed to start motor: {type(e).__name__}: {e}")
            return False
    
    def stop(self) -> None:
        """Stop the RPLIDAR motor."""
        if self._use_protocol and self._protocol and self._running:
            if self._motor_gpio is not None:
                try:
                    self._motor_gpio.off()
                    self._motor_gpio.close()
                except Exception as e:
                    logger.debug(f"GPIO motor stop: {e}")
                self._motor_gpio = None
            try:
                self._protocol.stop_motor()
            except Exception as e:
                logger.debug(f"Motor stop skipped or failed: {e}")
            self._running = False
            logger.info("RPLIDAR motor stopped")
        elif self.lidar and self._running:
            try:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
            except Exception as e:
                logger.debug(f"Motor stop skipped or failed: {e}")
            self._running = False
            self._scan_generator = None
            self._scan_generator_func = None
            logger.info("RPLIDAR motor stopped")
    
    def disconnect(self) -> None:
        """Disconnect from RPLIDAR sensor."""
        self.stop()
        if self._use_protocol and self._protocol:
            try:
                self._protocol.disconnect()
                logger.info("RPLIDAR disconnected")
            except Exception as e:
                logger.warning(f"Error disconnecting RPLIDAR: {e}")
            self._protocol = None
        elif self.lidar:
            try:
                self.lidar.disconnect()
                logger.info("RPLIDAR disconnected")
            except Exception as e:
                logger.warning(f"Error disconnecting RPLIDAR: {e}")
    
    def _protocol_points_to_scan(self, points: List[Tuple]) -> List[Tuple[float, float]]:
        """Convert protocol (angle_deg, distance_mm, quality) to (angle_deg, distance_m)."""
        out = []
        for item in points:
            angle_deg, distance_mm, _ = item[0], item[1], item[2] if len(item) >= 3 else 0
            # Use same range as protocol (1–12000 mm) so we don't drop valid points
            if 1.0 <= distance_mm <= 12000.0:
                out.append((angle_deg, distance_mm / 1000.0))
        return out

    def get_scan(self) -> Optional[List[Tuple[float, float]]]:
        """
        Read a single complete scan from the RPLIDAR.
        
        Returns:
            List of tuples [(angle_degrees, distance_meters), ...]
            Returns None if scan failed or sensor not running.
            
        Note:
            - Angles are in degrees (0-359)
            - Distances are in meters
            - Invalid measurements (out of range) are filtered
        """
        if self._use_protocol:
            if not self._protocol or not self._running:
                logger.warning("Cannot get scan: RPLIDAR not running")
                return None
            try:
                for points in self._protocol.iter_scans():
                    return self._protocol_points_to_scan(points)
                return None
            except Exception as e:
                logger.error(f"Error reading scan: {type(e).__name__}: {e}")
                return None
        if not self.lidar or not self._running or not self._scan_generator_func:
            logger.warning("Cannot get scan: RPLIDAR not running")
            return None

        try:
            scan_data = []
            prev_angle = None
            points_collected = False
            
            # Get generator if not already created
            if self._scan_generator is None:
                self._scan_generator = self._scan_generator_func()
            
            # Collect points until we detect a complete sweep
            for scan in self._scan_generator:
                # Filter valid measurements
                if MIN_DISTANCE_MM <= scan.distance <= MAX_DISTANCE_MM:
                    # Convert distance from millimeters to meters
                    distance_m = scan.distance / 1000.0
                    angle_deg = scan.angle
                    scan_data.append((angle_deg, distance_m))
                    points_collected = True
                
                # Detect sweep completion (angle wraps around)
                if prev_angle is not None and scan.angle < prev_angle:
                    break
                
                prev_angle = scan.angle
            
            return scan_data if points_collected else None
            
        except StopIteration:
            logger.warning("Scan iteration stopped")
            return None
        except Exception as e:
            logger.error(f"Error reading scan: {type(e).__name__}: {e}")
            return None
    
    def iter_scans(self) -> Generator[List[Tuple[float, float]], None, None]:
        """
        Generator that yields continuous scan data.
        
        Yields:
            List of tuples [(angle_degrees, distance_meters), ...]
            Format compatible with logic.process_scan()
        """
        if self._use_protocol:
            if not self._protocol or not self._running:
                logger.error("Cannot iterate scans: RPLIDAR not running")
                return
            try:
                for points in self._protocol.iter_scans():
                    converted = self._protocol_points_to_scan(points)
                    # Always yield so caller sees every sweep (even 0 points)
                    yield converted
            except Exception as e:
                logger.error(f"Error in scan iteration: {type(e).__name__}: {e}")
                raise
            return
        if not self.lidar or not self._running or not self._scan_generator_func:
            logger.error("Cannot iterate scans: RPLIDAR not running")
            return

        try:
            # Get generator if not already created
            if self._scan_generator is None:
                self._scan_generator = self._scan_generator_func()
            
            points = []
            prev_angle = None
            
            for scan in self._scan_generator:
                # Collect valid measurements
                if MIN_DISTANCE_MM <= scan.distance <= MAX_DISTANCE_MM:
                    distance_m = scan.distance / 1000.0
                    angle_deg = scan.angle
                    points.append((angle_deg, distance_m))
                
                # Detect sweep completion (angle wraps around)
                if prev_angle is not None and scan.angle < prev_angle:
                    if points:  # Only yield if we have data
                        yield points
                    points = []  # Reset for next scan
                
                prev_angle = scan.angle
                
        except Exception as e:
            logger.error(f"Error in scan iteration: {type(e).__name__}: {e}")
            raise
    
    def export_scan_ascii(self, scan_data: List[Tuple[float, float]], 
                         filename: str) -> bool:
        """
        Export scan data to ASCII format compatible with PyLidar.
        
        Args:
            scan_data: List of tuples [(angle, distance), ...]
            filename: Output filename
            
        Returns:
            True if export successful, False otherwise
        """
        try:
            with open(filename, 'w') as f:
                # Write header
                f.write("# RPLIDAR A1 Scan Data\n")
                f.write("# Format: angle_degrees distance_meters\n")
                f.write("# Compatible with PyLidar ASCII format\n")
                
                # Write data points
                for angle, distance in scan_data:
                    f.write(f"{angle:.2f} {distance:.4f}\n")
            
            logger.info(f"Scan data exported to {filename}")
            return True
        except Exception as e:
            logger.error(f"Failed to export scan data: {e}")
            return False


# Global instance for standalone usage
_reader_instance: Optional[RPLidarReader] = None


def signal_handler(sig, frame):
    """Handle interrupt signal for graceful shutdown."""
    logger.info("Interrupt signal received, shutting down...")
    if _reader_instance:
        _reader_instance.disconnect()
    sys.exit(0)


def main():
    """
    Standalone execution: continuously read and display RPLIDAR scan data.
    
    Usage: python -m hardware.rplidar_reader [port] [motor_gpio]
    Default port: /dev/ttyUSB0. Use /dev/ttyAMA0 for Pi UART (GPIO 14/15).
    For UART: pass motor_gpio (BCM pin, e.g. 12) if LiDAR motor is wired to
    a GPIO—Pi UART DTR often does not drive the motor. Wire that GPIO to RPLIDAR MTR.
    
    For integration with obstacle detection, use the RPLidarReader class
    directly and pass scan data to logic.process_scan().
    """
    global _reader_instance
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    port = DEFAULT_PORT
    motor_gpio = None
    uart_baud = None
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            motor_gpio = int(sys.argv[2])
        except ValueError:
            logger.warning(f"Invalid motor GPIO '{sys.argv[2]}', ignoring")
    if len(sys.argv) > 3:
        try:
            uart_baud = int(sys.argv[3])
        except ValueError:
            logger.warning(f"Invalid UART baud '{sys.argv[3]}', ignoring")
    
    # Initialize reader (use motor_gpio for UART so motor spins on Pi pins)
    reader = RPLidarReader(port=port, motor_gpio_pin=motor_gpio, uart_baudrate=uart_baud)
    _reader_instance = reader
    
    # Connect and start
    if not reader.connect():
        logger.error("Failed to connect to RPLIDAR")
        sys.exit(1)
    
    if not reader.start():
        logger.error("Failed to start RPLIDAR")
        reader.disconnect()
        sys.exit(1)
    
    logger.info("RPLIDAR scanning active. Press Ctrl+C to stop.")
    logger.info("Scan data format: (angle_degrees, distance_meters)")
    
    try:
        scan_count = 0
        for scan_data in reader.iter_scans():
            scan_count += 1
            logger.info(f"Scan #{scan_count}: {len(scan_data)} valid points")
            
            # Display sample of scan data
            if scan_data:
                sample_size = min(5, len(scan_data))
                for angle, distance in scan_data[:sample_size]:
                    logger.info(f"  Angle: {angle:6.2f}°, Distance: {distance:6.3f}m")
                if len(scan_data) > sample_size:
                    logger.info(f"  ... and {len(scan_data) - sample_size} more points")
            elif scan_count <= 3:
                logger.warning("No points in this sweep—check LiDAR view and wiring.")
    
    except KeyboardInterrupt:
        logger.info("Scan interrupted by user")
    except Exception as e:
        logger.error(f"Unexpected error: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        reader.disconnect()
        logger.info("RPLIDAR reader shutdown complete")


if __name__ == "__main__":
    main()

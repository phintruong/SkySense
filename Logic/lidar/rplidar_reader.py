#!/usr/bin/env python3
"""
RPLIDAR A1 Hardware Interface Module

Provides a professional interface for reading scan data from RPLIDAR A1 sensors.
Uses the 'rplidar' package (Skoltech/SkRobo) when available for reliable A1 support;
falls back to PyRPlidar for other models. Integrates with the obstacle detection
logic module and supports data export for post-processing analysis.

References:
- rplidar (A1-compatible): pip install rplidar
- PyRPlidar: https://github.com/SkoltechRobotics/pyrplidar (A2+ protocol)
"""

import sys
import signal
import logging
import time
import platform
from typing import List, Tuple, Optional, Generator, Any

# Prefer 'rplidar' package (Skoltech/SkRobo) - implements A1 protocol correctly.
# PyRPlidar expects a 7-byte descriptor response that A1 doesn't send -> IndexError.
try:
    from rplidar import RPLidar as SkRPLidar
    SK_RPLIDAR_AVAILABLE = True
except ImportError:
    SkRPLidar = None
    SK_RPLIDAR_AVAILABLE = False

from pyrplidar import PyRPlidar

# GPIO imports for direct pin motor control (Raspberry Pi only)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration constants - auto-detect platform
# Windows: 'COM3', 'COM4', etc. | Linux/Pi: '/dev/ttyUSB0' for USB, '/dev/serial0' for GPIO UART
DEFAULT_PORT = 'COM3' if platform.system() == 'Windows' else '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200  # RPLidar A1 default; 460800 can cause protocol/parsing errors
DEFAULT_TIMEOUT = 6.0  # A1 can be slow to respond; short timeout -> partial read -> IndexError
DEFAULT_MOTOR_PWM = 500  # Motor PWM value (0-1023)
MIN_DISTANCE_MM = 50  # Minimum valid distance in millimeters (5 cm)
MAX_DISTANCE_MM = 3000  # Maximum valid distance in millimeters (300 cm)

# GPIO Motor Control (for direct pin connections on Raspberry Pi)
MOTOR_PWM_PIN = 12  # GPIO12 (Physical pin 32) - Hardware PWM0
MOTOR_PWM_FREQ = 25000  # 25 kHz PWM frequency for motor control
MOTOR_DUTY_CYCLE = 50  # 50% duty cycle (adjust 0-100 for motor speed)

# Drone frame: sensor 0° = back of drone; 180 offset so 0° = front, then mirror L/R
ANGLE_OFFSET_DEG = 180


def _drone_angle(angle_deg: float) -> float:
    """Convert sensor angle (0° = back) to drone frame (0° = front), mirrored left/right."""
    # (180 - x) % 360 = 180° offset (0=front) + mirror (left<->right)
    return (ANGLE_OFFSET_DEG - angle_deg) % 360.0


class RPLidarReader:
    """
    Professional interface for RPLIDAR A1 sensor communication using PyRPlidar.
    
    Handles sensor initialization, data acquisition, and cleanup operations.
    Provides scan data in format compatible with obstacle detection logic.
    """
    
    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 motor_pwm: int = DEFAULT_MOTOR_PWM, use_gpio_motor: bool = False,
                 motor_pin: int = MOTOR_PWM_PIN, motor_duty_cycle: int = MOTOR_DUTY_CYCLE):
        """
        Initialize RPLIDAR reader.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0' for USB, '/dev/serial0' for direct UART)
            baudrate: Serial communication baud rate (default: 115200 for RPLidar A1)
            motor_pwm: Motor PWM value 0-1023 (default: 500) - used when use_gpio_motor=False
            use_gpio_motor: If True, control motor via GPIO pin instead of serial command
            motor_pin: GPIO pin number for motor control (BCM numbering, default: 12)
            motor_duty_cycle: PWM duty cycle 0-100% for GPIO motor control (default: 50)
        """
        self.port = port
        self.baudrate = baudrate
        self.motor_pwm = motor_pwm
        self.use_gpio_motor = use_gpio_motor
        self.motor_pin = motor_pin
        self.motor_duty_cycle = motor_duty_cycle
        self.lidar: Any = None  # PyRPlidar or SkRPLidar
        self._use_sk_rplidar = False  # True when using 'rplidar' package (A1-compatible)
        self._running = False
        self._scan_generator_func = None
        self._scan_generator: Optional[Generator] = None
        self._gpio_pwm = None  # GPIO PWM instance

        # Validate GPIO availability if needed
        if self.use_gpio_motor and not GPIO_AVAILABLE:
            logger.warning("GPIO motor control requested but RPi.GPIO not available")
            logger.warning("Falling back to serial motor control")
            self.use_gpio_motor = False
        
    def connect(self) -> bool:
        """
        Establish connection to RPLIDAR sensor.
        Prefers 'rplidar' package when available (correct A1 protocol); else PyRPlidar.
        
        Returns:
            True if connection successful, False otherwise
        """
        # Prefer SkRPLidar for A1 (avoids 7-byte descriptor IndexError with pyrplidar)
        if not self.use_gpio_motor and SK_RPLIDAR_AVAILABLE:
            try:
                logger.info(f"Connecting to RPLIDAR on {self.port} (rplidar backend, A1-compatible)...")
                self.lidar = SkRPLidar(
                    self.port,
                    baudrate=self.baudrate,
                    timeout=max(3, int(DEFAULT_TIMEOUT))
                )
                self._use_sk_rplidar = True
                logger.info("RPLIDAR connection established (rplidar backend)")
                return True
            except Exception as e:
                logger.warning(f"rplidar backend failed: {e}; falling back to PyRPlidar")
                self._use_sk_rplidar = False
                self.lidar = None

        try:
            logger.info(f"Connecting to RPLIDAR on {self.port} at {self.baudrate} baud...")
            self.lidar = PyRPlidar()
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=DEFAULT_TIMEOUT)
            self._use_sk_rplidar = False
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
        With rplidar backend, motor is already started in connect().

        Returns:
            True if motor started successfully, False otherwise
        """
        if not self.lidar:
            logger.error("Cannot start: RPLIDAR not connected")
            return False

        if self._use_sk_rplidar:
            self._running = True
            self._scan_generator_func = True  # marker; iter_scans uses lidar.iter_scans()
            logger.info("RPLIDAR scanning initiated (rplidar backend)")
            return True

        try:
            # Motor control: GPIO vs Serial (PyRPlidar backend)
            if self.use_gpio_motor:
                # Initialize GPIO motor control
                logger.info(f"Initializing GPIO motor control on pin {self.motor_pin}")
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.motor_pin, GPIO.OUT)
                self._gpio_pwm = GPIO.PWM(self.motor_pin, MOTOR_PWM_FREQ)
                self._gpio_pwm.start(self.motor_duty_cycle)
                logger.info(f"GPIO PWM started: {MOTOR_PWM_FREQ}Hz, {self.motor_duty_cycle}% duty cycle")
                time.sleep(2)  # Allow motor to stabilize
            else:
                # Use serial motor control (USB adapter)
                self.lidar.set_motor_pwm(self.motor_pwm)
                logger.info(f"Serial motor PWM set to {self.motor_pwm}")
                time.sleep(4)  # A1 needs longer spin-up before responding to scan

            # Use standard scan (0x20). Library reads 7-byte descriptor; partial read -> IndexError.
            # Retry a few times in case A1 is slow to respond.
            for attempt in range(3):
                try:
                    self._scan_generator_func = self.lidar.start_scan()
                    break
                except IndexError:
                    if attempt < 2:
                        logger.warning("Scan descriptor read failed, retrying in 1.5s...")
                        time.sleep(1.5)
                    else:
                        raise
            self._scan_generator = None
            self._running = True
            logger.info("RPLIDAR scanning initiated")
            return True
        except Exception as e:
            logger.error(f"Failed to start motor: {type(e).__name__}: {e}")
            # Stop motor so it doesn't keep spinning after disconnect
            try:
                if self.use_gpio_motor and self._gpio_pwm:
                    self._gpio_pwm.stop()
                    GPIO.cleanup(self.motor_pin)
                    self._gpio_pwm = None
                else:
                    self.lidar.set_motor_pwm(0)
            except Exception:
                pass
            return False
    
    def stop(self) -> None:
        """Stop the RPLIDAR motor."""
        if not self.lidar or not self._running:
            return
        try:
            if self._use_sk_rplidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                logger.info("RPLIDAR motor stopped (rplidar backend)")
            else:
                self.lidar.stop()
                if self.use_gpio_motor and self._gpio_pwm:
                    self._gpio_pwm.stop()
                    GPIO.cleanup(self.motor_pin)
                    self._gpio_pwm = None
                    logger.info("GPIO motor PWM stopped")
                else:
                    self.lidar.set_motor_pwm(0)
                    logger.info("Serial motor PWM stopped")
            self._running = False
            self._scan_generator = None
            self._scan_generator_func = None
            logger.info("RPLIDAR motor stopped")
        except Exception as e:
            logger.warning(f"Error stopping motor: {e}")
    
    def disconnect(self) -> None:
        """Disconnect from RPLIDAR sensor."""
        self.stop()
        
        if self.lidar:
            try:
                self.lidar.disconnect()
                logger.info("RPLIDAR disconnected")
            except Exception as e:
                logger.warning(f"Error disconnecting RPLIDAR: {e}")
    
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
        if not self.lidar or not self._running:
            logger.warning("Cannot get scan: RPLIDAR not running")
            return None

        if self._use_sk_rplidar:
            try:
                scan = next(self.lidar.iter_scans())
                return [
                    (_drone_angle(angle_deg), dist_mm / 1000.0)
                    for _q, angle_deg, dist_mm in scan
                    if MIN_DISTANCE_MM <= dist_mm <= MAX_DISTANCE_MM
                ] or None
            except (StopIteration, Exception) as e:
                logger.warning(f"get_scan failed: {e}")
                return None
        
        if not self._scan_generator_func:
            return None
        
        try:
            scan_data = []
            prev_angle = None
            points_collected = False
            
            # Get generator if not already created (PyRPlidar backend)
            if self._scan_generator is None:
                self._scan_generator = self._scan_generator_func()
            
            # Collect points until we detect a complete sweep
            for scan in self._scan_generator:
                # Filter valid measurements
                if MIN_DISTANCE_MM <= scan.distance <= MAX_DISTANCE_MM:
                    # Convert distance from millimeters to meters
                    distance_m = scan.distance / 1000.0
                    scan_data.append((_drone_angle(scan.angle), distance_m))
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
        if not self.lidar or not self._running:
            logger.error("Cannot iterate scans: RPLIDAR not running")
            return

        if self._use_sk_rplidar:
            # rplidar package: iter_scans() yields list of (quality, angle_deg, distance_mm)
            try:
                for scan in self.lidar.iter_scans():
                    points = []
                    for _quality, angle_deg, dist_mm in scan:
                        if MIN_DISTANCE_MM <= dist_mm <= MAX_DISTANCE_MM:
                            points.append((_drone_angle(angle_deg), dist_mm / 1000.0))
                    if points:
                        yield points
            except Exception as e:
                logger.error(f"Error in scan iteration: {type(e).__name__}: {e}")
                raise
            return

        if not self._scan_generator_func:
            logger.error("Cannot iterate scans: RPLIDAR not running")
            return
        
        try:
            # Get generator if not already created (PyRPlidar backend)
            if self._scan_generator is None:
                self._scan_generator = self._scan_generator_func()
            
            points = []
            prev_angle = None
            
            for scan in self._scan_generator:
                # Collect valid measurements
                if MIN_DISTANCE_MM <= scan.distance <= MAX_DISTANCE_MM:
                    distance_m = scan.distance / 1000.0
                    points.append((_drone_angle(scan.angle), distance_m))
                
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
    
    For integration with obstacle detection, use the RPLidarReader class
    directly and pass scan data to logic.process_scan().
    """
    global _reader_instance
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize reader
    reader = RPLidarReader()
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

#!/usr/bin/env python3
"""
RPLIDAR A1 Hardware Interface Module

Provides a professional interface for reading scan data from RPLIDAR A1 sensors
using the PyRPlidar library. Integrates with the obstacle detection logic module
and supports data export for post-processing analysis.

References:
- PyLidar: https://www.pylidar.org/en/latest/ (for post-processing LiDAR data files)
- PyRPlidar: https://github.com/SkoltechRobotics/pyrplidar
"""

import sys
import signal
import logging
import time
from typing import List, Tuple, Optional, Generator
from pyrplidar import PyRPlidar

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration constants
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 460800  # Higher baudrate for better performance
DEFAULT_TIMEOUT = 3.0
DEFAULT_MOTOR_PWM = 500  # Motor PWM value (0-1023)
MIN_DISTANCE_MM = 50  # Minimum valid distance in millimeters (5 cm)
MAX_DISTANCE_MM = 3000  # Maximum valid distance in millimeters (300 cm)


class RPLidarReader:
    """
    Professional interface for RPLIDAR A1 sensor communication using PyRPlidar.
    
    Handles sensor initialization, data acquisition, and cleanup operations.
    Provides scan data in format compatible with obstacle detection logic.
    """
    
    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE,
                 motor_pwm: int = DEFAULT_MOTOR_PWM):
        """
        Initialize RPLIDAR reader.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0' on Linux, 'COM3' on Windows)
            baudrate: Serial communication baud rate (default: 460800)
            motor_pwm: Motor PWM value 0-1023 (default: 500)
        """
        self.port = port
        self.baudrate = baudrate
        self.motor_pwm = motor_pwm
        self.lidar: Optional[PyRPlidar] = None
        self._running = False
        self._scan_generator_func = None
        self._scan_generator: Optional[Generator] = None
        
    def connect(self) -> bool:
        """
        Establish connection to RPLIDAR sensor.
        
        Returns:
            True if connection successful, False otherwise
        """
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
        if not self.lidar:
            logger.error("Cannot start: RPLIDAR not connected")
            return False
        
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
        except Exception as e:
            logger.error(f"Failed to start motor: {type(e).__name__}: {e}")
            return False
    
    def stop(self) -> None:
        """Stop the RPLIDAR motor."""
        if self.lidar and self._running:
            try:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
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

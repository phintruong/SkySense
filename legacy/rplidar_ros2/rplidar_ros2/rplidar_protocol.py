#!/usr/bin/env python3
"""
RPLIDAR A1 Serial Protocol Implementation

Implements the official RPLIDAR A1 serial communication protocol.
Based on RPLIDAR A1 SDK protocol specification.
"""

import serial
import struct
import time
import logging
from typing import Optional, Tuple, List
from enum import IntEnum

logger = logging.getLogger(__name__)


class RPLidarCommand(IntEnum):
    """RPLIDAR command codes."""
    STOP = 0x25
    RESET = 0x40
    SCAN = 0x20
    FORCE_SCAN = 0x21
    GET_INFO = 0x50
    GET_HEALTH = 0x52
    GET_SAMPLE_RATE = 0x59


class RPLidarResponse(IntEnum):
    """RPLIDAR response descriptor types."""
    DESCRIPTOR = 0xA5
    INFO = 0x04
    HEALTH = 0x06
    SCAN = 0x81
    SAMPLE_RATE = 0x15


class RPLidarHealth(IntEnum):
    """RPLIDAR health status."""
    GOOD = 0
    WARNING = 1
    ERROR = 2


class RPLidarProtocol:
    """
    RPLIDAR A1 serial protocol handler.
    
    Implements low-level communication with RPLIDAR A1 sensor
    using the official serial protocol.
    """
    
    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize RPLIDAR protocol handler.
        
        Args:
            port: Serial port path
            baudrate: Serial baud rate (115200 for A1)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.motor_running = False
        
    def connect(self) -> bool:
        """
        Connect to RPLIDAR device.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            logger.info(f"Connecting to RPLIDAR on {self.port} at {self.baudrate} baud...")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            # Clear any existing data
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Wait for device to be ready
            time.sleep(0.1)
            
            logger.info("RPLIDAR connected successfully")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to connect to RPLIDAR: {e}")
            return False
        except Exception as e:
            logger.error(f"Unexpected error connecting to RPLIDAR: {type(e).__name__}: {e}")
            return False
    
    def disconnect(self) -> None:
        """Disconnect from RPLIDAR device."""
        self.stop_motor()
        if self.serial and self.serial.is_open:
            self.serial.close()
            logger.info("RPLIDAR disconnected")
    
    def _send_command(self, cmd: int, payload: bytes = b'') -> bool:
        """
        Send command to RPLIDAR.
        
        Args:
            cmd: Command byte
            payload: Optional command payload
            
        Returns:
            True if command sent successfully
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Cannot send command: serial port not open")
            return False
        
        try:
            # Build command packet
            cmd_len = len(payload)
            packet = struct.pack('<BB', self.SYNC_BYTE, cmd)
            packet += struct.pack('<B', cmd_len)
            packet += payload
            
            # Calculate checksum (XOR of all bytes except sync byte)
            checksum = 0
            for b in packet[1:]:
                checksum ^= b
            packet += struct.pack('<B', checksum)
            
            self.serial.write(packet)
            self.serial.flush()
            return True
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False
    
    def _read_response(self, expected_type: int, timeout: float = 1.0) -> Optional[bytes]:
        """
        Read response from RPLIDAR.
        
        Args:
            expected_type: Expected response descriptor type
            timeout: Read timeout in seconds
            
        Returns:
            Response payload bytes, or None if failed
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        start_time = time.time()
        
        try:
            # Wait for sync byte
            while time.time() - start_time < timeout:
                if self.serial.in_waiting > 0:
                    byte = self.serial.read(1)
                    if byte[0] == self.SYNC_BYTE:
                        break
                time.sleep(0.01)
            else:
                logger.warning("Timeout waiting for sync byte")
                return None
            
            # Read descriptor
            descriptor = self.serial.read(4)
            if len(descriptor) < 4:
                logger.warning("Incomplete descriptor")
                return None
            
            resp_type, data_len = struct.unpack('<BBH', descriptor)
            
            if resp_type != expected_type:
                logger.warning(f"Unexpected response type: {resp_type} (expected {expected_type})")
                return None
            
            # Read payload
            if data_len > 0:
                payload = self.serial.read(data_len)
                if len(payload) < data_len:
                    logger.warning("Incomplete payload")
                    return None
                return payload
            else:
                return b''
                
        except Exception as e:
            logger.error(f"Error reading response: {e}")
            return None
    
    def start_motor(self, pwm: int = 660) -> bool:
        """
        Start RPLIDAR motor.
        
        Args:
            pwm: Motor PWM value (0-1023, default 660)
            
        Returns:
            True if motor started successfully
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Cannot start motor: serial port not open")
            return False
        
        try:
            # Set motor PWM via DTR/RTS (hardware control)
            # For RPLIDAR A1, motor is controlled via DTR pin
            self.serial.dtr = False  # Enable motor
            time.sleep(0.1)
            self.motor_running = True
            logger.info(f"Motor started (PWM control via DTR)")
            return True
            
        except Exception as e:
            logger.error(f"Error starting motor: {e}")
            return False
    
    def stop_motor(self) -> None:
        """Stop RPLIDAR motor."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.dtr = True  # Disable motor
                self.motor_running = False
                logger.info("Motor stopped")
            except Exception as e:
                logger.warning(f"Error stopping motor: {e}")
    
    def get_info(self) -> Optional[dict]:
        """
        Get device information.
        
        Returns:
            Dictionary with device info, or None if failed
        """
        if not self._send_command(RPLidarCommand.GET_INFO):
            return None
        
        payload = self._read_response(RPLidarResponse.INFO)
        if payload is None or len(payload) < 20:
            return None
        
        try:
            model, firmware_minor, firmware_major, hardware = struct.unpack('<BBBB', payload[0:4])
            serial_num = struct.unpack('<I', payload[4:8])[0]
            
            return {
                'model': model,
                'firmware': (firmware_major, firmware_minor),
                'hardware': hardware,
                'serial': serial_num
            }
        except Exception as e:
            logger.error(f"Error parsing device info: {e}")
            return None
    
    def get_health(self) -> Optional[dict]:
        """
        Get device health status.
        
        Returns:
            Dictionary with health status, or None if failed
        """
        if not self._send_command(RPLidarCommand.GET_HEALTH):
            return None
        
        payload = self._read_response(RPLidarResponse.HEALTH)
        if payload is None or len(payload) < 3:
            return None
        
        try:
            status, error_code = struct.unpack('<BH', payload[0:3])
            return {
                'status': RPLidarHealth(status),
                'error_code': error_code
            }
        except Exception as e:
            logger.error(f"Error parsing health status: {e}")
            return None
    
    def start_scan(self) -> bool:
        """
        Start scanning mode.
        
        Returns:
            True if scan started successfully
        """
        return self._send_command(RPLidarCommand.SCAN)
    
    def iter_scans(self, max_buf_meas: int = 3000):
        """
        Iterator for scan data.
        
        Args:
            max_buf_meas: Maximum buffer measurements
            
        Yields:
            List of tuples [(angle_deg, distance_mm, quality), ...]
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Cannot scan: serial port not open")
            return
        
        # Start scan
        if not self.start_scan():
            logger.error("Failed to start scan")
            return
        
        # Wait for scan data
        time.sleep(0.1)
        
        points = []
        prev_angle = None
        
        try:
            while True:
                # Read scan packet
                if self.serial.in_waiting < 5:
                    time.sleep(0.001)
                    continue
                
                # Check for sync byte
                byte = self.serial.read(1)
                if byte[0] != 0xA5:
                    continue
                
                # Read descriptor
                descriptor = self.serial.read(4)
                if len(descriptor) < 4:
                    continue
                
                resp_type, data_len = struct.unpack('<BBH', descriptor)
                
                if resp_type != RPLidarResponse.SCAN:
                    # Skip non-scan data
                    if data_len > 0:
                        self.serial.read(data_len)
                    continue
                
                # Read scan data
                if data_len < 5:
                    continue
                
                scan_data = self.serial.read(data_len)
                if len(scan_data) < data_len:
                    continue
                
                # Parse scan data (each measurement is 5 bytes)
                num_meas = (data_len - 1) // 5
                
                for i in range(num_meas):
                    offset = i * 5
                    if offset + 5 > len(scan_data):
                        break
                    
                    # Parse measurement
                    byte_data = scan_data[offset:offset+5]
                    
                    # Check quality (bit 0 of first byte)
                    quality = byte_data[0] >> 2
                    check_bit = (byte_data[0] >> 1) & 0x01
                    
                    if check_bit != 1:
                        continue  # Invalid measurement
                    
                    # Parse angle and distance
                    angle_raw = ((byte_data[1] >> 1) | (byte_data[2] << 7)) / 64.0
                    distance_raw = (byte_data[3] | (byte_data[4] << 8)) / 4.0
                    
                    # Convert to degrees and millimeters
                    angle_deg = angle_raw
                    distance_mm = distance_raw
                    
                    # Filter invalid measurements
                    if distance_mm < 1.0 or distance_mm > 12000.0:
                        continue
                    
                    points.append((angle_deg, distance_mm, quality))
                    
                    # Detect sweep completion (angle wraps around)
                    if prev_angle is not None and angle_deg < prev_angle:
                        if points:
                            yield points
                        points = []
                    
                    prev_angle = angle_deg
                    
        except Exception as e:
            logger.error(f"Error in scan iteration: {e}")
            raise


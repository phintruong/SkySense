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
    SCAN_ALT = 0x82  # A1 can use 0x81 or 0x82 for scan data
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
    # Some units / clones send 0x50 as second byte (e.g. after GET_INFO) instead of 0x5A
    SYNC_BYTE2_ALT = 0x50
    
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
            # Build command packet: [sync] [cmd]
            packet = struct.pack('<BB', self.SYNC_BYTE, cmd)

            # Only append size + payload + checksum for commands WITH payload.
            # Commands without payload are just 2 bytes per the RPLIDAR protocol.
            if payload:
                packet += struct.pack('<B', len(payload))
                packet += payload
                checksum = 0
                for b in packet[1:]:
                    checksum ^= b
                packet += struct.pack('<B', checksum)

            logger.debug(f"TX: {packet.hex()}")
            self.serial.write(packet)
            self.serial.flush()
            return True

        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False
    
    def _read_response(self, expected_type: int, timeout: float = 1.0) -> Optional[bytes]:
        """
        Read response from RPLIDAR.

        Standard descriptor (after sync): 5 bytes
            [4 bytes: 30-bit data length + 2-bit send mode] [1 byte: data type]
        Fallback for non-standard devices: 2 bytes [type] [length]

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
            # Wait for sync bytes 0xA5 0x5A/0x50
            sync_ok = False
            while time.time() - start_time < timeout:
                if self.serial.in_waiting >= 1:
                    b = self.serial.read(1)[0]
                    if b == self.SYNC_BYTE:
                        t2 = time.time()
                        while time.time() - t2 < 0.2:
                            if self.serial.in_waiting >= 1:
                                b2 = self.serial.read(1)[0]
                                if b2 in (self.SYNC_BYTE2, self.SYNC_BYTE2_ALT):
                                    sync_ok = True
                                    break
                                break
                            time.sleep(0.005)
                        if sync_ok:
                            break
                time.sleep(0.01)
            if not sync_ok:
                logger.warning("Timeout waiting for sync byte")
                return None

            # Standard RPLIDAR descriptor is 5 bytes after sync:
            #   bytes 0-3: 30-bit data length + 2-bit send mode (little-endian uint32)
            #   byte 4:    data type
            # Some non-standard units send fewer bytes; fall back to 2-byte descriptor.
            desc_deadline = time.time() + 0.3
            while time.time() < desc_deadline and self.serial.in_waiting < 5:
                time.sleep(0.01)

            avail = min(self.serial.in_waiting, 5)
            descriptor = self.serial.read(avail)
            logger.debug(f"Descriptor raw ({len(descriptor)} bytes): {descriptor.hex()}")

            if len(descriptor) >= 5:
                # Standard 5-byte descriptor
                raw_len_mode = struct.unpack('<I', descriptor[0:4])[0]
                data_len = raw_len_mode & 0x3FFFFFFF
                resp_type = descriptor[4]
            elif len(descriptor) >= 2:
                # Short descriptor fallback (non-standard device)
                resp_type = descriptor[0]
                data_len = descriptor[1]
                logger.info(
                    f"Short descriptor ({len(descriptor)} bytes): "
                    f"type=0x{resp_type:02x}, len={data_len}"
                )
            else:
                logger.warning(f"Incomplete descriptor ({len(descriptor)} bytes)")
                if descriptor:
                    logger.debug(f"  raw: {descriptor.hex()}")
                return None

            logger.debug(f"Parsed descriptor: type=0x{resp_type:02x}, len={data_len}")

            # Accept type 0 as wildcard — some units send 0x00 for all response types
            if resp_type != expected_type and resp_type != 0:
                logger.warning(
                    f"Unexpected response type: 0x{resp_type:02x} "
                    f"(expected 0x{expected_type:02x})"
                )
                return None

            # Cap payload length to something sane (protocol max is ~64 KB)
            if data_len > 1024:
                logger.warning(f"Suspiciously large payload length {data_len}, capping to 256")
                data_len = 256

            # Read payload; wait for full payload if it arrives slowly
            if data_len > 0:
                payload_deadline = time.time() + 0.5
                payload = b''
                while len(payload) < data_len and time.time() < payload_deadline:
                    chunk = self.serial.read(data_len - len(payload))
                    if not chunk:
                        time.sleep(0.01)
                        continue
                    payload += chunk
                if len(payload) < data_len:
                    logger.warning(
                        f"Incomplete payload ({len(payload)}/{data_len} bytes)"
                    )
                    if payload:
                        logger.debug(f"  partial payload: {payload[:32].hex()}")
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
        Uses FORCE_SCAN (0x21) for better compatibility with RPLIDAR A1 on UART.
        """
        return self._send_command(RPLidarCommand.FORCE_SCAN)
    
    def iter_scans(self, max_buf_meas: int = 3000):
        """
        Iterator for scan data.

        This LIDAR streams raw 5-byte measurement packets after FORCE_SCAN,
        without the standard A5 5A response wrapper.

        Args:
            max_buf_meas: Maximum buffer measurements

        Yields:
            List of tuples [(angle_deg, distance_mm, quality), ...]
        """
        if not self.serial or not self.serial.is_open:
            logger.error("Cannot scan: serial port not open")
            return

        # Test communication first (only if motor not yet running)
        if not self.motor_running:
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            time.sleep(0.05)
            logger.info("Testing communication with GET_INFO...")
            info = self.get_info()
            if info:
                logger.info(f"Device info: Model={info.get('model')}, Serial={info.get('serial')}")
            else:
                logger.warning("GET_INFO failed - device may not be responding.")

        # Start scan
        if not self.start_scan():
            logger.error("Failed to start scan")
            return

        # Wait for scan data to start arriving
        time.sleep(0.3)
        self.serial.reset_input_buffer()
        time.sleep(0.1)

        points = []
        prev_angle = None
        _debug_count = 0
        _debug_last_log = time.time()

        try:
            while True:
                # Wait for at least 5 bytes (one measurement packet)
                if self.serial.in_waiting < 5:
                    time.sleep(0.001)
                    # Debug: log every 3s if no data
                    if _debug_count == 0 and (time.time() - _debug_last_log) > 3.0:
                        logger.warning(
                            f"No scan data yet. in_waiting={self.serial.in_waiting}"
                        )
                        _debug_last_log = time.time()
                    continue

                # Read one 5-byte measurement packet
                # Format: [S][Angle_L][Angle_H][Dist_L][Dist_H]
                # S: bit 0 = start flag (new 360° sweep), bits 1-7 = quality
                byte_data = self.serial.read(5)
                if len(byte_data) < 5:
                    continue

                _debug_count += 1

                # Parse measurement
                quality = byte_data[0] >> 1
                start_flag = byte_data[0] & 0x01

                # Angle: 15-bit value in bits [1..15] of bytes [1..2]
                angle_raw = ((byte_data[1] >> 1) | (byte_data[2] << 7)) / 64.0

                # Distance: 16-bit value in bytes [3..4]
                distance_raw = (byte_data[3] | (byte_data[4] << 8)) / 4.0

                # Convert to degrees and millimeters
                angle_deg = angle_raw
                distance_mm = distance_raw

                # Filter invalid measurements
                if 1.0 <= distance_mm <= 12000.0:
                    points.append((angle_deg, distance_mm, quality))

                # Detect sweep completion
                # Method 1: start_flag indicates new sweep
                if start_flag == 1 and len(points) > 10:
                    yield points
                    points = []

                # Method 2: angle wraps around (fallback)
                if prev_angle is not None and angle_deg < prev_angle and len(points) > 10:
                    if start_flag == 0:  # Only yield if we didn't already yield via start_flag
                        yield points
                        points = []

                prev_angle = angle_deg

        except Exception as e:
            logger.error(f"Error in scan iteration: {e}")
            raise


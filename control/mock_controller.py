"""
Mock serial controller for the 6-axis robotic arm.

Simulates an Arduino/STM32 microcontroller that accepts binary commands,
applies realistic timing delays, and returns mock status responses.
"""

import logging
import time
from typing import Optional
import numpy as np

from control.protocol import SerialProtocol


# Configure logging for the module
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Add console handler if not already present
if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


class MockController:
    """
    Simulates a serial-connected microcontroller for the 6-axis robotic arm.

    Does not open actual serial ports. Instead, stores configuration and simulates
    command processing with realistic delays and mock response generation.
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200) -> None:
        """
        Initialize the mock controller.

        Args:
            port: Serial port name (not actually opened; stored for logging).
            baudrate: Baud rate (not used in simulation, stored for reference).
        """
        self._port: str = port
        self._baudrate: int = baudrate
        self._connected: bool = False
        self._enabled: bool = False

        # Mock state
        self._current_angles: np.ndarray = np.zeros(6, dtype=np.float32)
        self._current_e: float = 0.0     # Extrusion position in mm
        self._temperature: float = 25.0  # Celsius
        self._fan_speed: int = 0         # 0-255
        self._error_flags: int = 0
        self._move_count: int = 0

        logger.info(f"MockController initialized: port={port}, baudrate={baudrate}")

    def connect(self) -> bool:
        """
        Simulate connection to the controller.

        Logs a connection message and sets the connected flag.

        Returns:
            True (always successful in mock).
        """
        self._connected = True
        logger.info(f"Connected to mock controller on {self._port}")
        return True

    def disconnect(self) -> None:
        """
        Simulate disconnection from the controller.

        Clears the connected flag.
        """
        self._connected = False
        logger.info("Disconnected from mock controller")

    def send_command(self, cmd_bytes: bytes) -> Optional[bytes]:
        """
        Send a command to the mock controller and receive response.

        Decodes the incoming command, applies a small random delay (10-50 ms),
        updates internal state, and returns an appropriate mock response.

        Args:
            cmd_bytes: Encoded command frame as bytes.

        Returns:
            Mock ACK/NACK response frame, or None if not connected.

        Raises:
            ValueError: If the frame format is invalid.
        """
        if not self._connected:
            logger.warning("Cannot send command: not connected")
            return None

        # Log command (first few bytes in hex)
        logger.debug(f"Sending command: {cmd_bytes.hex()[:40]}...")

        # Parse frame
        if len(cmd_bytes) < 4:
            logger.error("Invalid command frame: too short")
            raise ValueError("Command frame too short")

        if cmd_bytes[0] != SerialProtocol.SYNC_BYTE_1 or cmd_bytes[1] != SerialProtocol.SYNC_BYTE_2:
            logger.error(f"Invalid sync bytes: {cmd_bytes[0]:02x} {cmd_bytes[1]:02x}")
            raise ValueError("Invalid sync bytes")

        command = cmd_bytes[2]
        length = cmd_bytes[3]
        payload = cmd_bytes[4:4+length] if length > 0 else b''
        checksum_received = cmd_bytes[4+length] if len(cmd_bytes) > 4 + length else 0

        # Verify checksum
        checksum_calc = SerialProtocol.checksum(payload)
        if checksum_received != checksum_calc:
            logger.warning(f"Checksum mismatch: expected {checksum_calc:02x}, got {checksum_received:02x}")

        # Process command and update state
        logger.debug(f"Processing command: 0x{command:02x}")

        if command == SerialProtocol.CMD_MOVE:
            if len(payload) < 12 + 4 + 4:  # 6 int16 + 1 int32 + 1 float32 = 20
                logger.error("MOVE command: insufficient data")
                return self._encode_nack(0x01)  # Invalid data length

            # Extract angles, E position and feedrate
            angles = np.array([
                SerialProtocol._decode_int16(payload, i*2) / SerialProtocol.ANGLE_SCALE
                for i in range(6)
            ], dtype=np.float32)

            e_int = int.from_bytes(payload[12:16], byteorder='little', signed=True)
            e_pos = e_int / SerialProtocol.EXTRUDER_SCALE

            feedrate = SerialProtocol._decode_float32(payload, 16)

            self._current_angles = angles
            self._current_e = e_pos
            self._move_count += 1
            
            # Slight temp variation, simulating cooling
            if self._temperature > 25.0:
                 self._temperature -= (self._fan_speed / 255.0) * 0.1

            logger.info(f"MOVE command: axes={angles}, e={e_pos}, feedrate={feedrate}%")

        elif command == SerialProtocol.CMD_HOME:
            self._current_angles = np.zeros(6, dtype=np.float32)
            self._current_e = 0.0
            logger.info("HOME command: all axes zeroed")

        elif command == SerialProtocol.CMD_SET_SPEED:
            if len(payload) < 4:
                logger.error("SET_SPEED command: insufficient data")
                return self._encode_nack(0x02)
            speed = SerialProtocol._decode_float32(payload, 0)
            logger.info(f"SET_SPEED command: speed={speed}%")

        elif command == SerialProtocol.CMD_SET_TEMP:
            if len(payload) < 4:
                return self._encode_nack(0x03)
            target_temp = SerialProtocol._decode_float32(payload, 0)
            self._temperature = target_temp  # Instant heat for mock
            logger.info(f"SET_TEMP command: temp={target_temp}C")

        elif command == SerialProtocol.CMD_SET_FAN:
            if len(payload) < 1:
                return self._encode_nack(0x04)
            self._fan_speed = payload[0]
            logger.info(f"SET_FAN command: speed={self._fan_speed}")

        elif command == SerialProtocol.CMD_ENABLE:
            self._enabled = True
            logger.info("ENABLE command: servos enabled")

        elif command == SerialProtocol.CMD_DISABLE:
            self._enabled = False
            logger.info("DISABLE command: servos disabled")

        elif command == SerialProtocol.CMD_QUERY:
            logger.debug("QUERY command: returning status")
            # Simulate delay for query
            time.sleep(np.random.uniform(0.01, 0.05))
            return self._encode_data_response()

        elif command == SerialProtocol.CMD_EMERGENCY_STOP:
            self._enabled = False
            self._error_flags |= 0x0001
            logger.warning("EMERGENCY_STOP command issued")

        else:
            logger.warning(f"Unknown command: 0x{command:02x}")
            return self._encode_nack(0xFF)

        # Simulate transmission delay
        delay = np.random.uniform(0.01, 0.05)
        time.sleep(delay)
        logger.debug(f"Command processed (delay: {delay*1000:.1f}ms)")

        return self._encode_ack()

    def move_to(self, joint_angles: np.ndarray, feedrate: float = 100.0) -> bool:
        """
        High-level move command: encode angles, send, verify ACK.

        Args:
            joint_angles: Array of 6 target angles in degrees.
            feedrate: Feedrate percentage (default 100%).

        Returns:
            True if move command succeeded, False otherwise.
        """
        logger.info(f"move_to called: angles={joint_angles}, feedrate={feedrate}")

        cmd = SerialProtocol.encode_move(joint_angles, feedrate)
        response = self.send_command(cmd)

        if response is None:
            logger.error("move_to: no response from controller")
            return False

        result = SerialProtocol.decode_response(response)
        success = result.get('valid', False) and result.get('status') == 'ACK'

        if success:
            logger.info(f"move_to: command accepted")
        else:
            logger.error(f"move_to: command rejected or invalid response")

        return success

    def home(self) -> bool:
        """
        Send a home (zero all axes) command.

        Returns:
            True if home command succeeded, False otherwise.
        """
        logger.info("home called")

        cmd = SerialProtocol.encode_home()
        response = self.send_command(cmd)

        if response is None:
            logger.error("home: no response from controller")
            return False

        result = SerialProtocol.decode_response(response)
        success = result.get('valid', False) and result.get('status') == 'ACK'

        if success:
            logger.info("home: all axes zeroed successfully")
        else:
            logger.error("home: command failed")

        return success

    def emergency_stop(self) -> bool:
        """
        Send an emergency stop command.

        Returns:
            True if command succeeded, False otherwise.
        """
        logger.warning("emergency_stop called")

        cmd = SerialProtocol.encode_emergency_stop()
        response = self.send_command(cmd)

        if response is None:
            logger.error("emergency_stop: no response from controller")
            return False

        result = SerialProtocol.decode_response(response)
        success = result.get('valid', False)

        if success:
            logger.warning("emergency_stop: arm halted")
        else:
            logger.error("emergency_stop: command failed")

        return success

    def get_status(self) -> dict:
        """
        Query controller status (non-blocking mock implementation).

        Returns:
            Dictionary with keys:
                - 'connected': bool, whether controller is connected
                - 'enabled': bool, whether servos are enabled
                - 'current_angles': np.ndarray, current joint angles
                - 'temperature': float, mock temperature reading
                - 'error_flags': int, bitmask of error conditions
                - 'move_count': int, number of moves executed
        """
        return {
            'connected': self._connected,
            'enabled': self._enabled,
            'current_angles': self._current_angles.copy(),
            'current_e': self._current_e,
            'temperature': round(self._temperature, 2),
            'fan_speed': self._fan_speed,
            'error_flags': self._error_flags,
            'move_count': self._move_count,
        }

    def execute_trajectory(
        self,
        trajectory: np.ndarray,
        dt: float = 0.05
    ) -> bool:
        """
        Execute a trajectory (sequence of waypoints).

        Sends each row of the trajectory with timing between moves.

        Args:
            trajectory: Shape (N, 7) array where each row is joint angles + E in degrees/mm.
            dt: Time interval (in seconds) between waypoint sends (default 0.05s).

        Returns:
            True if all moves succeeded, False if any failed.

        Raises:
            ValueError: If trajectory does not have 7 columns.
        """
        if trajectory.ndim < 2 or len(trajectory) == 0:
            logger.warning("execute_trajectory: empty trajectory, nothing to send.")
            return True
        if trajectory.shape[1] != 7:
            logger.error(f"execute_trajectory: invalid trajectory shape {trajectory.shape}")
            raise ValueError(f"Trajectory must have 7 columns, got {trajectory.shape[1]}")

        logger.info(f"execute_trajectory: {len(trajectory)} waypoints, dt={dt}s")

        all_success = True
        for i, waypoint in enumerate(trajectory):
            logger.debug(f"Trajectory waypoint {i+1}/{len(trajectory)}: {waypoint}")

            if not self.move_to(waypoint):
                logger.error(f"execute_trajectory: move {i+1} failed")
                all_success = False
                break

            if i < len(trajectory) - 1:
                time.sleep(dt)

        if all_success:
            logger.info("execute_trajectory: completed successfully")
        else:
            logger.warning("execute_trajectory: interrupted")

        return all_success

    def _encode_ack(self) -> bytes:
        """Generate an ACK response frame."""
        frame = bytes([
            SerialProtocol.SYNC_BYTE_1,
            SerialProtocol.SYNC_BYTE_2,
            SerialProtocol.RESP_ACK,
            0  # No payload
        ])
        frame += bytes([SerialProtocol.checksum(b'')])
        return frame

    def _encode_nack(self, error_code: int) -> bytes:
        """Generate a NACK response frame with error code."""
        payload = bytes([error_code])
        frame = bytes([
            SerialProtocol.SYNC_BYTE_1,
            SerialProtocol.SYNC_BYTE_2,
            SerialProtocol.RESP_NACK,
            len(payload)
        ])
        frame += payload
        frame += bytes([SerialProtocol.checksum(payload)])
        return frame

    def _encode_data_response(self) -> bytes:
        """Generate a DATA response frame with current status."""
        payload = b''

        # Add 6 joint angles as int16
        for angle in self._current_angles:
            angle_int = int(angle * SerialProtocol.ANGLE_SCALE)
            payload += SerialProtocol._encode_int16(angle_int)

        # Add E pos as int32
        e_int = int(self._current_e * SerialProtocol.EXTRUDER_SCALE)
        payload += e_int.to_bytes(4, byteorder='little', signed=True)

        # Add temperature as float32
        payload += SerialProtocol._encode_float32(self._temperature)

        # Add 2 bytes of error flags
        payload += self._error_flags.to_bytes(2, byteorder='little')

        frame = bytes([
            SerialProtocol.SYNC_BYTE_1,
            SerialProtocol.SYNC_BYTE_2,
            SerialProtocol.RESP_DATA,
            len(payload)
        ])
        frame += payload
        frame += bytes([SerialProtocol.checksum(payload)])

        return frame

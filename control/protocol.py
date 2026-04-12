"""
Serial communication protocol for the 6-axis robotic arm.

Defines the binary frame format, command encoding, and response decoding
for communicating with the embedded controller (Arduino/STM32).
"""

from typing import Dict, Optional
import numpy as np


class SerialProtocol:
    """
    Manages binary protocol for serial communication with the robotic arm controller.

    Frame format:
        [SYNC_BYTE_1, SYNC_BYTE_2, COMMAND, LENGTH, DATA..., CHECKSUM]

    All multi-byte integers are little-endian. Joint angles encoded as int16 in 0.01° units.
    Feedrate encoded as float (4 bytes).
    """

    # Frame synchronization
    SYNC_BYTE_1: int = 0xAA
    SYNC_BYTE_2: int = 0x55

    # Command codes
    CMD_MOVE: int = 0x01
    CMD_HOME: int = 0x02
    CMD_SET_SPEED: int = 0x03
    CMD_ENABLE: int = 0x04
    CMD_DISABLE: int = 0x05
    CMD_QUERY: int = 0x06
    CMD_SET_TEMP: int = 0x07
    CMD_SET_FAN: int = 0x08
    CMD_EMERGENCY_STOP: int = 0xFF

    # Response codes
    RESP_ACK: int = 0xAA
    RESP_NACK: int = 0x55
    RESP_DATA: int = 0x66

    # Constants
    NUM_AXES: int = 7  # 6 joints + 1 extruder
    ANGLE_SCALE: float = 100.0  # 0.01° per unit
    EXTRUDER_SCALE: float = 1000.0 # 0.001 mm per unit

    @staticmethod
    def checksum(data: bytes) -> int:
        """
        Calculate XOR checksum over data bytes.

        Args:
            data: Byte sequence to checksum.

        Returns:
            XOR of all bytes in data.
        """
        result = 0
        for byte in data:
            result ^= byte
        return result & 0xFF

    @staticmethod
    def _encode_int16(value: int) -> bytes:
        """Encode int16 as little-endian bytes."""
        return int(value).to_bytes(2, byteorder='little', signed=True)

    @staticmethod
    def _encode_float32(value: float) -> bytes:
        """Encode float32 as little-endian bytes."""
        return np.float32(value).tobytes()

    @staticmethod
    def _decode_int16(data: bytes, offset: int) -> int:
        """Decode int16 from little-endian bytes at offset."""
        return int.from_bytes(data[offset:offset+2], byteorder='little', signed=True)

    @staticmethod
    def _decode_float32(data: bytes, offset: int) -> float:
        """Decode float32 from little-endian bytes at offset."""
        return np.frombuffer(data[offset:offset+4], dtype=np.float32)[0]

    @classmethod
    def encode_move(
        cls,
        axes: np.ndarray,
        feedrate: float = 100.0
    ) -> bytes:
        """
        Encode a move command with 6 joint angles, 1 extruder position, and feedrate.

        Args:
            axes: Array of 7 values (6 angles in degrees, 1 extruder pos in mm).
            feedrate: Feedrate percentage (default 100.0 for full speed).

        Returns:
            Binary frame ready to send to controller.

        Raises:
            ValueError: If axes does not have exactly 7 elements.
        """
        if len(axes) != cls.NUM_AXES:
            raise ValueError(f"Expected {cls.NUM_AXES} axes, got {len(axes)}")

        # Encode joint angles as int16 (0.01° units)
        data = b''
        for i in range(6):
            angle_int = int(axes[i] * cls.ANGLE_SCALE)
            data += cls._encode_int16(angle_int)
        
        # Encode extruder pos as int32 (0.001 mm units) - 4 bytes
        e_int = int(axes[6] * cls.EXTRUDER_SCALE)
        data += e_int.to_bytes(4, byteorder='little', signed=True)

        # Encode feedrate as float32
        data += cls._encode_float32(feedrate)

        # Build frame
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_MOVE, len(data)])
        frame += data
        frame += bytes([cls.checksum(data)])

        return frame

    @classmethod
    def encode_home(cls) -> bytes:
        """
        Encode a home (zero all axes) command.

        Returns:
            Binary frame ready to send to controller.
        """
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_HOME, 0])
        frame += bytes([cls.checksum(b'')])
        return frame

    @classmethod
    def encode_emergency_stop(cls) -> bytes:
        """
        Encode an emergency stop command.

        Returns:
            Binary frame ready to send to controller.
        """
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_EMERGENCY_STOP, 0])
        frame += bytes([cls.checksum(b'')])
        return frame

    @classmethod
    def encode_set_temp(cls, temperature: float) -> bytes:
        """
        Encode a set hotend temperature command.
        """
        data = cls._encode_float32(temperature)
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_SET_TEMP, len(data)])
        frame += data
        frame += bytes([cls.checksum(data)])
        return frame

    @classmethod
    def encode_set_fan(cls, fan_speed: int) -> bytes:
        """
        Encode a set cooling fan speed command (0-255).
        """
        data = bytes([fan_speed & 0xFF])
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_SET_FAN, len(data)])
        frame += data
        frame += bytes([cls.checksum(data)])
        return frame

    @classmethod
    def encode_set_speed(cls, speed_percent: float) -> bytes:
        """
        Encode a set speed command.

        Args:
            speed_percent: Speed as percentage (0-100).

        Returns:
            Binary frame ready to send to controller.
        """
        data = cls._encode_float32(speed_percent)
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_SET_SPEED, len(data)])
        frame += data
        frame += bytes([cls.checksum(data)])
        return frame

    @classmethod
    def encode_enable(cls) -> bytes:
        """
        Encode an enable servos command.

        Returns:
            Binary frame ready to send to controller.
        """
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_ENABLE, 0])
        frame += bytes([cls.checksum(b'')])
        return frame

    @classmethod
    def encode_disable(cls) -> bytes:
        """
        Encode a disable servos command.

        Returns:
            Binary frame ready to send to controller.
        """
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_DISABLE, 0])
        frame += bytes([cls.checksum(b'')])
        return frame

    @classmethod
    def encode_query(cls) -> bytes:
        """
        Encode a query status command.

        Returns:
            Binary frame ready to send to controller.
        """
        frame = bytes([cls.SYNC_BYTE_1, cls.SYNC_BYTE_2, cls.CMD_QUERY, 0])
        frame += bytes([cls.checksum(b'')])
        return frame

    @classmethod
    def decode_response(cls, data: bytes) -> Dict[str, any]:
        """
        Parse a response frame from the controller.

        Expected frame: [SYNC1, SYNC2, RESPONSE_CODE, LENGTH, DATA..., CHECKSUM]

        Args:
            data: Raw bytes received from controller.

        Returns:
            Dictionary with keys: 'valid', 'response_code', 'status', 'angles', 'temperature', 'error_flags'
            If frame is invalid, 'valid' is False and other keys may be absent.
        """
        result: Dict[str, any] = {'valid': False}

        # Minimum frame: SYNC1, SYNC2, CODE, LEN, CHECKSUM = 5 bytes
        if len(data) < 5:
            return result

        # Check sync bytes
        if data[0] != cls.SYNC_BYTE_1 or data[1] != cls.SYNC_BYTE_2:
            return result

        response_code = data[2]
        length = data[3]

        # Verify frame length
        if len(data) < 5 + length:
            return result

        payload = data[4:4+length]
        checksum_received = data[4+length]
        checksum_calc = cls.checksum(payload)

        if checksum_received != checksum_calc:
            return result

        result['valid'] = True
        result['response_code'] = response_code

        # Parse based on response code
        if response_code == cls.RESP_ACK:
            result['status'] = 'ACK'
        elif response_code == cls.RESP_NACK:
            result['status'] = 'NACK'
            if len(payload) > 0:
                result['error_code'] = payload[0]
        elif response_code == cls.RESP_DATA and len(payload) >= 22:
            # Data response: 6 int16 angles (12 bytes) + 1 int32 E pos (4 bytes) + temperature float32 (4 bytes) + 2 bytes error flags = 22 bytes
            result['status'] = 'DATA'
            result['angles'] = np.array([
                cls._decode_int16(payload, i*2) / cls.ANGLE_SCALE
                for i in range(6)
            ])
            # Decode E position (bytes 12-15)
            e_int = int.from_bytes(payload[12:16], byteorder='little', signed=True)
            result['e_pos'] = e_int / cls.EXTRUDER_SCALE
            # Decode Temperature (bytes 16-19)
            result['temperature'] = cls._decode_float32(payload, 16)
            # Decode error flags (bytes 20-21)
            result['error_flags'] = int.from_bytes(payload[20:22], byteorder='little')

        return result

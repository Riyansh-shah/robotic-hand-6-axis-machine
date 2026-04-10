"""
Control module for 6-axis robotic arm mock serial controller.

Provides a simulated serial interface for testing joint movement commands,
communication protocol encoding/decoding, and trajectory execution.
"""

from control.protocol import SerialProtocol
from control.mock_controller import MockController

__all__ = ["MockController", "SerialProtocol"]

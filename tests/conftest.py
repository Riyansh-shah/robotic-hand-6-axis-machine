"""Shared pytest fixtures for PBL project tests."""

import pytest
import numpy as np
import tempfile
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from kinematics import DH6R, get_dh_table
from gcode_parser import GCodeParser


@pytest.fixture
def dh_table():
    """Fixture providing the standard DH parameter table for the 6R arm."""
    return get_dh_table()


@pytest.fixture
def sample_joint_angles():
    """Fixture providing several known joint angle configurations for testing.

    Returns a dictionary with named configurations:
    - 'home': all zeros (home position)
    - 'config_1': moderate angles
    - 'config_2': different moderate angles
    - 'config_3': random valid angles
    """
    return {
        'home': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'config_1': np.array([0.5, 0.3, -0.2, 0.1, -0.4, 0.6]),
        'config_2': np.array([-0.3, 0.6, 0.2, -0.5, 0.1, -0.3]),
        'config_3': np.array([0.7, -0.4, 0.5, -0.2, 0.3, -0.6]),
    }


@pytest.fixture
def sample_gcode_file(tmp_path):
    """Fixture that creates a temporary G-code file with standard commands.

    The file includes:
    - G28: Home command
    - G1: Linear move with extrusion
    - G0: Rapid move
    - G1: Another move
    - Comments and temperature commands

    Args:
        tmp_path: pytest built-in fixture for temporary directory

    Returns:
        Path object pointing to the temporary G-code file
    """
    gcode_content = """; Test G-code file
; Generated for unit testing
G28 ; Home all axes
G1 X10 Y20 Z0.3 E1.5 F1200 ; Linear move with extrusion
G0 X0 Y0 Z5 ; Rapid move to safe height
G1 X15 Y25 Z0.3 E2.0 F1200 ; Another linear move
M104 S200 ; Set nozzle temperature
G1 X30 Y30 Z0.5 E3.0 F1000 ; Final move
"""
    gcode_file = tmp_path / "test.gcode"
    gcode_file.write_text(gcode_content)
    return gcode_file

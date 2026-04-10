"""G-code parser for extracting Cartesian waypoints from CNC/3D printer files."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import re


@dataclass
class Waypoint:
    """Represents a single waypoint in the tool path.

    Attributes:
        x, y, z: Cartesian coordinates in metres.
        e: Extrusion amount in millimetres.
        feedrate: Feed rate in mm/min.
        move_type: Type of move ('rapid', 'linear', or 'home').
    """
    x: float
    y: float
    z: float
    e: float = 0.0
    feedrate: float = 0.0
    move_type: str = "linear"


class GCodeParser:
    """Parser for standard G-code files from Cura/PrusaSlicer.

    Reads G-code line-by-line, tracks position state, and extracts Cartesian
    waypoints with extrusion information. Ignores comments and unknown commands
    gracefully.
    """

    def __init__(self, scale_factor: float = 0.001) -> None:
        """Initialize the parser.

        Args:
            scale_factor: Factor to convert mm to metres. Default 0.001 (mm→m).
        """
        self.scale_factor = scale_factor
        self._waypoints: list[Waypoint] = []
        self._metadata: dict = {}

        # Track current position (in original units, usually mm)
        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_z: float = 0.0
        self._current_e: float = 0.0
        self._current_feedrate: float = 0.0

        # Track extrusion mode (absolute or relative)
        self._relative_extrusion: bool = False

    def parse_file(self, filepath: str) -> list[Waypoint]:
        """Read and parse a G-code file.

        Args:
            filepath: Path to the .gcode file.

        Returns:
            List of Waypoint objects extracted from the file.
        """
        self._waypoints = []
        self._metadata = {}
        self._reset_position()

        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                for line in f:
                    waypoint = self.parse_line(line.strip())
                    if waypoint is not None:
                        self._waypoints.append(waypoint)
        except FileNotFoundError:
            raise FileNotFoundError(f"G-code file not found: {filepath}")
        except Exception as e:
            raise RuntimeError(f"Error parsing G-code file {filepath}: {e}")

        return self._waypoints

    def parse_line(self, line: str) -> Optional[Waypoint]:
        """Parse a single G-code line.

        Handles:
        - G0: Rapid move
        - G1: Linear move with extrusion
        - G28: Home
        - G92: Set position
        - M104/M109: Set temperature (stored as metadata)
        - M106/M107: Fan control (stored as metadata)
        - Comments (lines starting with ;)
        - Unknown commands (ignored gracefully)

        Args:
            line: A single G-code line.

        Returns:
            A Waypoint if the line represents a movement, None otherwise.
        """
        # Strip comments
        if ';' in line:
            line = line[:line.index(';')]

        line = line.strip()
        if not line:
            return None

        # Parse command and parameters
        tokens = line.split()
        if not tokens:
            return None

        command = tokens[0].upper()
        params = self._parse_params(line)

        # Handle temperature commands
        if command == 'M104' or command == 'M109':
            if 'S' in params:
                temp = int(params['S'])
                if 'nozzle_temp' not in self._metadata:
                    self._metadata['nozzle_temp'] = temp
            return None

        # Handle fan control
        if command == 'M106':
            if 'S' in params:
                fan_speed = int(params['S'])
                self._metadata['fan_speed'] = fan_speed
            return None

        if command == 'M107':
            self._metadata['fan_speed'] = 0
            return None

        # Handle set position
        if command == 'G92':
            if 'X' in params:
                self._current_x = float(params['X'])
            if 'Y' in params:
                self._current_y = float(params['Y'])
            if 'Z' in params:
                self._current_z = float(params['Z'])
            if 'E' in params:
                self._current_e = float(params['E'])
            return None

        # Handle homing
        if command == 'G28':
            # Home all or specified axes
            if 'X' in params or 'Y' in params or 'Z' in params or not params:
                # Return waypoint for home position (typically 0, 0, 0)
                self._current_x = 0.0
                self._current_y = 0.0
                self._current_z = 0.0
                return Waypoint(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    e=self._current_e,
                    feedrate=self._current_feedrate,
                    move_type="home"
                )
            return None

        # Handle extrusion mode
        if command == 'G90':
            self._relative_extrusion = False
            return None

        if command == 'G91':
            self._relative_extrusion = True
            return None

        # Handle rapid (G0) and linear (G1) moves
        if command == 'G0' or command == 'G1':
            move_type = "rapid" if command == 'G0' else "linear"

            # Extract coordinates (all optional)
            x = float(params['X']) if 'X' in params else self._current_x
            y = float(params['Y']) if 'Y' in params else self._current_y
            z = float(params['Z']) if 'Z' in params else self._current_z
            feedrate = float(params['F']) if 'F' in params else self._current_feedrate

            # Extract extrusion
            e = self._current_e
            if 'E' in params:
                e_value = float(params['E'])
                if self._relative_extrusion:
                    e = self._current_e + e_value
                else:
                    e = e_value

            # Update state
            self._current_x = x
            self._current_y = y
            self._current_z = z
            self._current_e = e
            self._current_feedrate = feedrate

            # Only emit waypoint if coordinates actually changed
            waypoint = Waypoint(
                x=x * self.scale_factor,
                y=y * self.scale_factor,
                z=z * self.scale_factor,
                e=e,
                feedrate=feedrate,
                move_type=move_type
            )
            return waypoint

        # Ignore unknown commands gracefully
        return None

    def get_waypoints(self) -> list[Waypoint]:
        """Return the list of parsed waypoints.

        Returns:
            List of Waypoint objects.
        """
        return self._waypoints

    def get_metadata(self) -> dict:
        """Return metadata collected during parsing.

        Returns:
            Dictionary with keys like 'nozzle_temp', 'fan_speed', etc.
        """
        return self._metadata.copy()

    def _reset_position(self) -> None:
        """Reset position tracking to origin."""
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_z = 0.0
        self._current_e = 0.0
        self._current_feedrate = 0.0
        self._relative_extrusion = False

    @staticmethod
    def _parse_params(line: str) -> dict:
        """Parse G-code parameters from a line.

        G-code format: COMMAND PARAM1 PARAM2 ...
        where each parameter is a letter followed by a number.
        e.g., "G1 X10 Y20 Z5 F3000" -> {'X': '10', 'Y': '20', 'Z': '5', 'F': '3000'}

        Args:
            line: A G-code line (without comments).

        Returns:
            Dictionary mapping parameter letters to their values.
        """
        params = {}
        # Match parameter patterns: letter followed by optional sign and digits/decimals
        pattern = r'([A-Z])(-?\d+\.?\d*)'
        for match in re.finditer(pattern, line):
            param_letter = match.group(1)
            param_value = match.group(2)
            params[param_letter] = param_value
        return params

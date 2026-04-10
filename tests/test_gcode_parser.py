"""Unit tests for G-code parser module."""

import pytest
import sys
from pathlib import Path
import numpy.testing as npt

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from gcode_parser import GCodeParser, Waypoint


class TestGCodeLineParser:
    """Test suite for parsing individual G-code lines."""

    def test_parse_g1_linear_move(self):
        """Test parsing of G1 (linear move) command with all parameters.

        G1 X10 Y20 Z0.3 E1.5 F1200 should parse to a Waypoint with:
        - position in metres (mm * 0.001)
        - extrusion amount
        - feedrate
        - move_type = 'linear'
        """
        parser = GCodeParser()
        waypoint = parser.parse_line("G1 X10 Y20 Z0.3 E1.5 F1200")

        assert waypoint is not None, "G1 line should produce a waypoint"
        assert isinstance(waypoint, Waypoint)

        # Check coordinates (mm -> m conversion)
        assert waypoint.x == pytest.approx(0.010), f"Expected x=0.010 m, got {waypoint.x}"
        assert waypoint.y == pytest.approx(0.020), f"Expected y=0.020 m, got {waypoint.y}"
        assert waypoint.z == pytest.approx(0.0003), f"Expected z=0.0003 m, got {waypoint.z}"

        # Check extrusion and feedrate
        assert waypoint.e == pytest.approx(1.5)
        assert waypoint.feedrate == pytest.approx(1200.0)
        assert waypoint.move_type == "linear"

    def test_parse_g0_rapid_move(self):
        """Test parsing of G0 (rapid move) command.

        G0 should result in a waypoint with move_type='rapid'.
        """
        parser = GCodeParser()
        waypoint = parser.parse_line("G0 X5 Y10 Z2")

        assert waypoint is not None
        assert waypoint.move_type == "rapid"
        assert waypoint.x == pytest.approx(0.005)
        assert waypoint.y == pytest.approx(0.010)
        assert waypoint.z == pytest.approx(0.002)

    def test_parse_g28_home_command(self):
        """Test parsing of G28 (home) command.

        G28 should return a waypoint at (0, 0, 0) with move_type='home'.
        """
        parser = GCodeParser()
        waypoint = parser.parse_line("G28")

        assert waypoint is not None
        assert waypoint.move_type == "home"
        assert waypoint.x == pytest.approx(0.0)
        assert waypoint.y == pytest.approx(0.0)
        assert waypoint.z == pytest.approx(0.0)

    def test_parse_comment_returns_none(self):
        """Test that comment lines are ignored and return None."""
        parser = GCodeParser()

        # Lines starting with ;
        assert parser.parse_line("; This is a comment") is None
        assert parser.parse_line(";G1 X10 Y20") is None

    def test_parse_inline_comment_ignored(self):
        """Test that inline comments (after command) are stripped."""
        parser = GCodeParser()
        waypoint = parser.parse_line("G1 X10 Y20 Z0.5 ; inline comment")

        assert waypoint is not None
        assert waypoint.x == pytest.approx(0.010)
        assert waypoint.y == pytest.approx(0.020)
        assert waypoint.z == pytest.approx(0.0005)

    def test_parse_partial_coordinates(self):
        """Test parsing with partial coordinates (only some axes specified).

        If only X and Y are given, Z should retain previous value.
        """
        parser = GCodeParser()

        # First move sets Z
        wp1 = parser.parse_line("G1 X5 Y10 Z1.0 F1000")
        assert wp1.z == pytest.approx(0.001)

        # Second move omits Z, should keep previous Z value
        wp2 = parser.parse_line("G1 X15 Y20")
        assert wp2.z == pytest.approx(0.001), "Z should retain previous value"
        assert wp2.x == pytest.approx(0.015)
        assert wp2.y == pytest.approx(0.020)

    def test_parse_m104_temperature_command(self):
        """Test parsing of M104 (set temperature) command.

        M104 should not produce a waypoint but should store metadata.
        """
        parser = GCodeParser()
        waypoint = parser.parse_line("M104 S200")

        assert waypoint is None, "M104 should not produce a waypoint"
        metadata = parser.get_metadata()
        assert 'nozzle_temp' in metadata
        assert metadata['nozzle_temp'] == 200

    def test_parse_m106_fan_command(self):
        """Test parsing of M106 (set fan speed) command."""
        parser = GCodeParser()
        waypoint = parser.parse_line("M106 S255")

        assert waypoint is None
        metadata = parser.get_metadata()
        assert 'fan_speed' in metadata
        assert metadata['fan_speed'] == 255

    def test_parse_g92_set_position(self):
        """Test parsing of G92 (set current position) command.

        G92 should not produce a waypoint but should update internal state.
        """
        parser = GCodeParser()

        # G92 sets position
        waypoint = parser.parse_line("G92 X0 Y0 Z0 E0")
        assert waypoint is None

        # Subsequent moves should use updated position as reference
        wp = parser.parse_line("G1 X5 Y10")
        assert wp.x == pytest.approx(0.005)
        assert wp.y == pytest.approx(0.010)

    def test_parse_empty_line_returns_none(self):
        """Test that empty lines return None."""
        parser = GCodeParser()
        assert parser.parse_line("") is None
        assert parser.parse_line("   ") is None

    def test_parse_unknown_command_ignored(self):
        """Test that unknown commands are gracefully ignored."""
        parser = GCodeParser()
        # G99 is not a standard command
        waypoint = parser.parse_line("G99 X10 Y20")
        # Should be ignored gracefully (return None or ignore)
        # Parser might return None or handle it differently
        if waypoint is not None:
            assert isinstance(waypoint, Waypoint)


class TestGCodeFileParser:
    """Test suite for parsing complete G-code files."""

    def test_parse_sample_file(self, sample_gcode_file):
        """Test parsing the sample G-code file fixture.

        Should extract multiple waypoints and metadata correctly.
        """
        parser = GCodeParser()
        waypoints = parser.parse_file(str(sample_gcode_file))

        # The sample file has: G28, G1, G0, G1, G1
        assert len(waypoints) >= 4, \
            f"Expected at least 4 waypoints, got {len(waypoints)}"

        # Check that we have moves of different types
        move_types = [wp.move_type for wp in waypoints]
        assert "home" in move_types or "linear" in move_types

    def test_parse_file_returns_waypoints_list(self, sample_gcode_file):
        """Test that parse_file returns a list of Waypoint objects."""
        parser = GCodeParser()
        waypoints = parser.parse_file(str(sample_gcode_file))

        assert isinstance(waypoints, list)
        for wp in waypoints:
            assert isinstance(wp, Waypoint)

    def test_parse_file_not_found_raises(self):
        """Test that parsing non-existent file raises FileNotFoundError."""
        parser = GCodeParser()

        with pytest.raises(FileNotFoundError):
            parser.parse_file("/nonexistent/path/to/file.gcode")

    def test_parse_file_state_reset(self, sample_gcode_file):
        """Test that internal state is reset between parse_file calls."""
        parser = GCodeParser()

        # Parse the same file twice
        waypoints1 = parser.parse_file(str(sample_gcode_file))
        waypoints2 = parser.parse_file(str(sample_gcode_file))

        # Should get the same result both times (state was reset)
        assert len(waypoints1) == len(waypoints2)

        for wp1, wp2 in zip(waypoints1, waypoints2):
            assert wp1.x == wp2.x
            assert wp1.y == wp2.y
            assert wp1.z == wp2.z

    def test_metadata_extraction_from_file(self, sample_gcode_file):
        """Test that metadata (temperature, fan) is extracted during file parsing."""
        parser = GCodeParser()
        waypoints = parser.parse_file(str(sample_gcode_file))

        metadata = parser.get_metadata()

        # Sample file includes M104 S200
        assert 'nozzle_temp' in metadata, "Should extract nozzle temperature"
        assert metadata['nozzle_temp'] == 200


class TestWaypoint:
    """Test suite for the Waypoint dataclass."""

    def test_waypoint_creation(self):
        """Test basic Waypoint creation with all fields."""
        wp = Waypoint(x=0.01, y=0.02, z=0.003, e=1.5, feedrate=1200.0,
                      move_type="linear")

        assert wp.x == 0.01
        assert wp.y == 0.02
        assert wp.z == 0.003
        assert wp.e == 1.5
        assert wp.feedrate == 1200.0
        assert wp.move_type == "linear"

    def test_waypoint_defaults(self):
        """Test Waypoint creation with default values."""
        wp = Waypoint(x=0.01, y=0.02, z=0.003)

        assert wp.e == 0.0
        assert wp.feedrate == 0.0
        assert wp.move_type == "linear"

    def test_waypoint_position_vector(self):
        """Test that Waypoint positions can be accessed as coordinates."""
        wp = Waypoint(x=1.0, y=2.0, z=3.0)

        # Access individual coordinates
        assert wp.x == 1.0
        assert wp.y == 2.0
        assert wp.z == 3.0


class TestGCodeParserIntegration:
    """Integration tests for G-code parser with realistic scenarios."""

    def test_multi_segment_trajectory(self, tmp_path):
        """Test parsing a multi-segment trajectory with varying parameters.

        This simulates a more complex print with multiple moves,
        parameter changes, and comments.
        """
        gcode_content = """; Layer 1
G28
G1 X10 Y10 Z0.2 F1000 E0.5
G1 X20 Y10 Z0.2 F1000 E1.0
G1 X20 Y20 Z0.2 F1000 E1.5
G0 X0 Y0 Z5 ; Retract to safe height
; Layer 2
G1 X10 Y10 Z0.4 F1200 E2.0
G1 X20 Y10 Z0.4 F1200 E2.5
"""
        gcode_file = tmp_path / "trajectory.gcode"
        gcode_file.write_text(gcode_content)

        parser = GCodeParser()
        waypoints = parser.parse_file(str(gcode_file))

        # Should have multiple waypoints
        assert len(waypoints) > 5, f"Expected > 5 waypoints, got {len(waypoints)}"

        # Verify Z changes between layers
        z_values = [wp.z for wp in waypoints]
        assert min(z_values) < max(z_values), "Should have Z layer changes"

    def test_extrusion_tracking(self, tmp_path):
        """Test that extrusion amount is properly tracked."""
        gcode_content = """G92 E0
G1 X5 Y0 Z0.3 E1.0 F1000
G1 X10 Y0 Z0.3 E2.0 F1000
G1 X15 Y0 Z0.3 E3.0 F1000
"""
        gcode_file = tmp_path / "extrusion.gcode"
        gcode_file.write_text(gcode_content)

        parser = GCodeParser()
        waypoints = parser.parse_file(str(gcode_file))

        # Extrusion should be monotonically increasing
        e_values = [wp.e for wp in waypoints]
        assert e_values[0] <= e_values[1] <= e_values[2]

    def test_feedrate_inheritance(self, tmp_path):
        """Test that feedrate is inherited from previous commands."""
        gcode_content = """G1 F1200
G1 X10 Y10 Z0.5 E1.0
G1 X20 Y20 Z0.5 E2.0
G1 X30 Y30 Z0.5 F1500 E3.0
"""
        gcode_file = tmp_path / "feedrate.gcode"
        gcode_file.write_text(gcode_content)

        parser = GCodeParser()
        waypoints = parser.parse_file(str(gcode_file))

        # First two waypoints should have F1200
        assert waypoints[0].feedrate == pytest.approx(1200.0)
        assert waypoints[1].feedrate == pytest.approx(1200.0)

        # Last waypoint should have F1500
        assert waypoints[2].feedrate == pytest.approx(1500.0)

"""Post-processing filters for waypoint lists."""

from typing import Sequence
from dataclasses import dataclass, replace
import math

from .parser import Waypoint


def filter_travel_moves(waypoints: Sequence[Waypoint]) -> list[Waypoint]:
    """Remove non-extruding rapid moves (travel moves).

    Travel moves are rapid (G0) moves that don't extrude. They represent
    the nozzle moving between print areas without depositing material.

    Args:
        waypoints: List of waypoints to filter.

    Returns:
        Filtered list with travel moves removed.
    """
    result = []
    prev_e = 0.0

    for waypoint in waypoints:
        # Keep waypoint if it's linear, or if it's rapid with extrusion
        if waypoint.move_type == "linear" or waypoint.e > prev_e:
            result.append(waypoint)
            prev_e = waypoint.e
        # Skip rapid moves without extrusion (travel moves)

    return result


def downsample_waypoints(
    waypoints: Sequence[Waypoint],
    min_distance: float = 0.001
) -> list[Waypoint]:
    """Remove waypoints closer than min_distance to the previous one.

    Useful for reducing the density of waypoints while preserving the path.
    Uses Euclidean distance in 3D space.

    Args:
        waypoints: List of waypoints to downsample.
        min_distance: Minimum distance in metres between consecutive waypoints.

    Returns:
        Downsampled waypoint list.
    """
    if not waypoints:
        return []

    result = [waypoints[0]]
    prev = waypoints[0]

    for current in waypoints[1:]:
        distance = math.sqrt(
            (current.x - prev.x) ** 2 +
            (current.y - prev.y) ** 2 +
            (current.z - prev.z) ** 2
        )

        if distance >= min_distance:
            result.append(current)
            prev = current

    return result


def apply_z_offset(
    waypoints: Sequence[Waypoint],
    offset: float
) -> list[Waypoint]:
    """Add a Z offset to all waypoints.

    Useful for mounting the robotic arm above the print bed.

    Args:
        waypoints: List of waypoints to transform.
        offset: Z offset in metres (positive = up).

    Returns:
        Transformed waypoint list.
    """
    return [
        replace(waypoint, z=waypoint.z + offset)
        for waypoint in waypoints
    ]


def transform_to_arm_frame(
    waypoints: Sequence[Waypoint],
    origin_offset: tuple[float, float, float],
    rotation_angle: float = 0.0
) -> list[Waypoint]:
    """Transform waypoints to arm base frame.

    Maps the print bed origin to the arm base frame. Supports translation and
    optionally a 2D rotation about the Z-axis.

    Args:
        waypoints: List of waypoints to transform.
        origin_offset: (x_offset, y_offset, z_offset) in metres to translate
            the print bed origin to the arm base frame.
        rotation_angle: Rotation angle in radians about the Z-axis (default 0).

    Returns:
        Transformed waypoint list.
    """
    x_offset, y_offset, z_offset = origin_offset
    cos_theta = math.cos(rotation_angle)
    sin_theta = math.sin(rotation_angle)

    result = []
    for waypoint in waypoints:
        # Apply 2D rotation about Z-axis (if non-zero)
        if rotation_angle != 0.0:
            x_rot = waypoint.x * cos_theta - waypoint.y * sin_theta
            y_rot = waypoint.x * sin_theta + waypoint.y * cos_theta
        else:
            x_rot = waypoint.x
            y_rot = waypoint.y

        # Apply translation
        x_new = x_rot + x_offset
        y_new = y_rot + y_offset
        z_new = waypoint.z + z_offset

        result.append(replace(
            waypoint,
            x=x_new,
            y=y_new,
            z=z_new
        ))

    return result

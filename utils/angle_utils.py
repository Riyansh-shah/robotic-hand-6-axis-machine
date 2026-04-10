"""Angle utility helpers."""

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap *angle* (radians) to the interval [-π, π]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def deg2rad(degrees: float) -> float:
    """Convert degrees to radians."""
    return np.deg2rad(degrees)


def rad2deg(radians: float) -> float:
    """Convert radians to degrees."""
    return np.rad2deg(radians)

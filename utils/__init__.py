"""
utils — helper functions shared across all modules.

Exports:
    rot_x, rot_y, rot_z          — elementary rotation matrices (3×3 ndarray)
    homogeneous                   — build 4×4 homogeneous transform
    dh_transform                  — single DH row → 4×4 T matrix
    euler_to_rot                  — ZYX Euler angles → rotation matrix
    rot_to_euler                  — rotation matrix → ZYX Euler angles
    wrap_angle                    — wrap angle to [-π, π]
    deg2rad, rad2deg              — thin wrappers (prefer np.radians/degrees)
    plot_frame                    — draw a coordinate frame on a 3-D axes
"""

from .transforms import (
    rot_x, rot_y, rot_z,
    homogeneous, dh_transform,
    euler_to_rot, rot_to_euler,
)
from .angle_utils import wrap_angle, deg2rad, rad2deg
from .plotting import plot_frame

__all__ = [
    "rot_x", "rot_y", "rot_z",
    "homogeneous", "dh_transform",
    "euler_to_rot", "rot_to_euler",
    "wrap_angle", "deg2rad", "rad2deg",
    "plot_frame",
]

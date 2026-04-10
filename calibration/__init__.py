"""
Calibration module for the 6-DOF robotic arm.

Provides TCP (Tool Center Point) calibration and accuracy measurement tools.
"""

from .tcp_calibration import calibrate_tcp_four_point, apply_tcp_offset
from .accuracy_measurement import (
    AccuracyReport,
    compute_accuracy,
    compute_repeatability,
    generate_accuracy_report,
    plot_accuracy,
)

__all__ = [
    # TCP calibration
    "calibrate_tcp_four_point",
    "apply_tcp_offset",
    # Accuracy measurement
    "AccuracyReport",
    "compute_accuracy",
    "compute_repeatability",
    "generate_accuracy_report",
    "plot_accuracy",
]

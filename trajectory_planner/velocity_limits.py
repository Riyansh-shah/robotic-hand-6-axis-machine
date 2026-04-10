"""
Joint velocity and acceleration limits for the 6R robotic arm.

Provides default velocity and acceleration limits realistic for NEMA17 stepper motors
with gearbox reduction, and utility functions to check trajectory compliance.
"""

import numpy as np
from typing import Tuple


# Default velocity limits for each joint (rad/s)
# NEMA17 stepper with typical gearbox gives ~2 rad/s at end joint
DEFAULT_JOINT_VELOCITY_LIMITS = np.array([
    2.0,  # Joint 1 (Base rotation)
    2.0,  # Joint 2 (Shoulder)
    2.0,  # Joint 3 (Elbow)
    2.0,  # Joint 4 (Wrist 1)
    2.0,  # Joint 5 (Wrist 2)
    2.0,  # Joint 6 (Wrist 3)
])

# Default acceleration limits for each joint (rad/s²)
# Typical stepper can achieve ~5 rad/s² with gearbox
DEFAULT_JOINT_ACCEL_LIMITS = np.array([
    5.0,  # Joint 1
    5.0,  # Joint 2
    5.0,  # Joint 3
    5.0,  # Joint 4
    5.0,  # Joint 5
    5.0,  # Joint 6
])


def check_velocity_limits(
    trajectory: np.ndarray,
    dt: float = 0.01,
    v_limits: np.ndarray = None,
) -> Tuple[bool, np.ndarray]:
    """
    Verify that a joint trajectory respects velocity limits.

    Computes joint velocities by finite difference and checks against limits.

    Parameters
    ----------
    trajectory : np.ndarray, shape (N, 6)
        Joint angle trajectory. Each row is [q1, q2, q3, q4, q5, q6] at a timestep.
    dt : float, optional
        Timestep interval (seconds). Default: 0.01 s.
    v_limits : np.ndarray, shape (6,), optional
        Maximum velocity for each joint (rad/s).
        If None, uses DEFAULT_JOINT_VELOCITY_LIMITS.

    Returns
    -------
    is_valid : bool
        True if all velocities are within limits, False otherwise.
    velocities : np.ndarray, shape (N-1, 6)
        Computed joint velocities at each step.
    """
    if v_limits is None:
        v_limits = DEFAULT_JOINT_VELOCITY_LIMITS

    trajectory = np.asarray(trajectory)
    assert trajectory.shape[1] == 6, f"Expected shape (N, 6), got {trajectory.shape}"
    assert trajectory.ndim == 2, "Trajectory must be 2D array"

    # Compute velocities via finite difference
    dq = np.diff(trajectory, axis=0) / dt

    # Check against limits
    v_abs = np.abs(dq)
    violations = np.any(v_abs > v_limits[np.newaxis, :], axis=1)

    is_valid = not np.any(violations)

    return is_valid, dq


def check_acceleration_limits(
    trajectory: np.ndarray,
    dt: float = 0.01,
    a_limits: np.ndarray = None,
) -> Tuple[bool, np.ndarray]:
    """
    Verify that a joint trajectory respects acceleration limits.

    Computes joint accelerations via finite difference and checks against limits.

    Parameters
    ----------
    trajectory : np.ndarray, shape (N, 6)
        Joint angle trajectory. Each row is [q1, q2, q3, q4, q5, q6] at a timestep.
    dt : float, optional
        Timestep interval (seconds). Default: 0.01 s.
    a_limits : np.ndarray, shape (6,), optional
        Maximum acceleration for each joint (rad/s²).
        If None, uses DEFAULT_JOINT_ACCEL_LIMITS.

    Returns
    -------
    is_valid : bool
        True if all accelerations are within limits, False otherwise.
    accelerations : np.ndarray, shape (N-2, 6)
        Computed joint accelerations at each step.
    """
    if a_limits is None:
        a_limits = DEFAULT_JOINT_ACCEL_LIMITS

    trajectory = np.asarray(trajectory)
    assert trajectory.shape[1] == 6, f"Expected shape (N, 6), got {trajectory.shape}"
    assert trajectory.ndim == 2, "Trajectory must be 2D array"

    # Compute velocities first
    dq = np.diff(trajectory, axis=0) / dt

    # Compute accelerations from velocity
    ddq = np.diff(dq, axis=0) / dt

    # Check against limits
    a_abs = np.abs(ddq)
    violations = np.any(a_abs > a_limits[np.newaxis, :], axis=1)

    is_valid = not np.any(violations)

    return is_valid, ddq

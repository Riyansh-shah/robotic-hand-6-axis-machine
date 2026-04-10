"""
Trajectory interpolation methods for smooth joint motion.

Provides trapezoidal velocity profiles, linear interpolation, and trajectory chaining.
All trajectories synchronize joint motion to finish at the same time.
"""

import numpy as np
from typing import List


def trapezoidal_velocity_profile(
    q_start: np.ndarray,
    q_end: np.ndarray,
    v_max: float = 1.0,
    a_max: float = 2.0,
    dt: float = 0.01,
) -> np.ndarray:
    """
    Generate a smooth trajectory between two joint configurations using a trapezoidal
    velocity profile.

    The trajectory consists of three phases:
      1. Acceleration: constant acceleration from 0 to v_max
      2. Cruise: constant velocity at v_max
      3. Deceleration: constant deceleration from v_max to 0

    All 6 joints are synchronized to finish at the same time.

    Parameters
    ----------
    q_start : np.ndarray, shape (6,)
        Starting joint angles (radians).
    q_end : np.ndarray, shape (6,)
        Ending joint angles (radians).
    v_max : float, optional
        Maximum velocity in normalized units (path parameter rate). Default: 1.0.
    a_max : float, optional
        Maximum acceleration in normalized units. Default: 2.0.
    dt : float, optional
        Timestep for trajectory sampling (seconds). Default: 0.01 s.

    Returns
    -------
    trajectory : np.ndarray, shape (N, 6)
        Interpolated joint angles at each timestep.
        First row is q_start, last row is q_end.

    Notes
    -----
    The motion is parameterized by a normalized path parameter s in [0, 1].
    A trapezoidal velocity profile is applied to s over time.
    """
    q_start = np.asarray(q_start).ravel()
    q_end = np.asarray(q_end).ravel()

    assert q_start.shape == (6,), f"q_start must have 6 elements, got {q_start.shape}"
    assert q_end.shape == (6,), f"q_end must have 6 elements, got {q_end.shape}"

    # Compute total distance (normalized to [0, 1])
    dq = q_end - q_start
    total_distance = 1.0  # Normalized path parameter

    # Compute time for each phase of the trapezoidal profile
    # Acceleration and deceleration have equal duration by symmetry
    # Time to reach v_max: t_accel = v_max / a_max
    t_accel = v_max / a_max

    # Distance covered during acceleration/deceleration
    s_accel = 0.5 * a_max * t_accel ** 2

    # Check if we can reach v_max (if 2*s_accel > 1, we can't cruise)
    if 2.0 * s_accel > total_distance:
        # Triangular profile: no cruise phase
        t_peak = np.sqrt(total_distance / a_max)
        t_total = 2.0 * t_peak
    else:
        # Full trapezoidal profile
        s_cruise = total_distance - 2.0 * s_accel
        t_cruise = s_cruise / v_max
        t_total = 2.0 * t_accel + t_cruise

    # Generate time samples
    n_samples = int(np.ceil(t_total / dt)) + 1
    times = np.linspace(0, t_total, n_samples)

    # Compute path parameter s(t) from trapezoidal velocity profile
    s_trajectory = np.zeros_like(times)

    for i, t in enumerate(times):
        if 2.0 * s_accel > total_distance:
            # Triangular profile
            if t <= t_peak:
                s_trajectory[i] = 0.5 * a_max * t ** 2
            else:
                s_trajectory[i] = total_distance - 0.5 * a_max * (t_total - t) ** 2
        else:
            # Trapezoidal profile
            if t <= t_accel:
                s_trajectory[i] = 0.5 * a_max * t ** 2
            elif t <= t_accel + t_cruise:
                s_trajectory[i] = s_accel + v_max * (t - t_accel)
            else:
                t_decel = t - t_accel - t_cruise
                s_trajectory[i] = s_accel + s_cruise + v_max * t_decel - 0.5 * a_max * t_decel ** 2

    # Clamp to [0, 1] to avoid numerical issues
    s_trajectory = np.clip(s_trajectory, 0.0, 1.0)

    # Interpolate joint angles along the path
    trajectory = np.zeros((n_samples, 6))
    for i, s in enumerate(s_trajectory):
        trajectory[i] = q_start + s * dq

    return trajectory


def interpolate_trajectory(
    joint_waypoints: List[np.ndarray],
    v_max: float = 1.0,
    a_max: float = 2.0,
    dt: float = 0.01,
) -> np.ndarray:
    """
    Chain multiple trapezoidal profiles to create a complete trajectory.

    Connects a sequence of joint configurations with smooth trapezoidal profiles.

    Parameters
    ----------
    joint_waypoints : List[np.ndarray]
        Sequence of joint configurations, each shape (6,).
    v_max : float, optional
        Maximum velocity. Default: 1.0.
    a_max : float, optional
        Maximum acceleration. Default: 2.0.
    dt : float, optional
        Timestep for sampling (seconds). Default: 0.01 s.

    Returns
    -------
    full_trajectory : np.ndarray, shape (N, 6)
        Complete trajectory connecting all waypoints.

    Notes
    -----
    Each segment is generated independently and concatenated.
    The first point of each segment (except the first) is removed to avoid duplication.
    """
    if len(joint_waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")

    full_trajectory = None

    for i in range(len(joint_waypoints) - 1):
        q_start = np.asarray(joint_waypoints[i]).ravel()
        q_end = np.asarray(joint_waypoints[i + 1]).ravel()

        # Generate segment trajectory
        segment = trapezoidal_velocity_profile(q_start, q_end, v_max, a_max, dt)

        if full_trajectory is None:
            full_trajectory = segment
        else:
            # Concatenate, removing duplicate starting point
            full_trajectory = np.vstack([full_trajectory, segment[1:, :]])

    return full_trajectory


def linear_interpolation(
    q_start: np.ndarray,
    q_end: np.ndarray,
    n_steps: int = 50,
) -> np.ndarray:
    """
    Simple linear interpolation between two joint configurations.

    This is a fallback method for cases where trapezoidal profiles are not needed.

    Parameters
    ----------
    q_start : np.ndarray, shape (6,)
        Starting joint angles (radians).
    q_end : np.ndarray, shape (6,)
        Ending joint angles (radians).
    n_steps : int, optional
        Number of intermediate points (including endpoints). Default: 50.

    Returns
    -------
    trajectory : np.ndarray, shape (n_steps, 6)
        Linearly interpolated joint angles.
    """
    q_start = np.asarray(q_start).ravel()
    q_end = np.asarray(q_end).ravel()

    assert q_start.shape == (6,), f"q_start must have 6 elements, got {q_start.shape}"
    assert q_end.shape == (6,), f"q_end must have 6 elements, got {q_end.shape}"
    assert n_steps >= 2, "n_steps must be at least 2"

    # Generate parameter alpha in [0, 1]
    alpha = np.linspace(0, 1, n_steps)

    # Linear interpolation
    trajectory = np.zeros((n_steps, 6))
    for i, a in enumerate(alpha):
        trajectory[i] = (1.0 - a) * q_start + a * q_end

    return trajectory

"""
Trajectory planning module for the 6R robotic arm.

Converts Cartesian waypoints to smooth joint trajectories through:
  1. Inverse kinematics (Cartesian -> joint space)
  2. Trajectory interpolation (discrete waypoints -> smooth paths)
  3. Velocity limit checking (validation against hardware constraints)

Main entry points:
  - waypoints_to_joint_trajectory: G-code waypoints -> joint angles
  - interpolate_trajectory: joint waypoints -> smooth trajectory
  - trapezoidal_velocity_profile: segment-wise trajectory with acceleration limits
"""

from .cartesian_to_joint import waypoints_to_joint_trajectory, build_target_pose
from .interpolation import (
    trapezoidal_velocity_profile,
    interpolate_trajectory,
    linear_interpolation,
)
from .velocity_limits import (
    DEFAULT_JOINT_VELOCITY_LIMITS,
    DEFAULT_JOINT_ACCEL_LIMITS,
    check_velocity_limits,
    check_acceleration_limits,
)

__all__ = [
    # Cartesian to joint conversion
    "waypoints_to_joint_trajectory",
    "build_target_pose",
    # Interpolation
    "trapezoidal_velocity_profile",
    "interpolate_trajectory",
    "linear_interpolation",
    # Velocity limits
    "DEFAULT_JOINT_VELOCITY_LIMITS",
    "DEFAULT_JOINT_ACCEL_LIMITS",
    "check_velocity_limits",
    "check_acceleration_limits",
]

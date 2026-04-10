"""
Convert Cartesian waypoints to joint trajectories.

Handles the conversion from end-effector positions (Cartesian space) to joint
configurations (joint space) using inverse kinematics, with continuity across
waypoints.
"""

import logging
import numpy as np
from typing import List, Optional
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from gcode_parser import Waypoint
from kinematics.inverse_kinematics import ik_numerical, IKError
from kinematics.dh_params import DH6R, get_dh_table
from utils.transforms import rot_z, rot_y, rot_x, homogeneous


logger = logging.getLogger(__name__)


def build_target_pose(
    x: float,
    y: float,
    z: float,
    ee_orientation: Optional[np.ndarray] = None,
) -> np.ndarray:
    """
    Construct a 4×4 target pose from Cartesian position and orientation.

    Parameters
    ----------
    x, y, z : float
        End-effector target position (metres).
    ee_orientation : np.ndarray, shape (3, 3), optional
        Target rotation matrix (3×3). If None, uses a fixed downward-pointing
        orientation (Z-axis pointing down, or Z in local EE frame pointing -Z).
        Default orientation: X-axis forward (along X), Y-axis side, Z-axis down.

    Returns
    -------
    T_target : np.ndarray, shape (4, 4)
        Homogeneous transformation matrix with position [x, y, z] and
        specified or default orientation.

    Notes
    -----
    The default "Z-down" orientation is useful for printing/milling applications
    where the tool naturally points downward. It corresponds to:
    - Roll = pi (180°), Pitch = 0, Yaw = 0 (in ZYX Euler convention)
    which gives the rotation matrix for pointing straight down.
    """
    position = np.array([x, y, z])

    if ee_orientation is None:
        # Default: Z-axis pointing down (tool looking at the ground)
        # This is achieved by rotating 180° around the X-axis
        ee_orientation = rot_x(np.pi)

    else:
        ee_orientation = np.asarray(ee_orientation)
        assert ee_orientation.shape == (3, 3), \
            f"Orientation must be 3×3, got {ee_orientation.shape}"

    T_target = homogeneous(ee_orientation, position)
    return T_target


def waypoints_to_joint_trajectory(
    waypoints: List[Waypoint],
    dh_table: Optional[DH6R] = None,
    q_init: Optional[np.ndarray] = None,
) -> List[np.ndarray]:
    """
    Convert a list of Cartesian waypoints to joint angle configurations.

    For each waypoint, solves the inverse kinematics problem to find joint angles
    that position the end-effector at (x, y, z). Uses a continuous warm-start
    approach: the solution from each waypoint is used as the initial guess for
    the next, ensuring smooth trajectories.

    Parameters
    ----------
    waypoints : List[Waypoint]
        List of Cartesian waypoints with x, y, z in metres.
    dh_table : DH6R, optional
        DH parameter table. If None, uses the default 6R arm parameters.
    q_init : np.ndarray, shape (6,), optional
        Initial joint angle guess for the first waypoint. If None, starts from
        zero angles.

    Returns
    -------
    joint_waypoints : List[np.ndarray]
        List of joint angle arrays (each shape (6,)), one per Cartesian waypoint.
        Waypoints that fail IK are skipped with a warning logged.

    Notes
    -----
    - Orientation is fixed to Z-down (tool pointing downward) for all waypoints.
    - If IK fails to converge for a waypoint, a warning is logged and that waypoint
      is skipped.
    - Solution continuity is maintained by warm-starting from the previous solution.
    """
    if dh_table is None:
        dh_table = get_dh_table()

    if q_init is None:
        q_current = np.zeros(6)
    else:
        q_current = np.asarray(q_init).ravel()

    joint_waypoints = []

    for i, wp in enumerate(waypoints):
        # Build target pose with Z-down orientation
        T_target = build_target_pose(wp.x, wp.y, wp.z)

        try:
            # Solve IK with warm-start from previous solution
            q_solution = ik_numerical(
                target_pose=T_target,
                dh_table=dh_table,
                q_init=q_current,
                tol=1e-6,
                max_iter=200,
            )

            joint_waypoints.append(q_solution)
            q_current = q_solution  # Warm-start next iteration

        except IKError as e:
            logger.warning(
                f"IK failed for waypoint {i} at ({wp.x:.4f}, {wp.y:.4f}, {wp.z:.4f}): {e}"
            )
            # Skip this waypoint but continue with the rest

    if len(joint_waypoints) == 0:
        logger.error("No valid IK solutions found for any waypoint!")
        return []

    return joint_waypoints

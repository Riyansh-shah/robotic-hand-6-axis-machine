"""
TCP (Tool Center Point) calibration for the 6-DOF robotic arm.

Provides functions to calibrate the TCP offset vector (position of the tool
relative to the end-effector flange) using the standard 4-point calibration
method and to apply the calibrated offset to end-effector transforms.
"""

from typing import List, Union
import numpy as np
from scipy.optimize import least_squares
import sys
from pathlib import Path

# Add parent directory to path to import kinematics and utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from kinematics.forward_kinematics import forward_kinematics
from kinematics.dh_params import DH6R
from utils.transforms import homogeneous


def calibrate_tcp_four_point(
    joint_configs: List[Union[List[float], np.ndarray]],
    dh_table: DH6R,
) -> np.ndarray:
    """
    Calibrate TCP offset using the 4-point method.

    Classic TCP calibration: given 4 joint configurations where the TCP touches
    the same physical point, solve for the TCP offset vector in the flange frame
    using least-squares minimization.

    The objective is to minimize the difference between the transformed positions
    (end-effector transform applied to the TCP offset) across all configurations,
    since they should all point to the same physical location.

    Parameters
    ----------
    joint_configs : List[array-like], len 4
        Four joint angle configurations, each with shape (6,), where the TCP
        touches the same physical point in space.
    dh_table : DH6R
        DH parameter dataclass for the arm.

    Returns
    -------
    tcp_offset : np.ndarray, shape (3,)
        Calibrated TCP offset vector in the flange frame [x, y, z] (metres).
    """
    # Validate input
    if len(joint_configs) < 3:
        raise ValueError(f"Need at least 3 configurations for calibration, got {len(joint_configs)}")

    joint_configs = [np.asarray(config).ravel() for config in joint_configs]
    for i, config in enumerate(joint_configs):
        if config.shape != (6,):
            raise ValueError(f"Configuration {i} has shape {config.shape}, expected (6,)")

    # Objective function: compute residuals (differences in TCP positions)
    def residual_fn(tcp_offset_flat: np.ndarray) -> np.ndarray:
        """
        Compute residuals as deviations from the mean TCP position.

        Parameters
        ----------
        tcp_offset_flat : array-like, shape (3,)
            TCP offset vector to test.

        Returns
        -------
        residuals : np.ndarray, shape (n_configs * 3,)
            Concatenated position differences from the mean position.
        """
        tcp_offset = tcp_offset_flat.reshape(3, 1)

        # Compute TCP positions for all configurations
        positions = []
        for config in joint_configs:
            T_ee = forward_kinematics(config, dh_table)
            # Position in world frame = R * tcp_offset + t
            pos = T_ee[:3, :3] @ tcp_offset + T_ee[:3, 3:4]
            positions.append(pos.ravel())

        positions = np.array(positions)  # shape (n_configs, 3)

        # Compute mean position
        mean_pos = np.mean(positions, axis=0)

        # Residuals: deviations from mean
        residuals = (positions - mean_pos).ravel()

        return residuals

    # Initial guess: small offset in flange Z direction
    tcp_init = np.array([0.0, 0.0, 0.05])

    # Solve least-squares problem
    result = least_squares(
        residual_fn,
        tcp_init,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
        max_nfev=10000,
    )

    if not result.success:
        raise RuntimeError(f"TCP calibration failed: {result.message}")

    tcp_offset = result.x

    return tcp_offset


def apply_tcp_offset(
    ee_transform: np.ndarray,
    tcp_offset: np.ndarray,
) -> np.ndarray:
    """
    Apply TCP offset to an end-effector transform.

    Transforms a point (the TCP) from the flange frame to the world frame.
    The TCP is initially at position tcp_offset in the flange frame, and we
    compute its position in the world frame after applying the end-effector
    transformation.

    Parameters
    ----------
    ee_transform : np.ndarray, shape (4, 4)
        Homogeneous transformation matrix of the end-effector (flange) frame.
    tcp_offset : np.ndarray, shape (3,)
        TCP offset vector in the flange frame [x, y, z] (metres).

    Returns
    -------
    corrected_transform : np.ndarray, shape (4, 4)
        Homogeneous transformation matrix with the TCP offset applied.
        The position component is T[:3, :3] @ tcp_offset + T[:3, 3],
        and the rotation is unchanged.
    """
    ee_transform = np.asarray(ee_transform)
    tcp_offset = np.asarray(tcp_offset).ravel()

    if ee_transform.shape != (4, 4):
        raise ValueError(f"Expected transform shape (4, 4), got {ee_transform.shape}")
    if tcp_offset.shape != (3,):
        raise ValueError(f"Expected tcp_offset shape (3,), got {tcp_offset.shape}")

    # Extract rotation and position
    R = ee_transform[:3, :3]
    t = ee_transform[:3, 3]

    # Apply TCP offset: transform tcp_offset from flange to world frame
    tcp_world = R @ tcp_offset + t

    # Build corrected transform with the same rotation but TCP position
    corrected_transform = homogeneous(R, tcp_world)

    return corrected_transform

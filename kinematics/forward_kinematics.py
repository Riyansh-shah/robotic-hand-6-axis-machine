"""
Forward kinematics for the 6-DOF robotic arm.

Computes end-effector and intermediate joint transforms from joint angles
using the standard DH convention.
"""

from typing import List, Union
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.transforms import dh_transform
from .dh_params import DH6R, get_dh_table


def forward_kinematics(
    joint_angles: Union[List[float], np.ndarray],
    dh_table: DH6R,
) -> np.ndarray:
    """
    Compute the end-effector pose from joint angles.

    Uses the standard DH forward kinematics: multiply all single-joint
    transformation matrices in sequence.

    Parameters
    ----------
    joint_angles : array-like, shape (6,)
        Joint angles in radians [q1, q2, q3, q4, q5, q6].
    dh_table : DH6R
        DH parameter dataclass.

    Returns
    -------
    T : np.ndarray, shape (4, 4)
        Homogeneous transformation matrix of the end-effector.
        T[:3, :3] is the 3×3 rotation matrix, T[:3, 3] is the position.
    """
    joint_angles = np.asarray(joint_angles).ravel()
    assert joint_angles.shape == (6,), f"Expected 6 joint angles, got {joint_angles.shape}"

    # Start with identity
    T = np.eye(4)

    # Multiply transformations for each joint
    for i in range(6):
        a_i = dh_table.a[i]
        alpha_i = dh_table.alpha[i]
        d_i = dh_table.d[i]
        theta_i = joint_angles[i]

        T_i = dh_transform(a_i, alpha_i, d_i, theta_i)
        T = T @ T_i

    return T


def get_all_transforms(
    joint_angles: Union[List[float], np.ndarray],
    dh_table: DH6R,
) -> List[np.ndarray]:
    """
    Compute transforms for each joint frame (for visualization).

    Computes T_0^0, T_0^1, T_0^2, ..., T_0^6 (end-effector).

    Parameters
    ----------
    joint_angles : array-like, shape (6,)
        Joint angles in radians.
    dh_table : DH6R
        DH parameter dataclass.

    Returns
    -------
    transforms : List[np.ndarray]
        List of 7 transformation matrices (base frame + 6 joint frames).
        transforms[0] is the base frame (identity), transforms[6] is the EE.
    """
    joint_angles = np.asarray(joint_angles).ravel()
    assert joint_angles.shape == (6,), f"Expected 6 joint angles, got {joint_angles.shape}"

    transforms = [np.eye(4)]  # Base frame

    T = np.eye(4)
    for i in range(6):
        a_i = dh_table.a[i]
        alpha_i = dh_table.alpha[i]
        d_i = dh_table.d[i]
        theta_i = joint_angles[i]

        T_i = dh_transform(a_i, alpha_i, d_i, theta_i)
        T = T @ T_i
        transforms.append(T.copy())

    return transforms

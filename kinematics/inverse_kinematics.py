"""
Inverse kinematics solvers for the 6-DOF robotic arm.

Provides two approaches:
  1. Numerical IK: damped least-squares Jacobian-based solver with finite differences
  2. ikpy-based: analytical/numerical solver from the ikpy library (if available)
"""

from typing import List, Optional, Tuple, Union
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.transforms import rot_to_euler
from utils.angle_utils import wrap_angle
from .dh_params import DH6R, get_dh_table
from .forward_kinematics import forward_kinematics


class IKError(Exception):
    """Raised when inverse kinematics solver fails to find a solution."""
    pass


def ik_numerical(
    target_pose: np.ndarray,
    dh_table: DH6R,
    q_init: Optional[np.ndarray] = None,
    tol: float = 1e-6,
    max_iter: int = 200,
    damping: float = 0.01,
) -> np.ndarray:
    """
    Numerical inverse kinematics using damped least-squares (Levenberg-Marquardt).

    Iteratively refines joint angles to match a target pose. The Jacobian is
    computed via finite differences.

    Parameters
    ----------
    target_pose : np.ndarray, shape (4, 4)
        Desired end-effector homogeneous transformation matrix.
    dh_table : DH6R
        DH parameter dataclass.
    q_init : np.ndarray, shape (6,), optional
        Initial guess for joint angles (radians). Default: zero angles.
    tol : float, optional
        Convergence tolerance for pose error (metres). Default: 1e-6.
    max_iter : int, optional
        Maximum iterations. Default: 200.
    damping : float, optional
        Damping factor λ for regularization: J_pinv = J^T @ (J @ J^T + λI)^-1.
        Larger λ favours stability over accuracy. Default: 0.01.

    Returns
    -------
    q_solution : np.ndarray, shape (6,)
        Joint angles that approximately achieve the target pose.

    Raises
    ------
    IKError
        If solver fails to converge within max_iter.

    Notes
    -----
    Objective: minimize ||p_ee - p_target||^2 + w_rot * ||R_ee - R_target||_F^2
    where p denotes position and R denotes rotation. Uses finite differences
    to approximate the geometric Jacobian.
    """
    if q_init is None:
        q_init = np.zeros(6)
    else:
        q_init = np.asarray(q_init).ravel()

    q = q_init.copy()
    dt = 1e-5  # Step size for finite differences
    target_pos = target_pose[:3, 3]
    target_rot = target_pose[:3, :3]

    # Adaptive damping parameters
    lambda_init = damping
    lambda_factor = 10.0
    lambda_val = lambda_init

    prev_error = float('inf')

    for iteration in range(max_iter):
        # Current end-effector pose
        T_ee = forward_kinematics(q, dh_table)
        pos_ee = T_ee[:3, 3]
        rot_ee = T_ee[:3, :3]

        # Position error
        pos_error = target_pos - pos_ee
        pos_err_norm = np.linalg.norm(pos_error)

        if pos_err_norm < tol:
            return q

        # Rotation error (as 3D vector via angle-axis)
        # Using error matrix E = R_target @ R_ee^T
        E = target_rot @ rot_ee.T
        # Clamp to avoid numerical issues in arccos
        trace = np.clip(np.trace(E), -1.0, 3.0)
        angle = np.arccos((trace - 1.0) / 2.0) if trace < 3.0 else 0.0
        if angle > 1e-6:
            rot_error = angle * np.array([E[2, 1], E[0, 2], E[1, 0]]) / np.sin(angle)
        else:
            rot_error = np.array([E[2, 1], E[0, 2], E[1, 0]]) * 0.5

        # Combined error vector (6D: 3D position + 3D rotation)
        error = np.concatenate([pos_error, rot_error * 0.5])  # Weight rotation less
        error_norm = np.linalg.norm(error)

        # Compute Jacobian via finite differences (analytical would be better)
        J = np.zeros((6, 6))
        for i in range(6):
            q_plus = q.copy()
            q_plus[i] += dt
            T_plus = forward_kinematics(q_plus, dh_table)
            pos_plus = T_plus[:3, 3]
            rot_plus = T_plus[:3, :3]

            # Numerical derivative of position
            dp = (pos_plus - pos_ee) / dt
            J[:3, i] = dp

            # Numerical derivative of rotation (using angle-axis)
            dR = rot_plus @ rot_ee.T
            dtrace = np.clip(np.trace(dR), -1.0, 3.0)
            dangle = np.arccos((dtrace - 1.0) / 2.0) if dtrace < 3.0 else 0.0
            if dangle > 1e-6:
                drot = dangle * np.array([dR[2, 1], dR[0, 2], dR[1, 0]]) / (np.sin(dangle) * dt)
            else:
                drot = np.array([dR[2, 1], dR[0, 2], dR[1, 0]]) * 0.5 / dt

            J[3:, i] = drot * 0.5  # Weight rotation less

        # Damped least-squares pseudoinverse: (J^T @ J + λI)^-1 @ J^T
        JtJ = J.T @ J
        damped_JtJ = JtJ + lambda_val * np.eye(6)

        try:
            JtJ_inv = np.linalg.inv(damped_JtJ)
            J_pinv = JtJ_inv @ J.T
        except np.linalg.LinAlgError:
            # Fallback to SVD if inversion fails
            u, s, vt = np.linalg.svd(J, full_matrices=False)
            s_inv = np.where(s > 1e-10, 1.0 / s, 0.0)
            J_pinv = vt.T @ np.diag(s_inv) @ u.T

        # Update joint angles with line search
        dq = J_pinv @ error
        step_size = 1.0

        # Try step with backtracking if error increased
        q_new = q + step_size * dq
        q_new = np.array([wrap_angle(qi) for qi in q_new])
        T_new = forward_kinematics(q_new, dh_table)
        pos_new = T_new[:3, 3]

        new_error = np.linalg.norm(target_pos - pos_new)

        # Adaptive damping: decrease if improving, increase if not
        if new_error < error_norm:
            q = q_new
            lambda_val = lambda_val / lambda_factor
            prev_error = error_norm
        else:
            lambda_val = lambda_val * lambda_factor

        # Prevent lambda from getting too extreme
        lambda_val = np.clip(lambda_val, 1e-6, 1e2)

    # If we reach here, convergence failed
    T_final = forward_kinematics(q, dh_table)
    final_error = np.linalg.norm(target_pos - T_final[:3, 3])
    raise IKError(
        f"IK did not converge after {max_iter} iterations. "
        f"Final position error: {final_error:.6f} m"
    )


def ik_ikpy(
    target_position: Union[List[float], np.ndarray],
    target_orientation: Optional[Union[List[float], np.ndarray]] = None,
    dh_table: Optional[DH6R] = None,
) -> np.ndarray:
    """
    Inverse kinematics using the ikpy library (if installed).

    Builds a kinematic chain from DH parameters and solves for joint angles
    that position the end-effector at the target.

    Parameters
    ----------
    target_position : array-like, shape (3,)
        Target position in metres [x, y, z].
    target_orientation : array-like, shape (3, 3) or None, optional
        Target rotation matrix (3×3). If None, orientation is ignored.
    dh_table : DH6R, optional
        DH parameter dataclass. If None, uses default 6R arm.

    Returns
    -------
    q_solution : np.ndarray, shape (6,)
        Joint angles in radians.

    Raises
    ------
    ImportError
        If ikpy is not installed.
    IKError
        If ikpy fails to find a solution.

    Notes
    -----
    Requires: pip install ikpy
    This wrapper is provided for convenience and comparison with numerical IK.
    """
    try:
        import ikpy.chain
        import ikpy.link
    except ImportError:
        raise ImportError(
            "ikpy not installed. Install with: pip install ikpy"
        )

    if dh_table is None:
        dh_table = get_dh_table()

    target_position = np.asarray(target_position).ravel()
    assert target_position.shape == (3,), f"Expected 3D position, got {target_position.shape}"

    # Build ikpy chain from DH parameters
    links = []
    for i in range(6):
        # ikpy uses DH convention
        link = ikpy.link.DHLink(
            name=dh_table.names[i],
            a=dh_table.a[i],
            alpha=dh_table.alpha[i],
            d=dh_table.d[i],
            d_min=None,
            d_max=None,
            theta_offset=0.0,
            bounds=(dh_table.q_min[i], dh_table.q_max[i]),
        )
        links.append(link)

    chain = ikpy.chain.Chain(name="6R_Arm", links=links, active_links_mask=[True] * 6)

    # Build target as 4×4 homogeneous transform or 3D position
    if target_orientation is not None:
        target_orientation = np.asarray(target_orientation)
        target_matrix = np.eye(4)
        target_matrix[:3, :3] = target_orientation
        target_matrix[:3, 3] = target_position
    else:
        target_matrix = np.eye(4)
        target_matrix[:3, 3] = target_position

    # Solve
    try:
        ik_result = chain.inverse_kinematics(target_matrix)
        # ikpy returns 7 values (includes base link); extract joint angles
        return np.array(ik_result[1:7])
    except Exception as e:
        raise IKError(f"ikpy solver failed: {e}")

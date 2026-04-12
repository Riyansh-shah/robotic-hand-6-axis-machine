"""
Rotation matrices, homogeneous transforms, and DH convention helpers.
All angles are in radians unless stated otherwise.
"""

import numpy as np


# ---------------------------------------------------------------------------
# Elementary rotation matrices (3×3)
# ---------------------------------------------------------------------------

def rot_x(theta: float) -> np.ndarray:
    """Rotation matrix about the X axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c],
    ])


def rot_y(theta: float) -> np.ndarray:
    """Rotation matrix about the Y axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c,  0,  s],
        [ 0,  1,  0],
        [-s,  0,  c],
    ])


def rot_z(theta: float) -> np.ndarray:
    """Rotation matrix about the Z axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s,  0],
        [s,  c,  0],
        [0,  0,  1],
    ])


# ---------------------------------------------------------------------------
# Homogeneous transform helpers
# ---------------------------------------------------------------------------

def homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Build a 4×4 homogeneous transformation matrix from a 3×3 rotation
    matrix R and a 3-element translation vector t.
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = np.asarray(t).ravel()
    return T


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Standard Denavit-Hartenberg transformation matrix.

    Parameters
    ----------
    a     : link length          (metres)
    alpha : link twist           (radians)
    d     : link offset          (metres)
    theta : joint angle          (radians)

    Returns
    -------
    T : 4×4 ndarray
        T_{i-1,i} transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,  -st * ca,   st * sa,  a * ct],
        [st,   ct * ca,  -ct * sa,  a * st],
        [ 0,        sa,       ca,       d],
        [ 0,         0,        0,       1],
    ])


# ---------------------------------------------------------------------------
# Euler angle conversions (ZYX / RPY convention)
# ---------------------------------------------------------------------------

def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    ZYX intrinsic Euler angles (roll about X, pitch about Y, yaw about Z).
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def rot_to_euler(R: np.ndarray) -> tuple[float, float, float]:
    """
    Extract ZYX (roll, pitch, yaw) from a 3×3 rotation matrix.
    Handles the gimbal-lock edge cases.

    Returns
    -------
    (roll, pitch, yaw) in radians.
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2( R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2( R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw


def vector_to_rotation_matrix(vec: np.ndarray) -> np.ndarray:
    """
    Convert a 3D direction vector into a 3x3 rotation matrix.
    Assumes the vector represents the local Z-axis (tool pointing direction).
    Constructs an orthonormal basis using an arbitrary but consistent up-vector.
    
    Parameters
    ----------
    vec : np.ndarray
        Shape (3,), direction vector [i, j, k].
        
    Returns
    -------
    np.ndarray
        Shape (3, 3) rotation matrix.
    """
    z_axis = np.asarray(vec).astype(float)
    norm = np.linalg.norm(z_axis)
    if norm < 1e-6:
        # Default Z-down if vector is degenerate
        return rot_x(np.pi)
    
    z_axis = z_axis / norm
    
    # Choose an arbitrary up vector to orthogonalize against
    # If z_axis is almost vertical, use a different arbitrary vector
    if abs(z_axis[0]) > 0.9:
        ref_vec = np.array([0.0, 1.0, 0.0])
    else:
        ref_vec = np.array([1.0, 0.0, 0.0])
        
    y_axis = np.cross(z_axis, ref_vec)
    y_axis /= np.linalg.norm(y_axis)
    
    x_axis = np.cross(y_axis, z_axis)
    
    # Create the rotation matrix
    # Columns are X, Y, Z axes
    R = np.column_stack((x_axis, y_axis, z_axis))
    return R

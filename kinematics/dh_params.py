"""
DH parameter definitions for a 6R (6-revolute) articulated desktop arm.

Standard DH convention is used. The arm is designed as a compact, desktop-scale
robot with a total reach of approximately 500 mm.
"""

from dataclasses import dataclass
from typing import List, Dict
import numpy as np


@dataclass
class DH6R:
    """
    Standard Denavit-Hartenberg parameters for a 6R articulated arm.

    All distances in metres, angles in radians.

    Attributes
    ----------
    a : List[float]
        Link lengths for joints 1-6 (metres).
    alpha : List[float]
        Link twists for joints 1-6 (radians). Standard DH convention.
    d : List[float]
        Link offsets (prismatic d values) for joints 1-6 (metres).
    q_min : List[float]
        Joint angle lower bounds (radians).
    q_max : List[float]
        Joint angle upper bounds (radians).
    names : List[str]
        Joint names for labeling.
    """

    a: List[float]
    alpha: List[float]
    d: List[float]
    q_min: List[float]
    q_max: List[float]
    names: List[str]

    def to_dict(self) -> Dict:
        """Convert to dict for easier iteration."""
        return {
            "a": self.a,
            "alpha": self.alpha,
            "d": self.d,
            "q_min": self.q_min,
            "q_max": self.q_max,
            "names": self.names,
        }

    def get_row(self, i: int) -> tuple:
        """Get DH parameters for joint i (0-indexed)."""
        return (self.a[i], self.alpha[i], self.d[i], self.q_min[i], self.q_max[i])


def get_dh_table() -> DH6R:
    """
    Return the standard DH parameters for a 6R articulated desktop arm.

    The arm follows a typical SCARA-like configuration:
      - Joint 1 (base): rotates about Z-axis
      - Joints 2, 3 (shoulder, elbow): horizontal plane rotations
      - Joint 4 (wrist 1): vertical rotation
      - Joints 5, 6 (wrist 2, wrist 3): fine orientation control

    Link lengths and offsets are realistic for a desktop arm with ~500 mm reach:
      - d1 = 0.10 m  (base height to shoulder)
      - a2 = 0.20 m  (upper arm)
      - a3 = 0.15 m  (forearm)
      - d4 = 0.08 m  (wrist offset)
      - a4 = 0.05 m  (wrist length 1)
      - d6 = 0.05 m  (tool offset)

    Joint limits are ±160° for revolute joints (typical for industrial arms).

    Returns
    -------
    DH6R
        Dataclass with a, alpha, d, q_min, q_max, names.
    """

    # Link lengths (a) and offsets (d) in metres
    a = [0.0,   0.20,  0.15,  0.05,  0.0,   0.0  ]
    d = [0.10,  0.0,   0.0,   0.08,  0.0,   0.05 ]

    # Link twists (alpha) in radians
    # Standard convention for 6R arm
    alpha = [
        0.0,                    # Joint 1
        np.pi / 2,              # Joint 2
        0.0,                    # Joint 3
        -np.pi / 2,             # Joint 4
        np.pi / 2,              # Joint 5
        -np.pi / 2,             # Joint 6
    ]

    # Joint angle limits (radians): ±160°
    angle_limit = np.deg2rad(160.0)
    q_min = [-angle_limit] * 6
    q_max = [angle_limit] * 6

    names = ["Joint 1 (Base)", "Joint 2 (Shoulder)", "Joint 3 (Elbow)",
             "Joint 4 (Wrist 1)", "Joint 5 (Wrist 2)", "Joint 6 (Wrist 3)"]

    return DH6R(
        a=a,
        alpha=alpha,
        d=d,
        q_min=q_min,
        q_max=q_max,
        names=names,
    )

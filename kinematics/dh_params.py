"""
DH parameter definitions for the AR2 6R articulated desktop robot arm.

Standard Denavit-Hartenberg convention (Craig notation) is used.
Parameters are derived from the official AR2 calibration file (ARbot.cal)
distributed with the AR2 2.0 software by Chris Annin (anninrobotics.com).

Physical summary
----------------
  d1  = 169.77 mm  — base height to J2 shoulder pivot
  a1  =  64.20 mm  — horizontal J1→J2 axis offset
  a2  = 305.00 mm  — upper-arm link length (J2→J3)
  d4  = 222.63 mm  — forearm / wrist-offset length (J3→spherical-wrist centre)
  d6  =  36.25 mm  — tool / end-effector offset (wrist centre→tip)
  Max horizontal reach ≈ a1 + a2 ≈ 369 mm from J1 axis
  Total arm extension   ≈ d1 + a2 + d4 + d6 ≈ 634 mm
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
    theta_offset : List[float]
        Constant angle added to each joint variable before the DH transform,
        i.e.  θ_DH = q_i + theta_offset[i].  Used to align the DH zero position
        with the robot's physical home / calibration position.
        AR2 values (radians): [0, 0, −π/2, 0, 0, π].
    q_min : List[float]
        Joint angle lower bounds in native joint space (radians).
    q_max : List[float]
        Joint angle upper bounds in native joint space (radians).
    names : List[str]
        Joint names for labeling.
    """

    a: List[float]
    alpha: List[float]
    d: List[float]
    theta_offset: List[float]
    q_min: List[float]
    q_max: List[float]
    names: List[str]

    def to_dict(self) -> Dict:
        """Convert to dict for easier iteration."""
        return {
            "a": self.a,
            "alpha": self.alpha,
            "d": self.d,
            "theta_offset": self.theta_offset,
            "q_min": self.q_min,
            "q_max": self.q_max,
            "names": self.names,
        }

    def get_row(self, i: int) -> tuple:
        """Get DH parameters for joint i (0-indexed)."""
        return (self.a[i], self.alpha[i], self.d[i], self.theta_offset[i],
                self.q_min[i], self.q_max[i])


def get_dh_table() -> DH6R:
    """
    Return the AR2 DH parameters sourced from the official ARbot.cal file.

    DH convention: standard (Craig) — T = Rz(θ)·Tz(d)·Tx(a)·Rx(α).

    Joint axes
    ----------
      J1 — base rotation (Z)
      J2 — shoulder pitch (Y)
      J3 — elbow pitch (Y)
      J4 — wrist roll (Z of forearm)
      J5 — wrist pitch (Y)
      J6 — wrist roll / tool spin (Z)

    DH table (AR2 ARbot.cal, converted to SI)
    -----------------------------------------
    Joint |  a (m)   |  alpha (°) |  d (m)    | Notes
    ------+----------+------------+-----------+-------------------------------
      1   |  0.06420 |   -90      |  0.16977  | base→shoulder, 64 mm offset
      2   |  0.30500 |     0      |  0.0      | upper arm 305 mm
      3   |  0.0     |   +90      |  0.0      | elbow to wrist centre
      4   |  0.0     |   -90      | -0.22263  | forearm 222.6 mm (-Z)
      5   |  0.0     |   +90      |  0.0      | wrist pitch
      6   |  0.0     |     0      | -0.03625  | tool offset 36.25 mm (-Z)

    Joint limits (from ARbot.cal)
    -----------------------------
      J1 : −170° … +170°  (base rotation, cable-limited)
      J2 : −132° …   0°   (shoulder; 0° = arm pointing up, −132° = arm back)
      J3 :   +1° … +141°  (elbow; effectively forward-flex only)
      J4 : −165° … +165°  (wrist roll)
      J5 : −105° … +105°  (wrist pitch)
      J6 : −155° … +155°  (wrist spin)

    Returns
    -------
    DH6R
        Dataclass with a, alpha, d, q_min, q_max, names.
    """

    # Link lengths (a) and offsets (d) — metres, straight from ARbot.cal ÷ 1000
    a = [0.06420, 0.30500, 0.0,  0.0,  0.0,  0.0    ]
    d = [0.16977, 0.0,     0.0, -0.22263, 0.0, -0.03625]

    # Link twists (alpha) — radians, from ARbot.cal DHr values (in degrees)
    alpha = [
        -np.pi / 2,   # J1: −90°
         0.0,          # J2:   0°
         np.pi / 2,   # J3: +90°
        -np.pi / 2,   # J4: −90°
         np.pi / 2,   # J5: +90°
         0.0,          # J6:   0°
    ]

    # Theta offsets (DHt) — radians, from ARbot.cal indices 71–76
    # Applied as: θ_DH = q_joint + theta_offset
    # AR2 home calibration uses DHt3=−90° and DHt6=+180° to align
    # the DH zero-angle with the physical home position.
    theta_offset = [
        0.0,           # J1: DHt1 = 0°
        0.0,           # J2: DHt2 = 0°
        -np.pi / 2,   # J3: DHt3 = −90°
        0.0,           # J4: DHt4 = 0°
        0.0,           # J5: DHt5 = 0°
        np.pi,         # J6: DHt6 = +180°
    ]

    # Joint angle limits — radians, from ARbot.cal indices 35–51
    q_min = [
        np.deg2rad(-170),   # J1
        np.deg2rad(-132),   # J2
        np.deg2rad(   1),   # J3
        np.deg2rad(-165),   # J4
        np.deg2rad(-105),   # J5
        np.deg2rad(-155),   # J6
    ]
    q_max = [
        np.deg2rad( 170),   # J1
        np.deg2rad(   0),   # J2
        np.deg2rad( 141),   # J3
        np.deg2rad( 165),   # J4
        np.deg2rad( 105),   # J5
        np.deg2rad( 155),   # J6
    ]

    names = [
        "J1 Base",
        "J2 Shoulder",
        "J3 Elbow",
        "J4 Wrist Roll",
        "J5 Wrist Pitch",
        "J6 Tool Spin",
    ]

    return DH6R(
        a=a,
        alpha=alpha,
        d=d,
        theta_offset=theta_offset,
        q_min=q_min,
        q_max=q_max,
        names=names,
    )

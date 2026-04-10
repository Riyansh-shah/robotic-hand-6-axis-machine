"""
3-D plotting helpers shared by kinematics visualiser and simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3D projection


def plot_frame(
    ax,
    T: np.ndarray,
    scale: float = 0.05,
    label: str = "",
    linewidth: float = 1.5,
) -> None:
    """
    Draw a coordinate frame (RGB = XYZ) at the pose given by 4×4 matrix T.

    Parameters
    ----------
    ax     : matplotlib 3D axes
    T      : 4×4 homogeneous transform
    scale  : arrow length in world units (metres)
    label  : text label placed at origin of the frame
    """
    origin = T[:3, 3]
    x_axis = T[:3, 0] * scale
    y_axis = T[:3, 1] * scale
    z_axis = T[:3, 2] * scale

    for vec, color in zip([x_axis, y_axis, z_axis], ["r", "g", "b"]):
        ax.quiver(
            *origin, *vec,
            color=color, linewidth=linewidth,
            arrow_length_ratio=0.3,
        )
    if label:
        ax.text(*origin, f"  {label}", fontsize=7, color="white")


def make_3d_axes(title: str = "", figsize=(8, 7)) -> tuple:
    """Create a styled 3D figure/axes pair with a dark background."""
    fig = plt.figure(figsize=figsize, facecolor="#020617")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#020617")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#334155")
    ax.tick_params(colors="#94a3b8", labelsize=7)
    ax.set_xlabel("X (m)", color="#94a3b8", labelpad=6)
    ax.set_ylabel("Y (m)", color="#94a3b8", labelpad=6)
    ax.set_zlabel("Z (m)", color="#94a3b8", labelpad=6)
    if title:
        ax.set_title(title, color="#e2e8f0", fontsize=11, pad=12)
    return fig, ax

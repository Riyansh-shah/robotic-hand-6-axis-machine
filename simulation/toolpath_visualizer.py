"""
Toolpath and arm trajectory visualization for simulation outputs.

Provides functions to visualize Cartesian waypoints from g-code with extrusion
coloring, animate the arm tracing through joint trajectories, and compute
Monte Carlo workspace cross-sections.
"""

from typing import List, Optional, Union
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap
import sys
from pathlib import Path

# Add parent directory to path to import kinematics and utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from kinematics.dh_params import DH6R, get_dh_table
from kinematics.forward_kinematics import forward_kinematics, get_all_transforms
from utils.plotting import make_3d_axes, plot_frame
from .arm_simulator import ArmSimulator


def visualize_toolpath(
    waypoints: List,
    title: str = "Toolpath",
    figsize: tuple = (10, 8),
) -> tuple:
    """
    Plot Cartesian waypoints in 3D with color-coded extrusion.

    Visualizes a sequence of waypoints from g-code or other trajectory sources.
    Travel moves (no extrusion) are shown in blue; extrusion moves in red.

    Parameters
    ----------
    waypoints : List[Waypoint]
        List of Waypoint objects from gcode_parser, each with x, y, z, e, move_type.
    title : str, optional
        Figure title. Default: "Toolpath".
    figsize : tuple, optional
        Figure size (width, height). Default: (10, 8).

    Returns
    -------
    fig : matplotlib.figure.Figure
        The matplotlib figure.
    ax : mpl_toolkits.mplot3d.axes3d.Axes3D
        The 3D axes.
    """
    fig, ax = make_3d_axes(title=title, figsize=figsize)

    if not waypoints:
        return fig, ax

    # Extract positions and extrusion status
    positions = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])
    extrusion_status = np.array([wp.e > 0 for wp in waypoints])

    # Separate travel (no extrusion) and extrusion moves
    travel_mask = ~extrusion_status
    extrusion_mask = extrusion_status

    # Plot travel moves (blue, dashed)
    if np.any(travel_mask):
        travel_pts = positions[travel_mask]
        ax.plot(
            travel_pts[:, 0], travel_pts[:, 1], travel_pts[:, 2],
            color="#3b82f6", linewidth=1.5, linestyle="--",
            alpha=0.6, label="Travel"
        )
        ax.scatter(
            travel_pts[:, 0], travel_pts[:, 1], travel_pts[:, 2],
            color="#3b82f6", s=15, alpha=0.5
        )

    # Plot extrusion moves (red, solid)
    if np.any(extrusion_mask):
        extrusion_pts = positions[extrusion_mask]
        ax.plot(
            extrusion_pts[:, 0], extrusion_pts[:, 1], extrusion_pts[:, 2],
            color="#ef4444", linewidth=2.0, alpha=0.8, label="Extrude"
        )
        ax.scatter(
            extrusion_pts[:, 0], extrusion_pts[:, 1], extrusion_pts[:, 2],
            color="#ef4444", s=20, alpha=0.7
        )

    # Set axis properties
    ax.set_xlabel("X (m)", color="#94a3b8")
    ax.set_ylabel("Y (m)", color="#94a3b8")
    ax.set_zlabel("Z (m)", color="#94a3b8")

    # Auto-scale axes
    ranges = [
        np.max(positions[:, i]) - np.min(positions[:, i])
        for i in range(3)
    ]
    max_range = max(ranges) * 0.6
    centers = [np.mean(positions[:, i]) for i in range(3)]

    ax.set_xlim([centers[0] - max_range, centers[0] + max_range])
    ax.set_ylim([centers[1] - max_range, centers[1] + max_range])
    ax.set_zlim([centers[2] - max_range, centers[2] + max_range])

    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    plt.tight_layout()

    return fig, ax


def visualize_arm_on_toolpath(
    simulator: ArmSimulator,
    trajectory: List[Union[List[float], np.ndarray]],
    waypoints: Optional[List] = None,
    interval: int = 80,
    figsize: tuple = (12, 9),
) -> FuncAnimation:
    """
    Animate the arm tracing through a joint trajectory.

    Shows the arm linkage moving through a sequence of joint configurations,
    with optional overlay of the Cartesian toolpath. Displays arm links,
    joint coordinate frames, and end-effector trace line.

    Parameters
    ----------
    simulator : ArmSimulator
        Arm simulator instance (used to compute FK).
    trajectory : List[array-like]
        Sequence of joint configurations, each shape (6,).
    waypoints : List[Waypoint], optional
        Cartesian waypoints to overlay as reference. If provided, travel
        moves (blue dashed) and extrusion moves (red solid) are shown.
        Default: None (no waypoint overlay).
    interval : int, optional
        Delay between animation frames in milliseconds. Default: 80 ms (~12 fps).
    figsize : tuple, optional
        Figure size (width, height). Default: (12, 9).

    Returns
    -------
    anim : FuncAnimation
        Animation object. Call plt.show() to display.
    """
    dh_table = simulator.dh_table
    trajectory = [np.asarray(q).ravel() for q in trajectory]

    # Execute trajectory to get EE trace
    simulator.execute_trajectory(trajectory)
    ee_trace = simulator.get_ee_trace()

    # Precompute all joint transforms for each frame
    all_transforms = [get_all_transforms(q, dh_table) for q in trajectory]

    # Create figure
    fig, ax = make_3d_axes(title="Arm Tracing Toolpath", figsize=figsize)

    # Plot waypoint reference if provided
    if waypoints is not None:
        wp_positions = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])
        extrusion_status = np.array([wp.e > 0 for wp in waypoints])

        travel_mask = ~extrusion_status
        extrusion_mask = extrusion_status

        if np.any(travel_mask):
            travel_pts = wp_positions[travel_mask]
            ax.plot(
                travel_pts[:, 0], travel_pts[:, 1], travel_pts[:, 2],
                color="#3b82f6", linewidth=1.0, linestyle="--",
                alpha=0.4, label="Waypoint Travel"
            )

        if np.any(extrusion_mask):
            extrusion_pts = wp_positions[extrusion_mask]
            ax.plot(
                extrusion_pts[:, 0], extrusion_pts[:, 1], extrusion_pts[:, 2],
                color="#ef4444", linewidth=1.5, alpha=0.5, label="Waypoint Extrude"
            )

    # Line for arm linkage
    line, = ax.plot([], [], [], color="#60a5fa", linewidth=2.5, marker="o",
                     markersize=4, markerfacecolor="#f97316", markeredgecolor="#60a5fa",
                     label="Arm Linkage")

    # Scatter for EE trace
    trace_scatter = ax.scatter([], [], [], color="#22c55e", s=25, alpha=0.7,
                                label="EE Trace")

    # Text for frame info
    text_info = ax.text2D(0.05, 0.95, "", transform=ax.transAxes,
                          fontsize=10, color="#e2e8f0", verticalalignment="top",
                          bbox=dict(boxstyle="round", facecolor="#0f172a", alpha=0.8))

    # Compute axis limits from all poses
    all_positions = np.vstack([
        np.array([T[:3, 3] for T in transforms])
        for transforms in all_transforms
    ])

    if waypoints is not None:
        wp_positions = np.array([[wp.x, wp.y, wp.z] for wp in waypoints])
        all_positions = np.vstack([all_positions, wp_positions])

    ranges = [
        np.max(all_positions[:, i]) - np.min(all_positions[:, i])
        for i in range(3)
    ]
    max_range = max(ranges) * 0.6
    centers = [np.mean(all_positions[:, i]) for i in range(3)]

    ax.set_xlim([centers[0] - max_range, centers[0] + max_range])
    ax.set_ylim([centers[1] - max_range, centers[1] + max_range])
    ax.set_zlim([centers[2] - max_range, centers[2] + max_range])

    # Collections for frame artists and link artists
    frame_artists = []
    link_artists = []

    def update(frame_idx: int):
        """Update function for animation frame."""
        # Get current joint transforms and positions
        transforms = all_transforms[frame_idx]
        positions = np.array([T[:3, 3] for T in transforms])

        # Update arm linkage line
        line.set_data(positions[:, 0], positions[:, 1])
        line.set_3d_properties(positions[:, 2])

        # Update EE trace scatter
        if len(ee_trace) > 0:
            trace_pts = ee_trace[:frame_idx + 1]
            trace_scatter.set_offsets(trace_pts[:, :2])
            trace_scatter.set_3d_properties(trace_pts[:, 2])

        # Clear and redraw coordinate frames
        for artist in frame_artists:
            artist.remove()
        frame_artists.clear()

        frame_scale = 0.025
        for i, T in enumerate(transforms):
            origin = T[:3, 3]
            x_axis = T[:3, 0] * frame_scale
            y_axis = T[:3, 1] * frame_scale
            z_axis = T[:3, 2] * frame_scale

            for vec, color in zip([x_axis, y_axis, z_axis], ["r", "g", "b"]):
                q = ax.quiver(*origin, *vec, color=color, linewidth=1.2,
                              arrow_length_ratio=0.25, alpha=0.6)
                frame_artists.append(q)

        # Update text info
        q_str = ", ".join([f"{q:.2f}" for q in trajectory[frame_idx][:3]])
        text_info.set_text(
            f"Frame {frame_idx + 1}/{len(trajectory)}\nq1-3: [{q_str}] rad"
        )

        return line, trace_scatter, text_info, *frame_artists

    anim = FuncAnimation(
        fig, update, frames=len(trajectory), interval=interval,
        blit=False, repeat=True
    )

    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    plt.tight_layout()

    return anim


def plot_workspace_slice(
    dh_table: Optional[DH6R] = None,
    joint_idx: int = 0,
    n_samples: int = 5000,
    figsize: tuple = (10, 8),
) -> tuple:
    """
    Compute and plot a workspace cross-section using Monte Carlo sampling.

    Randomizes all joint angles within their limits and computes the
    reachable end-effector positions. Plots a 2D scatter of positions
    to visualize the workspace.

    Parameters
    ----------
    dh_table : DH6R, optional
        DH parameter dataclass. If None, uses default DH6R arm.
    joint_idx : int, optional
        Projection axis index (0=XY, 1=YZ, 2=XZ plane). Default: 0 (XY plane).
    n_samples : int, optional
        Number of random samples to generate. Default: 5000.
    figsize : tuple, optional
        Figure size (width, height). Default: (10, 8).

    Returns
    -------
    fig : matplotlib.figure.Figure
        The matplotlib figure.
    ax : matplotlib.axes.Axes
        The 2D axes with scatter plot.
    """
    if dh_table is None:
        dh_table = get_dh_table()

    # Generate random joint configurations within joint limits
    samples = np.random.uniform(
        low=dh_table.q_min,
        high=dh_table.q_max,
        size=(n_samples, 6)
    )

    # Compute end-effector positions for all samples
    ee_positions = []
    for sample in samples:
        T = forward_kinematics(sample, dh_table)
        ee_positions.append(T[:3, 3])

    ee_positions = np.array(ee_positions)

    # Create 2D projection based on joint_idx
    axis_labels = {
        0: ("X (m)", "Y (m)", "XY Plane"),
        1: ("Y (m)", "Z (m)", "YZ Plane"),
        2: ("X (m)", "Z (m)", "XZ Plane"),
    }
    idx_pairs = {
        0: (0, 1),
        1: (1, 2),
        2: (0, 2),
    }

    if joint_idx not in idx_pairs:
        joint_idx = 0

    i, j = idx_pairs[joint_idx]
    x_label, y_label, title = axis_labels[joint_idx]

    # Plot workspace
    fig, ax = plt.subplots(figsize=figsize, facecolor="#020617")
    ax.set_facecolor("#020617")

    scatter = ax.scatter(
        ee_positions[:, i], ee_positions[:, j],
        c=ee_positions[:, 2], cmap="viridis", s=5, alpha=0.6,
        edgecolors="none"
    )

    ax.set_xlabel(x_label, color="#94a3b8", fontsize=11)
    ax.set_ylabel(y_label, color="#94a3b8", fontsize=11)
    ax.set_title(f"Workspace: {title}", color="#e2e8f0", fontsize=12, pad=12)

    ax.tick_params(colors="#94a3b8", labelsize=9)
    ax.spines["bottom"].set_color("#334155")
    ax.spines["left"].set_color("#334155")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # Add colorbar for Z-height
    cbar = fig.colorbar(scatter, ax=ax, pad=0.02)
    cbar.set_label("Z (m)", color="#94a3b8", labelpad=10)
    cbar.ax.tick_params(colors="#94a3b8", labelsize=8)

    # Grid
    ax.grid(True, alpha=0.15, color="#334155", linestyle="-", linewidth=0.5)

    plt.tight_layout()

    return fig, ax

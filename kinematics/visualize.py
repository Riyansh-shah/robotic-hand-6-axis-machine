"""
3D visualization for the 6-DOF robotic arm.

Provides functions to plot arm linkage and animate trajectories.
"""

from typing import List, Optional, Union
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

try:
    from stl import mesh
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    HAS_STL = True
except ImportError:
    HAS_STL = False
import sys
from pathlib import Path

# Add parent directory to path to import utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.plotting import make_3d_axes, plot_frame
from .dh_params import DH6R, get_dh_table
from .forward_kinematics import get_all_transforms


def plot_arm(
    joint_angles: Union[List[float], np.ndarray],
    dh_table: DH6R,
    ax: Optional[plt.Axes] = None,
    show_frames: bool = True,
    frame_scale: float = 0.03,
    link_linewidth: float = 2.5,
    plot_cad: bool = True,
    cad_dir: str = "CAD/STL",
) -> tuple:
    """
    Plot the arm linkage in 3D with joint coordinate frames.

    Draws the arm skeleton (joints connected by links) and optionally shows
    the coordinate frame at each joint.

    Parameters
    ----------
    joint_angles : array-like, shape (6,)
        Joint angles in radians.
    dh_table : DH6R
        DH parameter dataclass.
    ax : matplotlib 3D axes, optional
        Axes to plot on. If None, creates a new figure and axes.
    show_frames : bool, optional
        If True, draw coordinate frames at each joint. Default: True.
    frame_scale : float, optional
        Length of frame arrows in metres. Default: 0.03 m.
    link_linewidth : float, optional
        Line width for the arm linkage. Default: 2.5.
    plot_cad : bool, optional
        Whether to attempt loading and plotting STL files. Default: True.
    cad_dir : str, optional
        Directory to look for link0.stl, link1.stl, etc. Default: "CAD/STL".

    Returns
    -------
    fig : matplotlib figure
    ax : matplotlib 3D axes
    """
    if ax is None:
        fig, ax = make_3d_axes(title="6-DOF Arm Kinematics")
    else:
        fig = ax.get_figure()

    joint_angles = np.asarray(joint_angles).ravel()

    # Get all joint transforms
    transforms = get_all_transforms(joint_angles, dh_table)

    # Extract joint positions (origins of each frame)
    positions = np.array([T[:3, 3] for T in transforms])

    # Draw linkage (thick lines connecting joints)
    ax.plot(
        positions[:, 0], positions[:, 1], positions[:, 2],
        color="#60a5fa", linewidth=link_linewidth, marker="o",
        markersize=4, markerfacecolor="#f97316", markeredgecolor="#60a5fa",
        label="Linkage",
    )

    # Draw coordinate frames at each joint
    if show_frames:
        for i, T in enumerate(transforms):
            if i == 0:
                label = "Base"
            elif i <= len(dh_table.names):
                label = dh_table.names[i - 1]
            else:
                label = f"Frame {i}"
            plot_frame(ax, T, scale=frame_scale, label=label)

    # Plot CAD models if requested and available
    if plot_cad and HAS_STL:
        # Resolve path
        proj_root = Path(__file__).parent.parent
        cad_path = proj_root / cad_dir
        
        for i, T in enumerate(transforms):
            stl_file = cad_path / f"link{i}.stl"
            if stl_file.exists():
                try:
                    stl_mesh = mesh.Mesh.from_file(str(stl_file))
                    # Transform vertices
                    verts = stl_mesh.vectors.reshape(-1, 3)
                    verts_h = np.hstack((verts, np.ones((verts.shape[0], 1))))
                    transformed_verts_h = (T @ verts_h.T).T
                    transformed_vectors = transformed_verts_h[:, :3].reshape(-1, 3, 3)
                    
                    # Create 3D collection and add to plot
                    collection = Poly3DCollection(
                        transformed_vectors, 
                        facecolors='#94a3b8', 
                        linewidths=0.1, 
                        edgecolors='#334155', 
                        alpha=0.6
                    )
                    ax.add_collection3d(collection)
                except Exception as e:
                    print(f"Warning: Failed to load or plot CAD file {stl_file}: {e}")

    # Set axis properties
    ax.set_xlabel("X (m)", color="#94a3b8")
    ax.set_ylabel("Y (m)", color="#94a3b8")
    ax.set_zlabel("Z (m)", color="#94a3b8")

    # Auto-scale axes to arm workspace
    all_pts = positions
    if len(all_pts) > 0:
        ranges = [
            np.max(all_pts[:, i]) - np.min(all_pts[:, i])
            for i in range(3)
        ]
        max_range = max(ranges) * 0.6
        centers = [np.mean(all_pts[:, i]) for i in range(3)]

        for i, (center, axis) in enumerate(zip(centers, [ax.set_xlim, ax.set_ylim, ax.set_zlim])):
            axis([center - max_range, center + max_range])

    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    plt.tight_layout()

    return fig, ax


def animate_arm(
    trajectory: List[Union[List[float], np.ndarray]],
    dh_table: Optional[DH6R] = None,
    interval: int = 50,
    show_ee_path: bool = True,
    frame_scale: float = 0.02,
    figsize: tuple = (10, 8),
    plot_cad: bool = True,
    cad_dir: str = "CAD/STL",
) -> FuncAnimation:
    """
    Animate the arm moving through a trajectory of joint configurations.

    Creates a matplotlib animation showing the arm linkage moving through
    a sequence of poses, optionally with the end-effector path traced.

    Parameters
    ----------
    trajectory : List[array-like]
        List of joint angle configurations, each shape (6,).
    dh_table : DH6R, optional
        DH parameter dataclass. If None, uses default 6R arm.
    interval : int, optional
        Delay between frames in milliseconds. Default: 50 ms (~20 fps).
    show_ee_path : bool, optional
        If True, traces the end-effector path. Default: True.
    frame_scale : float, optional
        Scale of coordinate frames. Default: 0.02 m.
    figsize : tuple, optional
        Figure size (width, height). Default: (10, 8).

    Returns
    -------
    anim : FuncAnimation
        Animation object. Call .show() or plt.show() to display.
    """
    if dh_table is None:
        dh_table = get_dh_table()

    trajectory = [np.asarray(q).ravel() for q in trajectory]

    # Precompute all transforms
    all_transforms = [get_all_transforms(q, dh_table) for q in trajectory]

    # Precompute end-effector positions for path tracing
    ee_positions = np.array([T[-1][:3, 3] for T in all_transforms])

    # Create figure
    fig, ax = make_3d_axes(title="6-DOF Arm Trajectory Animation", figsize=figsize)

    # Line collection for linkage
    line, = ax.plot([], [], [], color="#60a5fa", linewidth=2.5, marker="o",
                     markersize=4, markerfacecolor="#f97316", markeredgecolor="#60a5fa")

    # Scatter for EE path
    path_scatter = ax.scatter([], [], [], color="#22c55e", s=20, alpha=0.6, label="EE Path")

    # Text for frame counter
    text_info = ax.text2D(0.05, 0.95, "", transform=ax.transAxes,
                          fontsize=10, color="#e2e8f0", verticalalignment="top",
                          bbox=dict(boxstyle="round", facecolor="#0f172a", alpha=0.8))

    # Set axis limits based on all poses
    all_positions = np.vstack([np.array([T[:3, 3] for T in transforms])
                               for transforms in all_transforms])
    ranges = [np.max(all_positions[:, i]) - np.min(all_positions[:, i])
              for i in range(3)]
    max_range = max(ranges) * 0.6
    centers = [np.mean(all_positions[:, i]) for i in range(3)]

    ax.set_xlim([centers[0] - max_range, centers[0] + max_range])
    ax.set_ylim([centers[1] - max_range, centers[1] + max_range])
    ax.set_zlim([centers[2] - max_range, centers[2] + max_range])

    # Pre-load CAD models into memory if available
    cad_models = []
    if plot_cad and HAS_STL:
        proj_root = Path(__file__).parent.parent
        cad_path = proj_root / cad_dir
        for i in range(len(all_transforms[0])):
            stl_file = cad_path / f"link{i}.stl"
            if stl_file.exists():
                try:
                    cad_models.append(mesh.Mesh.from_file(str(stl_file)))
                except Exception as e:
                    print(f"Warning: Failed to load {stl_file}: {e}")
                    cad_models.append(None)
            else:
                cad_models.append(None)
    else:
        cad_models = [None] * len(all_transforms[0])

    # Quiver collections for coordinate frames and collections for CAD models
    frame_artists = []
    cad_artists = []

    def update(frame_idx: int):
        """Update function for animation."""
        # Get current transforms
        transforms = all_transforms[frame_idx]
        positions = np.array([T[:3, 3] for T in transforms])

        # Update linkage line
        line.set_data(positions[:, 0], positions[:, 1])
        line.set_3d_properties(positions[:, 2])

        # Update EE path
        if show_ee_path:
            path_pts = ee_positions[:frame_idx + 1]
            path_scatter.set_offsets(path_pts[:, :2])
            path_scatter.set_3d_properties(path_pts[:, 2])

        # Update coordinate frames
        # Clear old frames
        for artist in frame_artists:
            artist.remove()
        frame_artists.clear()
        
        # Clear old CAD overlays
        for artist in cad_artists:
            artist.remove()
        cad_artists.clear()

        # Draw new frames and CAD models
        for i, T in enumerate(transforms):
            # Draw frames
            origin = T[:3, 3]
            x_axis = T[:3, 0] * frame_scale
            y_axis = T[:3, 1] * frame_scale
            z_axis = T[:3, 2] * frame_scale

            for vec, color in zip([x_axis, y_axis, z_axis], ["r", "g", "b"]):
                q = ax.quiver(*origin, *vec, color=color, linewidth=1.5,
                              arrow_length_ratio=0.3, alpha=0.7)
                frame_artists.append(q)
                
            # Render CAD models
            if i < len(cad_models) and cad_models[i] is not None:
                stl_mesh = cad_models[i]
                verts = stl_mesh.vectors.reshape(-1, 3)
                verts_h = np.hstack((verts, np.ones((verts.shape[0], 1))))
                transformed_verts_h = (T @ verts_h.T).T
                transformed_vectors = transformed_verts_h[:, :3].reshape(-1, 3, 3)
                
                collection = Poly3DCollection(
                    transformed_vectors, 
                    facecolors='#94a3b8', 
                    linewidths=0.1, 
                    edgecolors='#334155', 
                    alpha=0.6
                )
                ax.add_collection3d(collection)
                cad_artists.append(collection)

        # Update text info
        q_str = ", ".join([f"{q:.2f}" for q in trajectory[frame_idx][:3]])
        text_info.set_text(f"Frame {frame_idx + 1}/{len(trajectory)}\nq1-3: [{q_str}] rad")

        return line, path_scatter, text_info, *frame_artists, *cad_artists

    anim = FuncAnimation(
        fig, update, frames=len(trajectory), interval=interval,
        blit=False, repeat=True
    )

    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    plt.tight_layout()

    return anim

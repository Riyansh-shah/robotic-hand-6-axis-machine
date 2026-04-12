#!/usr/bin/env python3
"""
visualize_slider.py — Interactive AR2 arm + toolpath viewer with timeline slider.

Drag the slider to scrub through any joint trajectory frame-by-frame.
The arm is rendered with realistic cylindrical link geometry and the
Bondtech extruder STL as the end-effector.

Usage
-----
    python visualize_slider.py                               # sphere gcode (default)
    python visualize_slider.py examples/sample_cube.gcode
    python visualize_slider.py examples/sample_sphere.gcode --frames 600
"""

from __future__ import annotations
import sys, argparse, logging
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from kinematics.dh_params import get_dh_table
from kinematics.forward_kinematics import get_all_transforms
from gcode_parser import GCodeParser
from gcode_parser.filters import (
    filter_travel_moves, downsample_waypoints, transform_to_arm_frame
)
from trajectory_planner.cartesian_to_joint import waypoints_to_joint_trajectory
from trajectory_planner.interpolation import interpolate_trajectory

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s  %(levelname)-8s  %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("slider")

# ─── theme ───────────────────────────────────────────────────────────────────
BG         = "#020617"
GRID_COL   = "#0f172a"
LINK_COLS  = ["#1e3a5f", "#1d4ed8", "#2563eb", "#3b82f6", "#60a5fa", "#93c5fd"]
JOINT_COL  = "#f97316"
EE_COL     = "#22c55e"
TRACE_COL  = "#60a5fa"
PATH_COL   = "#ef4444"

# Link tube radii (metres) — base is widest, wrist is narrowest
LINK_RADII = [0.038, 0.026, 0.020, 0.016, 0.013, 0.011]
JOINT_R    = 0.014   # joint sphere radius


# ─── geometry helpers ─────────────────────────────────────────────────────────

def _rotation_z_to(v_hat: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix that rotates [0,0,1] onto unit vector v_hat."""
    z = np.array([0., 0., 1.])
    dot = np.dot(z, v_hat)
    if dot > 0.9999:
        return np.eye(3)
    if dot < -0.9999:
        return np.diag([1., -1., -1.])
    axis = np.cross(z, v_hat)
    axis /= np.linalg.norm(axis)
    ang = np.arccos(np.clip(dot, -1., 1.))
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def _cylinder_faces(p1, p2, r: float, n: int = 14) -> np.ndarray:
    """
    Return (n, 4, 3) array of quad face vertices for a cylinder from p1 to p2.
    Returns empty array if the segment is too short.
    """
    v = np.asarray(p2, float) - np.asarray(p1, float)
    L = np.linalg.norm(v)
    if L < 1e-5:
        return np.empty((0, 4, 3))
    R = _rotation_z_to(v / L)
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.column_stack([r * np.cos(theta), r * np.sin(theta), np.zeros(n)])
    bot = (R @ ring.T).T + p1
    top = (R @ (ring + [0, 0, L]).T).T + p1
    # quad faces: current, next, next+top, top
    nxt = np.roll(bot, -1, axis=0)
    nxt_t = np.roll(top, -1, axis=0)
    return np.stack([bot, nxt, nxt_t, top], axis=1)   # (n, 4, 3)


def _sphere_faces(centre, r: float, nu: int = 10, nv: int = 7) -> np.ndarray:
    """Return (N, 3, 3) triangle face array for a UV sphere."""
    u = np.linspace(0, 2 * np.pi, nu, endpoint=False)
    v = np.linspace(0, np.pi, nv + 1)
    faces = []
    for i in range(nu):
        for j in range(nv):
            u0, u1 = u[i], u[(i + 1) % nu]
            v0, v1 = v[j], v[j + 1]
            def pt(uu, vv):
                return centre + r * np.array([np.sin(vv)*np.cos(uu),
                                              np.sin(vv)*np.sin(uu),
                                              np.cos(vv)])
            a, b, c, d = pt(u0,v0), pt(u1,v0), pt(u1,v1), pt(u0,v1)
            faces.append([a, b, c])
            faces.append([a, c, d])
    return np.array(faces)


def _add_collection(ax, faces, color, alpha=0.92, edge="none"):
    col = Poly3DCollection(faces, alpha=alpha)
    col.set_facecolor(color)
    col.set_edgecolor(edge)
    ax.add_collection3d(col)
    return col


# ─── STL loader ───────────────────────────────────────────────────────────────

def _load_stl(path: Path, scale: float = 0.001, max_faces: int = 800) -> np.ndarray | None:
    """
    Load an STL file, centre at origin, scale (mm → m by default).
    Returns (N, 3, 3) face array, or None on failure.
    Subsamples to max_faces triangles if the mesh is too dense for fast rendering.
    """
    try:
        from stl import mesh as stl_mod
        m = stl_mod.Mesh.from_file(str(path))
        faces = m.vectors.copy().astype(float) * scale
        faces -= faces.reshape(-1, 3).mean(axis=0)   # centre
        if len(faces) > max_faces:
            idx = np.linspace(0, len(faces) - 1, max_faces, dtype=int)
            faces = faces[idx]
        return faces
    except Exception as e:
        log.warning("STL load failed (%s): %s", path.name, e)
        return None


def _transform_faces(faces: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply 4×4 homogeneous transform T to (N, 3, 3) mesh faces."""
    pts = faces.reshape(-1, 3)
    pts_h = np.hstack([pts, np.ones((len(pts), 1))])
    pts_t = (T @ pts_h.T).T[:, :3]
    return pts_t.reshape(faces.shape)


# ─── pipeline ────────────────────────────────────────────────────────────────

def run_pipeline(gcode_path: Path, dh_table):
    """Full gcode → waypoints → IK → interpolated trajectory pipeline."""
    parser = GCodeParser(scale_factor=0.001)
    wps = parser.parse_file(str(gcode_path))
    wps = filter_travel_moves(wps)
    wps = downsample_waypoints(wps, min_distance=0.001)
    wps = transform_to_arm_frame(wps, origin_offset=(0.22, -0.015, 0.22))
    log.info("Waypoints: %d", len(wps))

    joint_traj = waypoints_to_joint_trajectory(wps, dh_table=dh_table,
                                                q_init=np.zeros(6))
    log.info("IK solved: %d / %d", len(joint_traj), len(wps))

    if len(joint_traj) >= 2:
        traj = interpolate_trajectory(joint_traj, v_max=1.5, a_max=3.0, dt=0.02)
    else:
        traj = np.array(joint_traj)
    log.info("Trajectory samples: %d", len(traj))

    wp_pts = np.array([[w.x, w.y, w.z] for w in wps])
    return traj, wp_pts


# ─── arm renderer (manages its own artist list) ───────────────────────────────

class ArmRenderer:
    def __init__(self, ax, extruder_faces):
        self.ax = ax
        self.extruder_faces = extruder_faces
        self._artists: list = []

    def clear(self):
        for a in self._artists:
            try:
                a.remove()
            except Exception:
                pass
        self._artists.clear()

    def draw(self, joint_pos: np.ndarray, T_ee: np.ndarray):
        """
        Draw the arm for one frame.

        Parameters
        ----------
        joint_pos : (7, 3) array of joint origins (base + 6 joints)
        T_ee      : (4, 4) end-effector homogeneous transform
        """
        self.clear()

        # ── links (cylinders) ──────────────────────────────────────────────
        for i in range(6):
            faces = _cylinder_faces(joint_pos[i], joint_pos[i + 1],
                                    LINK_RADII[i], n=14)
            if len(faces):
                c = _add_collection(self.ax, faces, LINK_COLS[i], alpha=0.93)
                self._artists.append(c)

        # ── joints (spheres) ───────────────────────────────────────────────
        for i, pos in enumerate(joint_pos):
            r = JOINT_R * 1.3 if i == 0 else JOINT_R
            col = JOINT_COL if i > 0 else "#94a3b8"
            faces = _sphere_faces(pos, r)
            if len(faces):
                c = _add_collection(self.ax, faces, col, alpha=0.95)
                self._artists.append(c)

        # ── extruder STL at end-effector ───────────────────────────────────
        if self.extruder_faces is not None:
            # Mount extruder 30 mm back along the tool Z axis
            T_ext = T_ee.copy()
            T_ext[:3, 3] += T_ee[:3, 2] * (-0.030)
            faces_t = _transform_faces(self.extruder_faces, T_ext)
            c = _add_collection(self.ax, faces_t, EE_COL, alpha=0.88)
            self._artists.append(c)
        else:
            # Fallback: bright sphere at EE
            faces = _sphere_faces(T_ee[:3, 3], 0.018)
            c = _add_collection(self.ax, faces, EE_COL, alpha=0.95)
            self._artists.append(c)


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="AR2 arm trajectory slider viewer")
    ap.add_argument("gcode", nargs="?",
                    default=str(PROJECT_ROOT / "examples" / "sample_sphere.gcode"))
    ap.add_argument("--frames", type=int, default=400,
                    help="Slider resolution (frames). Default: 400")
    args = ap.parse_args()

    gcode_path = Path(args.gcode)
    if not gcode_path.exists():
        print(f"Error: {gcode_path} not found"); sys.exit(1)

    dh_table = get_dh_table()

    # ── run pipeline ─────────────────────────────────────────────────────────
    traj, wp_pts = run_pipeline(gcode_path, dh_table)

    # Subsample to n_frames evenly-spaced frames
    n_frames = min(args.frames, len(traj))
    idx = np.linspace(0, len(traj) - 1, n_frames, dtype=int)
    traj_sub = traj[idx, :6]

    # Pre-compute FK for all slider frames
    log.info("Pre-computing FK for %d frames …", n_frames)
    all_joint_pos = np.zeros((n_frames, 7, 3))
    all_T_ee = []
    for fi, q in enumerate(traj_sub):
        transforms = get_all_transforms(q, dh_table)
        all_joint_pos[fi] = [T[:3, 3] for T in transforms]
        all_T_ee.append(transforms[-1])
    log.info("FK ready.")

    # ── load extruder STL ────────────────────────────────────────────────────
    extruder_path = (PROJECT_ROOT / "CAD" / "STL" / "extruder"
                     / "housing_core_x1_rev16.STL")
    extruder_faces = _load_stl(extruder_path)
    if extruder_faces is not None:
        log.info("Extruder STL loaded: %d faces", len(extruder_faces))
    else:
        log.warning("Extruder STL not found — using sphere fallback at EE.")

    # ── figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9), facecolor=BG)
    fig.patch.set_facecolor(BG)

    # 3-D axes (leaves room at bottom for slider + info strip)
    ax = fig.add_axes([0.01, 0.14, 0.98, 0.84], projection="3d")
    ax.set_facecolor(BG)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor(GRID_COL)
    ax.grid(True, color=GRID_COL, linewidth=0.5)
    ax.tick_params(colors="#1e293b", labelsize=7)
    ax.set_xlabel("X (m)", color="#334155", labelpad=3)
    ax.set_ylabel("Y (m)", color="#334155", labelpad=3)
    ax.set_zlabel("Z (m)", color="#334155", labelpad=3)
    ax.set_title(f"AR2 Arm — {gcode_path.stem}", color="#e2e8f0",
                 fontsize=12, fontweight="bold", pad=6)

    # Axis limits: cover toolpath + arm reach
    all_pts = np.vstack([wp_pts, all_joint_pos.reshape(-1, 3)])
    centre = all_pts.mean(axis=0)
    span = max(np.ptp(all_pts, axis=0).max() * 0.62, 0.45)
    ax.set_xlim(centre[0] - span, centre[0] + span)
    ax.set_ylim(centre[1] - span, centre[1] + span)
    ax.set_zlim(max(0., centre[2] - span), centre[2] + span)

    # Static: full toolpath (faint red)
    ax.plot(wp_pts[:, 0], wp_pts[:, 1], wp_pts[:, 2],
            color=PATH_COL, linewidth=0.9, alpha=0.35, zorder=0, label="Toolpath")

    # Dynamic: completed EE trace (blue) + current EE dot (green)
    trace_line, = ax.plot([], [], [], color=TRACE_COL, lw=1.6,
                          alpha=0.85, zorder=3, label="EE trace")
    ee_dot, = ax.plot([], [], [], "o", color=EE_COL, ms=9, zorder=6)

    ax.legend(loc="upper right", fontsize=8,
              facecolor="#0f172a", edgecolor="#1e293b", labelcolor="#94a3b8")

    # ── slider ────────────────────────────────────────────────────────────────
    ax_slider = fig.add_axes([0.10, 0.055, 0.80, 0.028], facecolor="#0f172a")
    slider = mwidgets.Slider(
        ax_slider, "◀ Frame ▶", 0, n_frames - 1,
        valinit=0, valstep=1, color="#1d4ed8",
    )
    slider.label.set_color("#94a3b8")
    slider.valtext.set_color("#64748b")
    slider.label.set_fontsize(9)

    # ── info text overlay ─────────────────────────────────────────────────────
    info = ax.text2D(0.01, 0.98, "", transform=ax.transAxes, fontsize=8,
                     color="#64748b", va="top", fontfamily="monospace",
                     bbox=dict(boxstyle="round,pad=0.3", fc="#020617",
                               ec="#1e293b", alpha=0.85))

    # ── arm renderer ──────────────────────────────────────────────────────────
    renderer = ArmRenderer(ax, extruder_faces)

    # ── update callback ───────────────────────────────────────────────────────
    def update(val):
        fi = int(slider.val)

        # Redraw arm
        renderer.draw(all_joint_pos[fi], all_T_ee[fi])

        # EE trace up to current frame
        trace = all_joint_pos[:fi + 1, -1, :]
        trace_line.set_data(trace[:, 0], trace[:, 1])
        trace_line.set_3d_properties(trace[:, 2])

        # Current EE position
        ee = all_joint_pos[fi, -1]
        ee_dot.set_data([ee[0]], [ee[1]])
        ee_dot.set_3d_properties([ee[2]])

        # Joint angle info
        q_deg = np.degrees(traj_sub[fi])
        t_s = idx[fi] * 0.02
        info.set_text(
            f" t = {t_s:6.1f} s    frame {fi+1:>4}/{n_frames}\n"
            f" J1={q_deg[0]:+6.1f}°  J2={q_deg[1]:+6.1f}°  J3={q_deg[2]:+6.1f}°\n"
            f" J4={q_deg[3]:+6.1f}°  J5={q_deg[4]:+6.1f}°  J6={q_deg[5]:+6.1f}° "
        )

        fig.canvas.draw_idle()

    slider.on_changed(update)

    # ── keyboard: left/right arrow to step one frame ──────────────────────────
    def on_key(event):
        fi = int(slider.val)
        if event.key == "right":
            slider.set_val(min(fi + 1, n_frames - 1))
        elif event.key == "left":
            slider.set_val(max(fi - 1, 0))
        elif event.key == "right" or event.key == "up":
            slider.set_val(min(fi + 10, n_frames - 1))
        elif event.key == "down":
            slider.set_val(max(fi - 10, 0))

    fig.canvas.mpl_connect("key_press_event", on_key)

    # Draw initial frame
    update(0)

    print("\n  Drag the slider or use ← → arrow keys to scrub through the trajectory.\n")
    plt.show()


if __name__ == "__main__":
    main()

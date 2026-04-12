#!/usr/bin/env python3
"""
visualize_slider.py — Interactive AR2 arm + toolpath viewer with collision checker.

Features
--------
- Scrub through trajectory frame-by-frame with the slider or ← → keys
- Collision timeline strip: green = safe, yellow = near-floor, red = self-collision
- Colliding links flash red in the 3D view
- AR2 arm geometry: cylindrical links, joint markers, extruder body + nozzle tip
- Plain-colour EE trace (no Z-height gimmicks)

Usage
-----
    python visualize_slider.py
    python visualize_slider.py examples/sample_cube.gcode
    python visualize_slider.py examples/sample_sphere.gcode --frames 500
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
from kinematics.collision import check_capsule_collision
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

# ─── theme ────────────────────────────────────────────────────────────────────
BG        = "#020617"
GRID_COL  = "#0f172a"
# Per-link colours (base → wrist, blue gradient)
LINK_COLS  = ["#1e3a5f", "#1d4ed8", "#2563eb", "#3b82f6", "#60a5fa", "#93c5fd"]
LINK_COL_COLL = "#ef4444"   # colour when link is in collision
JOINT_COL  = "#f97316"
BASE_COL   = "#475569"
EE_COL     = "#22c55e"      # extruder body
NOZZLE_COL = "#fbbf24"      # nozzle tip
TRACE_COL  = "#38bdf8"      # EE trace — single clean colour
PATH_COL   = "#334155"      # static background toolpath

# ─── physical radii ───────────────────────────────────────────────────────────
# Visual render radii (m)
LINK_RADII = [0.038, 0.026, 0.020, 0.016, 0.013, 0.011]
# Collision capsule radii — slightly smaller than visual to avoid false positives
COLL_RADII = [0.022, 0.016, 0.014, 0.012, 0.010, 0.009]
FLOOR_CLEARANCE = 0.018   # warn if any joint tip drops below this (m)


# ─── geometry helpers ─────────────────────────────────────────────────────────

def _rot_z_to(v_hat: np.ndarray) -> np.ndarray:
    """Minimal rotation from [0,0,1] onto unit vector v_hat."""
    z = np.array([0., 0., 1.])
    d = float(np.dot(z, v_hat))
    if d > 0.9999:  return np.eye(3)
    if d < -0.9999: return np.diag([1., -1., -1.])
    ax = np.cross(z, v_hat); ax /= np.linalg.norm(ax)
    a = np.arccos(np.clip(d, -1., 1.))
    K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    return np.eye(3) + np.sin(a)*K + (1-np.cos(a))*(K@K)


def _cylinder(p1, p2, r: float, n: int = 12) -> np.ndarray:
    """(n,4,3) quad-face array for a capped cylinder from p1 to p2."""
    v = np.asarray(p2, float) - np.asarray(p1, float)
    L = np.linalg.norm(v)
    if L < 1e-5: return np.empty((0,4,3))
    R = _rot_z_to(v / L)
    t = np.linspace(0, 2*np.pi, n, endpoint=False)
    ring = np.column_stack([r*np.cos(t), r*np.sin(t), np.zeros(n)])
    bot = (R @ ring.T).T + p1
    top = (R @ (ring + [0,0,L]).T).T + p1
    nxt, nxt_t = np.roll(bot,-1,0), np.roll(top,-1,0)
    return np.stack([bot, nxt, nxt_t, top], axis=1)


def _add_poly(ax, faces, color, alpha=0.90):
    """Add a Poly3DCollection and return the artist."""
    col = Poly3DCollection(faces, alpha=alpha)
    col.set_facecolor(color); col.set_edgecolor("none")
    ax.add_collection3d(col)
    return col


# ─── STL loader ───────────────────────────────────────────────────────────────

def _load_stl(path: Path, scale: float = 0.001, max_faces: int = 700) -> np.ndarray | None:
    try:
        from stl import mesh as sm
        m = sm.Mesh.from_file(str(path))
        f = m.vectors.copy().astype(float) * scale
        f -= f.reshape(-1,3).mean(axis=0)
        if len(f) > max_faces:
            f = f[np.linspace(0, len(f)-1, max_faces, dtype=int)]
        return f
    except Exception as e:
        log.warning("STL load failed (%s): %s", path.name, e); return None


def _apply_T(faces: np.ndarray, T: np.ndarray) -> np.ndarray:
    pts = faces.reshape(-1,3)
    pts_h = np.hstack([pts, np.ones((len(pts),1))])
    return ((T @ pts_h.T).T[:,:3]).reshape(faces.shape)


# ─── collision check (per frame) ─────────────────────────────────────────────

def _check_frame(joint_pos: np.ndarray) -> tuple[set, list]:
    """
    Returns (colliding_link_indices, warning_messages).
    colliding_link_indices: set of link numbers (0-5) involved in a collision.
    """
    colliding = set()
    msgs = []

    # Floor clearance
    for i in range(1, 7):
        if joint_pos[i, 2] < FLOOR_CLEARANCE:
            colliding.add(i - 1)
            msgs.append(f"Link {i} near floor (Z={joint_pos[i,2]*1000:.0f} mm)")

    # Self-collision (skip adjacent links — they always share a joint)
    for i in range(6):
        for j in range(i + 2, 6):
            if check_capsule_collision(
                joint_pos[i], joint_pos[i+1], COLL_RADII[i],
                joint_pos[j], joint_pos[j+1], COLL_RADII[j],
            ):
                colliding.add(i); colliding.add(j)
                msgs.append(f"L{i}↔L{j}")

    return colliding, msgs


# ─── arm renderer ─────────────────────────────────────────────────────────────

class ArmRenderer:
    def __init__(self, ax, extruder_faces):
        self.ax = ax
        self.extruder_faces = extruder_faces
        self._artists: list = []

    def clear(self):
        for a in self._artists:
            try: a.remove()
            except Exception: pass
        self._artists.clear()

    def draw(self, joint_pos: np.ndarray, T_ee: np.ndarray,
             colliding: set | None = None):
        """
        joint_pos : (7, 3) — base origin + 6 joint origins
        T_ee      : (4, 4) — end-effector transform
        colliding : set of link indices (0-5) that are in collision
        """
        self.clear()
        if colliding is None:
            colliding = set()

        # ── links ─────────────────────────────────────────────────────────
        for i in range(6):
            col   = LINK_COL_COLL if i in colliding else LINK_COLS[i]
            alpha = 0.97           if i in colliding else 0.88
            faces = _cylinder(joint_pos[i], joint_pos[i+1], LINK_RADII[i])
            if len(faces):
                self._artists.append(_add_poly(self.ax, faces, col, alpha))

        # ── joint markers (scatter — fast, no sphere mesh overhead) ───────
        # Base joint (grey, bigger)
        s0 = self.ax.scatter(*joint_pos[0], s=90, c=BASE_COL,
                              zorder=8, depthshade=False, marker='o')
        self._artists.append(s0)
        # J1–J6
        jp = joint_pos[1:]
        jc = [LINK_COL_COLL if (i in colliding) else JOINT_COL
              for i in range(6)]
        s1 = self.ax.scatter(jp[:,0], jp[:,1], jp[:,2],
                              s=55, c=jc, zorder=9, depthshade=False, marker='o')
        self._artists.append(s1)

        # ── extruder body (STL or fallback box) ───────────────────────────
        if self.extruder_faces is not None:
            T_ext = T_ee.copy()
            T_ext[:3, 3] += T_ee[:3, 2] * (-0.030)   # 30 mm back along tool Z
            self._artists.append(
                _add_poly(self.ax, _apply_T(self.extruder_faces, T_ext),
                          EE_COL, alpha=0.85))
        else:
            # Fallback: small cylinder representing the extruder body
            body_base = T_ee[:3, 3] - T_ee[:3, 2] * 0.050
            faces = _cylinder(body_base, T_ee[:3, 3], 0.016, n=8)
            if len(faces):
                self._artists.append(_add_poly(self.ax, faces, EE_COL, alpha=0.85))

        # ── nozzle tip ─────────────────────────────────────────────────────
        # Small bright cylinder: tool tip → 25 mm along tool Z
        nozzle_root = T_ee[:3, 3].copy()
        nozzle_tip  = nozzle_root + T_ee[:3, 2] * 0.025
        faces = _cylinder(nozzle_root, nozzle_tip, 0.004, n=8)
        if len(faces):
            self._artists.append(_add_poly(self.ax, faces, NOZZLE_COL, alpha=0.97))
        # Tip dot
        s2 = self.ax.scatter(*nozzle_tip, s=30, c=NOZZLE_COL,
                              zorder=12, depthshade=False)
        self._artists.append(s2)


# ─── pipeline ─────────────────────────────────────────────────────────────────

def run_pipeline(gcode_path: Path, dh_table):
    parser = GCodeParser(scale_factor=0.001)
    wps = parser.parse_file(str(gcode_path))
    wps = filter_travel_moves(wps)
    wps = downsample_waypoints(wps, min_distance=0.001)
    wps = transform_to_arm_frame(wps, origin_offset=(0.22, -0.015, 0.22))
    log.info("Waypoints: %d", len(wps))
    jt = waypoints_to_joint_trajectory(wps, dh_table=dh_table, q_init=np.zeros(6))
    log.info("IK: %d / %d solved", len(jt), len(wps))
    traj = interpolate_trajectory(jt, v_max=1.5, a_max=3.0, dt=0.02) if len(jt) >= 2 else np.array(jt)
    log.info("Trajectory: %d samples", len(traj))
    wp_pts = np.array([[w.x, w.y, w.z] for w in wps])
    return traj, wp_pts


# ─── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="AR2 arm trajectory + collision viewer")
    ap.add_argument("gcode", nargs="?",
                    default=str(PROJECT_ROOT / "examples" / "sample_sphere.gcode"))
    ap.add_argument("--frames", type=int, default=400)
    args = ap.parse_args()

    gcode_path = Path(args.gcode)
    if not gcode_path.exists():
        print(f"Error: {gcode_path} not found"); sys.exit(1)

    dh_table = get_dh_table()

    # ── pipeline ──────────────────────────────────────────────────────────────
    traj, wp_pts = run_pipeline(gcode_path, dh_table)
    n_frames = min(args.frames, len(traj))
    idx = np.linspace(0, len(traj)-1, n_frames, dtype=int)
    traj_sub = traj[idx, :6]

    # ── pre-compute FK ────────────────────────────────────────────────────────
    log.info("Pre-computing FK for %d frames …", n_frames)
    all_joint_pos = np.zeros((n_frames, 7, 3))
    all_T_ee: list = []
    for fi, q in enumerate(traj_sub):
        transforms = get_all_transforms(q, dh_table)
        all_joint_pos[fi] = [T[:3,3] for T in transforms]
        all_T_ee.append(transforms[-1])

    # ── pre-compute collisions ────────────────────────────────────────────────
    log.info("Checking collisions …")
    coll_sets: list[set]  = []
    coll_msgs: list[list] = []
    for fi in range(n_frames):
        s, m = _check_frame(all_joint_pos[fi])
        coll_sets.append(s); coll_msgs.append(m)

    n_coll  = sum(1 for s in coll_sets if s)
    pct     = 100 * n_coll / n_frames
    log.info("Collision summary: %d / %d frames flagged (%.1f%%)", n_coll, n_frames, pct)

    # Collision strip image: shape (1, n_frames, 3)
    strip = np.zeros((1, n_frames, 3))
    for fi in range(n_frames):
        if coll_sets[fi]:
            strip[0, fi] = [0.94, 0.27, 0.27]   # red  — self-collision
        else:
            strip[0, fi] = [0.13, 0.77, 0.37]   # green — safe

    # ── load extruder STL ─────────────────────────────────────────────────────
    ext_path = PROJECT_ROOT / "CAD" / "STL" / "extruder" / "housing_core_x1_rev16.STL"
    extruder_faces = _load_stl(ext_path)
    if extruder_faces is not None:
        log.info("Extruder STL loaded: %d faces", len(extruder_faces))
    else:
        log.warning("Extruder STL missing — using cylinder fallback.")

    # ── figure layout ─────────────────────────────────────────────────────────
    #   [3D ax   ]  0.20 → 1.00 (80% height)
    #   [coll strip]  0.13 → 0.18  (5%)
    #   [slider   ]  0.05 → 0.10  (5%)
    fig = plt.figure(figsize=(14, 9.5), facecolor=BG)
    fig.patch.set_facecolor(BG)

    ax = fig.add_axes([0.02, 0.19, 0.96, 0.79], projection="3d")
    ax.set_facecolor(BG)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False; pane.set_edgecolor(GRID_COL)
    ax.grid(True, color=GRID_COL, linewidth=0.4)
    ax.tick_params(colors="#1e293b", labelsize=7)
    ax.set_xlabel("X (m)", color="#1e293b", labelpad=3)
    ax.set_ylabel("Y (m)", color="#1e293b", labelpad=3)
    ax.set_zlabel("Z (m)", color="#1e293b", labelpad=3)
    ax.set_title(f"AR2  ·  {gcode_path.stem}  ·  {n_frames} frames",
                 color="#e2e8f0", fontsize=11, fontweight="bold", pad=5)

    # Axis limits
    all_pts = np.vstack([wp_pts, all_joint_pos.reshape(-1,3)])
    cen = all_pts.mean(axis=0)
    span = max(np.ptp(all_pts, axis=0).max() * 0.62, 0.45)
    ax.set_xlim(cen[0]-span, cen[0]+span)
    ax.set_ylim(cen[1]-span, cen[1]+span)
    ax.set_zlim(max(0., cen[2]-span), cen[2]+span)

    # Static toolpath background (dark, unobtrusive)
    ax.plot(wp_pts[:,0], wp_pts[:,1], wp_pts[:,2],
            color=PATH_COL, lw=0.8, alpha=0.5, zorder=0)

    # Dynamic EE trace (single bright colour — no Z-map)
    trace_line, = ax.plot([], [], [], color=TRACE_COL, lw=1.5,
                          alpha=0.9, zorder=3, label="EE trace")
    ee_dot, = ax.plot([], [], [], "o", color=NOZZLE_COL, ms=7, zorder=7)

    ax.legend(loc="upper right", fontsize=7,
              facecolor="#0f172a", edgecolor="#1e293b", labelcolor="#94a3b8")

    # ── info overlay ──────────────────────────────────────────────────────────
    info_text = ax.text2D(
        0.01, 0.99, "", transform=ax.transAxes,
        fontsize=7.5, color="#94a3b8", va="top", fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.35", fc="#020617", ec="#1e293b", alpha=0.88)
    )
    coll_text = ax.text2D(
        0.99, 0.99, "", transform=ax.transAxes,
        fontsize=8, va="top", ha="right", fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.35", fc="#020617", ec="#1e293b", alpha=0.88)
    )

    # ── collision strip ───────────────────────────────────────────────────────
    ax_coll = fig.add_axes([0.10, 0.135, 0.80, 0.032], facecolor=BG)
    ax_coll.imshow(strip, aspect="auto", extent=[0, n_frames, 0, 1],
                   interpolation="nearest", origin="upper")
    ax_coll.set_xlim(0, n_frames)
    ax_coll.set_yticks([])
    ax_coll.tick_params(colors="#334155", labelsize=6)
    ax_coll.set_xlabel("Frame →", color="#334155", fontsize=7, labelpad=2)
    ax_coll.text(-0.01, 0.5, "Collision\nCheck", transform=ax_coll.transAxes,
                 fontsize=6, color="#475569", va="center", ha="right",
                 fontfamily="monospace")
    # Summary label
    safe_pct = 100 - pct
    ax_coll.text(1.01, 0.5, f"{safe_pct:.0f}% safe",
                 transform=ax_coll.transAxes, fontsize=6,
                 color="#22c55e" if safe_pct > 90 else "#fbbf24",
                 va="center", fontfamily="monospace")
    # Vertical cursor line
    coll_cursor = ax_coll.axvline(0, color="white", lw=1.2, alpha=0.85)
    for spine in ax_coll.spines.values():
        spine.set_edgecolor("#1e293b")

    # ── slider ────────────────────────────────────────────────────────────────
    ax_sl = fig.add_axes([0.10, 0.05, 0.80, 0.030], facecolor="#0f172a")
    slider = mwidgets.Slider(ax_sl, "◀ Frame ▶", 0, n_frames-1,
                              valinit=0, valstep=1, color="#1d4ed8")
    slider.label.set_color("#475569"); slider.label.set_fontsize(8)
    slider.valtext.set_color("#334155")
    for spine in ax_sl.spines.values():
        spine.set_edgecolor("#1e293b")

    # ── arm renderer ──────────────────────────────────────────────────────────
    renderer = ArmRenderer(ax, extruder_faces)

    # ── update ────────────────────────────────────────────────────────────────
    def update(val):
        fi = int(slider.val)
        colliding = coll_sets[fi]
        msgs      = coll_msgs[fi]

        # Arm
        renderer.draw(all_joint_pos[fi], all_T_ee[fi], colliding)

        # EE trace
        trace = all_joint_pos[:fi+1, -1, :]
        trace_line.set_data(trace[:,0], trace[:,1])
        trace_line.set_3d_properties(trace[:,2])

        # Nozzle dot follows nozzle tip (along tool Z)
        T_ee = all_T_ee[fi]
        noz = T_ee[:3,3] + T_ee[:3,2] * 0.025
        ee_dot.set_data([noz[0]], [noz[1]])
        ee_dot.set_3d_properties([noz[2]])

        # Collision strip cursor
        coll_cursor.set_xdata([fi, fi])

        # Joint angle info
        q_deg = np.degrees(traj_sub[fi])
        t_s   = idx[fi] * 0.02
        info_text.set_text(
            f" t = {t_s:6.1f} s   [{fi+1}/{n_frames}]\n"
            f" J1={q_deg[0]:+6.1f}°  J4={q_deg[3]:+6.1f}°\n"
            f" J2={q_deg[1]:+6.1f}°  J5={q_deg[4]:+6.1f}°\n"
            f" J3={q_deg[2]:+6.1f}°  J6={q_deg[5]:+6.1f}°"
        )

        # Collision status badge
        if colliding:
            coll_text.set_text(" ⚠ COLLISION\n " + "\n ".join(msgs[:3]))
            coll_text.set_color("#ef4444")
            coll_text.get_bbox_patch().set_edgecolor("#7f1d1d")
        else:
            coll_text.set_text(" ✓ CLEAR")
            coll_text.set_color("#22c55e")
            coll_text.get_bbox_patch().set_edgecolor("#14532d")

        fig.canvas.draw_idle()

    slider.on_changed(update)

    # ── keyboard ──────────────────────────────────────────────────────────────
    def on_key(evt):
        fi = int(slider.val)
        if   evt.key == "right": slider.set_val(min(fi+1,  n_frames-1))
        elif evt.key == "left":  slider.set_val(max(fi-1,  0))
        elif evt.key == "up":    slider.set_val(min(fi+10, n_frames-1))
        elif evt.key == "down":  slider.set_val(max(fi-10, 0))
    fig.canvas.mpl_connect("key_press_event", on_key)

    update(0)
    print(f"\n  ← → step 1 frame   ↑ ↓ step 10 frames\n"
          f"  Collision: {n_coll}/{n_frames} frames flagged ({pct:.1f}%)\n")
    plt.show()


if __name__ == "__main__":
    main()

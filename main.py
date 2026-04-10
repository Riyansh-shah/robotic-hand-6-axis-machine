#!/usr/bin/env python3
"""
main.py — Single entry-point that demonstrates the full 6-axis robotic arm
3D-printing pipeline:

    G-code  →  Parse  →  Inverse Kinematics  →  Trajectory Planning
            →  Mock Controller  →  3-D Visualisation

Usage
-----
    python main.py                         # default: examples/sample_cube.gcode
    python main.py path/to/custom.gcode    # use a custom G-code file

Requirements
------------
    pip install -r requirements.txt
"""

from __future__ import annotations

import os
import sys
import logging
import argparse
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Ensure the project root is on sys.path so all modules are importable.
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from kinematics.dh_params import DH6R
from kinematics.forward_kinematics import forward_kinematics
from kinematics.inverse_kinematics import ik_numerical, IKError
from kinematics.visualize import plot_arm

from gcode_parser import GCodeParser
from gcode_parser.filters import (
    filter_travel_moves,
    downsample_waypoints,
    apply_z_offset,
)

from trajectory_planner.cartesian_to_joint import (
    waypoints_to_joint_trajectory,
    build_target_pose,
)
from trajectory_planner.interpolation import (
    interpolate_trajectory,
    linear_interpolation,
)
from trajectory_planner.velocity_limits import (
    check_velocity_limits,
    DEFAULT_JOINT_VELOCITY_LIMITS,
)

from control import MockController

from simulation.arm_simulator import ArmSimulator
from simulation.toolpath_visualizer import visualize_toolpath

from calibration.accuracy_measurement import compute_accuracy

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")


# ---------------------------------------------------------------------------
# Pipeline steps
# ---------------------------------------------------------------------------

def step_parse_gcode(gcode_path: Path) -> list:
    """Parse a G-code file and return filtered Cartesian waypoints."""
    log.info("STEP 1 — Parsing G-code: %s", gcode_path)
    parser = GCodeParser(scale_factor=0.001)           # mm → m
    waypoints = parser.parse_file(str(gcode_path))
    meta = parser.get_metadata()
    log.info("  Raw waypoints : %d", len(waypoints))
    log.info("  Metadata      : %s", meta)

    # Filter: keep only extrusion moves, downsample dense points
    waypoints = filter_travel_moves(waypoints)
    waypoints = downsample_waypoints(waypoints, min_distance=0.001)
    # Shift Z up so the toolpath sits within the arm's workspace
    waypoints = apply_z_offset(waypoints, offset=0.15)

    log.info("  Filtered waypoints: %d", len(waypoints))
    if waypoints:
        xs = [w.x for w in waypoints]
        ys = [w.y for w in waypoints]
        zs = [w.z for w in waypoints]
        log.info(
            "  Bounding box  : X [%.3f, %.3f]  Y [%.3f, %.3f]  Z [%.3f, %.3f] m",
            min(xs), max(xs), min(ys), max(ys), min(zs), max(zs),
        )
    return waypoints


def step_inverse_kinematics(waypoints: list, dh_table: np.ndarray) -> list:
    """Convert Cartesian waypoints to joint-angle configurations via IK."""
    log.info("STEP 2 — Inverse Kinematics (%d waypoints)", len(waypoints))
    joint_traj = waypoints_to_joint_trajectory(
        waypoints, dh_table=dh_table, q_init=np.zeros(6)
    )
    log.info("  Solved configurations: %d / %d", len(joint_traj), len(waypoints))
    return joint_traj


def step_trajectory_planning(joint_waypoints: list) -> np.ndarray:
    """Interpolate joint waypoints into a smooth, time-parameterised trajectory."""
    log.info("STEP 3 — Trajectory Planning")
    if len(joint_waypoints) < 2:
        log.warning("  Not enough waypoints for interpolation — using raw data.")
        return np.array(joint_waypoints)

    trajectory = interpolate_trajectory(
        joint_waypoints, v_max=1.5, a_max=3.0, dt=0.02
    )
    log.info("  Trajectory samples: %d  (dt = 0.02 s)", len(trajectory))

    within = check_velocity_limits(
        trajectory, dt=0.02, v_limits=DEFAULT_JOINT_VELOCITY_LIMITS
    )
    log.info("  Velocity limits respected: %s", within)
    return trajectory


def step_mock_control(trajectory: np.ndarray) -> None:
    """Send the trajectory to the mock serial controller."""
    log.info("STEP 4 — Mock Controller Execution")
    ctrl = MockController(port="/dev/ttyUSB0", baudrate=115200)
    ctrl.connect()
    # Send a small slice so the demo doesn't take forever
    n = min(len(trajectory), 20)
    ctrl.execute_trajectory(trajectory[:n], dt=0.02)
    log.info("  Sent %d commands to mock controller.", n)
    ctrl.disconnect()


def step_visualise(
    waypoints: list,
    trajectory: np.ndarray,
    dh_table: np.ndarray,
) -> None:
    """Plot the toolpath and the arm at the first/last configuration."""
    log.info("STEP 5 — Visualisation")
    try:
        import matplotlib

        matplotlib.use("Agg")          # non-interactive backend for CI
        import matplotlib.pyplot as plt

        # --- Toolpath 3D scatter ---
        fig1, ax1 = plt.subplots(
            subplot_kw={"projection": "3d"}, figsize=(7, 6), facecolor="#020617"
        )
        ax1.set_facecolor("#020617")
        xs = [w.x for w in waypoints]
        ys = [w.y for w in waypoints]
        zs = [w.z for w in waypoints]
        ax1.scatter(xs, ys, zs, c=zs, cmap="plasma", s=4)
        ax1.set_title("Parsed Toolpath", color="#e2e8f0", fontsize=11)
        ax1.tick_params(colors="#94a3b8", labelsize=7)
        out_dir = PROJECT_ROOT / "examples" / "output"
        out_dir.mkdir(exist_ok=True)
        fig1.savefig(out_dir / "toolpath.png", dpi=150, bbox_inches="tight")
        log.info("  Saved toolpath plot → examples/output/toolpath.png")

        # --- Arm at first & last configs ---
        if len(trajectory) >= 1:
            fig2, (ax2a, ax2b) = plt.subplots(
                1, 2, subplot_kw={"projection": "3d"},
                figsize=(13, 6), facecolor="#020617",
            )
            plot_arm(trajectory[0], dh_table, ax=ax2a)
            ax2a.set_title("Start Pose", color="#e2e8f0", fontsize=10)
            plot_arm(trajectory[-1], dh_table, ax=ax2b)
            ax2b.set_title("End Pose", color="#e2e8f0", fontsize=10)
            fig2.savefig(out_dir / "arm_poses.png", dpi=150, bbox_inches="tight")
            log.info("  Saved arm poses  → examples/output/arm_poses.png")

        plt.close("all")
    except ImportError:
        log.warning("  matplotlib not available — skipping plots.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="6-Axis Robotic Arm 3D-Printing Pipeline Demo",
    )
    parser.add_argument(
        "gcode",
        nargs="?",
        default=str(PROJECT_ROOT / "examples" / "sample_cube.gcode"),
        help="Path to a .gcode file (default: examples/sample_cube.gcode)",
    )
    args = parser.parse_args()
    gcode_path = Path(args.gcode)
    if not gcode_path.exists():
        log.error("G-code file not found: %s", gcode_path)
        sys.exit(1)

    arm = DH6R.default()
    dh_table = arm.table

    print()
    print("=" * 62)
    print("  6-Axis Robotic Arm — Full Pipeline Demo")
    print("=" * 62)
    print()

    waypoints      = step_parse_gcode(gcode_path)
    if not waypoints:
        log.error("No usable waypoints after filtering. Exiting.")
        sys.exit(1)

    joint_traj     = step_inverse_kinematics(waypoints, dh_table)
    trajectory     = step_trajectory_planning(joint_traj)
    step_mock_control(trajectory)
    step_visualise(waypoints, trajectory, dh_table)

    print()
    print("-" * 62)
    print("  Pipeline complete.")
    print("-" * 62)
    print()


if __name__ == "__main__":
    main()

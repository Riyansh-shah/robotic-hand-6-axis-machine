#!/usr/bin/env python3
"""
Full pipeline demo: G-code → Parse → IK → Trajectory → Visualization

Demonstrates the complete workflow from a G-code file through parsing,
inverse kinematics, trajectory planning, and arm visualization.

Usage:
    python examples/demo_pipeline.py

This script:
  1. Parses sample_cube.gcode to extract Cartesian waypoints
  2. Prints parsing summary (waypoints, bounding box)
  3. Converts waypoints to joint trajectory using IK
  4. Interpolates with trapezoidal velocity profile
  5. Executes on MockController
  6. Visualizes the arm at first and last waypoints
  7. Saves output plot to examples/output/pipeline_demo.png
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add parent directory to path so we can import from the project modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gcode_parser import GCodeParser
from trajectory_planner import (
    waypoints_to_joint_trajectory,
    trapezoidal_velocity_profile,
    interpolate_trajectory,
)
from control import MockController
from kinematics import plot_arm, forward_kinematics, DH6R
from simulation import ArmSimulator


def main():
    """Run the full pipeline demo."""

    # ===== 1. Parse G-code =====
    print("=" * 60)
    print("STEP 1: Parsing G-code")
    print("=" * 60)

    gcode_file = os.path.join(os.path.dirname(__file__), "sample_cube.gcode")
    if not os.path.exists(gcode_file):
        print(f"Error: {gcode_file} not found")
        return

    parser = GCodeParser()
    parser.parse_file(gcode_file)
    waypoints = parser.get_waypoints()

    print(f"Parsed {gcode_file}")
    print(f"Total waypoints: {len(waypoints)}")

    if waypoints:
        xs = [w.x for w in waypoints]
        ys = [w.y for w in waypoints]
        zs = [w.z for w in waypoints]
        es = [w.e for w in waypoints]

        print(f"X range: {min(xs):.2f} to {max(xs):.2f} mm")
        print(f"Y range: {min(ys):.2f} to {max(ys):.2f} mm")
        print(f"Z range: {min(zs):.2f} to {max(zs):.2f} mm")
        print(f"Extrusion: {min(es):.2f} to {max(es):.2f} mm")
        print(f"Sample waypoints (first 5):")
        for i, w in enumerate(waypoints[:5]):
            print(f"  {i}: ({w.x:.2f}, {w.y:.2f}, {w.z:.2f}) E={w.e:.2f}")
    else:
        print("Warning: No waypoints extracted from G-code")
        return

    # ===== 2. Convert to joint trajectory =====
    print("\n" + "=" * 60)
    print("STEP 2: Inverse Kinematics (Cartesian → Joint)")
    print("=" * 60)

    try:
        joint_waypoints = waypoints_to_joint_trajectory(waypoints)
        print(f"Successfully converted {len(joint_waypoints)} waypoints to joint space")
        print(f"Joint dimensions: {len(joint_waypoints[0])} DOF")
        print(f"Sample joint angles (first waypoint, degrees):")
        print(f"  {np.degrees(joint_waypoints[0]).round(2)}")
    except Exception as e:
        print(f"Error during IK conversion: {e}")
        return

    # ===== 3. Interpolate trajectory =====
    print("\n" + "=" * 60)
    print("STEP 3: Trajectory Interpolation & Velocity Profile")
    print("=" * 60)

    try:
        # Interpolate between waypoints
        interpolated = interpolate_trajectory(joint_waypoints, num_points=20)
        print(f"Interpolated to {len(interpolated)} points")

        # Apply trapezoidal velocity profile
        trajectory = trapezoidal_velocity_profile(
            waypoints=interpolated,
            max_joint_vel=1.0,  # rad/s
            max_joint_accel=0.5,  # rad/s^2
        )
        print(f"Generated trajectory with {len(trajectory)} segments")
        print(f"Trajectory duration: {trajectory[-1]['time']:.2f} seconds")
    except Exception as e:
        print(f"Error during trajectory planning: {e}")
        return

    # ===== 4. Execute on mock controller =====
    print("\n" + "=" * 60)
    print("STEP 4: Controller Execution (Mock)")
    print("=" * 60)

    try:
        controller = MockController()
        controller.connect()
        print("Connected to MockController")

        controller.execute_trajectory(trajectory)
        print(f"Executed trajectory on MockController")

        controller.disconnect()
        print("Disconnected from MockController")
    except Exception as e:
        print(f"Error during controller execution: {e}")
        return

    # ===== 5. Visualize arm at key poses =====
    print("\n" + "=" * 60)
    print("STEP 5: Arm Visualization")
    print("=" * 60)

    try:
        fig = plt.figure(figsize=(16, 6))

        # Plot 1: Arm at first waypoint
        ax1 = fig.add_subplot(131, projection='3d')
        first_joints = joint_waypoints[0]
        plot_arm(first_joints, ax=ax1, title="Arm at Start (First Waypoint)")

        # Plot 2: Arm at last waypoint
        ax2 = fig.add_subplot(132, projection='3d')
        last_joints = joint_waypoints[-1]
        plot_arm(last_joints, ax=ax2, title="Arm at End (Last Waypoint)")

        # Plot 3: Toolpath waypoints (Cartesian)
        ax3 = fig.add_subplot(133)
        xs = [w.x for w in waypoints]
        ys = [w.y for w in waypoints]
        ax3.scatter(xs, ys, alpha=0.6, s=30)
        ax3.plot(xs, ys, 'b-', alpha=0.3, linewidth=0.5)
        ax3.set_xlabel("X (mm)")
        ax3.set_ylabel("Y (mm)")
        ax3.set_title("Toolpath (Top View)")
        ax3.grid(True, alpha=0.3)
        ax3.axis('equal')

        # Save figure
        output_dir = os.path.join(os.path.dirname(__file__), "output")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "pipeline_demo.png")
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {output_path}")

        plt.close()
    except Exception as e:
        print(f"Error during visualization: {e}")
        return

    # ===== Summary =====
    print("\n" + "=" * 60)
    print("PIPELINE DEMO COMPLETE")
    print("=" * 60)
    print(f"G-code file: {gcode_file}")
    print(f"Waypoints parsed: {len(waypoints)}")
    print(f"Joint waypoints: {len(joint_waypoints)}")
    print(f"Trajectory segments: {len(trajectory)}")
    print(f"Output visualization: {output_path}")


if __name__ == "__main__":
    main()

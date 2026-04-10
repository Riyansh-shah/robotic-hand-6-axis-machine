#!/usr/bin/env python3
"""
Quick visualization demo of the 6-axis robotic arm.

Demonstrates the kinematics visualizer by:
  1. Creating an ArmSimulator
  2. Setting several joint configurations
  3. Plotting the arm at each configuration
  4. Showing transitions between poses

Usage:
    python examples/demo_visualization.py

Output:
    Saves a figure showing the arm in multiple configurations.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from kinematics import plot_arm, DH6R, forward_kinematics, get_all_transforms
from simulation import ArmSimulator


def main():
    """Run the visualization demo."""

    print("=" * 60)
    print("ARM VISUALIZATION DEMO")
    print("=" * 60)

    # ===== Create ArmSimulator =====
    print("\nInitializing ArmSimulator...")
    simulator = ArmSimulator()
    print("ArmSimulator initialized")

    # ===== Define joint configurations =====
    print("\nDefining joint configurations...")

    # Configuration 1: Home position (all zeros)
    config_home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Configuration 2: Elbow up
    config_elbow_up = np.array([0.0, np.pi/4, 0.0, 0.0, 0.0, 0.0])

    # Configuration 3: Reach forward
    config_reach = np.array([0.0, np.pi/6, np.pi/3, 0.0, 0.0, 0.0])

    # Configuration 4: Rotated
    config_rotated = np.array([np.pi/4, np.pi/4, np.pi/6, 0.0, 0.0, 0.0])

    # Configuration 5: Complex pose
    config_complex = np.array([np.pi/3, np.pi/5, -np.pi/6, np.pi/4, np.pi/3, np.pi/6])

    configs = [
        ("Home (All zeros)", config_home),
        ("Elbow Up", config_elbow_up),
        ("Reach Forward", config_reach),
        ("Rotated", config_rotated),
        ("Complex Pose", config_complex),
    ]

    print(f"Defined {len(configs)} configurations")

    # ===== Compute forward kinematics for each config =====
    print("\nComputing forward kinematics for each configuration...")

    fig = plt.figure(figsize=(18, 12))
    fig.suptitle("6-Axis Robotic Arm: Multiple Configurations", fontsize=16, fontweight='bold')

    for idx, (name, joints) in enumerate(configs, 1):
        print(f"  [{idx}] {name}: {np.degrees(joints).round(1)}°")

        ax = fig.add_subplot(2, 3, idx, projection='3d')

        try:
            # Plot the arm
            plot_arm(joints, ax=ax, title=name, show=False)

            # Compute and display TCP position
            ee_pose = forward_kinematics(joints)
            tcp_pos = ee_pose[:3, 3]
            ax.scatter(*tcp_pos, color='red', s=100, marker='*', label='TCP')
            ax.legend()

            print(f"       TCP position: ({tcp_pos[0]:.2f}, {tcp_pos[1]:.2f}, {tcp_pos[2]:.2f})")

        except Exception as e:
            print(f"       Error plotting: {e}")
            ax.text(0.5, 0.5, f"Error:\n{str(e)[:50]}", ha='center', va='center')

    # ===== Workspace slice =====
    print("\nGenerating workspace slice...")
    try:
        ax_workspace = fig.add_subplot(2, 3, 6)

        # Monte Carlo workspace sampling
        n_samples = 1000
        np.random.seed(42)
        joint_ranges = [
            (-np.pi, np.pi),      # Joint 1
            (-np.pi/2, np.pi),    # Joint 2
            (-np.pi, np.pi),      # Joint 3
            (-np.pi, np.pi),      # Joint 4
            (-np.pi/2, np.pi/2),  # Joint 5
            (-np.pi, np.pi),      # Joint 6
        ]

        tcp_positions = []
        for _ in range(n_samples):
            random_joints = np.array([
                np.random.uniform(r[0], r[1]) for r in joint_ranges
            ])
            try:
                ee_pose = forward_kinematics(random_joints)
                tcp_pos = ee_pose[:3, 3]
                tcp_positions.append(tcp_pos)
            except:
                pass

        if tcp_positions:
            tcp_positions = np.array(tcp_positions)
            ax_workspace.scatter(tcp_positions[:, 0], tcp_positions[:, 1],
                               alpha=0.3, s=5, c=tcp_positions[:, 2], cmap='viridis')
            ax_workspace.set_xlabel("X (mm)")
            ax_workspace.set_ylabel("Y (mm)")
            ax_workspace.set_title("Workspace Slice (Top View, colored by Z)")
            ax_workspace.grid(True, alpha=0.3)
            print(f"  Plotted {len(tcp_positions)} reachable points")

    except Exception as e:
        print(f"  Error generating workspace: {e}")

    # ===== Save figure =====
    print("\nSaving visualization...")
    output_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "arm_visualization.png")

    try:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved to {output_path}")
    except Exception as e:
        print(f"Error saving figure: {e}")

    plt.close()

    # ===== Summary =====
    print("\n" + "=" * 60)
    print("VISUALIZATION DEMO COMPLETE")
    print("=" * 60)
    print(f"Configurations visualized: {len(configs)}")
    print(f"Output file: {output_path}")


if __name__ == "__main__":
    main()

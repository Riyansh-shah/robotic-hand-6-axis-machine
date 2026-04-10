"""
simulation — Simulate the 6-DOF arm tracing toolpaths in 3D.

Provides classes and functions for forward kinematics-based simulation
of joint trajectories, end-effector tracing, and Cartesian waypoint visualization.

Exports:
    ArmSimulator           — simulate arm joint trajectories and EE positions
    visualize_toolpath     — plot Cartesian waypoints with extrusion coloring
    visualize_arm_on_toolpath — animate arm tracing through joint trajectory
    plot_workspace_slice   — Monte Carlo workspace cross-section scatter
"""

from .arm_simulator import ArmSimulator
from .toolpath_visualizer import (
    visualize_toolpath,
    visualize_arm_on_toolpath,
    plot_workspace_slice,
)

__all__ = [
    "ArmSimulator",
    "visualize_toolpath",
    "visualize_arm_on_toolpath",
    "plot_workspace_slice",
]

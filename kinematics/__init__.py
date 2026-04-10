"""
kinematics — Forward kinematics, inverse kinematics, and visualization for the 6-DOF robotic arm.

Exports:
    DH6R                   — DH parameter dataclass for a 6R articulated arm
    forward_kinematics     — compute 4×4 end-effector transform from joint angles
    get_all_transforms     — compute transforms for all joints
    ik_numerical           — damped least-squares numerical IK solver
    ik_ikpy                — ikpy-based IK solver
    IKError                — inverse kinematics error exception
    plot_arm               — 3D visualization of arm linkage
    animate_arm            — animated arm trajectory visualization
"""

from .dh_params import DH6R, get_dh_table
from .forward_kinematics import forward_kinematics, get_all_transforms
from .inverse_kinematics import ik_numerical, ik_ikpy, IKError
from .visualize import plot_arm, animate_arm

__all__ = [
    "DH6R",
    "get_dh_table",
    "forward_kinematics",
    "get_all_transforms",
    "ik_numerical",
    "ik_ikpy",
    "IKError",
    "plot_arm",
    "animate_arm",
]

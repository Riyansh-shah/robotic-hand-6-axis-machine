"""
ArmSimulator for executing and tracing 6-DOF arm trajectories.

Simulates joint angle sequences using forward kinematics, recording end-effector
positions and intermediate joint locations for visualization and analysis.
"""

from typing import List, Optional, Union
import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import kinematics and utils
sys.path.insert(0, str(Path(__file__).parent.parent))

from kinematics.forward_kinematics import forward_kinematics, get_all_transforms
from kinematics.dh_params import DH6R, get_dh_table


class ArmSimulator:
    """
    Simulate the 6-DOF arm executing joint trajectories.

    Computes forward kinematics at each step, tracks end-effector positions
    and joint locations for visualization and trajectory analysis.

    Attributes
    ----------
    dh_table : DH6R
        DH parameter dataclass for the arm configuration.
    joint_angles : np.ndarray
        Current joint angle configuration [q1, q2, q3, q4, q5, q6].
    ee_trace : np.ndarray
        End-effector positions traced during execution (Nx3).
    joint_positions_trace : List[np.ndarray]
        Joint positions at each step of trajectory execution.
    """

    def __init__(self, dh_table: Optional[DH6R] = None) -> None:
        """
        Initialize the arm simulator.

        Parameters
        ----------
        dh_table : DH6R, optional
            DH parameter dataclass. If None, uses the default DH6R 6R arm.
        """
        self.dh_table = dh_table if dh_table is not None else get_dh_table()
        self.joint_angles: np.ndarray = np.zeros(6)
        self.ee_trace: np.ndarray = np.empty((0, 3))
        self.joint_positions_trace: List[np.ndarray] = []
        self._is_reset = True

    def set_joint_angles(self, angles: Union[List[float], np.ndarray]) -> None:
        """
        Set the current joint angle configuration.

        Parameters
        ----------
        angles : array-like, shape (6,)
            Joint angles in radians [q1, q2, q3, q4, q5, q6].

        Raises
        ------
        AssertionError
            If angles array is not shape (6,).
        """
        angles = np.asarray(angles).ravel()
        assert angles.shape == (6,), f"Expected 6 joint angles, got {angles.shape}"
        self.joint_angles = angles.copy()

    def get_ee_pose(self) -> np.ndarray:
        """
        Get the current end-effector pose via forward kinematics.

        Returns
        -------
        T : np.ndarray, shape (4, 4)
            Homogeneous transformation matrix of the end-effector.
            T[:3, :3] is the 3×3 rotation matrix, T[:3, 3] is position.
        """
        return forward_kinematics(self.joint_angles, self.dh_table)

    def get_joint_positions(self) -> List[np.ndarray]:
        """
        Get the 3D positions of all joints for the current configuration.

        Returns all joint positions from base (frame 0) through end-effector
        (frame 6), enabling visualization of the arm linkage.

        Returns
        -------
        positions : List[np.ndarray]
            List of 7 position vectors (3,):
            [base, joint_1, joint_2, ..., joint_6, ee]
        """
        transforms = get_all_transforms(self.joint_angles, self.dh_table)
        return [T[:3, 3] for T in transforms]

    def execute_trajectory(
        self,
        trajectory: List[Union[List[float], np.ndarray]],
    ) -> None:
        """
        Execute a sequence of joint angle configurations.

        Steps through each configuration in the trajectory, computing forward
        kinematics at each step. Records end-effector positions and joint
        locations for later retrieval via get_ee_trace() and joint_positions_trace.

        Parameters
        ----------
        trajectory : List[array-like]
            Sequence of joint configurations, each shape (6,).
            Each element is a joint angle vector in radians.
        """
        self.ee_trace = np.empty((0, 3))
        self.joint_positions_trace = []

        for angles in trajectory:
            self.set_joint_angles(angles)

            # Record end-effector position
            ee_pose = self.get_ee_pose()
            ee_position = ee_pose[:3, 3]
            self.ee_trace = np.vstack([self.ee_trace, ee_position])

            # Record all joint positions
            joint_positions = self.get_joint_positions()
            self.joint_positions_trace.append(joint_positions)

        self._is_reset = False

    def get_ee_trace(self) -> np.ndarray:
        """
        Get the end-effector positions traced during execution.

        Returns an Nx3 array of end-effector positions collected during
        the last call to execute_trajectory(). Returns empty array (0x3)
        if no trajectory has been executed.

        Returns
        -------
        trace : np.ndarray, shape (N, 3)
            End-effector positions [x, y, z] for each step in the trajectory.
        """
        return self.ee_trace.copy()

    def reset(self) -> None:
        """
        Reset the simulator to initial state.

        Clears joint angles, end-effector trace, and joint position history.
        """
        self.joint_angles = np.zeros(6)
        self.ee_trace = np.empty((0, 3))
        self.joint_positions_trace = []
        self._is_reset = True

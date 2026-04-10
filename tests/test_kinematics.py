"""Unit tests for kinematics module: forward kinematics and inverse kinematics."""

import pytest
import numpy as np
import numpy.testing as npt
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from kinematics import (
    DH6R,
    get_dh_table,
    forward_kinematics,
    get_all_transforms,
    ik_numerical,
    IKError,
)


class TestForwardKinematics:
    """Test suite for forward kinematics computation."""

    def test_fk_home_position(self, dh_table):
        """Test FK at zero angles gives expected home position.

        At zero angles, the end-effector position should approximately equal
        the sum of the link offsets in Z direction and link lengths in X direction,
        depending on the DH parameter configuration.
        """
        q = np.zeros(6)
        T = forward_kinematics(q, dh_table)

        # Check output is 4x4
        assert T.shape == (4, 4), f"Expected shape (4, 4), got {T.shape}"

        # Check homogeneous transform structure
        npt.assert_array_almost_equal(T[3, :], [0, 0, 0, 1])

        # At zero angles, the position should be finite and reasonable
        # (not NaN or Inf)
        position = T[:3, 3]
        assert np.all(np.isfinite(position)), \
            f"Position contains non-finite values: {position}"

        # Rotation should be proper orthogonal (R^T @ R = I)
        R = T[:3, :3]
        npt.assert_array_almost_equal(R.T @ R, np.eye(3), decimal=5)

    def test_fk_returns_4x4(self, dh_table):
        """Test that forward_kinematics always returns a 4x4 matrix."""
        test_angles = [
            np.zeros(6),
            np.array([0.5, 0.3, -0.2, 0.1, -0.4, 0.6]),
            np.array([-np.pi / 4, np.pi / 6, np.pi / 3, -np.pi / 6, 0.5, -0.5]),
        ]

        for q in test_angles:
            T = forward_kinematics(q, dh_table)
            assert T.shape == (4, 4), \
                f"FK should return 4x4, got {T.shape} for angles {q}"

    def test_fk_rotation_orthogonal(self, dh_table):
        """Test that the rotation part of FK output is orthogonal (R^T R = I).

        This is a fundamental property of rotation matrices.
        """
        q = np.array([0.3, 0.4, -0.5, 0.2, -0.1, 0.4])
        T = forward_kinematics(q, dh_table)
        R = T[:3, :3]

        # Check orthogonality: R^T @ R should equal identity
        npt.assert_array_almost_equal(R.T @ R, np.eye(3), decimal=5,
                                      err_msg="Rotation matrix is not orthogonal")

        # Check determinant is 1 (proper rotation, not reflection)
        det = np.linalg.det(R)
        npt.assert_almost_equal(det, 1.0, decimal=5,
                               err_msg=f"Determinant of R is {det}, expected 1.0")

    def test_fk_accepts_list_and_array(self, dh_table):
        """Test that FK accepts both list and numpy array inputs."""
        q_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        q_array = np.array(q_list)

        T_from_list = forward_kinematics(q_list, dh_table)
        T_from_array = forward_kinematics(q_array, dh_table)

        npt.assert_array_almost_equal(T_from_list, T_from_array)

    def test_fk_invalid_input_size_raises(self, dh_table):
        """Test that FK raises AssertionError with wrong input size."""
        # Test with 5 angles (should be 6)
        q_invalid = np.array([0.1, 0.2, 0.3, 0.4, 0.5])

        with pytest.raises((AssertionError, ValueError)):
            forward_kinematics(q_invalid, dh_table)

        # Test with 7 angles
        q_invalid = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])

        with pytest.raises((AssertionError, ValueError)):
            forward_kinematics(q_invalid, dh_table)


class TestAllTransforms:
    """Test suite for get_all_transforms (joint frame transforms)."""

    def test_all_transforms_length(self, dh_table):
        """Test that get_all_transforms returns 7 transforms (base + 6 joints).

        The list should contain:
        - transforms[0]: base frame (identity)
        - transforms[1-6]: cumulative transforms to each joint
        """
        q = np.array([0.1, 0.2, -0.3, 0.4, -0.5, 0.6])
        transforms = get_all_transforms(q, dh_table)

        assert len(transforms) == 7, \
            f"Expected 7 transforms, got {len(transforms)}"

    def test_all_transforms_base_is_identity(self, dh_table):
        """Test that the first transform (base frame) is identity."""
        q = np.array([0.5, 0.3, 0.2, 0.1, 0.4, 0.6])
        transforms = get_all_transforms(q, dh_table)

        npt.assert_array_almost_equal(transforms[0], np.eye(4),
                                     err_msg="Base frame should be identity")

    def test_all_transforms_last_matches_fk(self, dh_table):
        """Test that the last transform matches forward_kinematics output.

        get_all_transforms[6] should be identical to forward_kinematics output.
        """
        q = np.array([0.2, 0.4, -0.3, 0.5, -0.2, 0.1])

        T_fk = forward_kinematics(q, dh_table)
        transforms = get_all_transforms(q, dh_table)
        T_last = transforms[-1]

        npt.assert_array_almost_equal(T_fk, T_last,
                                     err_msg="Last transform should match FK result")

    def test_all_transforms_are_4x4(self, dh_table):
        """Test that each returned transform is 4x4."""
        q = np.zeros(6)
        transforms = get_all_transforms(q, dh_table)

        for i, T in enumerate(transforms):
            assert T.shape == (4, 4), \
                f"Transform {i} has shape {T.shape}, expected (4, 4)"

    def test_all_transforms_proper_rotations(self, dh_table):
        """Test that rotation matrices in all transforms are orthogonal."""
        q = np.array([0.3, 0.2, 0.4, 0.1, 0.5, 0.3])
        transforms = get_all_transforms(q, dh_table)

        for i, T in enumerate(transforms):
            R = T[:3, :3]
            # Check orthogonality
            npt.assert_array_almost_equal(R.T @ R, np.eye(3), decimal=5,
                                         err_msg=f"Transform {i} rotation not orthogonal")
            # Check determinant = 1
            det = np.linalg.det(R)
            npt.assert_almost_equal(det, 1.0, decimal=5,
                                   err_msg=f"Transform {i} rotation determinant not 1")


class TestInverseKinematics:
    """Test suite for numerical inverse kinematics solver."""

    def test_ik_numerical_convergence(self, dh_table):
        """Test that IK converges to a target pose generated from FK.

        This is the fundamental roundtrip test: FK(q) -> target, then IK(target) -> q'.
        The final FK(q') should approximately match the original target.
        """
        # Choose a valid joint configuration
        q_original = np.array([0.3, 0.4, -0.2, 0.1, 0.5, -0.3])

        # Compute target pose via FK
        T_target = forward_kinematics(q_original, dh_table)

        # Solve IK to recover joint angles
        q_recovered = ik_numerical(T_target, dh_table, q_init=np.zeros(6))

        # Compute FK of recovered angles
        T_recovered = forward_kinematics(q_recovered, dh_table)

        # Positions should match within 1e-4 m
        npt.assert_array_almost_equal(T_target[:3, 3], T_recovered[:3, 3],
                                     decimal=4,
                                     err_msg="IK roundtrip position error too large")

    def test_ik_with_different_initial_guesses(self, dh_table):
        """Test that IK converges from different initial guesses."""
        q_original = np.array([0.2, 0.3, -0.1, 0.4, 0.2, -0.5])
        T_target = forward_kinematics(q_original, dh_table)

        # Try several different initial guesses
        initial_guesses = [
            np.zeros(6),
            np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5]),
            np.array([-0.3, 0.2, -0.4, 0.1, -0.5, 0.3]),
        ]

        for q_init in initial_guesses:
            q_solution = ik_numerical(T_target, dh_table, q_init=q_init)
            T_solution = forward_kinematics(q_solution, dh_table)

            # Position should match
            npt.assert_array_almost_equal(T_target[:3, 3], T_solution[:3, 3],
                                         decimal=4)

    def test_ik_unreachable_target_raises(self, dh_table):
        """Test that IK raises IKError for targets far outside workspace.

        A target position very far away (e.g., 10 m away) should not be reachable
        for a desktop arm with ~0.5 m reach.
        """
        T_unreachable = np.eye(4)
        T_unreachable[:3, 3] = [10.0, 10.0, 10.0]  # Far beyond workspace

        with pytest.raises(IKError):
            ik_numerical(T_unreachable, dh_table, max_iter=100)

    def test_ik_default_initial_guess(self, dh_table):
        """Test that IK works with default (zero) initial guess."""
        q_original = np.array([0.4, -0.3, 0.2, -0.1, 0.3, 0.5])
        T_target = forward_kinematics(q_original, dh_table)

        # Call IK without specifying q_init (should default to zeros)
        q_solution = ik_numerical(T_target, dh_table)

        T_solution = forward_kinematics(q_solution, dh_table)
        npt.assert_array_almost_equal(T_target[:3, 3], T_solution[:3, 3],
                                     decimal=4)

    def test_ik_accepts_list_and_array(self, dh_table):
        """Test that IK accepts both 3D position arrays and 4x4 transforms."""
        q_original = np.array([0.2, 0.3, 0.1, 0.4, 0.5, 0.2])
        T_target = forward_kinematics(q_original, dh_table)

        # IK should accept 4x4 target
        q_from_T = ik_numerical(T_target, dh_table)
        T_result = forward_kinematics(q_from_T, dh_table)

        npt.assert_array_almost_equal(T_target[:3, 3], T_result[:3, 3],
                                     decimal=4)


class TestFKIKRoundtrip:
    """Integration tests for FK-IK roundtrip convergence."""

    @pytest.mark.parametrize("config_name", ["home", "config_1", "config_2", "config_3"])
    def test_fk_ik_roundtrip_multiple_configs(self, dh_table, sample_joint_angles,
                                               config_name):
        """Parametrized test: FK->IK roundtrip for multiple known configurations.

        For each configuration in sample_joint_angles, verify that:
        1. FK(q) computes forward kinematics
        2. IK(FK(q)) recovers the joint angles
        3. FK(IK(FK(q))) matches the original position within tolerance
        """
        q_original = sample_joint_angles[config_name]

        # FK: joint space -> Cartesian space
        T = forward_kinematics(q_original, dh_table)

        # IK: Cartesian space -> joint space
        q_recovered = ik_numerical(T, dh_table, q_init=np.zeros(6), max_iter=200)

        # FK again: should recover the same end-effector pose
        T_recovered = forward_kinematics(q_recovered, dh_table)

        # Position should match within 1e-4 m (0.1 mm)
        npt.assert_array_almost_equal(T[:3, 3], T_recovered[:3, 3],
                                     decimal=4,
                                     err_msg=f"FK-IK roundtrip failed for {config_name}")

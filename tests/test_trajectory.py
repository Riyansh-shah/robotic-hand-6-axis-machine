"""Unit tests for trajectory planning module."""

import pytest
import numpy as np
import numpy.testing as npt
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from trajectory_planner import (
    trapezoidal_velocity_profile,
    interpolate_trajectory,
    linear_interpolation,
    DEFAULT_JOINT_VELOCITY_LIMITS,
    DEFAULT_JOINT_ACCEL_LIMITS,
    check_velocity_limits,
    check_acceleration_limits,
)


class TestTrapezoidalVelocityProfile:
    """Test suite for trapezoidal velocity profile generation."""

    def test_trapezoidal_profile_endpoints(self):
        """Test that trajectory starts at q_start and ends at q_end.

        The first row should match q_start and the last row should match q_end
        to within numerical precision.
        """
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([1.0, 0.5, -0.5, 0.2, 0.3, -0.2])

        trajectory = trapezoidal_velocity_profile(q_start, q_end, v_max=1.0,
                                                  a_max=2.0, dt=0.01)

        # Check first and last points
        npt.assert_array_almost_equal(trajectory[0, :], q_start, decimal=5,
                                     err_msg="Trajectory should start at q_start")
        npt.assert_array_almost_equal(trajectory[-1, :], q_end, decimal=5,
                                     err_msg="Trajectory should end at q_end")

    def test_trapezoidal_profile_shape(self):
        """Test that trajectory has correct shape (N, 6)."""
        q_start = np.zeros(6)
        q_end = np.ones(6)

        trajectory = trapezoidal_velocity_profile(q_start, q_end, v_max=1.0,
                                                  a_max=2.0, dt=0.01)

        assert trajectory.ndim == 2, f"Trajectory should be 2D, got {trajectory.ndim}D"
        assert trajectory.shape[1] == 6, \
            f"Trajectory should have 6 columns, got {trajectory.shape[1]}"
        assert trajectory.shape[0] >= 2, "Trajectory should have at least 2 points"

    def test_trapezoidal_profile_smooth_velocity(self):
        """Test that velocity profile respects v_max within tolerance.

        Compute velocities via finite difference and verify no excessive peaks.
        We allow a small tolerance (10%) for numerical discretization effects.
        """
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([2.0, 1.5, -1.0, 0.5, 1.0, -0.5])
        v_max = 0.8
        a_max = 1.5
        dt = 0.01

        trajectory = trapezoidal_velocity_profile(q_start, q_end, v_max=v_max,
                                                  a_max=a_max, dt=dt)

        # Compute velocities via finite difference
        dq = np.diff(trajectory, axis=0) / dt
        v_abs = np.abs(dq)

        # Allow 10% tolerance due to discretization
        max_v = np.max(v_abs)
        assert max_v <= v_max * 1.1, \
            f"Max velocity {max_v} exceeds v_max * 1.1 = {v_max * 1.1}"

    def test_trapezoidal_profile_symmetric(self):
        """Test that acceleration and deceleration phases are symmetric.

        In a symmetric profile, the trajectory should be roughly symmetric
        in time around the midpoint.
        """
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        trajectory = trapezoidal_velocity_profile(q_start, q_end, v_max=1.0,
                                                  a_max=2.0, dt=0.01)

        n = len(trajectory)
        mid = n // 2

        # Compute cumulative distance (path parameter)
        distances = np.sqrt(np.sum(np.diff(trajectory, axis=0)**2, axis=1))
        cumsum = np.concatenate(([0], np.cumsum(distances)))

        # Check approximate symmetry: distance at first quarter ~= distance from third quarter to end
        quarter = n // 4
        three_quarter = 3 * n // 4

        dist_to_quarter = cumsum[quarter]
        dist_from_three_quarter = cumsum[-1] - cumsum[three_quarter]

        # Allow 20% tolerance due to discrete sampling
        ratio = max(dist_to_quarter, dist_from_three_quarter) / \
                min(dist_to_quarter, dist_from_three_quarter)
        assert ratio < 1.2, f"Profile not symmetric enough, ratio = {ratio}"

    def test_trapezoidal_profile_accepts_list_and_array(self):
        """Test that profile accepts both list and numpy array inputs."""
        q_start_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q_end_list = [1.0, 0.5, -0.5, 0.2, 0.3, -0.2]

        q_start_array = np.array(q_start_list)
        q_end_array = np.array(q_end_list)

        traj_from_lists = trapezoidal_velocity_profile(q_start_list, q_end_list)
        traj_from_arrays = trapezoidal_velocity_profile(q_start_array, q_end_array)

        npt.assert_array_almost_equal(traj_from_lists, traj_from_arrays)

    def test_trapezoidal_profile_zero_distance(self):
        """Test profile with zero distance (start = end)."""
        q = np.array([0.5, 0.3, 0.2, 0.1, 0.4, 0.6])

        trajectory = trapezoidal_velocity_profile(q, q, v_max=1.0, a_max=2.0)

        # Should return trajectory with at least one point (the position itself)
        assert len(trajectory) >= 1
        npt.assert_array_almost_equal(trajectory[0, :], q)

    def test_trapezoidal_profile_small_distance(self):
        """Test profile with very small distance (triangular profile).

        When 2*s_accel > total_distance, the profile is triangular (no cruise phase).
        """
        q_start = np.zeros(6)
        q_end = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])  # Very small distance
        v_max = 10.0  # High velocity limit
        a_max = 10.0  # High acceleration limit

        trajectory = trapezoidal_velocity_profile(q_start, q_end, v_max=v_max,
                                                  a_max=a_max, dt=0.001)

        # Should still reach the endpoint
        npt.assert_array_almost_equal(trajectory[-1, :], q_end, decimal=4)


class TestLinearInterpolation:
    """Test suite for linear interpolation method."""

    def test_linear_interpolation_endpoints(self):
        """Test that first and last points match the start and end."""
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([1.0, 0.5, -0.5, 0.2, 0.3, -0.2])
        n_steps = 50

        trajectory = linear_interpolation(q_start, q_end, n_steps=n_steps)

        npt.assert_array_almost_equal(trajectory[0, :], q_start)
        npt.assert_array_almost_equal(trajectory[-1, :], q_end)

    def test_linear_interpolation_shape(self):
        """Test that output shape is (n_steps, 6)."""
        q_start = np.zeros(6)
        q_end = np.ones(6)
        n_steps = 100

        trajectory = linear_interpolation(q_start, q_end, n_steps=n_steps)

        assert trajectory.shape == (n_steps, 6), \
            f"Expected shape ({n_steps}, 6), got {trajectory.shape}"

    def test_linear_interpolation_intermediate_points(self):
        """Test that intermediate points lie on the line between start and end."""
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([2.0, 4.0, 6.0, 2.0, 4.0, 6.0])
        n_steps = 11  # Easy arithmetic

        trajectory = linear_interpolation(q_start, q_end, n_steps=n_steps)

        # At step 5 (halfway), should be at midpoint
        midpoint_expected = (q_start + q_end) / 2.0
        npt.assert_array_almost_equal(trajectory[5, :], midpoint_expected, decimal=5)

        # At step 2 (1/5 of the way), should be 20% towards end
        quarter_expected = q_start + 0.2 * (q_end - q_start)
        npt.assert_array_almost_equal(trajectory[2, :], quarter_expected, decimal=5)

    def test_linear_interpolation_minimum_steps(self):
        """Test linear interpolation with minimum number of steps."""
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.ones(6)

        # Minimum n_steps should be 2 (start and end)
        trajectory = linear_interpolation(q_start, q_end, n_steps=2)

        assert trajectory.shape == (2, 6)
        npt.assert_array_almost_equal(trajectory[0, :], q_start)
        npt.assert_array_almost_equal(trajectory[1, :], q_end)

    def test_linear_interpolation_accepts_list_and_array(self):
        """Test that linear_interpolation accepts both list and array inputs."""
        q_start_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        q_end_list = [1.0, 0.5, -0.5, 0.2, 0.3, -0.2]

        q_start_array = np.array(q_start_list)
        q_end_array = np.array(q_end_list)

        traj_from_lists = linear_interpolation(q_start_list, q_end_list)
        traj_from_arrays = linear_interpolation(q_start_array, q_end_array)

        npt.assert_array_almost_equal(traj_from_lists, traj_from_arrays)


class TestInterpolateTrajectory:
    """Test suite for multi-waypoint trajectory interpolation."""

    def test_interpolate_trajectory_multiple_waypoints(self):
        """Test that interpolate_trajectory chains multiple segments."""
        waypoints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 0.5, -0.5, 0.2, 0.3, -0.2]),
            np.array([1.5, 1.0, 0.5, 0.1, 0.4, 0.1]),
        ]

        trajectory = interpolate_trajectory(waypoints, v_max=1.0, a_max=2.0, dt=0.01)

        # Should start at first waypoint
        npt.assert_array_almost_equal(trajectory[0, :], waypoints[0], decimal=5)

        # Should end at last waypoint
        npt.assert_array_almost_equal(trajectory[-1, :], waypoints[2], decimal=5)

    def test_interpolate_trajectory_passes_through_waypoints(self):
        """Test that trajectory approximately passes through intermediate waypoints.

        After chaining segments, the trajectory should reach or nearly reach
        each waypoint (might be off by a few samples due to sampling resolution).
        """
        waypoints = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0]),
        ]

        trajectory = interpolate_trajectory(waypoints, v_max=1.0, a_max=2.0, dt=0.01)

        # Find indices closest to each waypoint
        for i, waypoint in enumerate(waypoints):
            distances = np.linalg.norm(trajectory - waypoint, axis=1)
            min_dist = np.min(distances)

            # Should get very close to each waypoint
            assert min_dist < 0.05, \
                f"Trajectory doesn't reach waypoint {i}: min_dist = {min_dist}"

    def test_interpolate_trajectory_concatenation(self):
        """Test that multi-segment trajectory is longer than single segment.

        The full trajectory connecting 3 waypoints should be roughly longer than
        one segment (allowing for the fact that segments overlap at start points).
        """
        waypoints_2 = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        ]

        waypoints_3 = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0]),
        ]

        traj_2 = interpolate_trajectory(waypoints_2, v_max=1.0, a_max=2.0, dt=0.01)
        traj_3 = interpolate_trajectory(waypoints_3, v_max=1.0, a_max=2.0, dt=0.01)

        # Three-waypoint trajectory should be substantially longer
        assert len(traj_3) > len(traj_2), \
            f"3-waypoint traj ({len(traj_3)}) should be longer than 2-waypoint ({len(traj_2)})"

    def test_interpolate_trajectory_minimum_waypoints(self):
        """Test that interpolate_trajectory requires at least 2 waypoints."""
        waypoints_1 = [np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])]

        with pytest.raises(ValueError):
            interpolate_trajectory(waypoints_1)

    def test_interpolate_trajectory_accepts_lists(self):
        """Test that waypoints can be provided as a list of lists."""
        waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.5, -0.5, 0.2, 0.3, -0.2],
            [1.5, 1.0, 0.5, 0.1, 0.4, 0.1],
        ]

        trajectory = interpolate_trajectory(waypoints, v_max=1.0, a_max=2.0, dt=0.01)

        assert trajectory.shape[1] == 6
        assert len(trajectory) >= 3


class TestVelocityLimitChecking:
    """Test suite for velocity limit validation."""

    def test_check_velocity_limits_compliant_trajectory(self):
        """Test that a slow trajectory passes velocity limit check."""
        # Slow linear trajectory
        q_start = np.zeros(6)
        q_end = 0.1 * np.ones(6)  # Small movement
        trajectory = linear_interpolation(q_start, q_end, n_steps=1000)  # Many points = slow

        is_valid, velocities = check_velocity_limits(trajectory, dt=0.01)

        assert is_valid, "Slow trajectory should pass velocity limits"

    def test_check_velocity_limits_exceeding_trajectory(self):
        """Test that a fast trajectory fails velocity limit check.

        Create a trajectory where joints jump between positions in few steps,
        resulting in very high velocities.
        """
        trajectory = np.zeros((5, 6))
        trajectory[0, :] = np.zeros(6)
        trajectory[-1, :] = 10.0 * np.ones(6)  # Large jump in few steps

        # With slow dt, this creates very high velocities
        is_valid, velocities = check_velocity_limits(trajectory, dt=0.01)

        # Should fail (too fast)
        assert not is_valid, "Fast trajectory should fail velocity limits"

    def test_check_velocity_limits_returns_velocities(self):
        """Test that check_velocity_limits returns computed velocities."""
        trajectory = linear_interpolation(np.zeros(6), 0.5 * np.ones(6), n_steps=50)

        is_valid, velocities = check_velocity_limits(trajectory, dt=0.01)

        assert velocities.shape[0] == len(trajectory) - 1
        assert velocities.shape[1] == 6


class TestAccelerationLimitChecking:
    """Test suite for acceleration limit validation."""

    def test_check_acceleration_limits_compliant_trajectory(self):
        """Test that a smoothly accelerating trajectory passes limits."""
        # Trapezoidal profile should have smooth acceleration
        trajectory = trapezoidal_velocity_profile(
            np.zeros(6), np.ones(6), v_max=0.5, a_max=0.5, dt=0.01
        )

        is_valid, accelerations = check_acceleration_limits(trajectory, dt=0.01)

        # Trapezoidal profile respects acceleration limits by design
        assert is_valid, "Trapezoidal profile should pass acceleration limits"

    def test_check_acceleration_limits_returns_accelerations(self):
        """Test that function returns computed accelerations."""
        trajectory = linear_interpolation(np.zeros(6), np.ones(6), n_steps=50)

        is_valid, accelerations = check_acceleration_limits(trajectory, dt=0.01)

        assert accelerations.shape[0] == len(trajectory) - 2
        assert accelerations.shape[1] == 6

    def test_check_acceleration_limits_custom_limits(self):
        """Test that custom acceleration limits are respected."""
        # Linear interpolation creates uniform acceleration
        trajectory = linear_interpolation(np.zeros(6), 1.0 * np.ones(6), n_steps=20)

        # Very tight limits (should fail)
        tight_limits = 0.01 * np.ones(6)
        is_valid, accelerations = check_acceleration_limits(
            trajectory, dt=0.01, a_limits=tight_limits
        )

        assert not is_valid, "Should fail with very tight acceleration limits"

        # Loose limits (should pass)
        loose_limits = 100.0 * np.ones(6)
        is_valid, accelerations = check_acceleration_limits(
            trajectory, dt=0.01, a_limits=loose_limits
        )

        assert is_valid, "Should pass with loose acceleration limits"

"""
Accuracy and repeatability measurement tools for the 6-DOF robotic arm.

Provides functions to compute accuracy metrics, repeatability analysis,
and visualization of positional errors and accuracy reports.
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
from pathlib import Path

# Add parent directory to path to import utils if needed
sys.path.insert(0, str(Path(__file__).parent.parent))


@dataclass
class AccuracyReport:
    """
    Accuracy measurement report for TCP or end-effector positions.

    Attributes
    ----------
    rms_error : float
        Root mean squared error (metres).
    max_error : float
        Maximum position error (metres).
    mean_error : float
        Mean position error (metres).
    errors : List[float]
        List of individual position errors for each waypoint (metres).
    positions_measured : np.ndarray, shape (n_points, 3)
        Measured positions (metres).
    positions_expected : np.ndarray, shape (n_points, 3)
        Expected positions (metres).
    """
    rms_error: float
    max_error: float
    mean_error: float
    errors: List[float]
    positions_measured: np.ndarray
    positions_expected: np.ndarray


def compute_accuracy(
    measured_positions: np.ndarray,
    expected_positions: np.ndarray,
) -> AccuracyReport:
    """
    Compute accuracy metrics between measured and expected positions.

    Parameters
    ----------
    measured_positions : np.ndarray, shape (n_points, 3)
        Measured TCP/EE positions (metres).
    expected_positions : np.ndarray, shape (n_points, 3)
        Expected/reference TCP/EE positions (metres).

    Returns
    -------
    AccuracyReport
        Report containing RMS error, max error, mean error, and individual errors.

    Raises
    ------
    ValueError
        If array shapes are inconsistent or invalid.
    """
    measured_positions = np.asarray(measured_positions)
    expected_positions = np.asarray(expected_positions)

    if measured_positions.shape != expected_positions.shape:
        raise ValueError(
            f"Shape mismatch: measured {measured_positions.shape} vs "
            f"expected {expected_positions.shape}"
        )

    if measured_positions.ndim != 2 or measured_positions.shape[1] != 3:
        raise ValueError(
            f"Expected shape (n_points, 3), got {measured_positions.shape}"
        )

    # Compute position differences and errors
    deltas = measured_positions - expected_positions  # shape (n_points, 3)
    errors = np.linalg.norm(deltas, axis=1)  # shape (n_points,)

    # Compute metrics
    rms_error = np.sqrt(np.mean(errors ** 2))
    max_error = np.max(errors)
    mean_error = np.mean(errors)

    return AccuracyReport(
        rms_error=float(rms_error),
        max_error=float(max_error),
        mean_error=float(mean_error),
        errors=errors.tolist(),
        positions_measured=measured_positions.copy(),
        positions_expected=expected_positions.copy(),
    )


def compute_repeatability(
    measurement_sets: List[np.ndarray],
) -> dict:
    """
    Compute repeatability metrics from multiple measurement sets.

    Given multiple measurement sets of the same target positions, compute
    the standard deviation of position errors (repeatability). This measures
    how consistently the arm returns to the same location.

    Parameters
    ----------
    measurement_sets : List[np.ndarray]
        List of measurement arrays, each with shape (n_points, 3).
        All arrays must have the same shape.

    Returns
    -------
    dict
        Dictionary with keys:
        - 'mean_positions': np.ndarray, shape (n_points, 3) — mean position across sets
        - 'std_positions': np.ndarray, shape (n_points, 3) — std dev of positions
        - 'repeatability': np.ndarray, shape (n_points,) — position error std dev (metres)
        - 'mean_repeatability': float — average repeatability (metres)
        - 'max_repeatability': float — worst-case repeatability (metres)

    Raises
    ------
    ValueError
        If measurement_sets is empty or arrays have inconsistent shapes.
    """
    if not measurement_sets:
        raise ValueError("measurement_sets cannot be empty")

    measurement_sets = [np.asarray(m) for m in measurement_sets]

    # Validate shapes
    first_shape = measurement_sets[0].shape
    for i, m in enumerate(measurement_sets):
        if m.shape != first_shape:
            raise ValueError(
                f"Shape mismatch: set 0 has {first_shape}, "
                f"set {i} has {m.shape}"
            )

    # Stack into (n_sets, n_points, 3)
    stacked = np.array(measurement_sets)

    # Compute mean and std dev along the set axis (axis 0)
    mean_positions = np.mean(stacked, axis=0)
    std_positions = np.std(stacked, axis=0, ddof=1) if stacked.shape[0] > 1 else np.zeros_like(mean_positions)

    # Compute per-point repeatability (distance std dev)
    repeatability = np.linalg.norm(std_positions, axis=1)

    return {
        'mean_positions': mean_positions,
        'std_positions': std_positions,
        'repeatability': repeatability,
        'mean_repeatability': float(np.mean(repeatability)),
        'max_repeatability': float(np.max(repeatability)),
    }


def generate_accuracy_report(report: AccuracyReport) -> str:
    """
    Generate a formatted text report of accuracy metrics.

    Parameters
    ----------
    report : AccuracyReport
        Accuracy report object.

    Returns
    -------
    str
        Formatted text report.
    """
    n_points = len(report.errors)

    lines = [
        "=" * 60,
        "TCP/END-EFFECTOR ACCURACY REPORT",
        "=" * 60,
        f"Number of measured points: {n_points}",
        "",
        "ACCURACY METRICS:",
        f"  RMS Error:      {report.rms_error:.6f} m ({report.rms_error * 1000:.3f} mm)",
        f"  Mean Error:     {report.mean_error:.6f} m ({report.mean_error * 1000:.3f} mm)",
        f"  Max Error:      {report.max_error:.6f} m ({report.max_error * 1000:.3f} mm)",
        "",
        "ERROR STATISTICS:",
        f"  Std Dev:        {float(np.std(report.errors)):.6f} m ({float(np.std(report.errors)) * 1000:.3f} mm)",
        f"  Min Error:      {float(np.min(report.errors)):.6f} m ({float(np.min(report.errors)) * 1000:.3f} mm)",
        "",
        "POSITION DATA (first 5 points):",
    ]

    # Add sample position data
    for i in range(min(5, n_points)):
        exp = report.positions_expected[i]
        meas = report.positions_measured[i]
        err = report.errors[i]
        lines.append(
            f"  Point {i}: Expected [{exp[0]:.4f}, {exp[1]:.4f}, {exp[2]:.4f}], "
            f"Measured [{meas[0]:.4f}, {meas[1]:.4f}, {meas[2]:.4f}], "
            f"Error {err:.6f} m"
        )

    lines.extend([
        "",
        "=" * 60,
    ])

    return "\n".join(lines)


def plot_accuracy(
    report: AccuracyReport,
    save_path: Optional[str] = None,
) -> None:
    """
    Create a multi-panel figure visualizing accuracy metrics.

    Generates three subplots:
    1. 3D scatter plot of expected vs measured positions
    2. Histogram of error magnitudes
    3. Error vs waypoint index line plot

    Parameters
    ----------
    report : AccuracyReport
        Accuracy report object.
    save_path : Optional[str]
        Path to save the figure. If None, displays the figure.

    Returns
    -------
    None
    """
    fig = plt.figure(figsize=(15, 5))

    # --- Panel 1: 3D scatter plot of expected vs measured positions ---
    ax1 = fig.add_subplot(131, projection='3d')

    exp = report.positions_expected
    meas = report.positions_measured

    ax1.scatter(exp[:, 0], exp[:, 1], exp[:, 2], c='green', marker='x', s=100,
                label='Expected', alpha=0.7, linewidths=2)
    ax1.scatter(meas[:, 0], meas[:, 1], meas[:, 2], c='red', marker='o', s=50,
                label='Measured', alpha=0.6)

    # Draw lines connecting expected to measured
    for i in range(len(exp)):
        ax1.plot([exp[i, 0], meas[i, 0]],
                [exp[i, 1], meas[i, 1]],
                [exp[i, 2], meas[i, 2]],
                'b-', alpha=0.3, linewidth=0.8)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Expected vs Measured Positions')
    ax1.legend()

    # --- Panel 2: Histogram of error magnitudes ---
    ax2 = fig.add_subplot(132)
    errors_mm = [e * 1000 for e in report.errors]
    ax2.hist(errors_mm, bins=max(10, len(report.errors) // 5), edgecolor='black', alpha=0.7)
    ax2.axvline(report.rms_error * 1000, color='r', linestyle='--', linewidth=2, label='RMS Error')
    ax2.axvline(report.mean_error * 1000, color='g', linestyle='--', linewidth=2, label='Mean Error')
    ax2.set_xlabel('Error (mm)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Distribution of Position Errors')
    ax2.legend()
    ax2.grid(alpha=0.3)

    # --- Panel 3: Error vs waypoint index ---
    ax3 = fig.add_subplot(133)
    waypoint_indices = np.arange(len(report.errors))
    errors_mm = [e * 1000 for e in report.errors]
    ax3.plot(waypoint_indices, errors_mm, 'b-o', markersize=4, alpha=0.7, label='Error')
    ax3.axhline(report.rms_error * 1000, color='r', linestyle='--', linewidth=2, label='RMS')
    ax3.axhline(report.mean_error * 1000, color='g', linestyle='--', linewidth=2, label='Mean')
    ax3.fill_between(waypoint_indices, 0, errors_mm, alpha=0.2)
    ax3.set_xlabel('Waypoint Index')
    ax3.set_ylabel('Error (mm)')
    ax3.set_title('Error vs Waypoint Index')
    ax3.legend()
    ax3.grid(alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figure saved to {save_path}")
    else:
        plt.show()

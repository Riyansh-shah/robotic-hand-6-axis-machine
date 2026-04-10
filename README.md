# 6-Axis Robotic Arm 3D Printer

A desktop-scale 6-DOF robotic arm designed for non-planar 3D printing. Built as a PBL 2026 capstone project at Manipal University Jaipur, Dept. of Computer Science & Engineering.

The robot uses an articulated 6R kinematic chain with optimised link geometry to minimise arm mass while maintaining a usable print workspace. A standard Bowden-fed FDM extruder (E3D V6) is mounted at the end-effector, enabling true 5-axis printing on curved and tilted surfaces — something impossible with conventional Cartesian printers.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Repository Layout](#repository-layout)
3. [Installation](#installation)
4. [Quick Start](#quick-start)
5. [Module Reference](#module-reference)
   - [kinematics/](#kinematics)
   - [simulation/](#simulation)
   - [gcode_parser/](#gcode_parser)
   - [trajectory_planner/](#trajectory_planner)
   - [control/](#control)
   - [calibration/](#calibration)
   - [utils/](#utils)
6. [Tests](#tests)
7. [Examples](#examples)
8. [Documentation](#documentation)
9. [Bill of Materials](#bill-of-materials)
10. [CAD & Mechanical Design](#cad--mechanical-design)
11. [Presentation Website](#presentation-website)
12. [Contributing](#contributing)
13. [License](#license)

---

## System Architecture

The software pipeline converts standard G-code (from any slicer) into smooth joint-space trajectories and sends them to the arm's motor controller:

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│  G-code     │     │  Cartesian   │     │  Joint-angle    │
│  (.gcode)   │────▶│  Waypoints   │────▶│  Configurations │
│  from Slicer│     │  (X, Y, Z, E)│     │  via IK Solver  │
└─────────────┘     └──────────────┘     └────────┬────────┘
                                                   │
                    ┌──────────────┐     ┌─────────▼────────┐
                    │  Motor       │     │  Smooth Trajectory│
                    │  Controller  │◀────│  (Trapezoidal    │
                    │  (Serial)    │     │   Velocity)      │
                    └──────────────┘     └──────────────────┘
```

The arm's kinematics follow the standard Denavit-Hartenberg convention with these link parameters:

| Joint | a (m)  | α (rad) | d (m)  | θ offset |
|-------|--------|---------|--------|----------|
| 1     | 0.000  | π/2     | 0.100  | 0        |
| 2     | 0.200  | 0       | 0.000  | 0        |
| 3     | 0.150  | 0       | 0.000  | 0        |
| 4     | 0.000  | π/2     | 0.080  | 0        |
| 5     | 0.000  | −π/2    | 0.000  | 0        |
| 6     | 0.000  | 0       | 0.050  | 0        |

Total reach ≈ 480 mm from the base. All joints have ±160° travel.

---

## Repository Layout

```
.
├── main.py                  # Single entry-point — runs full pipeline demo
├── requirements.txt         # Python dependencies
├── LICENSE                  # MIT
├── .gitignore
│
├── kinematics/              # DH params, FK, IK, 3D visualisation
├── simulation/              # Arm simulator & toolpath visualiser
├── gcode_parser/            # G-code reader → Cartesian waypoints
├── trajectory_planner/      # IK + trapezoidal velocity interpolation
├── control/                 # Mock serial controller (Arduino/STM32)
├── calibration/             # TCP calibration & accuracy measurement
├── utils/                   # Rotation matrices, transforms, plotting
│
├── tests/                   # pytest unit tests
├── examples/                # Sample G-code, demo scripts
├── docs/                    # Jupyter notebooks (DH walkthrough, workspace)
├── BOM/                     # Bill of Materials (Markdown + CSV)
├── CAD/                     # Mechanical design guide + placeholder dirs
│
├── index.html               # PBL presentation website (landing page)
└── pages/                   # Presentation subpages
```

---

## Installation

**Prerequisites:** Python 3.10 or newer.

```bash
# Clone the repository
git clone https://github.com/your-org/6axis-arm-printer.git
cd 6axis-arm-printer

# Create a virtual environment (recommended)
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
# .venv\Scripts\activate       # Windows

# Install dependencies
pip install -r requirements.txt
```

---

## Quick Start

Run the full pipeline with the included sample G-code:

```bash
python main.py
```

This will parse `examples/sample_cube.gcode`, solve inverse kinematics for each waypoint, plan a smooth trajectory, send it to the mock controller, and save visualisation plots to `examples/output/`.

To use a custom G-code file:

```bash
python main.py path/to/your_model.gcode
```

---

## Module Reference

### `kinematics/`

Core robotics math for the 6R arm.

- **`dh_params.py`** — `DH6R` dataclass holding link dimensions, joint limits, and the DH parameter table. Call `DH6R.default()` for the standard desktop arm configuration.
- **`forward_kinematics.py`** — `forward_kinematics(q, dh_table)` returns the 4×4 end-effector homogeneous transform. `get_all_transforms()` returns intermediate frames for visualisation.
- **`inverse_kinematics.py`** — Two solvers: `ik_numerical()` (from-scratch damped least-squares with Jacobian via finite differences) and `ik_ikpy()` (wrapper around the ikpy library). Raises `IKError` for unreachable targets.
- **`visualize.py`** — `plot_arm()` draws the arm linkage in 3D; `animate_arm()` creates a matplotlib animation of the arm tracing a trajectory.

### `simulation/`

Higher-level simulation layer built on top of kinematics.

- **`arm_simulator.py`** — `ArmSimulator` manages joint state, executes trajectories, and records the end-effector trace.
- **`toolpath_visualizer.py`** — `visualize_toolpath()` plots Cartesian waypoints colour-coded by extrusion state; `visualize_arm_on_toolpath()` animates the arm following a toolpath; `plot_workspace_slice()` generates a Monte Carlo workspace scatter.

### `gcode_parser/`

Reads standard FDM G-code and extracts machine-agnostic waypoints.

- **`parser.py`** — `GCodeParser` processes G0, G1, G28, G90/G91, M104, M106, and more. Outputs a list of `Waypoint` dataclass instances with positions in metres.
- **`filters.py`** — Post-processing utilities: `filter_travel_moves()`, `downsample_waypoints()`, `apply_z_offset()`, `transform_to_arm_frame()`.

### `trajectory_planner/`

Converts Cartesian waypoints into smooth, hardware-safe joint trajectories.

- **`cartesian_to_joint.py`** — `waypoints_to_joint_trajectory()` runs IK on each waypoint with warm-start continuity from the previous solution.
- **`interpolation.py`** — `trapezoidal_velocity_profile()` generates acceleration-limited motion segments; `interpolate_trajectory()` chains them.
- **`velocity_limits.py`** — Default NEMA 17 speed/accel limits and constraint-checking functions.

### `control/`

Mock hardware interface for testing without a physical arm.

- **`protocol.py`** — `SerialProtocol` defines the binary frame format `[0xAA, 0x55, CMD, LEN, DATA…, XOR_CHECKSUM]` with commands for move, home, e-stop, etc.
- **`mock_controller.py`** — `MockController` simulates an Arduino Mega / STM32 over serial. Accepts trajectory arrays, tracks internal joint state, and logs all commands.

### `calibration/`

Tools for aligning the physical arm with the kinematic model.

- **`tcp_calibration.py`** — `calibrate_tcp_four_point()` solves for the TCP offset using four configurations that touch the same physical point (least-squares optimisation).
- **`accuracy_measurement.py`** — `compute_accuracy()` returns an `AccuracyReport` with RMS, max, and mean positional errors; `plot_accuracy()` visualises the results.

### `utils/`

Shared low-level helpers imported by every other module.

- **`transforms.py`** — `rot_x/y/z`, `homogeneous()`, `dh_transform()`, `euler_to_rot()`, `rot_to_euler()`.
- **`angle_utils.py`** — `wrap_angle()`, `deg2rad()`, `rad2deg()`.
- **`plotting.py`** — `plot_frame()` draws RGB coordinate axes; `make_3d_axes()` returns a dark-themed 3D figure.

---

## Tests

Tests live in `tests/` and use pytest:

```bash
# Run all tests
pytest tests/ -v

# Run with coverage
pytest tests/ --cov=kinematics --cov=gcode_parser --cov=trajectory_planner -v
```

Test modules cover FK/IK round-trips, G-code parsing edge cases, trajectory smoothness, and velocity-limit compliance.

---

## Examples

The `examples/` directory contains ready-to-run demos:

```bash
# Full pipeline (parse → IK → trajectory → control → plot)
python examples/demo_pipeline.py

# Arm visualisation only
python examples/demo_visualization.py
```

Sample G-code files included: `sample_cube.gcode` (planar 20 mm cube) and `sample_spiral.gcode` (non-planar helix showcasing the arm's 6-DOF advantage).

---

## Documentation

Interactive Jupyter notebooks in `docs/`:

```bash
jupyter notebook docs/
```

- **`dh_parameters_walkthrough.ipynb`** — Step-by-step derivation of the DH transformation matrices with inline LaTeX and live code.
- **`workspace_analysis.ipynb`** — Monte Carlo reachability study with 10 000 random configurations; XY, XZ, and 3D scatter plots of the workspace.

---

## Bill of Materials

See `BOM/BOM.md` for the full component list with specifications and prices in INR. A machine-readable version is at `BOM/bom.csv`. Estimated total build cost: ~₹17 000 (excluding optional heated bed).

---

## CAD & Mechanical Design

`CAD/README.md` contains detailed part descriptions, recommended dimensions, material choices, and links to open-source 6-DOF arm references (AR4, Niryo, Thor, BCN3D Moveo). Placeholder directories `CAD/STEP/` and `CAD/STL/` are ready for exported models.

---

## Presentation Website

The project includes a static HTML presentation site (`index.html` + `pages/`) built with Tailwind CSS, Chart.js, and Font Awesome. Open `index.html` in a browser to view the project overview, methodology, and results.

---

## Contributing

1. Fork the repo and create a feature branch.
2. Write tests for any new functionality.
3. Run `pytest tests/ -v` and ensure all tests pass.
4. Submit a pull request with a clear description.

---

## License

This project is released under the [MIT License](LICENSE).

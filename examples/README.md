# Examples Directory

This directory contains sample G-code files and demo scripts demonstrating the full pipeline of the 6-axis robotic arm control system.

## Files

### G-code Files

#### `sample_cube.gcode`
A realistic G-code file for a 20mm cube with 3 layers (0.3mm layer height).

**Contents:**
- PrusaSlicer-style header with filament and printer settings
- Temperature and fan commands (M104, M140, M106)
- Homing and extrusion reset (G28, G92)
- Multiple layers with outer and inner perimeters
- Travel moves (G0) and extrusion moves (G1)
- ~60 lines demonstrating typical planar 3D printer output

**Use cases:**
- Test the GCodeParser
- Validate Cartesian waypoint extraction
- Debug G-code to joint trajectory conversion

#### `sample_spiral.gcode`
A non-planar helical spiral toolpath (R=10mm, height=10mm, 30 points).

**Contents:**
- Helical pattern: X = R·cos(t), Y = R·sin(t), Z increases linearly
- Demonstrates the advantage of a robotic arm (non-planar printing)
- 31 extrusion moves forming a complete spiral
- Minimal temperature/homing overhead

**Use cases:**
- Showcase non-planar printing capabilities
- Test inverse kinematics on complex 3D paths
- Demonstrate trajectory smoothing

### Demo Scripts

#### `demo_pipeline.py`
Full end-to-end pipeline demonstration.

**Workflow:**
1. **Parse G-code** — Extract Cartesian waypoints from `sample_cube.gcode`
2. **Print summary** — Display waypoint count, bounding box, extrusion range
3. **Inverse kinematics** — Convert Cartesian waypoints to joint angles
4. **Interpolation** — Smooth the joint trajectory
5. **Velocity profile** — Apply trapezoidal acceleration limits
6. **Controller execution** — Send trajectory to MockController
7. **Visualization** — Plot the arm at first and last poses
8. **Save output** — Write visualization to `output/pipeline_demo.png`

**Usage:**
```bash
python examples/demo_pipeline.py
```

**Output:**
- Console logs showing parsing results, IK conversion, and trajectory stats
- `examples/output/pipeline_demo.png` with 3 subplots:
  - Arm at start position
  - Arm at end position
  - Toolpath in Cartesian space (top view)

**Dependencies:**
- `gcode_parser` — Parse G-code and extract waypoints
- `trajectory_planner` — IK, interpolation, velocity profiles
- `control` — MockController for simulated execution
- `kinematics` — Forward kinematics and arm visualization
- `matplotlib`, `numpy` — Plotting and numerical computation

#### `demo_visualization.py`
Standalone arm kinematics and visualization demo.

**What it does:**
1. Create an ArmSimulator
2. Define 5 joint configurations (home, elbow up, reach, rotated, complex)
3. Plot the arm in each configuration
4. Display TCP (tool center point) position for each pose
5. Generate a workspace slice via Monte Carlo sampling (1000 random poses)
6. Save the 6-panel figure to `output/arm_visualization.png`

**Usage:**
```bash
python examples/demo_visualization.py
```

**Output:**
- `examples/output/arm_visualization.png` with 6 subplots:
  - 5 arm configurations with TCP markers
  - 1 workspace slice (X-Y top view, colored by Z)

**Dependencies:**
- `kinematics` — Forward kinematics and plotting
- `simulation` — ArmSimulator
- `matplotlib`, `numpy` — Plotting and numerics

## Running the Examples

### Prerequisites
Ensure the main project is importable:
```bash
cd /path/to/PBL_project
export PYTHONPATH=".:$PYTHONPATH"
```

Or run from the project root:
```bash
python examples/demo_pipeline.py
python examples/demo_visualization.py
```

### Directory Structure
```
examples/
├── README.md                      # This file
├── sample_cube.gcode              # 3-layer cube G-code
├── sample_spiral.gcode            # Non-planar spiral G-code
├── demo_pipeline.py               # Full pipeline demo
├── demo_visualization.py          # Kinematics visualization demo
└── output/                        # Generated visualizations
    ├── pipeline_demo.png          # Output from demo_pipeline.py
    └── arm_visualization.png      # Output from demo_visualization.py
```

## Extending the Examples

### Add Your Own G-code
1. Place your G-code file in `examples/`
2. Modify the `gcode_file` path in `demo_pipeline.py`
3. Run the script to parse and visualize

### Custom Joint Configurations
Edit `demo_visualization.py` to add more configurations:
```python
config_custom = np.array([angle1, angle2, angle3, angle4, angle5, angle6])
configs.append(("My Pose", config_custom))
```

### Trajectory Animation
Use `kinematics.animate_arm()` to create animated arm movements:
```python
from kinematics import animate_arm
animate_arm(joint_trajectory, interval=50)
```

## Troubleshooting

**ModuleNotFoundError: No module named 'gcode_parser'**
- Ensure PYTHONPATH is set or you're running from the project root

**Empty waypoints from G-code**
- Check that the G-code file has valid G1 extrusion moves with X, Y, Z coordinates

**Inverse kinematics failure**
- Waypoints may be outside the arm's workspace
- Check the bounding box printed in demo_pipeline.py

**Visualization not saving**
- Ensure `examples/output/` directory is writable
- Check disk space and permissions

## G-code Format Notes

The examples use standard RepRap G-code:
- **G0** — Rapid move (travel, no extrusion)
- **G1** — Linear move (with optional extrusion via E parameter)
- **G28** — Home all axes
- **G92** — Set position (e.g., G92 E0 resets extrusion counter)
- **M104** — Set nozzle temperature (°C)
- **M140** — Set bed temperature (°C)
- **M106** — Set fan speed (0–255)
- **F** — Feedrate (mm/min)
- **E** — Extrusion distance (mm)

For more details, see the [RepRap Wiki](https://reprap.org/wiki/G-code).

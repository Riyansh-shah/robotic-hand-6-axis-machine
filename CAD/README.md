# CAD Design Documentation — 6-Axis Robotic Arm

This directory contains all CAD models, design specifications, and manufacturing files for the 6-DOF desktop robotic arm optimized for non-planar 3D printing. All parametric source models are stored in `STEP/` and printable mesh exports in `STL/`.

---

## Parts List and Dimensions

### Base Assembly

#### Base Plate
- **Material:** Aluminum 6061-T6
- **Dimensions:** 200×200×10 mm
- **Purpose:** Rigid foundation for the entire arm
- **Features:**
  - Four M5 corner mounting holes for vibration isolation feet
  - Central circular bore (Ø28 mm) for bearing J1 mount
  - Four M4 clearance holes around center for turret bolt-down
  - Bottom surface: optionally anodized for durability

#### Base Turret (Joint 1 Housing)
- **Material:** 3D-printed PETG or aluminum casting
- **Dimensions:** ~80 mm diameter, 60 mm tall
- **Purpose:** Rotating frame that enables continuous yaw (Z-axis rotation)
- **Features:**
  - Integrates a planetary gearbox (10:1 reduction ratio recommended) to connect motor to bearing
  - Bearing type: 6001ZZ (deep groove ball bearing) supporting radial and axial loads
  - Motor mount flange for NEMA23 stepper (~57 mm × 57 mm bolt pattern)
  - Cable routing channels through turret wall for pneumatic/electrical lines
  - Snap-fit or screw-together design for ease of assembly and maintenance

---

### Arm Links (Serial Chain)

#### Shoulder Link (L1)
- **Material:** Aluminum 7075-T73 (or 6061 for cost reduction)
- **Length:** 200 mm (matching DH parameter a₂)
- **Cross-section:** Rectangular box tube, 40×30 mm outer, 4 mm wall thickness
- **Mass Budget:** ~400 g
- **Purpose:** Connects J2 (pitch) to J3 (roll)
- **Features:**
  - Motor mount flanges at both ends for NEMA23 motors
  - Four M4 tapped holes on each end for secure coupling
  - Internal cable channel (10 mm diameter) running full length for Bowden tube routing
  - Lightening holes (Ø8 mm, spaced 50 mm) along the length to reduce mass
  - Chamfered edges for safety

#### Elbow Link (L2)
- **Material:** Aluminum 7075-T73 (hollow profile)
- **Length:** 150 mm (matching DH parameter a₃)
- **Cross-section:** Rectangular box tube, 30×25 mm outer, 3 mm wall thickness
- **Mass Budget:** ~250 g
- **Purpose:** Connects J3 (roll) to J4 (pitch)
- **Features:**
  - Similar motor mount design as L1 (NEMA17 or NEMA23, depending on load analysis)
  - Cable routing channel through center (8 mm diameter)
  - Lightening holes (Ø6 mm) to minimize inertia
  - Compact profile reduces arm footprint when fully extended

#### Wrist Link 1 (L3)
- **Material:** Aluminum 6061-T6 or titanium (cost permitting)
- **Length:** 80 mm (matching DH parameter d₄)
- **Profile:** Cylindrical, Ø35 mm outer diameter
- **Mass Budget:** ~150 g
- **Purpose:** Connects J4 (pitch) to J5 (roll)
- **Features:**
  - Hollow core (Ø20 mm) for weight reduction and cable routing
  - Integrated bearing journals at each end (6001ZZ or 6000ZZ)
  - Four M3 tapped holes for joint housing attachment

#### Wrist Link 2 (L4)
- **Material:** Aluminum or 3D-printed PETG
- **Dimensions:** Compact cylindrical, ~40 mm length, Ø28 mm diameter
- **Mass Budget:** ~80 g
- **Purpose:** Connects J5 (roll) to J6 (yaw)
- **Features:**
  - Precision bearing mount for J6 assembly
  - Flange interface for end-effector mounting (ISO 9409-1-50-4-M6 or custom)
  - Cable and cooling line routing holes

---

### End-Effector / Extruder Mount

#### Hotend Flange Adapter
- **Material:** 3D-printed nylon-CF (carbon-filled) or aluminum
- **Purpose:** Mounts an E3D V6 hotend (or Volcano variant) for 3D printing capability
- **Nozzle Stand-off:** 50 mm from J6 axis to nozzle tip (matching DH parameter d₆)
- **Features:**
  - M10×1 threaded bore for V6 hotend insertion
  - Bowden tube fitting (M10×1) with 1/4″ pneumatic quick-disconnect
  - Part cooling fan duct: dual 30×30×10 mm axial fan mounts (120° apart)
  - Temperature sensor bead pocket (NTC 100k thermistor 3mm diameter)
  - Two M3 mounting bosses for attachment to L4 (with 3 mm clearance for motion)

#### Part Cooling System
- **Fans:** Two 30×30×10 mm DC brushless fans, 24V, ~8 CFM each
- **Duct:** 3D-printed PETG, Y-shaped, directs air toward build area
- **Angle:** Dual ducts angled ±45° from nozzle axis for even cooling

#### Bowden Tube Routing
- **Tube Diameter:** 4 mm OD (2.4 mm ID recommended)
- **Quick-Disconnect:** 1/4″ NPT pneumatic connector at J6 for easy hotend swap
- **Friction Fittings:** PTFE tube guide at each joint housing inlet

---

### Joint Housings

#### Housing Design Principles
- **Type:** Modular 3D-printed enclosures, snap-fit or screw-together
- **Material:** PETG (better thermal stability than PLA)
- **Wall Thickness:** 3–4 mm for rigidity
- **Purpose:** Enclose motors, gearboxes, and bearings; provide cable routing infrastructure

#### Common Features (J1–J6)
- **Bearing Pockets:** Precision bore pockets (±0.1 mm) for 6001ZZ or equivalent
- **Motor Clamping:** Snap-fit tabs or four M3 screws per housing face
- **Gearbox Integration:** Planetary gearbox (8:1 to 10:1 reduction) pressed into housing bore
- **Cable Egress Channels:** 4 mm diameter holes with molded strain relief for motor leads and sensor lines
- **Maintenance Access:** Snap-fit or screw-cover design to allow bearing/gearbox replacement

#### Specific Housings
- **J1 (Yaw):** Integrated into base turret; supports largest loads (reaction torques from arm weight)
- **J2–J5 (Pitch/Roll):** Compact, stackable design; all use identical motor (NEMA23 recommended) and gearbox
- **J6 (Wrist Yaw):** Smaller housing for NEMA17 motor; interfaces directly with end-effector flange

---

## Design Guidelines

### Material Selection

| Component | Primary Material | Alternate | Rationale |
|-----------|------------------|-----------|-----------|
| Base plate | Aluminum 6061 | Cast iron | High stiffness, lightweight |
| Links (L1–L2) | Aluminum 7075 | Carbon tube | Strength-to-weight ratio |
| Links (L3–L4) | Aluminum 6061 | Titanium | Wrist precision; minimal inertia |
| Joint housings | PETG (printed) | Nylon-CF | Low cost, good dimensional stability |
| End-effector | Nylon-CF (printed) | Aluminum | High stiffness-to-weight for hotend mounting |

### Mass Budget

**Target Total Arm Mass:** ~2.0 kg (excluding base)

| Assembly | Target Mass (g) | Notes |
|----------|-----------------|-------|
| Base + J1 | 600 | Heaviest; includes motor and gearbox |
| L1 + J2 + J3 | 400 | Shoulder region; high acceleration load |
| L2 + J4 + J5 | 300 | Elbow; mid-range inertia |
| L3 + L4 + J6 | 250 | Wrist; precision-critical |
| End-effector + hotend | 350 | E3D V6 (~125g) + duct + flange |
| **Total** | **~2.0 kg** | Achievable with aluminum links and PETG housings |

Lightening holes, hollow link cores, and careful material selection keep total arm mass below 2 kg, enabling faster accelerations and lower motor torque requirements.

### Bearing Selection

**Standard Bearing:** 6001ZZ (deep groove ball bearing)
- **Bore:** 12 mm
- **OD:** 28 mm
- **Width:** 8 mm
- **Load Rating:** C_r ≈ 7,630 N (radial); useful for low-speed, high-torque joints

**Lubrication:** Grease-packed (ZZ shields); sealed for clean-room compatibility.

**Preload:** Light preload (~5 N axial) via spring washer between bearing and joint housing to eliminate backlash.

### Cable Routing & Pneumatics

#### Motor Cables
- **Gauge:** 18 AWG (1 mm²) twisted pair per phase (three pairs per motor)
- **Shielding:** Foil wrap for EMI protection
- **Strain Relief:** Molded boots at motor and control board connectors

#### Pneumatic Lines (Optional)
- **Tube:** 4 mm OD Polyurethane or nylon
- **Pressure:** 5–6 bar (standard industrial air supply)
- **Use Case:** Future pneumatic gripper or tool-changer integration
- **Routing:** Parallel to motor cables through link channels; quick-disconnect at wrist

#### Sensor Cables
- **Limit switches:** 2-conductor, 24 AWG
- **Thermistor (hotend):** 3-conductor, 24 AWG shielded
- **Cable dressing:** Nylon spiral wrap; tie points every 50 mm

### Assembly Order

1. **Prepare base:** Mill mounting holes, install bearing on base plate, test fit turret
2. **Assemble J1 (base turret):**
   - Press 6001ZZ bearing into turret bore
   - Slide planetary gearbox onto input shaft
   - Bolt NEMA23 motor to turret flange
   - Mount turret assembly to base plate (four M4 bolts)
3. **Assemble L1 + J2/J3:**
   - Mount L1 to J2 motor (via coupling)
   - Attach motor housing and gearbox to J2 flange
   - Thread motor cables and Bowden tube through L1 channel
4. **Assemble L2 + J4/J5:** Mirror of step 3
5. **Assemble wrist (L3, L4, J6):**
   - Mount L3 to L2 via J4 assembly
   - Mount L4 to L3 via J5 assembly
   - Attach compact J6 housing to L4
6. **Mount end-effector:**
   - Attach hotend flange to J6 interface
   - Route Bowden tube and cooling fan leads to flange connector
   - Secure cooling fans into duct mounting points
7. **Wiring & pneumatics:**
   - Solder motor leads to control board connectors; heat-shrink joints
   - Route all cables down through link channels with spiral wrap
   - Connect thermistor to ADC input on controller
   - (If equipped) connect pneumatic quick-disconnects at wrist

---

## Recommended CAD Software

### Fusion 360 (Recommended for education)
- **Cost:** Free for personal, academic, and startup use
- **Strengths:** Native parametric design, cloud collaboration, excellent animation and rendering
- **Drawback:** Requires internet and Autodesk account
- **Export:** Native `.f3d` format; exports STEP, IGES, STL
- **Learning Resources:** Abundant YouTube tutorials and Autodesk community forums

### SolidWorks (if university license available)
- **Cost:** Typically provided by universities (check with your institution)
- **Strengths:** Industry-standard, extremely robust parametric engine, superior surface modeling
- **Drawback:** Steep learning curve; resource-intensive on older machines
- **Export:** Native `.sldprt` / `.sldasm`; exports STEP, IGES, STL, PDF drawings
- **Advantage for PBL:** Can generate manufacturing drawings with GD&T annotations

### FreeCAD (open-source alternative)
- **Cost:** Free and open-source
- **Strengths:** No licensing restrictions; excellent for educational projects; strong community
- **Drawback:** Steeper learning curve than Fusion 360; assembly design can be slower
- **Export:** Native `.FCStd`; exports STEP, IGES, STL, OBJ
- **Best For:** Long-term project sustainability; no software cost; full design transparency

### STEP Format as Design Interchange
Regardless of primary software, always **export and commit STEP files** to version control. STEP is an ISO standard (ISO 10303) that preserves parametric intent across most CAD packages, enabling team members to work in different software and collaborate seamlessly.

---

## Reference Designs & Open-Source Projects

Learning from proven designs accelerates development. Below are active, well-documented open-source 6-DOF arm projects:

### 1. **AR4 Robot Arm** (Chris Annin)
- **Repository:** https://github.com/Chris-Annin/AR4
- **Design:** Budget-friendly (~$1,500–$2,500), uses stepper motors and 3D printing
- **Notable:** Desktop scale, Bowden tube extruder integration
- **CAD:** FreeCAD models included; STEP and STL exports available
- **Documentation:** Comprehensive assembly guides and electrical schematics

### 2. **Niryo One / Niryo Ned** (Niryo Robotics)
- **Repository:** https://github.com/NiryoRobotics
- **Design:** Professional 6-DOF arms with integrated vision and gripper support
- **Notable:** Production-grade design; ROS-compatible software stack
- **CAD:** Complete STEP models, mechanical drawings, BoM
- **Documentation:** Industrial-quality assembly and commissioning manuals

### 3. **Thor Robotic Arm** (AngelLM)
- **Repository:** https://github.com/AngelLM/Thor
- **Design:** Lightweight, 3D-printed links; NEMA17/23 stepper driven
- **Notable:** Inspired by UR robot aesthetics; compact wrist design
- **CAD:** Fusion 360 and STEP files; STL for 3D printing
- **Community:** Active contributor base; good for design variations and improvements

### 4. **BCN3D Moveo** (BCN3D Technologies)
- **Repository:** https://github.com/BCN3D/BCN3D-Moveo
- **Design:** Fully 3D-printed structure; educational focus
- **Notable:** Minimal material cost; modular joint design for easy customization
- **CAD:** Complete FreeCAD source files; manufacturing guides
- **Documentation:** Step-by-step 3D printing and assembly instructions

### 5. **KAUDA Robotic Arm** (Thingiverse / Community)
- **Resource:** https://www.thingiverse.com (search "6-DOF robotic arm" or "KAUDA")
- **Design:** Community-driven variations; excellent for studying mechanical joints
- **Notable:** Multiple versions with different motor/controller combinations
- **Files:** STL-focused; STEP models contributed by community members
- **Advantage:** Low barrier to entry; remix-friendly open licenses

---

## Directory Structure

### `CAD/STEP/`
Contains all parametric source models in **STEP format** (ISO 10303-21 standard).

**Why STEP:**
- Universal interchange format; readable by any modern CAD package
- Preserves part geometry, assemblies, and design intent
- Version-control friendly (text-based geometric data)
- Industry standard for manufacturing hand-offs

**Naming Convention:**
```
Base_Plate.STEP
Base_Turret_J1_Housing.STEP
Link_L1_Shoulder.STEP
Link_L2_Elbow.STEP
Link_L3_Wrist_1.STEP
Link_L4_Wrist_2.STEP
EndEffector_Hotend_Flange.STEP
Joint_Housing_Generic.STEP
Assembly_Full_Arm.STEP          (top-level assembly)
```

**Best Practice:**
- One STEP file per discrete part
- One assembly STEP combining all parts
- Commit both native CAD files (`.f3d`, `.sldprt`, or `.FCStd`) and STEP exports
- Use descriptive names with version suffixes if iterating (e.g., `Base_Plate_v2.STEP`)

### `CAD/STL/`
Contains all mesh models in **STL format** (binary, for 3D printing).

**Why STL:**
- Universal standard for 3D printing; compatible with all slicers (Cura, PrusaSlicer, Simplify3D)
- Lightweight and easy to manage; suitable for GitHub version control (compressed archives recommended for large files)
- Enables non-CAD users to slice and print without specialized software

**Naming Convention:**
```
Base_Turret_J1_Housing.STL
Joint_Housing_J2_J3.STL
Joint_Housing_J4_J5.STL
Joint_Housing_J6.STL
EndEffector_Duct_Upper.STL
EndEffector_Duct_Lower.STL
(metal parts typically not exported to STL; machining drawings used instead)
```

**Export Workflow:**
1. In CAD software: Export each 3D-printable part as binary STL
2. Verify geometry in free viewer (e.g., Fusion 360 online viewer, Thingiverse viewer)
3. Store in `CAD/STL/` with matching naming to STEP originals
4. Update whenever STEP models change

---

## Next Steps

1. **Choose CAD software** based on team preference and university resources (Fusion 360 recommended for rapid iteration)
2. **Model base assembly first** (base plate + J1 turret) to establish reference geometry
3. **Parameterize all dimensions** using variables tied to DH parameters (a₂, a₃, d₄, d₆) to enable link length optimization
4. **Export STEP + STL for each part** and commit to this directory
5. **Create 2D manufacturing drawings** from STEP models for machinists (aluminum links) and 3D printing prep (housings)
6. **Share designs with reference project communities** for peer review and design feedback

---

**Last Updated:** 2026-04-11  
**Project:** PBL 2026 — 6-Axis Robotic Arm for Non-Planar 3D Printing  
**Institution:** Manipal University Jaipur, Department of CSE

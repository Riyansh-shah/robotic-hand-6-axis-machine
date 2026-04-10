# Bill of Materials: 6-Axis Robotic Arm 3D Printer

**Project:** Desktop-scale 6-DOF robotic 3D printer with non-planar printing capability  
**Prices:** Approximate INR values as of April 2026  
**Platform:** Manipal University Jaipur, Department of CSE

---

## Components Table

| # | Component | Specification | Qty | Unit Price (INR) | Total (INR) | Supplier/Notes |
|---|-----------|---------------|-----|------------------|-------------|----------------|
| 1 | NEMA 17 Stepper Motor | 42mm, 1.8°/step, 0.4 Nm | 6 | 350 | 2,100 | Standard bipolar stepper; joints 1–6 actuation |
| 2 | Planetary Gearbox | 1:5 ratio (joints 1–3) | 3 | 800 | 2,400 | Reduces speed, increases torque for shoulder/elbow |
| 3 | Planetary Gearbox | 1:3 ratio (joints 4–6) | 3 | 800 | 2,400 | Wrist joint reduction; compact form factor |
| 4 | TMC2209 Stepper Driver | 2A RMS, SPI control | 6 | 250 | 1,500 | Silent operation; current limiting; one per motor |
| 5 | Arduino Mega 2560 | ATmega2560, 16 MHz | 1 | 650 | 650 | Main controller; alternative: STM32F407 (~₹1200) |
| 6 | RAMPS 1.4 Shield | Stepper driver breakout | 1 | 400 | 400 | Simplifies wiring; alternative: custom PCB |
| 7 | E3D V6 Hotend | All-metal, 1.75mm filament | 1 | 1,500 | 1,500 | Direct mount on end-effector; 24V heater |
| 8 | Bowden Extruder | BMG clone, 1.75mm, geared | 1 | 600 | 600 | Remote feeder; reduces arm inertia |
| 9 | PTFE Bowden Tube | 2m length, 1.75mm inner diameter | 1 | 150 | 150 | Filament conduit from extruder to hotend |
| 10 | 24V 15A Power Supply | Regulated PSU, ATX form factor | 1 | 900 | 900 | Supplies motors, heater, control electronics |
| 11 | 6001ZZ Deep Groove Bearing | 12mm OD, 6mm bore, sealed | 12 | 60 | 720 | Joint pivot points; two per primary joint |
| 12 | 5mm–8mm Shaft Coupling | Flexible jaw, aluminum | 6 | 120 | 720 | Motor-to-gearbox connections; vibration isolation |
| 13 | 2020 Aluminum Extrusion | 300mm lengths, anodized | 8 | 180 | 1,440 | Structural frame; joints and mounting rails |
| 14 | GT2 Timing Belt + Pulleys | 6mm wide, 20-tooth pulleys | 1 | 300 | 300 | Optional belt drive for non-stepper axes |
| 15 | 3D-Printed Joint Housings | PLA/PETG, designed parts (~500g) | 1 | 500 | 500 | Material cost; CAD designs provided in project repo |
| 16 | Limit Switches | Mechanical, NO/NC contact | 6 | 30 | 180 | Home sensing; one per joint |
| 17 | Wiring & Connectors Kit | Dupont connectors, terminal blocks, crimps | 1 | 400 | 400 | PSU cables, motor leads, sensor wiring |
| 18 | Heated Bed (Optional) | 200×200mm, 24V 300W | 1 | 800 | 800 | Improves print adhesion; optional add-on |
| 19 | PLA/PETG Filament | 1kg spool, 1.75mm diameter | 1 | 500 | 500 | Initial test prints; multiple colors available |
| 20 | Miscellaneous Hardware | M3/M5 screws, nuts, washers, standoffs, heat-shrink | 1 | 500 | 500 | Assembly consumables |
|    | **TOTAL**                | | | | **₹17,760** | **Excluding optional heated bed: ₹16,960** |

---

## Notes

- **Motor Selection:** NEMA 17 motors chosen for balance between torque and arm mass; six motors enable full 6-DOF control without redundancy.
- **Gearbox Ratios:** Shoulder and elbow joints (1–3) use 1:5 reduction for improved load-carrying capacity; wrist joints (4–6) use 1:3 for faster, more precise positioning.
- **Driver Choice:** TMC2209 offers silent operation with active current limiting and microstepping, reducing vibration during printing.
- **Control:** Arduino Mega 2560 runs custom firmware (based on standard robotics frameworks). STM32F407 recommended for higher clock speeds and floating-point performance.
- **Power:** 24V 15A supply (360W) sufficient for six motors + heater + control. Peak current draw ~12A under full acceleration.
- **Printing Head:** E3D V6 all-metal hotend avoids PTFE limitations; Bowden feeder reduces payload.
- **Frame:** 2020 extrusion provides rigid, modular structure. Aluminum anodized finish resists corrosion.
- **3D Printing:** Housings and structural brackets printed in PETG for strength and thermal stability; material cost amortized over full build.
- **Optional:** Heated bed improves PLA/PETG adhesion for large prints; not essential for basic operation.
- **Filament:** Budget includes one spool for commissioning tests; ongoing printing costs separate.
- **Contingency:** Miscellaneous row (₹500) covers unexpected fasteners, replacements, and assembly aids.

---

## Cost Summary

| Item Category | Subtotal (INR) |
|---------------|------------------|
| Motors & Drivers (6 motors + 6 drivers) | 3,600 |
| Gearboxes (6 units) | 4,800 |
| Controller & Shield | 1,050 |
| Extruder & Hotend | 2,250 |
| Power & Cooling | 900 |
| Structure (extrusion, bearings, couplings) | 2,880 |
| Sensors & Wiring | 580 |
| Filament & Materials | 1,000 |
| Miscellaneous | 500 |
| **Total (core)** | **₹17,560** |
| **Total (with optional heated bed)** | **₹18,360** |

---

## Assembly Recommendations

1. **Timeline:** Expect 40–60 hours for mechanical assembly, 20–30 hours for electrical integration, 10–20 hours for firmware tuning.
2. **Tools Required:** Hex keys, M3/M5 tap set, screwdrivers, multimeter, soldering iron, wire crimper.
3. **Sourcing:** Local suppliers (RS Components India, Robotics India, local electronics shops) typically stock >90% of items; lead times 3–5 days.
4. **Testing:** Start with zero-gravity calibration and range-of-motion tests before introducing print loads.


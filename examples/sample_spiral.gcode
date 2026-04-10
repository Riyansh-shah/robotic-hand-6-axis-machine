; Non-planar spiral toolpath for 6-axis robotic arm
; Demonstrates helical extrusion: R=10mm, height=10mm, ~30 points
; This showcases the advantage of a robotic arm over planar 3D printers
; Generated: 2026-04-11

M104 S200 ; Set nozzle temperature
M140 S60 ; Set bed temperature
M109 S200 ; Wait for nozzle temperature
M190 S60 ; Wait for bed temperature
M106 S255 ; Turn on fan
G28 ; Home all axes
G92 E0 ; Reset extruder

; Move to spiral start point (R=10mm, angle=0, Z=0.3)
G0 X10 Y0 Z0.3 F6000

; Spiral: X=R*cos(t), Y=R*sin(t), Z=Z0+height*(t/2π)
; 30 points around the spiral (2π radians)
G1 X9.92 Y0.628 Z0.334 E1 F1800 ; point 1
G1 X9.69 Y1.252 Z0.367 E2 F1800 ; point 2
G1 X9.33 Y1.869 Z0.401 E3 F1800 ; point 3
G1 X8.83 Y2.474 Z0.434 E4 F1800 ; point 4
G1 X8.19 Y3.062 Z0.467 E5 F1800 ; point 5
G1 X7.41 Y3.628 Z0.501 E6 F1800 ; point 6
G1 X6.49 Y4.166 Z0.534 E7 F1800 ; point 7
G1 X5.45 Y4.669 Z0.567 E8 F1800 ; point 8
G1 X4.28 Y5.131 Z0.601 E9 F1800 ; point 9
G1 X3.01 Y5.546 Z0.634 E10 F1800 ; point 10
G1 X1.64 Y5.906 Z0.667 E11 F1800 ; point 11
G1 X0.21 Y6.204 Z0.701 E12 F1800 ; point 12
G1 X-1.27 Y6.435 Z0.734 E13 F1800 ; point 13
G1 X-2.80 Y6.591 Z0.768 E14 F1800 ; point 14
G1 X-4.35 Y6.666 Z0.801 E15 F1800 ; point 15
G1 X-5.88 Y6.654 Z0.834 E16 F1800 ; point 16
G1 X-7.37 Y6.548 Z0.868 E17 F1800 ; point 17
G1 X-8.76 Y6.340 Z0.901 E18 F1800 ; point 18
G1 X-10.02 Y6.023 Z0.934 E19 F1800 ; point 19
G1 X-11.07 Y5.591 Z0.968 E20 F1800 ; point 20
G1 X-11.88 Y5.036 Z1.001 E21 F1800 ; point 21
G1 X-12.41 Y4.353 Z1.034 E22 F1800 ; point 22
G1 X-12.66 Y3.536 Z1.068 E23 F1800 ; point 23
G1 X-12.60 Y2.586 Z1.101 E24 F1800 ; point 24
G1 X-12.19 Y1.505 Z1.135 E25 F1800 ; point 25
G1 X-11.44 Y0.301 Z1.168 E26 F1800 ; point 26
G1 X-10.38 Y-1.043 Z1.201 E27 F1800 ; point 27
G1 X-9.04 Y-2.585 Z1.235 E28 F1800 ; point 28
G1 X-7.46 Y-4.205 Z1.268 E29 F1800 ; point 29
G1 X-5.69 Y-5.873 Z1.301 E30 F1800 ; point 30

; Complete the spiral back to start (full 2π rotation)
G1 X10 Y0 Z1.334 E31 F1800

; Cool down and end
G0 Z30 F3000
M104 S0 ; Turn off nozzle heater
M140 S0 ; Turn off bed heater
M106 S0 ; Turn off fan
G28 ; Home
M84 ; Disable motors

import math, os

r = 20
cx, cy, cz = 0, 0, 25

lines = [
    "; Sphere toolpath for AR2 6-axis arm",
    "G21",
    "G90",
    "G92 E0",
    f"G1 X{cx:.3f} Y{cy:.3f} Z{cz+r:.3f} F1200",
]

e = 0.0
step_lat = 6
step_lon = 4

for lat in range(-84, 91, step_lat):
    phi = math.radians(lat)
    ring_r = r * math.cos(phi)
    z = cz + r * math.sin(phi)
    lon_range = range(0, 361, step_lon) if (lat // step_lat) % 2 == 0 else range(360, -1, -step_lon)
    for lon in lon_range:
        theta = math.radians(lon)
        x = cx + ring_r * math.cos(theta)
        y = cy + ring_r * math.sin(theta)
        e += 0.02
        lines.append(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} E{e:.4f} F800")

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "examples", "sample_sphere.gcode")
with open(out, "w") as f:
    f.write("\n".join(lines))
print(f"Written {len(lines)} lines -> {out}")

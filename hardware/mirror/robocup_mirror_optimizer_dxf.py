"""
Catadioptric mirror optimizer (RoboCup Junior)

Features:
- 320x240 camera
- H FOV 65.9°, V FOV 51.8°
- Vertical camera
- Rotationally symmetric mirror
- Diameter 75 mm
- Mirror flat top/rim 65 mm above lens (the curved tip at the center
  sits closer to the camera than the flat rim)

Optimization target:
- Reach far distance (~2616 mm)
- Bias pixel density toward near field
- Derive mirror profile from desired image-row -> ground-radius mapping

Outputs:
- mirror_profile.csv
- mirror_profile.dxf  (smooth spline profile)
- mirror_profile_points.dxf (polyline fallback)
- mirror_rays.png (ray-trace diagram: camera -> mirror -> ground)
"""

import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

VFOV = math.radians(51.8)

IMG_H = 240
MIRROR_RADIUS = 37.5
TOP_HEIGHT = 65.0  # mirror top height above the camera lens

MIRROR_TOP_HEIGHT_ABOVE_GROUND = 210.0
# Lens (camera) height above the ground, derived from the two heights above.
CAMERA_HEIGHT_ABOVE_GROUND = MIRROR_TOP_HEIGHT_ABOVE_GROUND - TOP_HEIGHT
# Ground plane height, expressed in the same coordinate system as the mirror
# profile (z = 0 at the lens).
GROUND_Z = -CAMERA_HEIGHT_ABOVE_GROUND

MAX_FIELD_RADIUS = 2616.73461

# ---------- mapping ----------
# p=0 at image center, p=1 at outermost useful row
#
# EQUIDISTANT MAPPING: each equal step in image row / FOV angle (p) is
# mapped to an equal step in ground distance. This spreads the curvature
# demand evenly across the whole mirror instead of letting it collapse
# into a near-conical (straight-line) section further out, which is what
# happens when NEAR_FIELD_WEIGHT > 1 compresses far-field rows together.
NEAR_FIELD_WEIGHT = 1.4

def desired_ground_radius(p):
    return MAX_FIELD_RADIUS * (p ** NEAR_FIELD_WEIGHT)

# ---------- inverse mirror solve ----------
# NOTE: TOP_HEIGHT (65mm) is the height of the mirror's flat TOP/rim
# (at the outer edge, r = MIRROR_RADIUS) above the lens - not the curved
# tip at the center (r = 0). The tip bulges down, closer to the camera.
# So we integrate from the outer edge inward to the tip.
rows = np.linspace(0, 1, 500)

r_mirror = MIRROR_RADIUS * rows
ground_r = desired_ground_radius(rows)

z = np.zeros_like(rows)
z[-1] = TOP_HEIGHT  # outer edge / flat rim, 65mm above the lens

for i in range(len(rows) - 2, -1, -1):
    rm = r_mirror[i]
    rf = ground_r[i]

    vin = np.array([-rm, 0.0, -z[i+1]])
    vin /= np.linalg.norm(vin)

    vout = np.array([rf-rm, 0.0, GROUND_Z - z[i+1]])
    vout /= np.linalg.norm(vout)

    n = vin + vout
    n /= np.linalg.norm(n)

    dzdr = -n[0] / n[2]

    dr = r_mirror[i] - r_mirror[i+1]
    z[i] = z[i+1] + dzdr * dr

z += TOP_HEIGHT - z[-1]

# ---------- CSV ----------
with open("mirror_profile.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["radius_mm", "height_mm"])
    for rr, zz in zip(r_mirror, z):
        w.writerow([rr, zz])

# ---------- DXF spline ----------
def write_spline_dxf(filename, x, y):
    with open(filename, "w") as f:
        f.write("0\nSECTION\n2\nENTITIES\n")

        f.write("0\nSPLINE\n")
        f.write("70\n8\n")
        f.write(f"71\n3\n")
        f.write(f"73\n{len(x)}\n")

        for _ in range(len(x)+4):
            f.write("40\n0.0\n")

        for xx, yy in zip(x, y):
            f.write(f"10\n{xx}\n20\n{yy}\n30\n0\n")

        f.write("0\nENDSEC\n0\nEOF\n")

# ---------- DXF polyline fallback ----------
def write_poly_dxf(filename, x, y):
    with open(filename, "w") as f:
        f.write("0\nSECTION\n2\nENTITIES\n")
        f.write("0\nLWPOLYLINE\n")
        f.write(f"90\n{len(x)}\n")
        for xx, yy in zip(x, y):
            f.write(f"10\n{xx}\n20\n{yy}\n")
        f.write("0\nENDSEC\n0\nEOF\n")

write_spline_dxf("mirror_profile.dxf", r_mirror, z)
write_poly_dxf("mirror_profile_points.dxf", r_mirror, z)

print("Generated DXF and CSV outputs.")

# ---------- Ray-trace visualization ----------
# Camera (lens) sits at the origin, looking straight up along +z.
# Each traced ray:
#   camera (0,0) -> mirror surface point (r_mirror, z)   [incoming]
#   mirror surface point               -> ground point (ground_r, 0)  [reflected]
#
# Rays are picked so they are evenly spaced across the camera's field of
# view (equal steps in image-row fraction p), since that's what actually
# limits how finely the mirror can resolve distance steps on the ground.
# Equal ground-distance spacing is deliberately NOT used here.

N_RAYS = 12  # >= 10 requested

ray_indices = np.linspace(1, len(rows) - 1, N_RAYS).round().astype(int)

fig, ax = plt.subplots(figsize=(16, 4.5))

# ground line (below the camera, not at the camera's height)
ax.axhline(GROUND_Z, color="gray", linewidth=1.5, zorder=1)

# vertical stand-off from ground up to the camera, for reference
ax.plot([0, 0], [GROUND_Z, 0], color="black", linewidth=1.0, zorder=2)

# mirror profile, mirrored across the axis for a full cross-section silhouette
mirror_x = np.concatenate([-r_mirror[::-1], r_mirror])
mirror_y = np.concatenate([z[::-1], z])
ax.fill_between(mirror_x, mirror_y, TOP_HEIGHT, color="0.55", alpha=0.7, zorder=2)
ax.plot(mirror_x, mirror_y, color="black", linewidth=1.5, zorder=3)

# camera marker at the lens (origin)
ax.plot(0, 0, marker="o", color="black", markersize=7, zorder=6)
ax.annotate("camera", (0, 0), textcoords="offset points", xytext=(-6, 10),
            ha="right", fontsize=8)

for i in ray_indices:
    rm = r_mirror[i]
    zm = z[i]
    rf = ground_r[i]

    # camera -> mirror
    ax.plot([0, rm], [0, zm], linestyle="--", color="dimgray", linewidth=0.9, zorder=1)
    # mirror -> ground
    ax.plot([rm, rf], [zm, GROUND_Z], linestyle="--", color="dimgray", linewidth=0.9, zorder=1)

    ax.plot(rf, GROUND_Z, marker="o", color="black", markersize=4, zorder=5)
    ax.annotate(f"{rf:.0f}", (rf, GROUND_Z), textcoords="offset points", xytext=(0, 8),
                ha="center", fontsize=7)

ax.set_xlim(-60, MAX_FIELD_RADIUS * 1.03)
ax.set_ylim(GROUND_Z - 15, TOP_HEIGHT + 15)
ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("Ground radius from mirror axis (mm)")
ax.set_ylabel("Height (mm)")
ax.set_title(f"Mirror ray-trace: camera -> mirror -> ground ({N_RAYS} rays, equal ground-distance steps)")
plt.tight_layout()
plt.savefig("mirror_rays.png", dpi=150)
plt.close(fig)

print("Generated mirror_rays.png")
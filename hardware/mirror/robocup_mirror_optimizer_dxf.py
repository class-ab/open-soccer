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

# ---------- camera field of view limit ----------
# The camera's real field of view limits how far off-axis (from straight up)
# any ray leaving the lens can be. 55 degrees is the FULL angular extent on
# the limiting side; half of that, from the optical axis (straight up) out
# to the edge of frame, is the steepest incidence angle the camera can
# actually achieve.
CAMERA_FOV_DEG = 55.0
THETA_MAX = math.radians(CAMERA_FOV_DEG / 2.0)

# The angle needed to reach the mirror's physical outer rim (r=MIRROR_RADIUS,
# z=TOP_HEIGHT) in a straight line from the lens is roughly atan(37.5/65) =
# ~30 degrees - MORE than THETA_MAX (27.5 degrees). That's the cutoff: the
# camera's last usable image row never reaches the mirror's physical edge;
# a thin outer ring of the mirror blank sits outside the camera's view.

# ---------- mapping ----------
# Independent variable is now the camera's own incidence angle theta
# (0 = straight up, at the mirror tip; THETA_MAX = the camera's real edge
# of frame). EQUIDISTANT MAPPING: equal steps in theta map to equal steps
# in ground distance, and the max range is reached exactly at THETA_MAX -
# the camera's actual usable edge - not at the mirror's physical rim.
def desired_ground_radius(theta):
    return MAX_FIELD_RADIUS * (theta / THETA_MAX)

# ---------- inverse mirror solve ----------
# NOTE: TOP_HEIGHT (65mm) is the height of the mirror's flat TOP/rim, a
# fixed physical/mounting fact (at the outer edge, r = MIRROR_RADIUS) above
# the lens - not the curved tip at the center (r = 0). The tip bulges down,
# closer to the camera.
#
# We march outward in theta from the tip (theta=0, r=0) using the
# reflection law to get the local surface slope at each point, then
# intersect that tangent line with the next ray-line (through the lens at
# the next theta) to advance. Because the physical rim position (r=37.5,
# z=65) is a fixed mounting constraint - not something on the theta=THETA_MAX
# ray - we don't know the tip height in advance. We solve for it with a
# shooting method: try a tip height, march out past THETA_MAX and past the
# camera's usable range (continuing the same reflection law) until the
# curve crosses r = MIRROR_RADIUS, and adjust the tip height until the
# curve is at z = TOP_HEIGHT exactly when it crosses that radius.

N_POINTS = 4000
THETA_SEARCH_MAX = math.radians(45.0)  # generous upper bound, past the rim


def trace_profile(z_tip, theta_stop, n_points=N_POINTS):
    thetas = np.linspace(0.0, theta_stop, n_points)
    r = np.zeros(n_points)
    z = np.zeros(n_points)
    z[0] = z_tip

    for i in range(1, n_points):
        theta_prev, r_prev, z_prev = thetas[i-1], r[i-1], z[i-1]
        rf_prev = desired_ground_radius(theta_prev)

        vin = np.array([-r_prev, 0.0, -z_prev])
        vin_norm = np.linalg.norm(vin)
        vin = vin / vin_norm if vin_norm > 0 else np.array([0.0, 0.0, -1.0])

        vout = np.array([rf_prev - r_prev, 0.0, GROUND_Z - z_prev])
        vout /= np.linalg.norm(vout)

        n = vin + vout
        n /= np.linalg.norm(n)
        m = -n[0] / n[2]  # local tangent slope dz/dr at (r_prev, z_prev)

        theta_new = thetas[i]
        T = math.tan(theta_new)
        r_new = T * (z_prev - m * r_prev) / (1 - m * T)
        z_new = r_new / T if T != 0 else z_prev

        r[i] = r_new
        z[i] = z_new

    return thetas, r, z


def z_at_physical_rim(z_tip, theta_stop=THETA_SEARCH_MAX, n_points=N_POINTS):
    thetas, r, z = trace_profile(z_tip, theta_stop, n_points)
    idx = np.searchsorted(r, MIRROR_RADIUS)
    if idx == 0 or idx >= len(r):
        return None
    r0, r1 = r[idx-1], r[idx]
    z0, z1 = z[idx-1], z[idx]
    frac = (MIRROR_RADIUS - r0) / (r1 - r0)
    return z0 + frac * (z1 - z0)


# Bisection on tip height so the curve passes through the known rim point.
lo, hi = 21.0, TOP_HEIGHT - 0.01
f_lo = z_at_physical_rim(lo) - TOP_HEIGHT
f_hi = z_at_physical_rim(hi) - TOP_HEIGHT
for _ in range(60):
    mid = (lo + hi) / 2.0
    f_mid = z_at_physical_rim(mid) - TOP_HEIGHT
    if (f_mid > 0) == (f_lo > 0):
        lo, f_lo = mid, f_mid
    else:
        hi, f_hi = mid, f_mid
TIP_HEIGHT = mid

# Final profile, traced out to (and slightly past) the physical rim so we
# can trim exactly at r = MIRROR_RADIUS.
thetas, r_full, z_full = trace_profile(TIP_HEIGHT, THETA_SEARCH_MAX)
edge_idx = np.searchsorted(r_full, MIRROR_RADIUS)
r0, r1 = r_full[edge_idx-1], r_full[edge_idx]
z0, z1 = z_full[edge_idx-1], z_full[edge_idx]
frac = (MIRROR_RADIUS - r0) / (r1 - r0)
theta_edge = thetas[edge_idx-1] + frac * (thetas[edge_idx] - thetas[edge_idx-1])

r_mirror = np.concatenate([r_full[:edge_idx], [MIRROR_RADIUS]])
z = np.concatenate([z_full[:edge_idx], [TOP_HEIGHT]])
thetas_used = np.concatenate([thetas[:edge_idx], [theta_edge]])
ground_r = desired_ground_radius(thetas_used)

# Index of the last camera-visible row (theta <= THETA_MAX); everything
# beyond this in the arrays above is mirror material the camera can't see.
fov_cutoff_idx = int(np.searchsorted(thetas_used, THETA_MAX))

print(f"Camera max incidence angle: {math.degrees(THETA_MAX):.2f} deg")
print(f"Angle needed to reach physical rim: {math.degrees(theta_edge):.2f} deg")
print(f"Mirror radius actually used by camera: {r_mirror[fov_cutoff_idx]:.2f} mm "
      f"of {MIRROR_RADIUS} mm physical radius")

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
# Rays are picked so they are evenly spaced across the camera's actual
# field of view (equal steps in incidence angle theta, theta in [0, THETA_MAX]),
# since that's what actually limits how finely the mirror can resolve
# distance steps on the ground. The last ray (at THETA_MAX) is the true
# edge of the image - it lands short of the mirror's physical rim.

N_RAYS = 12  # >= 10 requested

ray_thetas = np.linspace(0.0, THETA_MAX, N_RAYS)
ray_indices = np.searchsorted(thetas_used, ray_thetas)
ray_indices = np.clip(ray_indices, 0, len(thetas_used) - 1)

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

# highlight the cutoff sliver of mirror the camera can't see
ax.plot(r_mirror[fov_cutoff_idx:], z[fov_cutoff_idx:], color="red", linewidth=2.5, zorder=4)
ax.plot(-r_mirror[fov_cutoff_idx:], z[fov_cutoff_idx:], color="red", linewidth=2.5, zorder=4)

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
ax.set_title(f"Mirror ray-trace: camera -> mirror -> ground ({N_RAYS} rays, equal steps in camera FOV angle; "
             f"red = mirror ring outside the {CAMERA_FOV_DEG:.0f} deg FOV)")
plt.tight_layout()
plt.savefig("mirror_rays.png", dpi=150)
plt.close(fig)

print("Generated mirror_rays.png")

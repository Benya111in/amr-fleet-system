"""rev5 (2026-09-22) — residual consistency fixes found while resuming the revision of perception-tracking.md.
E2. camera 3D ground-plane RMS per range band INCLUDING the iid depth-median term (rev4 E omitted it).
G2. LiDAR lateral variance split: rev4 G 'lateral sd' is the TOTAL (quantisation + shape); the brief's
    sigma_perp^2 = s^2/24 + sigma_lat^2 must use the SHAPE part only -> sigma_lat = sqrt(sd_tot^2 - s^2/24).
S.  RGB-depth skew pixel term: max shift vs uniform-skew sigma.
W.  bound on the line-of-sight rotation rate |omega_LOS| <= v_max(D)/D from robot_params.yaml safety limits.
R.  required confirmed-track range d_req = (v_r + v_o)(tau_warn + l) with the per-speed latency budgets.
"""
import math
import numpy as np

rng = np.random.default_rng(7)
P = print

# ---------------- common (config/sensors.yaml, sensor_calibration.md 2.2) ----------------
fx = 337.2098
def sig_d(Z):
    return math.sqrt(0.005 ** 2 + (0.002 * Z * Z) ** 2)

# ---------------- E2. camera 3D RMS per band with the iid median term ----------------
P("E2. camera ground-plane RMS (base frame), kappa=0.05, surface->centre offset corrected, central-50% ROI median")
kappa = 0.05
objs = {  # name: (visible width L [m], visible height Hh [m], sigma_delta [m])
    "medium box": (0.50, 0.30, 0.034),
    "person": (0.45, 1.70, 0.040),
}
for name, (L, Hh, sdel) in objs.items():
    for band in ((1, 3), (3, 6), (6, 10)):
        Zs = np.linspace(band[0], band[1], 81)
        e_sim, e_cons, e_med = [], [], []
        for Z in Zs:
            w, h = fx * L / Z, fx * Hh / Z
            m = max(1.0, 0.25 * w * h)
            s_med = math.sqrt(math.pi / 2) * sig_d(Z) / math.sqrt(m)     # iid pixels (simulation)
            sx = kappa * L
            e_sim.append(sx ** 2 + sdel ** 2 + s_med ** 2)
            e_cons.append(sx ** 2 + sdel ** 2 + sig_d(Z) ** 2)            # fully correlated pixels (real, conservative)
            e_med.append(s_med)
        P(f"   {name:10s} {band[0]:>2}-{band[1]:<2} m: RMS sim {math.sqrt(np.mean(e_sim))*100:4.1f} cm "
          f"(max in band {math.sqrt(max(e_sim))*100:4.1f}) | correlated {math.sqrt(np.mean(e_cons))*100:4.1f} cm "
          f"| iid-median sigma at far edge {e_med[-1]*100:.2f} cm")

# map frame: add localisation 0.05 m (spec straight-line) and yaw lever arm sigma_theta 0.01 rad at band far edge
P("   map-frame RMS = sqrt(base^2 + 0.05^2 + (Z*0.01)^2) at band far edge (medium box / person, sim):")
for band in ((1, 3), (3, 6), (6, 10)):
    row = []
    for name, (L, Hh, sdel) in objs.items():
        Z = band[1]; w, h = fx * L / Z, fx * Hh / Z; m = 0.25 * w * h
        base2 = (kappa * L) ** 2 + sdel ** 2 + (math.pi / 2) * sig_d(Z) ** 2 / m
        row.append(math.sqrt(base2 + 0.05 ** 2 + (Z * 0.01) ** 2))
    P(f"      {band[0]}-{band[1]} m: {row[0]*100:.1f} / {row[1]*100:.1f} cm")

# ---------------- G2. lateral variance split by 2D ray casting ----------------
dphi = math.radians(0.5); sig_r = 0.03

def ray_circle(dx, dy, cx, cy, r):
    b = -(cx * dx + cy * dy); c = cx ** 2 + cy ** 2 - r * r; disc = b * b - c
    t = -b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc >= 0) & (t > 0), t, np.inf)

def ray_rect(dx, dy, cx, cy, a, b, yaw):
    cth, sth = math.cos(-yaw), math.sin(-yaw)
    lx = cth * (-cx) - sth * (-cy); ly = sth * (-cx) + cth * (-cy)
    ldx = cth * dx - sth * dy; ldy = sth * dx + cth * dy
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-a - lx) / ldx; t2 = (a - lx) / ldx; t3 = (-b - ly) / ldy; t4 = (b - ly) / ldy
    tmin = np.maximum(np.minimum(t1, t2), np.minimum(t3, t4)); tmax = np.minimum(np.maximum(t1, t2), np.maximum(t3, t4))
    return np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)

def lateral_sd(shape, r, trials=12000, quantise=True):
    perp = []
    for _ in range(trials):
        ang0 = rng.uniform(-dphi / 2, dphi / 2) if quantise else 0.0
        angs = ang0 + np.arange(-60, 61) * dphi
        dx, dy = np.cos(angs), np.sin(angs)
        yaw = rng.uniform(0, 2 * np.pi)
        if shape[0] == "rect":
            t = ray_rect(dx, dy, r, 0.0, shape[1], shape[2], yaw)
        else:
            rho, sep = shape[1], shape[2] * rng.uniform(0.0, 1.0)
            c1 = (r + math.cos(yaw) * sep / 2, math.sin(yaw) * sep / 2); c2 = (r - math.cos(yaw) * sep / 2, -math.sin(yaw) * sep / 2)
            t = np.minimum(ray_circle(dx, dy, *c1, rho), ray_circle(dx, dy, *c2, rho))
        hit = np.isfinite(t)
        if hit.sum() < 3:
            continue
        rr = t[hit] + rng.normal(0, sig_r, hit.sum())
        perp.append((rr * dy[hit]).mean())
    return float(np.std(perp))

P("G2. LiDAR lateral centroid sd: total (random beam-grid offset) vs s/sqrt(24) -> shape part sqrt(tot^2 - s^2/24)")
shapes = {"person legs": ("legs", 0.06, 0.30), "large box 60x50": ("rect", 0.30, 0.25),
          "AMR+payload 60x40": ("rect", 0.30, 0.20), "forklift 1.2x1.0": ("rect", 0.60, 0.50)}
for name, sh in shapes.items():
    parts = []
    for r in (3.0, 5.0, 8.0):
        tot = lateral_sd(sh, r)
        q = r * dphi / math.sqrt(24)
        shape_part = math.sqrt(max(0.0, tot ** 2 - q ** 2))
        parts.append(shape_part)
        P(f"   {name:18s} r={r:.0f} m: total {tot:.4f} | s/sqrt24 {q:.4f} | shape part {shape_part:.4f} | "
          f"model sqrt(s^2/24+shape^2) {math.hypot(q, shape_part):.4f}")
    P(f"   {name:18s} -> sigma_lat (max shape part over 3/5/8 m) = {max(parts):.3f} m")

# ---------------- S. RGB-depth skew pixel term ----------------
w_max, dt_max = 1.5, 1 / 60   # rad/s (robot_params limits.max_angular_velocity), half RGB period
shift = fx * w_max * dt_max
P(f"S. skew pixel shift at 1.5 rad/s: max {shift:.1f} px; skew ~ U[-16.7, 16.7] ms -> sigma {shift/math.sqrt(3):.1f} px")

# ---------------- W. LOS rotation-rate bound from robot_params.yaml ----------------
a, t_r, d_stop = 1.0, 0.15, 0.30
def v_allowed(D):
    if D < 0.30: return 0.0
    if D < 0.50: return 0.2          # critical zone cap
    if D < 1.00: return 0.5          # warning zone cap
    return min(2.0, -a * t_r + math.sqrt((a * t_r) ** 2 + 2 * a * (D - d_stop)))
Ds = np.linspace(0.30, 8.0, 7701)
ratios = np.array([v_allowed(D) / D for D in Ds])
i = int(np.argmax(ratios))
P(f"W. |omega_LOS| <= v(D)/D (object at footprint clearance D; sensor range >= D): max {ratios[i]:.2f} rad/s at D={Ds[i]:.2f} m "
  f"(v={v_allowed(Ds[i]):.2f} m/s); D=2: {v_allowed(2.0)/2:.2f}, D=3: {v_allowed(3.0)/3:.2f}, inside warning zone max {0.5/0.5:.2f} (D=0.5)")

# ---------------- R. required confirmed range with per-speed latency budgets ----------------
tau_warn = 3.0
P("R. d_req = (v_r + v_o)(tau_warn + l), l = per-speed p90 budget (0.3 m/s: 0.8 s; 1.0/1.5 m/s: 0.5 s)")
for vr in (1.0, 2.0):
    for vo, l in ((0.3, 0.8), (1.0, 0.5), (1.5, 0.5)):
        P(f"   robot {vr:.1f} + obstacle {vo:.1f} m/s, l={l:.1f} s -> {(vr + vo) * (tau_warn + l):.2f} m")

"""rev4 (2026-09-22) — geometry + measurement-model checks for perception-tracking.md.
Sections: A camera geometry, B LiDAR plane visibility, C depth noise + ROI median, D camera surface offset,
E camera 3D error budget per range band, F pose-covariance propagation (base pivot + cross-cov) vs MC,
G LiDAR cluster model (lateral quantisation s^2/24, class-conditioned LOS bias) by 2D ray casting,
H NEES acceptance bands."""
import math
import numpy as np
from scipy.stats import chi2

rng = np.random.default_rng(42)
P = print

# ---------------- A. camera geometry (config/sensors.yaml) ----------------
W, H, hfov = 640, 480, 1.518436
fx = (W / 2) / math.tan(hfov / 2)
vfov = 2 * math.atan((H / 2) / fx)
cam_h = 0.18 + 0.25          # base_link_height + camera_link.z
lidar_h = 0.18 + 0.20        # base_link_height + lidar.z
P(f"A. fx=fy={fx:.1f} px (sensor_calibration.md measured 337.2098), vfov={math.degrees(vfov):.1f} deg, camera height {cam_h:.2f} m")
P(f"   nearest visible floor point ahead of camera: {cam_h / math.tan(vfov / 2):.2f} m; LiDAR plane height {lidar_h:.2f} m")

# ---------------- B. what the 2D LiDAR plane (0.38 m) can see ----------------
objs = {"small box 30x20x15": 0.15, "medium box 50x40x30": 0.30, "large box 60x50x40": 0.40,
        "AMR chassis (top)": 0.03 + 0.30, "AMR + medium payload": 0.33 + 0.30}
P("B. LiDAR plane 0.38 m vs object top height:")
for k, h in objs.items():
    P(f"   {k:24s} top={h:.2f} m -> {'VISIBLE' if h > lidar_h else 'invisible'} (margin {h - lidar_h:+.2f} m)")

# ---------------- C. depth noise model (sensors.yaml) and ROI median ----------------
def sig_d(Z):
    return math.sqrt(0.005 ** 2 + (0.002 * Z * Z) ** 2)
P("C. sigma_d(Z)=sqrt(0.005^2+(0.002 Z^2)^2):", ", ".join(f"{Z} m:{sig_d(Z)*100:.1f} cm" for Z in (1, 2, 3, 5, 8, 10)))
# median of m iid Gaussian pixels -> var ~ (pi/2) sigma^2/m ; check by MC
m = 200; s = 0.05
med = np.median(rng.normal(0, s, (20000, m)), axis=1)
P(f"   MC median std (m={m}, sigma={s}): {med.std():.5f} vs sqrt(pi/2)*sigma/sqrt(m)={math.sqrt(math.pi/2)*s/math.sqrt(m):.5f}")
for Z in (2, 5, 8):
    w = fx * 0.5 / Z; h = fx * 0.3 / Z; mpx = 0.25 * w * h
    P(f"   medium box at {Z} m: bbox {w:.0f}x{h:.0f} px, central-50% ROI m={mpx:.0f} px -> iid-median sigma={math.sqrt(math.pi/2)*sig_d(Z)/math.sqrt(mpx)*100:.2f} cm (real, correlated: up to {sig_d(Z)*100:.1f} cm)")

# ---------------- D. camera surface-vs-centre offset, random yaw ----------------
def rect_exit(a, b, phi):
    # distance from centre to boundary of rectangle (half-extents a,b) along direction phi
    c, s_ = abs(np.cos(phi)), abs(np.sin(phi))
    with np.errstate(divide="ignore"):
        return np.minimum(np.where(c > 1e-12, a / c, np.inf), np.where(s_ > 1e-12, b / s_, np.inf))
phi = rng.uniform(0, 2 * np.pi, 200000)
cam_off = {}
for name, (a, b) in {"medium box 50x40": (0.25, 0.20), "large box 60x50": (0.30, 0.25), "AMR 60x40": (0.30, 0.20),
                     "small box 30x20": (0.15, 0.10)}.items():
    t = rect_exit(a, b, phi)
    cam_off[name] = (t.mean(), t.std())
    P(f"D. camera ray-through-centre offset {name:18s}: mean {t.mean():.3f} m, std {t.std():.3f} m (range {t.min():.2f}-{t.max():.2f})")
P("   person torso ~ ellipse 0.40x0.25 m -> circle-like, median over central 50% width ~0.96 rho; use mu=0.12, sigma=0.04 (to calibrate)")

# ---------------- E. camera 3D error budget per band (base frame) ----------------
P("E. predicted ground-plane RMS error (base frame), kappa=0.05 bbox-centre jitter, offset corrected:")
kappa = 0.05
for name, L, (mu, sd) in [("medium box", 0.5, cam_off["medium box 50x40"]), ("person", 0.45, (0.12, 0.04))]:
    for band in ((1, 3), (3, 6), (6, 10)):
        Zs = np.linspace(*band, 50)
        # iid-sim median term (tiny) and the conservative correlated term
        errs_sim, errs_cons = [], []
        for Z in Zs:
            sx = kappa * L                              # (Z/fx)*(kappa*fx*L/Z)
            sz_cons = math.hypot(sig_d(Z), sd)
            errs_cons.append(sx ** 2 + sz_cons ** 2)
            errs_sim.append(sx ** 2 + sd ** 2)
        P(f"   {name:10s} {band[0]}-{band[1]} m: RMS(sim, iid pixels) {math.sqrt(np.mean(errs_sim))*100:.1f} cm | RMS(correlated depth noise) {math.sqrt(np.mean(errs_cons))*100:.1f} cm | uncorrected bias would add {mu*100:.0f} cm")

# ---------------- F. pose covariance propagation to map (base pivot, cross-cov) ----------------
xb, yb, th = 10.0, 5.0, 0.7
Sp = np.array([[0.03 ** 2, 0.0002, 0.0001], [0.0002, 0.03 ** 2, 0.00015], [0.0001, 0.00015, 0.01 ** 2]])
pb = np.array([4.0, 1.0])     # object in base frame
Rm = lambda t: np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
pm = Rm(th) @ pb + np.array([xb, yb])
G = np.array([-(pm[1] - yb), pm[0] - xb])
J = np.hstack([np.eye(2), G[:, None]])
Sig_lin = J @ Sp @ J.T
smp = rng.multivariate_normal([xb, yb, th], Sp, 200000)
pts = np.stack([np.cos(smp[:, 2]) * pb[0] - np.sin(smp[:, 2]) * pb[1] + smp[:, 0],
                np.sin(smp[:, 2]) * pb[0] + np.cos(smp[:, 2]) * pb[1] + smp[:, 1]], 1)
P("F. pose->map covariance, linear [I2 G_base] Sigma_pose [I2 G_base]^T:", np.round(Sig_lin, 6).tolist())
P("   Monte-Carlo:", np.round(np.cov(pts.T), 6).tolist())
Gcam = np.array([-(pm[1] - (yb + math.sin(th) * 0.18)), pm[0] - (xb + math.cos(th) * 0.18)])
P(f"   lever arm |G| base pivot {np.linalg.norm(G):.3f} m vs camera pivot {np.linalg.norm(Gcam):.3f} m (object 4.12 m ahead)")

# ---------------- G. LiDAR cluster model via 2D ray casting ----------------
dphi = math.radians(0.5); sig_r = 0.03

def ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx_, fy_ = ox - cx, oy - cy
    b = fx_ * dx + fy_ * dy; c = fx_ ** 2 + fy_ ** 2 - r * r; disc = b * b - c
    t = -b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc >= 0) & (t > 0), t, np.inf)

def ray_rect(ox, oy, dx, dy, cx, cy, a, b, yaw):
    # rotate ray into rect frame, slab test
    cth, sth = math.cos(-yaw), math.sin(-yaw)
    lx = cth * (ox - cx) - sth * (oy - cy); ly = sth * (ox - cx) + cth * (oy - cy)
    ldx = cth * dx - sth * dy; ldy = sth * dx + cth * dy
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-a - lx) / ldx; t2 = (a - lx) / ldx; t3 = (-b - ly) / ldy; t4 = (b - ly) / ldy
    tmin = np.maximum(np.minimum(t1, t2), np.minimum(t3, t4)); tmax = np.minimum(np.maximum(t1, t2), np.maximum(t3, t4))
    return np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)

def cluster_stats(shape, r, trials=4000):
    par, perp, ns = [], [], []
    for _ in range(trials):
        ang0 = rng.uniform(-dphi / 2, dphi / 2)         # beam-grid offset
        angs = ang0 + np.arange(-60, 61) * dphi
        dx, dy = np.cos(angs), np.sin(angs)
        yaw = rng.uniform(0, 2 * np.pi)
        if shape[0] == "rect":
            t = ray_rect(0.0, 0.0, dx, dy, r, 0.0, shape[1], shape[2], yaw)
        else:  # person legs: two circles radius rho, centres +-sep/2 along random stride direction
            rho, sep = shape[1], shape[2] * rng.uniform(0.0, 1.0)
            c1 = (r + math.cos(yaw) * sep / 2, math.sin(yaw) * sep / 2); c2 = (r - math.cos(yaw) * sep / 2, -math.sin(yaw) * sep / 2)
            t = np.minimum(ray_circle(0, 0, dx, dy, *c1, rho), ray_circle(0, 0, dx, dy, *c2, rho))
        hit = np.isfinite(t)
        if hit.sum() < 3:
            continue
        rr = t[hit] + rng.normal(0, sig_r, hit.sum())
        px, py = rr * dx[hit], rr * dy[hit]
        par.append(px.mean() - r); perp.append(py.mean()); ns.append(hit.sum())
    par, perp = np.array(par), np.array(perp)
    return par.mean(), par.std(), perp.std(), np.mean(ns), len(par) / trials

shapes = {"person legs (rho .06, stride<=.3)": ("legs", 0.06, 0.30), "large box 60x50": ("rect", 0.30, 0.25),
          "AMR+payload 60x40": ("rect", 0.30, 0.20), "forklift body 1.2x1.0 (approx)": ("rect", 0.60, 0.50)}
P("G. LiDAR cluster centroid vs object centre (LOS = +x), sigma_r=0.03, 0.5 deg:")
for name, sh in shapes.items():
    for r in (3.0, 5.0, 8.0):
        mu, sd, sdp, n, vis = cluster_stats(sh, r)
        s = r * dphi
        P(f"   {name:32s} r={r:.0f} m: n~{n:4.1f} seen {vis*100:3.0f}% | LOS bias mu={mu:+.3f} sd={sd:.3f} | lateral sd={sdp:.3f} (s/sqrt24={s/math.sqrt(24):.3f}, s/sqrt(12n)={s/math.sqrt(12*n):.3f})")
# pure lateral quantisation check: flat plate width Wd face-on, only grid offset varies
for r in (3.0, 5.0, 8.0):
    s = r * dphi; errs = []
    for Wd in rng.uniform(0.3, 0.6, 4000):
        u = rng.uniform(0, s); xs = -Wd / 2 + u + s * np.arange(0, int((Wd - u) / s) + 1)
        errs.append(xs.mean())
    P(f"   flat-plate lateral quantisation r={r}: sd={np.std(errs):.4f} vs s/sqrt(24)={s/math.sqrt(24):.4f}")

# ---------------- H. NEES acceptance bands (2-DoF, N independent samples) ----------------
for N in (100, 300, 1000):
    lo, hi = chi2.ppf(0.025, 2 * N) / N, chi2.ppf(0.975, 2 * N) / N
    P(f"H. mean NEES 2-DoF, N={N}: 95% band [{lo:.3f}, {hi:.3f}] (normal approx 2 +/- {1.96*math.sqrt(4/N):.3f})")

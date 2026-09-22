"""rev4 extra checks: (a) person-leg cluster visibility vs range and min_points,
(b) velocity test with jitter-variance (persistent bias removed from the slope-test noise) for the unknown class,
(c) ego-motion-adaptive v_min_eff = v_min + 2*sigma_delta*|omega_LOS| against rotating residual bias,
(d) tempered (beta=0.5) once-per-cycle class update."""
import math
import numpy as np
from scipy.stats import chi2
rng = np.random.default_rng(11)
P = print; dt = 0.1; dphi = math.radians(0.5); sig_r = 0.03

def ray_circle(dx, dy, cx, cy, r):
    b = -(cx * dx + cy * dy); c = cx * cx + cy * cy - r * r; disc = b * b - c
    t = -b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc >= 0) & (t > 0), t, np.inf)
P("(a) person legs (2 x rho 0.06 m, stride 0-0.3 m) at LiDAR plane 0.38 m: points per cluster")
for r in (4, 6, 8, 10, 12):
    ns = []
    for _ in range(4000):
        a0 = rng.uniform(-dphi / 2, dphi / 2); ang = a0 + np.arange(-30, 31) * dphi; dx, dy = np.cos(ang), np.sin(ang)
        yaw = rng.uniform(0, 2 * np.pi); sep = 0.3 * rng.uniform(0, 1)
        t = np.minimum(ray_circle(dx, dy, r + math.cos(yaw) * sep / 2, math.sin(yaw) * sep / 2, 0.06),
                       ray_circle(dx, dy, r - math.cos(yaw) * sep / 2, -math.sin(yaw) * sep / 2, 0.06))
        ns.append(np.isfinite(t).sum())
    ns = np.array(ns)
    P(f"   r={r:2d} m: mean n={ns.mean():.1f}; P(n>=3)={np.mean(ns>=3)*100:.0f}%  P(n>=2)={np.mean(ns>=2)*100:.0f}%  P(n>=1)={np.mean(ns>=1)*100:.0f}%")
    # 3-of-5 confirmation with independent per-scan detection prob p (approximation; stride changes per scan)
    for mp in (3, 2):
        p = np.mean(ns >= mp); pc = sum(math.comb(5, k) * p ** k * (1 - p) ** (5 - k) for k in (3, 4, 5))
        P(f"       min_points={mp}: P(confirm within 5 scans) ~ {pc*100:.0f}%")

def ls_T(Z, Rj, W=10, nmin=4):
    N, T, _ = Z.shape; Ts = np.zeros((N, T)); Vn = np.zeros((N, T))
    for k in range(T):
        n = min(k + 1, W)
        if n < nmin: continue
        t = np.arange(n) * dt; tc = t - t.mean(); Stt = (tc ** 2).sum(); z = Z[:, k - n + 1:k + 1, :]
        v = (tc[None, :, None] * (z - z.mean(1, keepdims=True))).sum(1) / Stt
        Ts[:, k] = v[:, 0] ** 2 / (Rj[0] / Stt) + v[:, 1] ** 2 / (Rj[1] / Stt); Vn[:, k] = np.hypot(v[:, 0], v[:, 1])
    return Ts, Vn
def fires(Ts, Vn, vmin, thr=chi2.ppf(0.999, 2), consec=2):
    ok = (Ts > thr) & (Vn > vmin); run = np.zeros(ok.shape, int)
    for k in range(ok.shape[1]): run[:, k] = np.where(ok[:, k], (run[:, k - 1] + 1) if k else 1, 0)
    return run >= consec
def sim(v, direction, jit, bias_sd=0.10, N=1200, T=40, om=0.0):
    t = np.arange(T) * dt; d = np.array([1.0, 0.0]) if direction == "los" else np.array([0.0, 1.0])
    Z = v * t[None, :, None] * d + rng.normal(0, 1, (N, T, 2)) * np.array(jit)
    b = rng.normal(0, bias_sd, (N, 1))                      # persistent residual LOS bias of this object
    ang = om * t[None, :] + rng.uniform(0, 2 * np.pi, (N, 1)) if om > 0 else np.zeros((N, T))
    Z = Z + b[..., None] * np.stack([np.cos(ang), np.sin(ang)], -1)
    return Z
jit = (math.sqrt(0.03 ** 2 / 10 + 0.03 ** 2), math.sqrt(0.009 ** 2 + 0.03 ** 2))   # sigma_seg = 0.03 m (to calibrate)
Rj = np.array([jit[0] ** 2, jit[1] ** 2])
P(f"(b) unknown class, slope-test noise = frame-to-frame jitter {jit[0]:.3f}/{jit[1]:.3f} m (persistent bias sd 0.10 m NOT white):")
for direction in ("los", "lateral"):
    row = []
    for v in (0.3, 0.5, 1.0, 1.5):
        Ts, Vn = ls_T(sim(v, direction, jit), Rj); F = fires(Ts, Vn, 0.15)
        first = np.where(F.any(1), F.argmax(1) * dt, np.nan)
        row.append(f"v={v}: med {np.nanmedian(first):.1f}s p90 {np.nanpercentile(first,90):.1f}s ({np.mean(~np.isnan(first))*100:.0f}%)")
    P(f"   {direction:7s}: " + " | ".join(row))
Ts, Vn = ls_T(sim(0.0, "los", jit, N=400, T=600), Rj); F = fires(Ts, Vn, 0.15)
P(f"   static (stationary robot): {(F[:,1:] & ~F[:,:-1]).sum()/(400*600*dt/3600):.2f} false rising edges / track-h")
P("(c) robot passing static unknown object: residual bias sd 0.10 m rotating at omega_LOS; v_min_eff = 0.15 + 2*0.10*|omega_LOS|")
for om in (0.5, 1.0, 2.0):
    Z = sim(0.0, "los", jit, om=om, N=1200, T=30); Ts, Vn = ls_T(Z, Rj)
    fa_fixed = fires(Ts, Vn, 0.15).any(1).mean(); vme = 0.15 + 2 * 0.10 * om; fa_ad = fires(Ts, Vn, vme).any(1).mean()
    Zm = sim(0.5, "lateral", jit, om=om, N=1200, T=30); Ts2, Vn2 = ls_T(Zm, Rj); det = fires(Ts2, Vn2, vme).any(1).mean()
    Zm3 = sim(0.3, "lateral", jit, om=om, N=1200, T=30); Ts3, Vn3 = ls_T(Zm3, Rj); det3 = fires(Ts3, Vn3, vme).any(1).mean()
    P(f"   omega_LOS={om}: FA(3 s) fixed v_min {fa_fixed*100:.1f}% -> adaptive v_min_eff={vme:.2f}: {fa_ad*100:.1f}% ; detection(3 s) of 0.5 m/s {det*100:.0f}%, 0.3 m/s {det3*100:.0f}%")
P("   LOS rate bound from robot_params: v_max(D)=-a t+sqrt((a t)^2+2a(D-0.3)), t=0.15:")
for D in (1.0, 2.0, 3.0, 5.0):
    v = -0.15 + math.sqrt(0.0225 + 2 * (D - 0.3)); v = min(v, 2.0)
    vz = 0.5 if D <= 1.0 else v
    P(f"      clearance D={D} m: v<= {vz:.2f} m/s -> |omega_LOS| <= v/D = {vz/D:.2f} rad/s")

C = ["box", "sign", "person", "forklift", "amr"]; pi0 = np.array([0.30, 0.05, 0.30, 0.15, 0.20])
M_hi = np.full((5, 5), 0.02); np.fill_diagonal(M_hi, 0.92); M_lo = np.full((5, 5), 0.075); np.fill_diagonal(M_lo, 0.70)
def upd(p, pred, M, beta=0.5, cap=0.95, eta=0.02):
    p = (1 - eta) * p + eta * pi0; p = p * M[:, pred] ** beta; p /= p.sum()
    if p.max() > cap:
        i = p.argmax(); ex = p[i] - cap; p[i] = cap; o = [j for j in range(5) if j != i]; p[o] += ex * p[o] / p[o].sum()
    return p
for name, M in (("high-score", M_hi), ("low-score", M_lo)):
    p = pi0.copy(); out = []
    for k in range(5): p = upd(p, 2, M); out.append(f"{p[2]:.2f}")
    P(f"(d) tempered beta=0.5, once per LiDAR cycle, {name} 'person' hits: p(person) k=1..5: {out}")
p = pi0.copy()
for k in range(3): p = upd(p, 2, M_hi)
out = []
for k in range(5): p = upd(p, 0, M_hi); out.append(f"box {p[0]:.2f}/person {p[2]:.2f}")
P("    then 5 contradicting high-score 'box' hits:", out)

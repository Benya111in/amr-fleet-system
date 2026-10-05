"""rev5 (2026-09-22) B1 LS-slope-test latency while the robot passes (rotating residual LOS bias),
with the ego-adaptive v_min_eff = 0.15 + 2 sigma_delta |omega_LOS|. Same noise/test as rev4_extra.py (b)/(c).
omega_LOS grid: 0 (robot stopped), 0.5, 1.04 rad/s (max from robot_params.yaml, rev5_fixes.out W).
Latency = first fire after first detection (object moving from the start), +0.05 s processing in the brief."""
import math
import numpy as np
from scipy.stats import chi2
rng = np.random.default_rng(23)
dt = 0.1
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
def sim(v, direction, jit, bias_sd, om, N=1500, T=40):
    t = np.arange(T) * dt; d = np.array([1.0, 0.0]) if direction == "los" else np.array([0.0, 1.0])
    Z = v * t[None, :, None] * d + rng.normal(0, 1, (N, T, 2)) * np.array(jit)
    b = rng.normal(0, bias_sd, (N, 1))
    ang = om * t[None, :] + rng.uniform(0, 2 * np.pi, (N, 1))
    return Z + b[..., None] * np.stack([np.cos(ang), np.sin(ang)], -1)
jit = (math.sqrt(0.03 ** 2 / 10 + 0.03 ** 2), math.sqrt(0.009 ** 2 + 0.03 ** 2)); Rj = np.array([jit[0] ** 2, jit[1] ** 2])
for cls, sd in (("unknown", 0.10), ("person", 0.027)):
    for om in (0.0, 0.5, 1.04):
        vmin = 0.15 + 2 * sd * om
        cells = []
        for v in (0.3, 0.5, 1.0, 1.5):
            worst = []
            for direction in ("los", "lateral"):
                Ts, Vn = ls_T(sim(v, direction, jit, sd, om), Rj); F = fires(Ts, Vn, vmin)
                first = np.where(F.any(1), F.argmax(1) * dt, np.nan)
                worst.append((np.nanpercentile(first, 90) if np.any(~np.isnan(first)) else np.inf, np.mean(~np.isnan(first))))
            p90 = max(w[0] for w in worst); frac = min(w[1] for w in worst)
            cells.append(f"v={v}: p90 {p90:.1f}s ({frac*100:.0f}% in 4 s)")
        Ts, Vn = ls_T(sim(0.0, "los", jit, sd, om, N=1500, T=30), Rj); F = fires(Ts, Vn, vmin)
        print(f"{cls:7s} omega_LOS={om:4.2f} v_min_eff={vmin:.2f}: " + " | ".join(cells) + f" | static false-dyn within 3 s {F.any(1).mean()*100:.1f}%")

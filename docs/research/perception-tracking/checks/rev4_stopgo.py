"""rev4: stop->go latency of the LS-slope velocity path (object tracked while stationary for 1.5 s, then moves).
Also the free-space path threshold rho* at which the DATMO score crosses 0.5 for an unknown-class cluster."""
import math, numpy as np
from scipy.stats import chi2
rng = np.random.default_rng(5); dt = 0.1
jit = np.array([0.031, 0.031]); R = jit ** 2
def run(v, direction, N=1500, T0=15, T=45, W=10, thr=chi2.ppf(0.999, 2), vmin=0.15, consec=2):
    t = np.arange(T) * dt; d = np.array([1., 0.]) if direction == "los" else np.array([0., 1.])
    mv = np.clip(t - T0 * dt, 0, None)
    Z = v * mv[None, :, None] * d + rng.normal(0, 1, (N, T, 2)) * jit
    run_ = np.zeros(N, int); first = np.full(N, np.nan)
    for k in range(4, T):
        n = min(k + 1, W); tt = np.arange(n) * dt; tc = tt - tt.mean(); Stt = (tc ** 2).sum()
        z = Z[:, k - n + 1:k + 1]; vh = (tc[None, :, None] * (z - z.mean(1, keepdims=True))).sum(1) / Stt
        Ts = (vh ** 2 / (R / Stt)).sum(1); ok = (Ts > thr) & (np.hypot(*vh.T) > vmin)
        run_ = np.where(ok, run_ + 1, 0)
        newly = (run_ >= consec) & np.isnan(first)
        first[newly] = (k - T0) * dt
    pre = np.mean(first < 0)
    f = first[first >= 0]
    return np.median(f), np.percentile(f, 90), pre
for direction in ("lateral", "los"):
    for v in (0.3, 1.0, 1.5):
        m, p90, pre = run(v, direction)
        print(f"stop->go {direction:7s} v={v}: time from motion onset to fire median {m:.2f}s p90 {p90:.2f}s (fired before onset: {pre*100:.1f}%)")
sig = lambda x: 1/(1+math.exp(-x)); logit = lambda p: math.log(p/(1-p))
pim = 0.382
rho_star = -logit(pim) / 4.0
print(f"DATMO unknown class: P(dyn)>0.5 when 4*rho_free > {-logit(pim):.3f} -> rho* = {rho_star:.3f}")
for v in (0.3, 1.0, 1.5):
    t = rho_star * 0.30 / v
    print(f"   lateral stop->go v={v}: strip v*t reaches rho*.W after {t:.2f}s -> + scan quantisation 0.1 + proc 0.05 = {t+0.15:.2f}s")

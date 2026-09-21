"""Windowed weighted-LS slope velocity test: latency per speed and false-alarm rate."""
import math, numpy as np
rng = np.random.default_rng(1)
dt = 0.1
def ls_T(ts, zs, sp, sq):
    # per-axis WLS slope with equal weights (isotropic approx using max sigma); 2-DoF statistic
    t = np.array(ts); z = np.array(zs); n = len(t)
    tc = t - t.mean(); Stt = (tc**2).sum()
    v = (tc[:, None]*(z - z.mean(0))).sum(0)/Stt
    var = np.array([sp**2, sq**2])/Stt
    return (v**2/var).sum(), v
def run(v, sp, sq, W=10, thr=13.8, nmin=3, T=4.0, N=600, rot_bias=None, jump=None):
    lat = []
    for _ in range(N):
        ang0 = rng.uniform(0, 2*math.pi); dirv = np.array([math.cos(ang0), math.sin(ang0)])
        ts, zs = [], []; done = None
        for k in range(int(T/dt)):
            z = v*k*dt*dirv + rng.normal(0, [sp, sq])
            if rot_bias is not None:
                b, om = rot_bias; a = om*k*dt; z = z + b*np.array([math.cos(a), math.sin(a)])
            if jump is not None and k >= jump[0]: z = z + np.array(jump[1])
            ts.append(k*dt); zs.append(z); ts = ts[-W:]; zs = zs[-W:]
            if len(ts) >= nmin:
                Tst, _ = ls_T(ts, zs, sp, sq)
                if Tst > thr: done = k*dt; break
        lat.append(done if done is not None else np.nan)
    lat = np.array(lat); ok = ~np.isnan(lat)
    return (np.nanmedian(lat) if ok.any() else float('nan')), (np.nanpercentile(lat, 90) if ok.any() else float('nan')), ok.mean()
for name, sp, sq in [("person@5m corrected", 0.051, 0.02), ("unknown@5m corrected", 0.12, 0.02), ("isotropic 0.05", 0.05, 0.05)]:
    print(f"--- {name}: sigma_par={sp} sigma_perp={sq}, W=10, thr=13.8 (chi2_2 0.999)")
    for v in (0.3, 0.5, 1.0, 1.5):
        m, p90, ok = run(v, sp, sq)
        print(f"   v={v}: first-fire median={m:.2f}s p90={p90:.2f}s rate(4s)={ok:.2f}")
    print(f"   static false-alarm rate over 5 s: {run(0.0, sp, sq, T=5.0)[2]:.3f}")
    print(f"   static, residual bias 0.05 m rotating 0.8 rad/s: {run(0.0, sp, sq, T=3.0, rot_bias=(0.05, 0.8))[2]:.3f}")
    print(f"   static, uncorrected bias 0.20 m rotating 0.8 rad/s: {run(0.0, sp, sq, T=3.0, rot_bias=(0.20, 0.8))[2]:.3f}")
    print(f"   static, 5 cm step jump at t=1 s (map-frame AMCL): {run(0.0, sp, sq, T=3.0, jump=(10, [0.05, 0]))[2]:.3f}")
    print(f"   static, 8 cm step jump at t=1 s (map-frame AMCL): {run(0.0, sp, sq, T=3.0, jump=(10, [0.08, 0]))[2]:.3f}")

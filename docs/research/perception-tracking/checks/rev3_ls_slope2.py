"""LS slope test, variant: T>13.8 AND |v|>v_min on 2 consecutive updates."""
import math, numpy as np
rng = np.random.default_rng(2); dt = 0.1
def ls(ts, zs, sp, sq):
    t = np.array(ts); z = np.array(zs); tc = t - t.mean(); Stt = (tc**2).sum()
    v = (tc[:, None]*(z - z.mean(0))).sum(0)/Stt; var = np.array([sp**2, sq**2])/Stt
    return (v**2/var).sum(), np.linalg.norm(v)
def run(v, sp, sq, W=10, thr=13.8, vmin=0.15, consec=2, nmin=3, T=4.0, N=800, rot_bias=None, jump=None):
    lat = []
    for _ in range(N):
        a0 = rng.uniform(0, 2*math.pi); d = np.array([math.cos(a0), math.sin(a0)]); ts, zs = [], []; done = None; cnt = 0
        for k in range(int(T/dt)):
            z = v*k*dt*d + rng.normal(0, [sp, sq])
            if rot_bias is not None: b, om = rot_bias; a = om*k*dt; z = z + b*np.array([math.cos(a), math.sin(a)])
            if jump is not None and k >= jump[0]: z = z + np.array(jump[1])
            ts.append(k*dt); zs.append(z); ts = ts[-W:]; zs = zs[-W:]
            if len(ts) >= nmin:
                Tst, vn = ls(ts, zs, sp, sq); cnt = cnt+1 if (Tst > thr and vn > vmin) else 0
                if cnt >= consec: done = k*dt; break
        lat.append(done if done is not None else np.nan)
    lat = np.array(lat); ok = ~np.isnan(lat)
    return (np.nanmedian(lat) if ok.any() else float('nan')), (np.nanpercentile(lat, 90) if ok.any() else float('nan')), ok.mean()
for name, sp, sq in [("person@5m", 0.051, 0.02), ("unknown@5m", 0.12, 0.02), ("unknown@10m floor .03", 0.12, 0.03)]:
    print(f"--- {name}: sp={sp} sq={sq}; W=10, thr=13.8, vmin=0.15, 2 consecutive")
    for v in (0.3, 0.5, 1.0, 1.5):
        m, p90, ok = run(v, sp, sq); print(f"   v={v}: median={m:.2f}s p90={p90:.2f}s rate(4s)={ok:.2f}")
    print(f"   static FA over 5 s: {run(0.0, sp, sq, T=5.0)[2]:.4f}")
    print(f"   static FA over 30 s: {run(0.0, sp, sq, T=30.0, N=300)[2]:.4f}")
    print(f"   residual bias .05 rotating .8 rad/s (3 s): {run(0.0, sp, sq, T=3.0, rot_bias=(0.05, 0.8))[2]:.3f}")
    print(f"   uncorrected bias .20 rotating .8 rad/s (3 s): {run(0.0, sp, sq, T=3.0, rot_bias=(0.20, 0.8))[2]:.3f}")
    print(f"   5 cm step (map-frame jump): {run(0.0, sp, sq, T=3.0, jump=(10, [0.05, 0]))[2]:.3f} | 8 cm: {run(0.0, sp, sq, T=3.0, jump=(10, [0.08, 0]))[2]:.3f}")

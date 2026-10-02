"""Full SEED pipeline Monte Carlo (sec 4.C): coarse grid -> capped-mean ranking (untrimmed, d_max 1 m, 90 beams)
-> top-k cluster suppression (2 m / 30 deg) -> fine grid 0.1 m x 2 deg in a +-0.5 m / +-8 deg box, 180 beams,
scored by the 80 % trimmed capped mean -> best / second-best mode.
Same synthetic repetitive-rack warehouse as seed_sim.py.
Reports per coarse-grid variant: P(truth inside fine window of some top-k candidate), P(final best within
0.15 m / 3 deg of truth), distribution of m_1 and of the margin m_2 - m_1, and lookup counts.
Run: python3 seed_pipeline_sim.py [n_trials]
"""
import sys
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

rng = np.random.default_rng(11)
RES = 0.05
W, H = 60.0, 40.0
nx, ny = int(W / RES), int(H / RES)
occ = np.zeros((ny, nx), dtype=bool)

def rect(x0, y0, x1, y1):
    occ[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = True

rect(0, 0, W, 0.2); rect(0, H - 0.2, W, H); rect(0, 0, 0.2, H); rect(W - 0.2, 0, W, H)
for xb in ((5, 17), (21, 33), (37, 49)):
    y = 6.0
    while y + 1.2 < 35:
        rect(xb[0], y, xb[1], y + 1.2)
        y += 4.2
for (px, py) in ((53, 10), (53, 20), (53, 30), (57, 15)):
    rect(px, py, px + 0.4, py + 0.4)
rect(50, 37, 58, 37.3)
edt = distance_transform_edt(~occ) * RES
DMAX = 1.0
edt_c = np.minimum(edt, DMAX).astype(np.float32)

def lookup(xs, ys):
    ix = (xs / RES).astype(np.int64); iy = (ys / RES).astype(np.int64)
    inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    out = np.full(xs.shape, DMAX, dtype=np.float32)
    out[inside] = edt_c[iy[inside], ix[inside]]
    return out

def raycast(x, y, th, angles, rmax=25.0, step=0.025):
    ss = np.arange(step, rmax, step)
    ranges = np.full(len(angles), np.inf)
    for i, a in enumerate(angles):
        xs = x + ss * math.cos(th + a); ys = y + ss * math.sin(th + a)
        ix = (xs / RES).astype(int); iy = (ys / RES).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        hit = np.zeros_like(ok); hit[ok] = occ[iy[ok], ix[ok]]; hit |= ~ok
        k = np.argmax(hit)
        if hit[k]:
            ranges[i] = ss[k]
    return ranges

def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def score(HX, HY, HT, rr, ang, trim=1.0, chunk=20000):
    out = np.empty(len(HX), dtype=np.float32)
    c0 = np.cos(ang).astype(np.float32); s0 = np.sin(ang).astype(np.float32)
    for a in range(0, len(HX), chunk):
        b = min(a + chunk, len(HX))
        ct = np.cos(HT[a:b]).astype(np.float32)[:, None]; st = np.sin(HT[a:b]).astype(np.float32)[:, None]
        c = ct * c0[None, :] - st * s0[None, :]; s = st * c0[None, :] + ct * s0[None, :]
        ex = HX[a:b, None] + rr[None, :] * c; ey = HY[a:b, None] + rr[None, :] * s
        d = lookup(ex, ey)
        if trim < 1.0:
            k = int(math.ceil(trim * d.shape[1]))
            d = np.partition(d, k - 1, axis=1)[:, :k]
        out[a:b] = d.mean(axis=1)
    return out

def topk_clusters(HX, HY, HT, sc, k, dpos=2.0, dang=math.radians(30)):
    order = np.argsort(sc); chosen = []
    for i in order:
        if all(not (math.hypot(HX[i] - HX[j], HY[i] - HY[j]) < dpos and abs(wrap(HT[i] - HT[j])) < dang) for j in chosen):
            chosen.append(i)
            if len(chosen) == k:
                break
    return chosen

def coarse_grid(step, dth_deg):
    gx, gy = np.meshgrid(np.arange(step / 2, W, step), np.arange(step / 2, H, step))
    gx, gy = gx.ravel(), gy.ravel()
    keep = edt[(gy / RES).astype(int), (gx / RES).astype(int)] > 0.3
    gx, gy = gx[keep], gy[keep]
    heads = np.radians(np.arange(0, 360, dth_deg))
    return (np.repeat(gx, len(heads)).astype(np.float32), np.repeat(gy, len(heads)).astype(np.float32),
            np.tile(heads, len(gx)).astype(np.float32))

fdx = np.arange(-0.5, 0.5001, 0.1); fdt = np.radians(np.arange(-8, 8.01, 2))
FX, FY, FT = [a.ravel().astype(np.float32) for a in np.meshgrid(fdx, fdx, fdt, indexing="ij")]

n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 25
ang90 = np.linspace(-math.pi, math.pi, 90, endpoint=False)
ang180 = np.linspace(-math.pi, math.pi, 180, endpoint=False)
variants = [(1.0, 15, 20), (0.5, 10, 20), (0.5, 5, 20), (0.5, 5, 50)]
grids = {v: coarse_grid(v[0], v[1]) for v in variants}
for v in variants:
    print(f"variant step {v[0]} m, {v[1]} deg, top-{v[2]}: {len(grids[v][0])} coarse hypotheses, coarse lookups {len(grids[v][0])*90/1e6:.1f} M, fine lookups {v[2]*len(FX)*180/1e6:.1f} M")
stats = {v: {"window": 0, "final": 0, "m1": [], "margin": [], "m1_ok_and_margin": 0, "false_accept": 0} for v in variants}
for t in range(n_trials):
    while True:
        x, y = rng.uniform(1, W - 1), rng.uniform(1, H - 1)
        if edt[int(y / RES), int(x / RES)] > 0.4:
            break
    th = rng.uniform(-math.pi, math.pi)
    r180 = raycast(x, y, th, ang180) + rng.normal(0, 0.03, 180)
    k0 = rng.integers(0, 180); idx = (k0 + np.arange(18)) % 180   # 10 % dynamic occluders
    r180[idx] = np.minimum(r180[idx], rng.uniform(0.5, 2.0))
    fin180 = np.isfinite(r180)
    r90 = r180[::2]; fin90 = np.isfinite(r90)
    for v in variants:
        HX, HY, HT = grids[v]
        sc = score(HX, HY, HT, r90[fin90].astype(np.float32), ang90[fin90])
        top = topk_clusters(HX, HY, HT, sc, v[2])
        inwin = [abs(HX[i] - x) <= 0.5 and abs(HY[i] - y) <= 0.5 and abs(wrap(HT[i] - th)) <= math.radians(8) for i in top]
        stats[v]["window"] += any(inwin)
        # fine stage per candidate (trimmed 80 %, 180 beams); keep best pose per candidate
        best = []
        for i in top:
            fs = score(HX[i] + FX, HY[i] + FY, HT[i] + FT, r180[fin180].astype(np.float32), ang180[fin180], trim=0.8)
            j = int(np.argmin(fs))
            best.append((float(fs[j]), float(HX[i] + FX[j]), float(HY[i] + FY[j]), float(HT[i] + FT[j])))
        best.sort()
        # distinct modes: drop candidates within 0.5 m / 10 deg of a better one
        modes = []
        for bsc in best:
            if all(math.hypot(bsc[1] - m[1], bsc[2] - m[2]) > 0.5 or abs(wrap(bsc[3] - m[3])) > math.radians(10) for m in modes):
                modes.append(bsc)
        m1 = modes[0]; m2 = modes[1] if len(modes) > 1 else (1.0,)
        ok = math.hypot(m1[1] - x, m1[2] - y) <= 0.15 and abs(wrap(m1[3] - th)) <= math.radians(3)
        stats[v]["final"] += ok
        stats[v]["m1"].append(m1[0]); stats[v]["margin"].append(m2[0] - m1[0])
        accept = m1[0] < 0.10 and (m2[0] - m1[0]) > 0.05
        stats[v]["m1_ok_and_margin"] += accept
        stats[v]["false_accept"] += accept and not ok
    print(f"trial {t+1}/{n_trials} done", flush=True)
for v in variants:
    s = stats[v]
    print(f"step {v[0]} m / {v[1]} deg / top-{v[2]}: truth in some fine window {s['window']}/{n_trials}; final best correct {s['final']}/{n_trials}; "
          f"median m1 {np.median(s['m1']):.3f}; margin median {np.median(s['margin']):.3f} (min {np.min(s['margin']):.3f}); "
          f"unimodal-accept (m1<0.10 & margin>0.05) {s['m1_ok_and_margin']}/{n_trials}, false accepts {s['false_accept']}")

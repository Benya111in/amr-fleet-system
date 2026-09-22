"""Monte Carlo check of the sec 4.C SEED stage claim: 'coarse 1 m x 15 deg grid scored with the capped
mean distance (d_max = 1 m) puts the true pose deterministically inside the basin'.
Synthetic 60 x 40 m warehouse with repetitive rack rows (worst case for aliasing), 0.05 m grid.
Measures, over random true poses: rank of the best coarse hypothesis within (0.75 m, 7.5 deg) of truth,
whether a correct hypothesis is among the top-20 after 2 m / 30 deg cluster suppression, with the plain
capped mean m and with the 80 % trimmed capped mean, with / without 10 % dynamic occluders.
Run: python3 seed_sim.py [n_trials]
"""
import sys
import math
import numpy as np
from scipy.ndimage import distance_transform_edt

rng = np.random.default_rng(7)
RES = 0.05
W, H = 60.0, 40.0
nx, ny = int(W / RES), int(H / RES)
occ = np.zeros((ny, nx), dtype=bool)

def rect(x0, y0, x1, y1):
    occ[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = True

# outer walls 0.2 m
rect(0, 0, W, 0.2); rect(0, H - 0.2, W, H); rect(0, 0, 0.2, H); rect(W - 0.2, 0, W, H)
# rack rows: 3 blocks along x, rows along y (1.2 m deep, 3.0 m aisles)
for xb in ((5, 17), (21, 33), (37, 49)):
    y = 6.0
    while y + 1.2 < 35:
        rect(xb[0], y, xb[1], y + 1.2)
        y += 4.2
# a few pillars in the open dock zone (x 50..60) and a charging wall
for (px, py) in ((53, 10), (53, 20), (53, 30), (57, 15)):
    rect(px, py, px + 0.4, py + 0.4)
rect(50, 37, 58, 37.3)

edt = distance_transform_edt(~occ) * RES          # distance to nearest occupied cell [m]
DMAX = 1.0
edt_c = np.minimum(edt, DMAX).astype(np.float32)

def lookup(xs, ys):
    ix = np.clip((xs / RES).astype(np.int64), 0, nx - 1)
    iy = np.clip((ys / RES).astype(np.int64), 0, ny - 1)
    out = edt_c[iy, ix]
    outside = (xs < 0) | (xs >= W) | (ys < 0) | (ys >= H)
    return np.where(outside, DMAX, out)

def raycast(x, y, th, angles, rmax=25.0, step=0.025):
    ss = np.arange(step, rmax, step)
    ranges = np.full(len(angles), np.inf)
    for i, a in enumerate(angles):
        xs = x + ss * math.cos(th + a)
        ys = y + ss * math.sin(th + a)
        ix = (xs / RES).astype(int); iy = (ys / RES).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        hit = np.zeros_like(ok)
        hit[ok] = occ[iy[ok], ix[ok]]
        hit |= ~ok
        k = np.argmax(hit)
        if hit[k]:
            ranges[i] = ss[k]
    return ranges

NB = 90
angles = np.linspace(-math.pi, math.pi, NB, endpoint=False)

# coarse hypothesis grid: 1 m positions with edt > 0.3 m, 15 deg headings
gx, gy = np.meshgrid(np.arange(0.5, W, 1.0), np.arange(0.5, H, 1.0))
gx, gy = gx.ravel(), gy.ravel()
keep = lookup(gx, gy) > 0.3
# edt capped at 1 -> use raw edt for the free test
ix = (gx / RES).astype(int); iy = (gy / RES).astype(int)
keep = edt[iy, ix] > 0.3
gx, gy = gx[keep], gy[keep]
heads = np.radians(np.arange(0, 360, 15))
HX = np.repeat(gx, len(heads)); HY = np.repeat(gy, len(heads)); HT = np.tile(heads, len(gx))
print(f"free-space coarse positions: {len(gx)}, hypotheses: {len(HX)}")

def score(ranges, trim):
    fin = np.isfinite(ranges)
    a = angles[fin]; rr = ranges[fin]
    # endpoints for all hypotheses: (Nh, Nbeams)
    c = np.cos(HT[:, None] + a[None, :]); s = np.sin(HT[:, None] + a[None, :])
    ex = HX[:, None] + rr[None, :] * c
    ey = HY[:, None] + rr[None, :] * s
    d = lookup(ex, ey)
    if trim < 1.0:
        k = int(math.ceil(trim * d.shape[1]))
        d = np.partition(d, k - 1, axis=1)[:, :k]
    return d.mean(axis=1)

def topk_clusters(sc, k=20, dpos=2.0, dang=math.radians(30)):
    order = np.argsort(sc)
    chosen = []
    for i in order:
        ok = True
        for j in chosen:
            if math.hypot(HX[i] - HX[j], HY[i] - HY[j]) < dpos and abs((HT[i] - HT[j] + math.pi) % (2 * math.pi) - math.pi) < dang:
                ok = False; break
        if ok:
            chosen.append(i)
            if len(chosen) == k:
                break
    return chosen

def correct(i, x, y, th):
    return math.hypot(HX[i] - x, HY[i] - y) <= 0.75 and abs((HT[i] - th + math.pi) % (2 * math.pi) - math.pi) <= math.radians(7.6)

n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 40
res = {}
for dyn in (0.0, 0.10):
    for trim in (1.0, 0.8):
        res[(dyn, trim)] = {"top1": 0, "top3": 0, "top20": 0, "m_true": [], "m_best_wrong": []}
for t in range(n_trials):
    while True:
        x, y = rng.uniform(1, W - 1), rng.uniform(1, H - 1)
        if edt[int(y / RES), int(x / RES)] > 0.4:
            break
    th = rng.uniform(-math.pi, math.pi)
    base = raycast(x, y, th, angles)
    for dyn in (0.0, 0.10):
        rr = base + rng.normal(0, 0.03, NB)
        if dyn > 0:  # dynamic occluders: shorten a contiguous 10 % block of beams to 0.5-2 m
            k0 = rng.integers(0, NB); nb = int(dyn * NB)
            idx = (k0 + np.arange(nb)) % NB
            rr[idx] = np.minimum(rr[idx], rng.uniform(0.5, 2.0))
        for trim in (1.0, 0.8):
            sc = score(rr, trim)
            top = topk_clusters(sc)
            flags = [correct(i, x, y, th) for i in top]
            r = res[(dyn, trim)]
            r["top1"] += flags[0]
            r["top3"] += any(flags[:3])
            r["top20"] += any(flags)
            corr_idx = [i for i in range(len(HX)) if False]
            # best correct-hypothesis score vs best wrong top cluster score
            cmask = (np.hypot(HX - x, HY - y) <= 0.75) & (np.abs((HT - th + np.pi) % (2 * np.pi) - np.pi) <= math.radians(7.6))
            r["m_true"].append(sc[cmask].min() if cmask.any() else np.nan)
            wrong = [sc[i] for i, f in zip(top, flags) if not f]
            r["m_best_wrong"].append(wrong[0] if wrong else np.nan)
for (dyn, trim), r in res.items():
    print(f"dyn={dyn:.2f} trim={trim:.1f}: correct@top1 {r['top1']}/{n_trials}, @top3 {r['top3']}/{n_trials}, @top20 {r['top20']}/{n_trials}; "
          f"median m(best correct coarse) {np.nanmedian(r['m_true']):.3f} m, median m(best wrong cluster) {np.nanmedian(r['m_best_wrong']):.3f} m")

# score of the exact true pose (fine-level) for m_ok calibration
mt = {0.0: [], 0.10: []}
for t in range(20):
    while True:
        x, y = rng.uniform(1, W - 1), rng.uniform(1, H - 1)
        if edt[int(y / RES), int(x / RES)] > 0.4:
            break
    th = rng.uniform(-math.pi, math.pi)
    base = raycast(x, y, th, angles)
    for dyn in (0.0, 0.10):
        rr = base + rng.normal(0, 0.03, NB)
        if dyn > 0:
            k0 = rng.integers(0, NB); nb = int(dyn * NB); idx = (k0 + np.arange(nb)) % NB
            rr[idx] = np.minimum(rr[idx], rng.uniform(0.5, 2.0))
        fin = np.isfinite(rr)
        d = lookup(x + rr[fin] * np.cos(th + angles[fin]), y + rr[fin] * np.sin(th + angles[fin]))
        k = int(math.ceil(0.8 * len(d)))
        mt[dyn].append((d.mean(), np.sort(d)[:k].mean()))
for dyn, v in mt.items():
    v = np.array(v)
    print(f"true pose, dyn={dyn:.2f}: capped mean m median {np.median(v[:,0]):.3f} (max {v[:,0].max():.3f}); 80% trimmed median {np.median(v[:,1]):.3f} (max {v[:,1].max():.3f})")

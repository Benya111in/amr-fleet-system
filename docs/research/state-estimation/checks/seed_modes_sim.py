"""Mode-ambiguity analysis for sec 4.C SEED (appended to seed_pipeline_sim.py setup): rank of the correct mode
 among fine modes by 80 % trimmed capped mean vs by dynamic-mask capped mean (see-through beams kept).
Run: python3 seed_modes_sim.py [n_trials]
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


# ---- mode-ambiguity analysis (appended): best variant only ----
ang90 = np.linspace(-math.pi, math.pi, 90, endpoint=False)
ang180 = np.linspace(-math.pi, math.pi, 180, endpoint=False)
def masked_score(x, y, th, r180, fin):
    # score at hypothesis with dynamic-beam mask: beams shorter than the expected map range by > 0.15 m are ignored,
    # beams longer than expected (see-through) are kept -> keeps discriminative evidence that trimming discards
    exp = raycast(x, y, th, ang180[fin])
    z = r180[fin]
    keep = ~(z < exp - 0.15)
    d = lookup((x + z * np.cos(th + ang180[fin]))[keep].astype(np.float32), (y + z * np.sin(th + ang180[fin]))[keep].astype(np.float32))
    return float(d.mean()), int(keep.sum())

n_trials = int(sys.argv[1]) if len(sys.argv) > 1 else 15
v = (0.5, 5, 50)
HX, HY, HT = coarse_grid(v[0], v[1])
res = []
for t in range(n_trials):
    while True:
        x, y = rng.uniform(1, W - 1), rng.uniform(1, H - 1)
        if edt[int(y / RES), int(x / RES)] > 0.4:
            break
    th = rng.uniform(-math.pi, math.pi)
    r180 = raycast(x, y, th, ang180) + rng.normal(0, 0.03, 180)
    k0 = rng.integers(0, 180); idx = (k0 + np.arange(18)) % 180
    r180[idx] = np.minimum(r180[idx], rng.uniform(0.5, 2.0))
    fin = np.isfinite(r180); r90 = r180[::2]; fin90 = np.isfinite(r90)
    sc = score(HX, HY, HT, r90[fin90].astype(np.float32), ang90[fin90])
    top = topk_clusters(HX, HY, HT, sc, v[2])
    best = []
    for i in top:
        fs = score(HX[i] + FX, HY[i] + FY, HT[i] + FT, r180[fin].astype(np.float32), ang180[fin], trim=0.8)
        j = int(np.argmin(fs)); best.append((float(fs[j]), float(HX[i] + FX[j]), float(HY[i] + FY[j]), float(HT[i] + FT[j])))
    best.sort(); modes = []
    for b_ in best:
        if all(math.hypot(b_[1] - m[1], b_[2] - m[2]) > 0.5 or abs(wrap(b_[3] - m[3])) > math.radians(10) for m in modes):
            modes.append(b_)
    corr = [math.hypot(m[1] - x, m[2] - y) <= 0.15 and abs(wrap(m[3] - th)) <= math.radians(3) for m in modes]
    rank_trim = corr.index(True) + 1 if any(corr) else None
    n_amb = sum(1 for m in modes if m[0] - modes[0][0] < 0.02)
    # re-score the 10 best modes with the masked score
    ms = [(masked_score(m[1], m[2], m[3], r180, fin)[0], c) for m, c in zip(modes[:10], corr[:10])]
    ms_sorted = sorted(ms, key=lambda q: q[0])
    rank_mask = [c for _, c in ms_sorted].index(True) + 1 if any(c for _, c in ms_sorted) else None
    margin_mask = ms_sorted[1][0] - ms_sorted[0][0] if len(ms_sorted) > 1 else 1.0
    res.append((rank_trim, n_amb, rank_mask, margin_mask, ms_sorted[0][1]))
    print(f"trial {t+1}: rank(trimmed) {rank_trim}, modes within 0.02 of best {n_amb}, rank(masked, top-10) {rank_mask}, masked margin {margin_mask:.3f}, masked-best correct {ms_sorted[0][1]}", flush=True)
rt = [r[0] for r in res]; rm = [r[2] for r in res]
print("SUMMARY: correct mode present in top-10 modes:", sum(1 for r in rt if r is not None and r <= 10), "/", n_trials,
      "; rank1 trimmed:", sum(1 for r in rt if r == 1), "; rank1 masked:", sum(1 for r in rm if r == 1),
      "; masked margin>0.02 & correct:", sum(1 for r in res if r[3] > 0.02 and r[4]), "; masked margin>0.02 & wrong:", sum(1 for r in res if r[3] > 0.02 and not r[4]),
      "; median ambiguity-set size:", float(np.median([r[1] for r in res])))

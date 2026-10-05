"""Second audit pass (2026-09-22): independent re-derivations for items still open after §9.1.

1. Euclidean tube raster size vs (2Lρ+πρ²)/r² (claimed '≤' in §4.2-3) and vs (2Lρ+4ρ²)/r².
2. SOR/Gauss-Seidel convergence of the Nav2 smoother sweep via exact spectral radius of the
   iteration matrix (independent of the time-stepping check in check_misc.py).
3. Post-processing (C3) worst-case work after the search deadline (§4.2-5 budget).
4. Safety gate vs C1 speed map if the continuous clearance limit acts on FORWARD clearance only
   (robot_params.yaml comment says '전방 여유 2.6 m'), zones isotropic.
5. Costmap CPU range for 5 robots x 5 Hz x (3..10) ms.
"""
import math
import numpy as np

r = 0.05

# ---------------------------------------------------------------- 1. tube
def seg_cells(L, ang):
    n = int(round(L / r)); a = math.radians(ang)
    return [(round(k * math.cos(a)), round(k * math.sin(a))) for k in range(n + 1)]

def euclid_tube(P, rho):
    k = int(math.ceil(rho / r)); disk = [(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)
                                          if math.hypot(i, j) * r <= rho + 1e-9]
    s = set()
    for (x, y) in P:
        for (i, j) in disk:
            s.add((x + i, y + j))
    return len(s)

print("1) Euclidean tube raster vs formulas")
worst_pi = 0.0; ok4 = True
for L in (0.5, 2.0, 6.0, 15.0):
    for rho in (1.0, 3.0):
        for ang in (0, 22.5, 45):
            n = euclid_tube(seg_cells(L, ang), rho)
            fpi = (2 * L * rho + math.pi * rho**2) / r**2
            f4 = (2 * L * rho + 4 * rho**2) / r**2
            worst_pi = max(worst_pi, n / fpi); ok4 &= n <= f4
            if L == 6.0:
                print(f"   L={L} rho={rho} ang={ang}: raster={n}  pi-formula={fpi:.0f} ({(n/fpi-1)*100:+.1f} %)  4-formula={f4:.0f}")
print(f"   max raster/pi-formula over grid = {worst_pi:.3f}  (>1 => '<=' claim false) ; raster <= 4-formula everywhere: {ok4}")

# ---------------------------------------------------------------- 2. smoother spectral radius
def gs_matrix(wd, ws, n=60):
    # interior unknowns y_1..y_{n-2}, ends fixed; x = 0 (error dynamics)
    m = n - 2
    D = np.eye(m) * (1 - wd - 2 * ws)       # coefficient on old y_i
    Lo = np.zeros((m, m)); Up = np.zeros((m, m))
    for i in range(m):
        if i > 0: Lo[i, i - 1] = ws           # y_{i-1} new
        if i < m - 1: Up[i, i + 1] = ws       # y_{i+1} old
    # y_new = D y_old + Lo y_new + Up y_old  => (I - Lo) y_new = (D + Up) y_old
    return np.linalg.solve(np.eye(m) - Lo, D + Up)

def jac_matrix(wd, ws, n=60):
    m = n - 2
    M = np.eye(m) * (1 - wd - 2 * ws)
    for i in range(m):
        if i > 0: M[i, i - 1] = ws
        if i < m - 1: M[i, i + 1] = ws
    return M

print("2) smoother spectral radius (n=60)")
for wd, ws in ((0.2, 0.3), (0.2, 0.5), (0.2, 0.85), (0.2, 0.89), (0.2, 0.91), (0.2, 0.95)):
    rg = max(abs(np.linalg.eigvals(gs_matrix(wd, ws))))
    rj = max(abs(np.linalg.eigvals(jac_matrix(wd, ws))))
    print(f"   (w_d,w_s)=({wd},{ws}) omega={wd+2*ws:.2f}: rho_GS={rg:.4f} rho_Jacobi={rj:.4f}")

# ---------------------------------------------------------------- 3. post-processing work
print("3) C3 post-processing worst-case lookups")
Lmax, ds_s, spiral_cells = 10.0, 0.25, sum(1 for i in range(-56, 57) for j in range(-56, 57) if math.hypot(i, j) <= 56)
for Lpath in (40, 100, 150):
    M = int(Lpath / r * 1.1)
    shortcut = 2 * M * (Lmax / (r / 2))            # 2 passes, each anchor may scan chords up to L_max, sampled r/2
    spiral = (Lpath / ds_s) * spiral_cells
    smooth = 30 * M * 10
    tot = shortcut + spiral + smooth
    print(f"   path {Lpath} m: M={M}, shortcut<={shortcut:.2e}, spiral<={spiral:.2e} (disk {spiral_cells}), smooth~{smooth:.1e}"
          f" -> total {tot:.2e} lookups = {tot*2e-9*1e3:.0f}-{tot*5e-9*1e3:.0f} ms at 2-5 ns")

# ---------------------------------------------------------------- 4. forward-only continuous limit
A, T, DSTOP, VMAX, VFLOOR, RINS, S = 1.0, 0.15, 0.30, 2.0, 0.2, 0.20, 2.0
def v_cont(D):
    return 0.0 if D <= DSTOP else min(VMAX, -A * T + math.sqrt((A * T) ** 2 + 2 * A * (D - DSTOP)))
def gate_iso(D):
    if D <= 0.3: return 0.0
    if D <= 0.5: return min(0.2, v_cont(D))
    if D < 1.0: return min(0.5, v_cont(D))
    return v_cont(D)
def gate_lat_fwdonly(D):      # lateral obstacle, continuous limit only on forward clearance
    if D <= 0.3: return 0.0
    if D <= 0.5: return 0.2
    if D < 1.0: return 0.5
    return VMAX
def d_lo(c): return RINS - math.log((c + 1) / 252) / S
def d_hi(c): return RINS - math.log(c / 252) / S
def v_lim(c): return min(max(math.sqrt(2 * A * max(d_lo(c) - RINS, 0)), VFLOOR), VMAX)
print("4) lower-bound property if continuous limit is forward-only (lateral obstacle)")
for R, name in ((1.2, "A"), (2.2, "B")):
    cells = {}
    n = int(R / r) + 2
    for i in range(n):
        for j in range(i, n):
            d = math.hypot(i, j) * r
            if RINS < d <= R + 1e-12:
                c = int(252 * math.exp(-S * (d - RINS)))
                if 1 <= c <= 252: cells[c] = d
    bad = sorted(c for c in cells if gate_lat_fwdonly(min(d_hi(c), R) - RINS) > v_lim(c) + 1e-9)
    bad_iso = sorted(c for c in cells if gate_iso(min(d_hi(c), R) - RINS) > v_lim(c) + 1e-9)
    print(f"   config {name}: violating c (fwd-only) = {bad[:3]}..{bad[-3:] if bad else ''} (n={len(bad)}), "
          f"edge D range {min((min(d_hi(c),R)-RINS) for c in bad) if bad else float('nan'):.2f}-"
          f"{max((min(d_hi(c),R)-RINS) for c in bad) if bad else float('nan'):.2f} m ; isotropic violations: {bad_iso}")
for w in (2.4, 3.0):
    D = w / 2 - RINS
    print(f"   aisle {w} m centred D={D:.2f}: isotropic gate {gate_iso(D):.2f} m/s, fwd-only gate {gate_lat_fwdonly(D):.2f} m/s")

# ---------------------------------------------------------------- 5. costmap CPU
print("5) costmap: 5 robots x 5 Hz x 3..10 ms =", 25 * 3e-3, "-", 25 * 10e-3, "core")

"""rev4 (2026-09-22) — dynamics / association / classification checks for perception-tracking.md.
V  windowed LS-slope velocity test: latency per speed & direction, static false alarms per track-hour,
   robustness to rotating residual bias (robot passing) and to AMCL jumps (map frame) vs odom frame.
FS free-space (ray) evidence: latency model per motion direction.
D  DATMO log-odds score truth table (corrected sign/prior/unknown-cell handling).
J  IMM pieces: CT Jacobian (Cartesian velocity) vs numeric, spread-of-means gate covariance,
   class speed-gate envelope under the 0.95 cap, Pi_c row sums, class posterior update.
K  augmented GNN assignment (miss/birth costs, NLL with log-det) vs brute force; 'coasting track steals' example.
L  prediction covariance growth (CWNA) and TTC detection-range requirement.
M  latency / GPU budget arithmetic."""
import math, itertools
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2, norm

rng = np.random.default_rng(7)
P = print
dt = 0.1
sig = lambda x: 1 / (1 + math.exp(-x)); logit = lambda p: math.log(p / (1 - p))

# ============ V. windowed LS-slope velocity test ============
def ls_test_series(Z, Rdiag, W=10, nmin=4):
    """Z: (N,T,2) centroids in LOS coords (x=LOS, y=lateral). returns T-stat and |v| arrays (N,T)."""
    N, T, _ = Z.shape
    Ts = np.zeros((N, T)); Vn = np.zeros((N, T))
    for k in range(T):
        n = min(k + 1, W)
        if n < nmin:
            continue
        t = np.arange(n) * dt; tc = t - t.mean(); Stt = (tc ** 2).sum()
        z = Z[:, k - n + 1:k + 1, :]
        v = (tc[None, :, None] * (z - z.mean(1, keepdims=True))).sum(1) / Stt
        Ts[:, k] = v[:, 0] ** 2 / (Rdiag[0] / Stt) + v[:, 1] ** 2 / (Rdiag[1] / Stt)
        Vn[:, k] = np.hypot(v[:, 0], v[:, 1])
    return Ts, Vn

def fire(Ts, Vn, thr, vmin, consec):
    ok = (Ts > thr) & (Vn > vmin)
    run = np.zeros_like(ok, dtype=int)
    for k in range(ok.shape[1]):
        run[:, k] = np.where(ok[:, k], (run[:, k - 1] + 1) if k > 0 else 1, 0)
    return run >= consec

def simulate(v, direction, R, N=1500, T=40, bias=None, jump=None):
    t = np.arange(T) * dt
    d = np.array([1.0, 0.0]) if direction == "los" else np.array([0.0, 1.0])
    Z = v * t[None, :, None] * d[None, None, :] + rng.normal(0, 1, (N, T, 2)) * np.sqrt(R)[None, None, :]
    if bias is not None:          # residual bias vector of length b rotating at omega (robot passing)
        b, om = bias; ang = om * t + rng.uniform(0, 2 * np.pi, (N, 1))
        Z = Z + b * np.stack([np.cos(ang), np.sin(ang)], -1)
    if jump is not None:          # map-frame AMCL correction step at k0
        k0, J = jump; Z[:, k0:, 0] += J
    return Z

THR, VMIN, CONS, WIN = chi2.ppf(0.999, 2), 0.15, 2, 10
P(f"V. LS-slope test: window W={WIN} scans, T > chi2_2(0.999)={THR:.2f} AND |v|>{VMIN} m/s on {CONS} consecutive scans")
classes_R = {"person@5m (0.031,0.020)": np.array([0.031 ** 2, 0.020 ** 2]),
             "unknown@5m (0.12,0.03)": np.array([0.12 ** 2, 0.03 ** 2])}
for name, R in classes_R.items():
    for direction in ("lateral", "los"):
        row = []
        for v in (0.3, 0.5, 1.0, 1.5):
            Ts, Vn = ls_test_series(simulate(v, direction, R), R, WIN)
            F = fire(Ts, Vn, THR, VMIN, CONS)
            first = np.where(F.any(1), F.argmax(1) * dt, np.nan)
            row.append(f"v={v}: med {np.nanmedian(first):.1f}s p90 {np.nanpercentile(first, 90):.1f}s ({np.mean(~np.isnan(first))*100:.0f}%)")
        P(f"   {name:24s} {direction:7s}: " + " | ".join(row))
# static false alarms: rising edges per track-hour
for name, R in classes_R.items():
    N, T = 400, 600
    Ts, Vn = ls_test_series(simulate(0.0, "los", R, N=N, T=T), R, WIN)
    F = fire(Ts, Vn, THR, VMIN, CONS)
    edges = (F[:, 1:] & ~F[:, :-1]).sum()
    P(f"   static {name}: {edges} rising edges in {N*T*dt/3600:.2f} track-h -> {edges/(N*T*dt/3600):.2f} /track-h")
# robot passing a static object: residual LOS bias rotating with the line of sight
R = classes_R["person@5m (0.031,0.020)"]
for b in (0.03, 0.06, 0.23):
    for om in (1.0, 2.0):
        Ts, Vn = ls_test_series(simulate(0.0, "los", R, N=800, T=30, bias=(b, om)), R, WIN)
        F = fire(Ts, Vn, THR, VMIN, CONS)
        P(f"   static, rotating bias {b:.2f} m @ {om} rad/s (apparent {b*om:.2f} m/s): false-dynamic runs {F.any(1).mean()*100:.1f}% (3 s)")
for J in (0.05, 0.08, 0.30):
    Ts, Vn = ls_test_series(simulate(0.0, "los", R, N=800, T=30, jump=(10, J)), R, WIN)
    F = fire(Ts, Vn, THR, VMIN, CONS)
    P(f"   map-frame AMCL step {J:.2f} m on a static object: false-dynamic runs {F.any(1).mean()*100:.1f}%  (odom frame: no step)")
P("   (baseline B1 KF chi2 3-consecutive test at 0.3 m/s: 0% within 3 s for q=0.26/0.5 -> see rev3_checks.py C / critique)")

# ============ FS. free-space (ray) evidence latency ============
P("FS. free-space violation evidence (beam endpoint >= 0.10 m beyond the point in >=2 of last K=5 scans):")
pstat = norm.sf(0.10 / (0.03 * math.sqrt(2)))
P(f"   static point: P(violation per scan pair)={pstat:.4f}; need 2 of 5 -> ~{1 - sum(math.comb(5, i) * pstat**i * (1-pstat)**(5-i) for i in (0, 1)):.1e} per point")
for v in (0.3, 1.0, 1.5):
    appr = 0.10 / v + dt                       # displacement past margin, next scan
    strip = 0.4 * v                            # leading strip seen free in >=2 of 5 scans (K-K_f+1=4 scans)
    lat_person = max(0.0, (0.5 * 0.30) / v) + dt   # stop->go: time until strip covers half of 0.30 m visible width
    P(f"   v={v}: approaching {appr:.2f} s | lateral steady-state rho_free=min(1,0.4v/W)= {min(1, strip/0.30):.2f} (person W=0.30) ; stop->go rho>=0.5 after {lat_person:.2f} s | receding: no evidence (occluded)")

# ============ D. DATMO consistency log-odds ============
pmove = dict(box=0.02, sign=0.01, person=0.60, forklift=0.50, amr=0.60)
pi0 = dict(box=0.30, sign=0.05, person=0.30, forklift=0.15, amr=0.20)   # LiDAR-visible cluster base rates (to be re-estimated)
pim = sum(pi0[c] * pmove[c] for c in pi0)
def score(rho_map, rho_free, vel, stat, p, w=(4.0, 4.0, 4.0, 2.0), rho0=0.5):
    w1, w2, w3, w4 = w
    pm = sum(p[c] * pmove[c] for c in p)
    l = logit(pim) + (logit(pm) - logit(pim))
    l += -w1 * max(0.0, (rho_map - rho0) / (1 - rho0)) + w2 * rho_free + (w3 if vel else 0.0) - (w4 if stat else 0.0)
    return sig(l)
U = pi0
box = dict(box=.8, sign=.05, person=.05, forklift=.05, amr=.05); per = dict(box=.05, sign=.05, person=.8, forklift=.05, amr=.05)
fork = dict(box=.05, sign=.05, person=.05, forklift=.8, amr=.05)
cases = [("mapped shelf leg", .95, 0, 0, 0, U), ("un-mapped static pallet, early", 0, 0, 0, 0, U),
         ("un-mapped static pallet, 1 s observed static", 0, 0, 0, 1, U), ("un-mapped static, noise rho_free .05", 0, .05, 0, 0, U),
         ("large box p(box)=.8", 0, 0, 0, 0, box), ("standing person p=.8", 0, 0, 0, 0, per),
         ("standing person p=.8, 1 s static", 0, 0, 0, 1, per), ("parked forklift p=.8, 1 s static", 0, 0, 0, 1, fork),
         ("walking person 0.2 m from shelf (rho_map .6), free .8, vel", .6, .8, 1, 0, U),
         ("person 0.3 m/s lateral, early (free .4, no vel)", 0, .4, 0, 0, U), ("person approaching (free 1.0), no class", 0, 1.0, 0, 0, U),
         ("person receding (vel only)", 0, 0, 1, 0, U), ("pushed box 1 m/s, p(box)=.8, free .9, vel", 0, .9, 1, 0, box)]
P(f"D. DATMO log-odds: pi_move = sum pi0*p_move = {pim:.3f} (logit {logit(pim):+.2f}); w=(w_map,w_free,w_vel,w_static)=(4,4,4,2)")
for name, rm, rf, vl, st, p in cases:
    P(f"   {name:58s} P(dyn)={score(rm, rf, vl, st, p):.3f}")
# the ORIGINAL formula for comparison (critique item)
pd_orig = dict(box=0.05, person=0.90, sign=0.02, forklift=0.80, amr=0.90)
pu = sum(pd_orig.values()) / 5
P(f"   original formula, un-mapped static, uniform class: P={sig(4*(0.5-0.0) + logit(pu)):.3f} (critique: 0.894)")

# ============ J. IMM pieces ============
def ct_f(x, T=dt):
    px, py, vx, vy, w = x
    if abs(w) < 1e-4:
        return np.array([px + T * vx - 0.5 * T * T * w * vy, py + T * vy + 0.5 * T * T * w * vx, vx - T * w * vy, vy + T * w * vx, w])
    s, c = math.sin(w * T), math.cos(w * T)
    return np.array([px + s / w * vx - (1 - c) / w * vy, py + (1 - c) / w * vx + s / w * vy, c * vx - s * vy, s * vx + c * vy, w])
def ct_F(x, T=dt):
    px, py, vx, vy, w = x
    F = np.eye(5)
    if abs(w) < 1e-4:
        F[0, 2] = T; F[1, 3] = T; F[0, 4] = -0.5 * T * T * vy; F[1, 4] = 0.5 * T * T * vx
        F[2, 4] = -T * vy; F[3, 4] = T * vx
        return F
    s, c = math.sin(w * T), math.cos(w * T)
    F[0, 2] = s / w; F[0, 3] = -(1 - c) / w; F[1, 2] = (1 - c) / w; F[1, 3] = s / w
    F[2, 2] = c; F[2, 3] = -s; F[3, 2] = s; F[3, 3] = c
    F[0, 4] = vx * (T * c * w - s) / w ** 2 - vy * (T * s * w - (1 - c)) / w ** 2
    F[1, 4] = vx * (T * s * w - (1 - c)) / w ** 2 + vy * (T * c * w - s) / w ** 2
    F[2, 4] = -T * s * vx - T * c * vy
    F[3, 4] = T * c * vx - T * s * vy
    return F
maxerr = 0.0
for x in [np.array([1, 2, 1.2, -0.4, 0.8]), np.array([0, 0, 0.0, 0.0, 1.2]), np.array([3, 1, 1.5, 0.3, 2e-5]), np.array([0, 0, -0.7, 1.1, -1.5])]:
    Fn = np.zeros((5, 5)); h = 1e-6
    for i in range(5):
        e = np.zeros(5); e[i] = h; Fn[:, i] = (ct_f(x + e) - ct_f(x - e)) / (2 * h)
    maxerr = max(maxerr, np.abs(Fn - ct_F(x)).max())
P(f"J. CT (Cartesian velocity) Jacobian vs central differences: max abs error {maxerr:.2e} (incl. v=0 and |w|<1e-4 branch)")
zhat = [np.array([0, 0.]), np.array([0.3, 0.])]; S = [np.eye(2) * 0.05, np.eye(2) * 0.05]; cb = [0.5, 0.5]
zb = sum(c * z for c, z in zip(cb, zhat))
P("   gate covariance diag: per-mode average", np.diag(sum(c * s for c, s in zip(cb, S))), "-> with spread-of-means", np.diag(sum(c * (s + np.outer(z - zb, z - zb)) for c, s, z in zip(cb, S, zhat))))
vmx = dict(box=0.5, sign=0.5, person=2.0, forklift=3.0, amr=2.0)
pc = dict(box=0.95, sign=0.0125, person=0.0125, forklift=0.0125, amr=0.0125)
for tau in (0.01, 0.0125, 0.02, 0.05):
    env = max(vmx[c] for c in pc if pc[c] >= tau)
    P(f"   speed-gate envelope under 0.95 cap, tau={tau}: v_gate={env} m/s  (mean-based: {sum(pc[c]*vmx[c] for c in pc):.3f})")
Pi = {"person": [[.90, .08, .02], [.05, .85, .10], [.05, .15, .80]], "forklift": [[.95, .05, 0], [.02, .90, .08], [.02, .08, .90]],
      "amr": [[.95, .05, 0], [.02, .93, .05], [.02, .08, .90]], "box": [[.99, .01, 0], [.30, .70, 0], [.30, .20, .50]],
      "sign": [[.99, .01, 0], [.30, .70, 0], [.30, .20, .50]]}
P("   Pi_c row sums:", {c: [round(sum(r), 6) for r in m] for c, m in Pi.items()})
# class posterior: score-binned confusion likelihood, once per LiDAR cycle, cap .95, forgetting eta
C = ["box", "sign", "person", "forklift", "amr"]
M_hi = np.full((5, 5), 0.02); np.fill_diagonal(M_hi, 0.92)   # rows: true class, cols: predicted (score >= 0.6)
M_lo = np.full((5, 5), 0.075); np.fill_diagonal(M_lo, 0.70)  # score in [0.35, 0.6)
def upd(p, pred, M, cap=0.95, eta=0.02, p0=np.array([pi0[c] for c in C])):
    p = (1 - eta) * p + eta * p0
    p = p * M[:, pred]; p = p / p.sum()
    if p.max() > cap:
        i = p.argmax(); ex = p[i] - cap; p[i] = cap; o = [j for j in range(5) if j != i]; p[o] += ex * p[o] / p[o].sum()
    return p
p = np.array([pi0[c] for c in C]); out = []
for k in range(4):
    p = upd(p, 2, M_hi); out.append(f"{p[2]:.3f}")
P("   p(person) after k=1..4 LiDAR cycles with a high-score 'person' hit:", out)
p = np.array([pi0[c] for c in C]); out = []
for k in range(4):
    p = upd(p, 2, M_lo); out.append(f"{p[2]:.3f}")
P("   ... with low-score hits:", out)
p = np.array([pi0[c] for c in C]); p = upd(p, 2, M_hi); p = upd(p, 2, M_hi)
hist = []
for k in range(1, 61):
    p = upd(p, 0, np.ones((5, 5)))   # no camera evidence (out of FOV): likelihood 1, forgetting only
    if k in (10, 30, 60): hist.append(f"{k*dt:.0f}s:{p[2]:.2f}")
P("   decay without camera evidence (eta=0.02/cycle):", hist)

# ============ K. augmented GNN assignment ============
def build(d2, logdet, PD, lamB, gate=chi2.ppf(0.99, 2), BIG=1e6):
    n, k = d2.shape; M = np.full((n + k, k + n), BIG)
    for i in range(n):
        for j in range(k):
            if d2[i, j] <= gate:
                M[i, j] = d2[i, j] + logdet[i] - 2 * math.log(PD[i])
        M[i, k + i] = -2 * math.log(1 - PD[i])
    for j in range(k):
        M[n + j, j] = -2 * math.log(lamB)
    M[n:, k:] = 0.0
    return M
def brute(M):
    n = M.shape[0]; best = None
    for perm in itertools.permutations(range(n)):
        s = sum(M[i, perm[i]] for i in range(n))
        if best is None or s < best[0] - 1e-9: best = (s, perm)
    return best
agree = 0; trials = 150
for _ in range(trials):
    n, k = rng.integers(1, 4), rng.integers(1, 4)
    d2 = rng.uniform(0, 14, (n, k)); ld = rng.uniform(-9, -4, n); PD = rng.uniform(0.5, 0.95, n)
    M = build(d2, ld, PD, 0.01)
    r, c = linear_sum_assignment(M)
    agree += abs(M[r, c].sum() - brute(M)[0]) < 1e-6
P(f"K. augmented (n+k)x(k+n) assignment: scipy LSA == brute force in {agree}/{trials} random problems (finite BIG, no inf)")
S_conf, S_coast = 0.1 ** 2, 0.5 ** 2
d_conf, d_coast = 0.15, 0.30
ld = lambda s2: math.log((2 * math.pi) ** 2 * s2 * s2)
d2 = np.array([[d_conf ** 2 / S_conf], [d_coast ** 2 / S_coast]])
Mx = build(d2, np.array([ld(S_conf), ld(S_coast)]), np.array([0.9, 0.5]), 0.01)
r, c = linear_sum_assignment(Mx)
P(f"   example: confirmed track (sigma .10, 0.15 m away, d2={d2[0,0]:.2f}) vs coasting track (sigma .50, 0.30 m away, d2={d2[1,0]:.2f}) -> det goes to track {int(r[list(c).index(0)])} (0=confirmed)")
P(f"   d2-only cost would pick track {int(np.argmin(d2[:, 0]))} (coasting steals)")
P(f"   birth cost -2 ln(lambda_B=0.01 m^-2) = {-2*math.log(0.01):.2f}; miss cost PD=.9: {-2*math.log(0.1):.2f}, PD=.5: {-2*math.log(0.5):.2f}")

# ============ L. prediction covariance growth, TTC range ============
def kf_ss(q, R, iters=400):
    F = np.eye(4); F[0, 2] = F[1, 3] = dt
    Q = q * np.array([[dt**3/3, 0, dt**2/2, 0], [0, dt**3/3, 0, dt**2/2], [dt**2/2, 0, dt, 0], [0, dt**2/2, 0, dt]])
    H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1; Rm = np.diag(R); Pm = np.eye(4)
    for _ in range(iters):
        Pm = F @ Pm @ F.T + Q; S_ = H @ Pm @ H.T + Rm; K = Pm @ H.T @ np.linalg.inv(S_); Pm = (np.eye(4) - K @ H) @ Pm
    return Pm
for q in (0.1, 0.25, 0.5):
    Pss = kf_ss(q, [0.031 ** 2, 0.02 ** 2])
    row = []
    for tau in (1.0, 1.5, 3.0):
        Pp = Pss[0, 0] + 2 * tau * Pss[0, 2] + tau ** 2 * Pss[2, 2] + q * tau ** 3 / 3
        row.append(f"tau={tau}s: {math.sqrt(Pp):.2f} m")
    P(f"L. CWNA q={q} m^2/s^3: steady sigma_v={math.sqrt(Pss[2,2]):.3f} m/s; predicted position sigma " + ", ".join(row) + f"; 1-s velocity-change sd sqrt(q)={math.sqrt(q):.2f} m/s")
for vr, vo in ((2.0, 1.0), (1.0, 1.0), (2.0, 1.5)):
    for lat in (0.3, 0.8):
        P(f"   head-on closing {vr}+{vo} m/s, tau_warn 3.0 s, perception latency {lat} s -> required confirmed-track range {(vr+vo)*(3.0+lat):.1f} m")
P(f"   robot circumscribed radius sqrt(0.3^2+0.2^2) = {math.hypot(0.3, 0.2):.3f} m")

# ============ M. latency / GPU budget ============
P(f"M. 5 streams x 30 FPS = 150 img/s -> per-image budget {1000/150:.2f} ms; batch of 5 at 30 Hz -> per-batch budget {1000/30:.1f} ms")
P(f"   cross-robot batching wait <= one frame period {1000/30:.1f} ms; depth pairing: RGB 30 Hz vs depth 15 Hz -> 3D rate 15 Hz, max RGB-depth skew {1000/60:.1f} ms with slop 17 ms")
P(f"   skew pixel shift at max yaw rate 1.5 rad/s: fx*w*dt = {337.2*1.5*(1/60):.1f} px ; object 1.5 m/s x 16.7 ms = {1.5/60*100:.1f} cm")
P(f"   CPU-budget ref (multi_robot.md s7): 32 thr x 80% = {32*0.8:.1f} cores")

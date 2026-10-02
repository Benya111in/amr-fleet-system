"""Re-derivation checks for the revised brief (perception-tracking, rev 2)."""
import math, itertools
import numpy as np
rng = np.random.default_rng(0)
sig = lambda x: 1/(1+math.exp(-x)); logit = lambda p: math.log(p/(1-p))
sig_r = 0.03; dphi = math.radians(0.5); dt = 0.1

# ---------- A. corrected R_cl ----------
def R_cl(r, L_perp, sigma_delta, L_perp_unobs=0.0, floor=0.02):
    n = max(3, int(L_perp/(r*dphi)))
    s_par2 = sig_r**2/n + sigma_delta**2
    s_perp2 = (r*dphi)**2/24 + L_perp_unobs**2/12
    return n, math.sqrt(max(s_par2, floor**2)), math.sqrt(max(s_perp2, floor**2))
print("A. R_cl (corrected): class-conditioned half-depth shift applied, residual sigma_delta")
for name, Lp, sd in [("person", 0.40, 0.05), ("box", 0.50, 0.10), ("unknown", 0.50, 0.12)]:
    for r in (3, 5, 10):
        n, sp, sq = R_cl(r, Lp, sd)
        print(f"   {name:8s} r={r:2d} m n={n:2d} sigma_par={sp:.3f} sigma_perp={sq:.3f}")

# ---------- B. camera 3D error with fx from 87 deg HFOV ----------
fx = 320/math.tan(math.radians(43.5)); kappa = 0.1
print(f"B. fx(87deg HFOV @640) = {fx:.0f} px ; sigma_X = kappa*L_obj (Z-independent)")
for Z in (2, 3, 5, 8):
    sZ = 0.01 + 0.002*Z*Z
    for L in (0.4, 0.5):
        w = fx*L/Z; su = kappa*w; sX = Z/fx*su
        print(f"   Z={Z} m L={L} m: w={w:.0f}px sigma_u={su:.1f}px sigma_X={sX*100:.1f} cm sigma_Z={sZ*100:.1f} cm RMS={math.hypot(sX,sZ)*100:.1f} cm")

# ---------- C. velocity-test latency Monte Carlo (CV-KF, corrected R) ----------
def sim(q, sp, sq, v, N=600, consec=3, thr=9.21, P0v=4.0, T=3.0, jump=None, rot_bias=None):
    F = np.eye(4); F[0,2] = dt; F[1,3] = dt
    Q = q*np.array([[dt**3/3,0,dt**2/2,0],[0,dt**3/3,0,dt**2/2],[dt**2/2,0,dt,0],[0,dt**2/2,0,dt]])
    H = np.zeros((2,4)); H[0,0] = 1; H[1,1] = 1
    R = np.diag([sp**2, sq**2]); steps = int(T/dt); lat = []
    for _ in range(N):
        x = np.array([0,0,v,0.0]); z0 = x[:2] + rng.multivariate_normal([0,0],R)
        xh = np.array([z0[0],z0[1],0,0.0]); P = np.diag([R[0,0],R[1,1],P0v,P0v]); cnt = 0; done = None
        for k in range(1, steps):
            x = F@x; z = x[:2] + rng.multivariate_normal([0,0],R)
            if jump is not None and k == jump[0]: z = z + np.array(jump[1])
            if rot_bias is not None:   # residual bias b rotating with LOS while robot passes
                b, omega = rot_bias; ang = omega*k*dt; z = z + b*np.array([math.cos(ang), math.sin(ang)])
            xh = F@xh; P = F@P@F.T + Q; S = H@P@H.T + R; K = P@H.T@np.linalg.inv(S); nu = z - H@xh
            xh = xh + K@nu; I = np.eye(4); P = (I-K@H)@P@(I-K@H).T + K@R@K.T
            Tst = xh[2:]@np.linalg.solve(P[2:,2:], xh[2:]); cnt = cnt+1 if Tst > thr else 0
            if cnt >= consec: done = k*dt; break
        lat.append(done if done is not None else np.nan)
    lat = np.array(lat); ok = ~np.isnan(lat)
    return (np.nanmedian(lat) if ok.any() else float('nan')), (np.nanpercentile(lat,90) if ok.any() else float('nan')), ok.mean()
print("C. velocity-path latency (3 consecutive chi2 tests), person@5m corrected R")
n, sp, sq = R_cl(5, 0.40, 0.05)
for q in (0.26, 0.5):
    for v in (0.3, 1.0, 1.5):
        m, p90, ok = sim(q, sp, sq, v)
        print(f"   q={q} v={v}: median={m:.2f}s p90={p90:.2f}s conf_rate(3s)={ok:.2f}")
    print(f"   q={q} static false-dyn rate (5 s): {sim(q, sp, sq, 0.0, T=5.0)[2]:.3f}")
    print(f"   q={q} static + 5 cm AMCL jump at t=1s (map frame): false-dyn rate {sim(q, sp, sq, 0.0, T=5.0, jump=(10,[0.05,0.0]))[2]:.3f}")
    print(f"   q={q} static + 8 cm AMCL jump at t=1s (map frame): false-dyn rate {sim(q, sp, sq, 0.0, T=5.0, jump=(10,[0.08,0.0]))[2]:.3f}")
    print(f"   q={q} static, uncorrected bias 0.20 m rotating at 0.8 rad/s (passing): {sim(q, sp, sq, 0.0, T=3.0, rot_bias=(0.20,0.8))[2]:.3f}")
    print(f"   q={q} static, residual bias 0.05 m rotating at 0.8 rad/s (passing): {sim(q, sp, sq, 0.0, T=3.0, rot_bias=(0.05,0.8))[2]:.3f}")

# ---------- D. free-space path latency (geometric) ----------
print("D. free-space path: time until rho_free >= 0.5 (lateral motion, K=10 scans window)")
for v in (0.3, 1.0, 1.5):
    for name, L in (("person lateral", 0.40), ("person approaching (arc depth 0.13)", 0.13), ("box lateral", 0.50)):
        t = 0.5*L/v
        print(f"   v={v} {name:36s}: {t:.2f} s (+0.1 s scan period, +lifecycle)")
print("   receding motion: leading cells were occluded -> no free-space evidence -> velocity path only")

# ---------- E. MPFS truth table (corrected formula) ----------
pdyn = dict(box=0.05, person=0.90, sign=0.02, forklift=0.80, amr=0.90)
pi0 = dict(box=0.55, person=0.15, sign=0.10, forklift=0.10, amr=0.10)
pd0 = sum(pi0[c]*pdyn[c] for c in pi0)
def score(rho_map, rho_free, vel, p, w, pi_dyn=0.15, rho0=0.5):
    w1, w2, w3 = w
    l = logit(pi_dyn) - w1*max(0.0, (rho_map-rho0)/(1-rho0)) + w2*rho_free + w3*(1 if vel else 0)
    pdc = sum(p[c]*pdyn[c] for c in p); l += logit(pdc) - logit(pd0)
    return sig(l)
U = pi0
B = dict(box=0.8, person=0.05, sign=0.05, forklift=0.05, amr=0.05)
Pp = dict(box=0.05, person=0.8, sign=0.05, forklift=0.05, amr=0.05)
cases = [("mapped shelf, no class", .95, 0, 0, U), ("un-mapped static box, no class", 0, 0, 0, U),
         ("un-mapped static box, p(box)=.8", 0, 0, 0, B), ("standing person, p(person)=.8", 0, 0, 0, Pp),
         ("walking person near shelf, no class", .6, .8, 1, U), ("walking person near shelf, p=.8", .6, .8, 1, Pp),
         ("walking person open floor, no class", 0, .9, 1, U), ("pushed box 1 m/s, p(box)=.8", 0, .9, 1, B),
         ("moving box, free-space only (early)", 0, .9, 0, B), ("walking person, vel only (receding)", 0, 0, 1, Pp),
         ("person, free-space only (early, lateral 0.3 s)", 0, .5, 0, Pp), ("unknown, free-space 0.5 only", 0, .5, 0, U),
         ("static object briefly in raytrace shadow edge (rho_free .2)", 0, .2, 0, U),
         ("parked forklift, p(forklift)=.8, no motion", 0, 0, 0, dict(box=.05, person=.05, sign=.05, forklift=.8, amr=.05))]
print(f"E. MPFS truth table; p_dyn(pi0)={pd0:.3f}, pi_dyn=0.15")
for w in [(3, 4, 3), (3, 5, 3)]:
    print(f"   --- w={w}")
    for name, rm, rf, vp, p in cases:
        print(f"      {name:58s} P(dyn)={score(rm, rf, vp, p, w):.3f}")

# ---------- F. IMM association covariance spread-of-means ----------
zhat = [np.array([0, 0.]), np.array([0.3, 0.])]; S = [np.eye(2)*0.05, np.eye(2)*0.05]; cbar = [0.5, 0.5]
zbar = sum(c*z for c, z in zip(cbar, zhat))
print("F. IMM S: naive", np.diag(sum(c*s for c, s in zip(cbar, S))), "with spread", np.diag(sum(c*(s+np.outer(z-zbar, z-zbar)) for c, s, z in zip(cbar, S, zhat))))

# ---------- G. augmented assignment with confidence ----------
def brute(M):
    n, m = M.shape; best = None
    for perm in itertools.permutations(range(m), n):
        s = sum(M[i, perm[i]] for i in range(n))
        if best is None or s < best[0]: best = (s, perm)
    return best
def augmented(d2, c, gamma=9.21, BIG=1e6, lam=2.0):
    n, k = d2.shape; M = np.full((n+k, k+n), BIG)
    for i in range(n):
        for j in range(k):
            M[i, j] = d2[i, j] - lam*math.log(c[i]) if d2[i, j] <= gamma else BIG
        M[i, k+i] = gamma
    for j in range(k): M[n+j, j] = gamma
    M[n:, k:] = 0.0
    return M
d2 = np.array([[4.0], [5.0]]); c = [0.3, 0.9]
print("G. 2 tracks compete for 1 det: d2=(4,5), conf=(0.3,0.9) ->", brute(augmented(d2, c))[1], "(row index 1 = confident track wins)")
c = [0.9, 0.9]; print("   same d2, equal conf ->", brute(augmented(d2, c))[1], "(nearest wins)")
d2 = np.array([[9.0, 30.0], [30.0, 8.0]]); c = [0.2, 0.9]
print("   2x2 in-gate diagonal, conf (0.2,0.9):", brute(augmented(d2, c))[1], "(both assigned: in-gate beats miss+birth)")

# ---------- H. class posterior update (new likelihood, once per LiDAR cycle) ----------
C = 5; Mconf = np.full((C, C), 0.025); np.fill_diagonal(Mconf, 0.9); s = 0.8
p = np.array([0.55, 0.15, 0.10, 0.10, 0.10]); cap = 0.95
print("H. class posterior after k consistent 'person' hits (s=0.8, 10 Hz, cap 0.95)")
for k in range(1, 6):
    lik = (1-s)/C + s*Mconf[1, :]; p = p*lik; p /= p.sum()
    if p.max() > cap:
        i = p.argmax(); excess = p[i]-cap; p[i] = cap; others = [j for j in range(C) if j != i]; p[others] += excess*p[others]/p[others].sum()
    print(f"   k={k} ({k*0.1:.1f} s): p(person)={p[1]:.3f}")

# ---------- I. NEES band ----------
for N in (100, 1000, 5000): print(f"I. NEES 2-DoF mean band N={N}: 2 +/- {1.96*math.sqrt(4/N):.3f}")

# ---------- J. GPU / E2E budget ----------
print(f"J. per-image {1000/150:.1f} ms, per-batch(5)@30Hz {1000/30:.1f} ms; batching wait <= 33 ms added to E2E")
# ---------- K. q(pi0) for B1 ----------
qc = dict(box=0.05, person=0.5, sign=0.05, forklift=1.0, amr=0.5)
print(f"K. q_CV(pi0) = {sum(pi0[c]*qc[c] for c in pi0):.3f} m^2/s^3 (B1 baseline uses this value)")

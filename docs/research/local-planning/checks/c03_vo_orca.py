"""c03: (a) arc vs chord deviation, (b) false-prune / miss rates of the ORIGINAL chord-velocity VO
against per-pose ground truth, (c) corrected ORCA half-plane: symmetric head-on test and the sign
error of the original formulation."""
import numpy as np
rng = np.random.default_rng(7)
def arc(v, w, t):
    if abs(w) < 1e-9: return np.stack([v*t, 0*t], -1)
    return np.stack([v/w*np.sin(w*t), v/w*(1-np.cos(w*t))], -1)
def chord_vel(v, w, tau):
    x = w*tau/2; s = np.sinc(x/np.pi)          # sin(x)/x
    return v*s*np.array([np.cos(x), np.sin(x)])
print("== (a) max_t |arc(t) - t*v_eff| and sagitta (v=1)")
for tau in (1.0, 2.0):
    for w in (0.1, 0.3, 0.5, 1.0, 1.5):
        t = np.linspace(0, tau, 2001); A = arc(1.0, w, t); C = np.outer(t, chord_vel(1.0, w, tau))
        print("  tau=%.1f w=%.1f  e_max=%.3f  sagitta=%.3f  |w|tau/2=%.2f rad" % (tau, w, np.abs(np.linalg.norm(A-C, axis=1)).max(), (1/w)*(1-np.cos(w*tau/2)), w*tau/2))
print("  v=2, w=0.5, tau=2: e_max=%.3f" % np.linalg.norm(arc(2,0.5,np.linspace(0,2,2001))-np.outer(np.linspace(0,2,2001),chord_vel(2,0.5,2)),axis=1).max())
# (b) random scenes: robot at origin heading +x, sample (v,w) in full V_s, obstacle CV
tau = 2.0; R = 0.6; N = 40000; fp = fn = pos = neg = 0
fp_small = n_small = 0
for _ in range(N):
    v = rng.uniform(0.2, 2.0); w = rng.uniform(-1.5, 1.5)
    p0 = rng.uniform([-1, -4], [6, 4]); u = rng.normal(0, 0.8, 2)
    if np.linalg.norm(p0) < R + 0.05: continue
    t = np.linspace(0, tau, 201)
    gt = (np.linalg.norm(arc(v, w, t) - (p0 + np.outer(t, u)), axis=1) <= R).any()
    ve = chord_vel(v, w, tau); r0 = p0; wv = u - ve
    a = wv@wv; b = 2*r0@wv; c = r0@r0 - R*R; disc = b*b - 4*a*c
    vo = False
    if disc >= 0 and a > 1e-12:
        t1 = (-b - np.sqrt(disc))/(2*a); vo = 0 <= t1 <= tau
    small = abs(w)*tau/2 <= 0.3
    if gt: pos += 1; fn += (not vo)
    else:
        neg += 1; fp += vo
        if small: n_small += 1; fp_small += vo
print("== (b) original chord-VO vs per-pose truth (tau=2, R=0.6, %d scenes)" % (pos+neg))
print("  false prune (VO says collide, arc free): %.3f of free samples; with |w|tau/2<=0.3: %.3f" % (fp/neg, fp_small/max(n_small,1)))
print("  missed collision (VO says free, arc collides): %.3f of colliding samples" % (fn/pos))
# (c) ORCA with truncated VO (van den Berg et al. 2011 convention)
def orca_u(p_rel, v_rel, R, tau):
    """p_rel = p_B - p_A, v_rel = v_A - v_B (A = self). Returns u, n (outward normal)."""
    w = v_rel - p_rel/tau; wl2 = w@w; dp = w@p_rel
    dist2 = p_rel@p_rel; R2 = R*R
    if dp < 0 and dp*dp > R2*wl2:      # project on cut-off circle
        wl = np.sqrt(wl2); n = w/wl; u = (R/tau - wl)*n
    else:                               # project on legs
        leg = np.sqrt(dist2 - R2)
        if p_rel[0]*w[1] - p_rel[1]*w[0] > 0:
            d = np.array([p_rel[0]*leg - p_rel[1]*R, p_rel[0]*R + p_rel[1]*leg])/dist2
        else:
            d = -np.array([p_rel[0]*leg + p_rel[1]*R, -p_rel[0]*R + p_rel[1]*leg])/dist2
        proj = v_rel@d; u = proj*d - v_rel
        n = np.array([-d[1], d[0]]) if p_rel[0]*w[1] - p_rel[1]*w[0] > 0 else np.array([d[1], -d[0]])
        # make n point from the VO boundary outward (direction of u when v_rel inside VO)
    return u, n
def in_vo(p_rel, v_rel, R, tau):
    t = np.linspace(0, tau, 2001); return (np.linalg.norm(np.outer(t, v_rel) - p_rel, axis=1) <= R).any()
pA, pB = np.array([0., 0.]), np.array([4., 0.05]); vA, vB = np.array([1., 0.]), np.array([-1., 0.])
R, tau = 0.72, 2.0
uA, _ = orca_u(pB-pA, vA-vB, R, tau); uB, _ = orca_u(pA-pB, vB-vA, R, tau)
nA = uA/np.linalg.norm(uA); nB = uB/np.linalg.norm(uB)
print("== (c) symmetric head-on: v_rel in VO:", in_vo(pB-pA, vA-vB, R, tau))
print("  u_A =", np.round(uA,4), " u_B =", np.round(uB,4), " u_A + u_B =", np.round(uA+uB,6), " n_A.n_B =", round(nA@nB,6))
vA2 = vA + 0.5*uA; vB2 = vB + 0.5*uB     # each takes half: minimal change onto ORCA boundary
print("  after both apply 1/2 u: v_rel' in VO:", in_vo(pB-pA, vA2-vB2, R, tau), " min dist over tau: %.3f (R=%.2f)" %
      (np.linalg.norm(np.outer(np.linspace(0,tau,2001), vA2-vB2) - (pB-pA), axis=1).min(), R))
# original (brief v1) convention: w = u_j - v_R and r0 = p_j - p_R; u computed in w-space then applied to v
w_opt = vB - vA
uw, _ = orca_u(pB-pA, w_opt, R, tau)    # misuse: treating w (= -v_rel) as if it were v_rel
vA_bad = vA + 0.5*uw
print("  original sign convention: v_A' =", np.round(vA_bad,3), "-> v_rel' in VO:", in_vo(pB-pA, vA_bad-vB, R, tau),
      "(correct v_A' =", np.round(vA2,3), ")")

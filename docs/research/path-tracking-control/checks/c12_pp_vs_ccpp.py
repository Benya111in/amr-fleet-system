"""M13 / expected gain: kinematic closed-loop simulation of PP, RPP-style lookahead and CC-PP on
straight 6 m -> 90 deg arc R -> straight 6 m, speed from v_cap (a_lat 0.8, j_lat 1.5, alpha 2.0 terms)
with the jerk filter + braking trigger, control 20 Hz, omega kappa-preserving, alpha_max rate limit 50 Hz."""
import numpy as np, importlib.util, os, sys
here = os.path.dirname(os.path.abspath(__file__))
def load(n):
    sp = importlib.util.spec_from_file_location(n, os.path.join(here, n + ".py")); m = importlib.util.module_from_spec(sp)
    so = sys.stdout; sys.stdout = open(os.devnull, "w"); sp.loader.exec_module(m); sys.stdout = so; return m
c05 = load("c05_jerk_filter_closed"); c06 = load("c06_envelope"); c08 = load("c08_lmin_schedule")
step, brake_dist, envelope, Lmin = c05.step_closed, c06.brake_dist, c06.envelope, c08.Lmin
ds = 0.05; tL = 0.8

def make_path(R, straight=6.0):
    pts = [np.array([x, 0.0]) for x in np.arange(0, straight, ds)]
    for ph in np.arange(0, np.pi/2, ds/R): pts.append(np.array([straight + R*np.sin(ph), R - R*np.cos(ph)]))
    for y in np.arange(R, R + straight + 1e-9, ds): pts.append(np.array([straight + R, y]))
    P = np.array(pts); d = np.diff(P, axis=0); seg = np.linalg.norm(d, axis=1); s = np.r_[0, np.cumsum(seg)]
    th = np.unwrap(np.arctan2(d[:, 1], d[:, 0])); th = np.r_[th, th[-1]]
    kap = np.gradient(th, s); w = int(round(0.5/ds)); kap_s = np.convolve(kap, np.ones(w)/w, mode="same")
    kap_true = np.where((P[:, 0] > straight - 1e-9) & (P[:, 1] < R - 1e-9) & (P[:, 1] > 1e-9), 1/R, 0.0)
    return P, s, th, kap_s, kap_true

def vcap_of(kap, s):
    kp = np.abs(np.gradient(kap, s)) + 1e-9; k = np.abs(kap) + 1e-9
    vc = np.minimum.reduce([np.full_like(k, 2.0), np.sqrt(0.8/k), 1.5/k, np.sqrt(2.0/kp), (1.5/kp)**(1/3)])
    vc[-1] = 0.0; return vc

def project(P, p, i0):
    lo, hi = max(0, i0 - 40), min(len(P) - 1, i0 + 40)
    best = None
    for i in range(lo, hi):
        a, b = P[i], P[i+1]; ab = b - a; t = np.clip((p - a) @ ab/(ab @ ab), 0, 1); q = a + t*ab; d = np.linalg.norm(p - q)
        if best is None or d < best[0]: best = (d, i, t, q, ab/np.linalg.norm(ab))
    return best

def lookahead(P, start_pt, i, L):
    for k in range(i + 1, len(P)):
        if np.linalg.norm(P[k] - start_pt) >= L:
            a, b = P[k-1], P[k]; lo, hi = 0.0, 1.0
            for _ in range(40):
                m = 0.5*(lo + hi)
                if np.linalg.norm(a + m*(b - a) - start_pt) >= L: hi = m
                else: lo = m
            return a + hi*(b - a), True
    return P[-1], False

def run(R, mode):
    P, s, th, kap_s, kap_true = make_path(R); vc = vcap_of(kap_s, s)
    B = np.where(np.abs(envelope(vc, range(len(vc))) - vc) < 1e-9)[0]
    x = P[0].copy(); yaw = 0.0; v = a = 0.0; w = 0.0; idx = 0; kcmd = 0.0; target = 0.0; log = []
    for k in range(int(60/0.01)):
        if k % 5 == 0:
            d, idx, tt, q, tv = project(P, x, idx); sr = s[idx] + tt*ds
            n = np.array([-tv[1], tv[0]]); e = (x - q) @ n
            L = min(max(v*tL, Lmin(max(v, 0.05))), 1.8) if mode != "rpp" else min(max(v*1.5, 0.3), 0.9)
            G, ok = lookahead(P, x, idx, L)
            if not ok: L = max(np.linalg.norm(G - x), 1e-3)
            rel = G - x; yg = -np.sin(yaw)*rel[0] + np.cos(yaw)*rel[1]
            if mode == "ccpp":
                G0, _ = lookahead(P, q, idx, L); r0 = G0 - q; yg0 = -tv[1]*r0[0] + tv[0]*r0[1]
                kcmd = np.interp(sr, s, kap_s) + 2*(yg - yg0)/L**2
            else:
                kcmd = 2*yg/L**2
            vT = np.interp(sr, s, vc); ahead = B[(s[B] > sr) & (vc[B] < v)]
            if len(ahead):
                hit = s[ahead] - sr <= brake_dist(v, a, vc[ahead]) + v*0.05
                if hit.any(): vT = min(vT, vc[ahead][hit].min())
            target = vT
            log.append((sr, e, np.interp(sr, s, kap_true)))
            if sr > s[-1] - 0.05 and v < 1e-3: break
        if k % 2 == 0:
            v, a = step(v, a, target, 0.02)
            w_des = np.clip(v*kcmd, -1.5, 1.5); w = w + np.clip(w_des - w, -2.0*0.02, 2.0*0.02)
        x = x + 0.01*v*np.array([np.cos(yaw), np.sin(yaw)]); yaw += 0.01*w
    log = np.array(log); sr, e, kt = log.T
    curve = np.zeros(len(sr), bool)
    s_arc0, s_arc1 = 6.0, 6.0 + R*np.pi/2
    curve = (sr > s_arc0 - 0.5) & (sr < s_arc1 + 0.5)
    straight = ~curve & (sr > 0.5)
    return np.abs(e[curve]).mean(), np.abs(e[curve]).max(), np.abs(e[straight]).mean(), np.abs(e[straight]).max()

for R in [1.5, 1.0]:
    for mode in ["pp", "rpp", "ccpp"]:
        cm, cx, sm, sx = run(R, mode)
        print(f"M13 R={R} {mode:5s}: curve mean|e|={100*cm:.2f} cm max={100*cx:.2f} cm | straight mean={100*sm:.2f} cm max={100*sx:.2f} cm")

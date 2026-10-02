"""Anchor-envelope audit (M4 / JRG), vectorised:
(i) local-minimum anchors are incomplete; (ii) tracking the min-of-cones envelope with a causal
jerk filter overshoots v_cap at accel->decel kinks; (iii) fix: exact envelope (min over all grid
cones) + online braking trigger using the filter's closed-form braking distance from (v, a)."""
import numpy as np, importlib.util, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sp = importlib.util.spec_from_file_location("c05", os.path.join(here, "c05_jerk_filter_closed.py")); c05 = importlib.util.module_from_spec(sp)
so = sys.stdout; sys.stdout = open(os.devnull, "w"); sp.loader.exec_module(c05); sys.stdout = so
step = c05.step_closed
A, J = 1.0, 2.0; ds, dt = 0.05, 0.02

def d_vec(v, vt):
    v = np.asarray(v, float); vt = np.asarray(vt, float); dv = np.maximum(v - vt, 0)
    return np.where(dv >= A*A/J, (v*v - vt*vt)/(2*A) + (v + vt)*A/(2*J), (v + vt)*np.sqrt(dv/J))
def dinv_vec(vt, D):
    vt = np.asarray(vt, float); D = np.asarray(D, float)
    vl = -A*A/(2*J) + np.sqrt(A**4/(4*J*J) + vt*vt - vt*A*A/J + 2*A*D)
    lo = vt.copy() + 0*D; hi = vt + A*A/J + 0*D
    for _ in range(60):
        mid = 0.5*(lo + hi); big = d_vec(mid, vt + 0*D) > D; hi = np.where(big, mid, hi); lo = np.where(big, lo, mid)
    return np.where(vl - vt >= A*A/J, vl, 0.5*(lo + hi))

def envelope(vcap, anchors):
    n = len(vcap); idx = np.arange(n); vp = vcap.copy()
    for k in anchors:
        vp = np.minimum(vp, dinv_vec(np.full(n, vcap[k]), np.abs(idx - k)*ds))
    return vp
def local_min_anchors(vcap):
    n = len(vcap); Aset = {0, n - 1}
    for i in range(1, n - 1):
        if vcap[i] <= vcap[i-1] and vcap[i] <= vcap[i+1] and (vcap[i] < vcap[i-1] or vcap[i] < vcap[i+1]): Aset.add(i)
    return sorted(Aset)

def brake_dist(v, a, vt):
    """closed-form time-optimal jerk-limited distance from (v, a) to (vt, 0), vt < v (vectorised in vt)."""
    vt = np.asarray(vt, float)
    if a >= 0:
        t1 = a/J; d1 = v*t1 + a*t1*t1/2 - J*t1**3/6; v1 = v + a*a/(2*J)
        return d1 + np.where(v1 > vt, d_vec(v1, np.minimum(vt, v1)), 0.0)
    tr = -a/J; vv = v + a*a/(2*J); ap = np.minimum(A, np.sqrt(J*np.maximum(vv - vt, 0)))
    dr = vv*tr - J*tr**3/6
    return np.where(-a <= ap, d_vec(vv, np.minimum(vt, vv)) - dr, v*tr + a*tr*tr/2 + J*tr**3/6)

def drive(vcap, vprof, trigger=False):
    n = len(vcap); sg = np.arange(n)*ds; s, v, a, t = 0.0, 0.0, 0.0, 0.0; worst, ws = -9, 0
    active = np.where(np.abs(vprof - vcap) < 1e-9)[0]
    while s < sg[-1] - 1e-3 and t < 300:
        vr = np.interp(s, sg, vprof)
        if trigger:
            ahead = active[(sg[active] > s) & (vcap[active] < v)]
            if len(ahead):
                hit = sg[ahead] - s <= brake_dist(v, a, vcap[ahead]) + v*dt
                if hit.any(): vr = min(vr, vcap[ahead][hit].min())
        v1, a1 = step(v, a, vr, dt); s += 0.5*(v + v1)*dt; v, a = v1, a1; t += dt
        exc = v - np.interp(s, sg, vcap)
        if exc > worst: worst, ws = exc, s
    return worst, ws, t

cases = {}
n = int(12/ds) + 1; s = np.arange(n)*ds; vc = np.full(n, 2.0); vc[(s >= 3) & (s < 6)] = np.sqrt(0.8*1.5); vc[-1] = 0.0
cases["A rest->3 m straight->R1.5 arc"] = vc
n = int(30/ds) + 1; s = np.arange(n)*ds; vc = np.full(n, 2.0); vc[(s >= 20) & (s < 23)] = np.sqrt(0.8*1.5); vc[-1] = 0.0
cases["B cruise 20 m->R1.5 arc"] = vc
n = int(30/ds) + 1; s = np.arange(n)*ds; vc = np.full(n, 2.0); m = (s >= 10) & (s <= 20); vc[m] = 1.0 - 0.01*(s[m] - 10); vc[-1] = 0.0
cases["C step down to slowly decreasing cap"] = vc

if __name__ == "__main__":
    # verify closed-form brake distance against the filter itself
    for (v, a, vt) in [(2.0, 0.0, 1.1), (1.5, 0.8, 1.1), (1.8, -0.5, 1.0), (1.2, 1.0, 0.3)]:
        vv, aa, d = v, a, 0.0
        for _ in range(100000):
            v1, a1 = step(vv, aa, vt, 1e-3); d += 0.5*(vv + v1)*1e-3; vv, aa = v1, a1
            if vv == vt and aa == 0.0: break
        print(f"brake_dist({v},{a}->{vt}): closed {float(brake_dist(v, a, vt)):.4f} m, filter-simulated {d:.4f} m")

    for name, vc in cases.items():
        Ad = local_min_anchors(vc); vpd = envelope(vc, Ad)
        vpe = envelope(vc, range(len(vc)))
        w1, s1, _ = drive(vc, vpd); w2, s2, _ = drive(vc, vpe); w3, s3, t3 = drive(vc, vpe, trigger=True)
        Tp = np.sum(ds/np.maximum(0.5*(vpe[1:] + vpe[:-1]), 1e-3))
        print(f"{name}: |A_localmin|={len(Ad)}; doc-anchor env max(v_prof-v_cap)={np.max(vpd-vc):+.3f}")
        print(f"   local-min anchors + filter : max(v - v_cap) = {w1:+.3f} m/s at s={s1:.2f}")
        print(f"   exact envelope + filter    : max(v - v_cap) = {w2:+.3f} m/s at s={s2:.2f}")
        print(f"   exact env + brake trigger  : max(v - v_cap) = {w3:+.3f} m/s at s={s3:.2f};  T_filt={t3:.2f} s vs T_pred={Tp:.2f} s ({100*(t3/Tp-1):+.1f} %)")

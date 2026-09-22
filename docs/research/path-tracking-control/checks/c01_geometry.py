"""Audit checks: CTE sign (M1), circle linearization (M2/P3), Stanley damping (M8),
governor operating points L*kappa (P3 claim 'L kappa <= 0.59')."""
import numpy as np

# ---- M1: CTE sign convention -------------------------------------------------
t = np.array([1.0, 0.0]); d = np.array([0.0, 1.0])            # robot to the LEFT of path
cross_td = t[0]*d[1] - t[1]*d[0]                                # (t x d).z
n = np.array([-t[1], t[0]])
print("M1  (t x (p-P)).z =", cross_td, " (p-P).n =", d @ n, " (old (p-P) x t).z =", d[0]*t[1]-d[1]*t[0])

# ---- M2: circle linearization by finite differences ---------------------------
def yg_on_circle(R, L, e, psi):
    """Robot at lateral offset e (left +) from a CCW circle of radius R (centre (0,R)),
    heading error psi. Returns y of the look-ahead intersection (robot frame)."""
    # path point at origin, tangent +x, circle centre (0, R) (left turn, kappa = 1/R)
    p = np.array([0.0, e]); th = psi
    # intersect circle x^2 + (y-R)^2 = R^2 with circle |q-p| = L, forward solution
    phis = np.linspace(-np.pi/2, 3*np.pi/2, 200001)
    q = np.stack([R*np.sin(phis), R - R*np.cos(phis)], 1)       # param by arc angle from origin
    dist = np.linalg.norm(q - p, axis=1) - L
    # first sign change going forward from nearest point (phis ~ 0)
    i0 = np.argmin(np.abs(phis))
    idx = np.where(np.sign(dist[i0:-1]) != np.sign(dist[i0+1:]))[0][0] + i0
    a = dist[idx]/(dist[idx]-dist[idx+1]); qg = q[idx] + a*(q[idx+1]-q[idx])
    rel = qg - p; c, s = np.cos(th), np.sin(th)
    return -s*rel[0] + c*rel[1]

for R, L in [(1.5, 1.6), (1.5, 0.9), (1.5, 0.876), (1.0, 0.9)]:
    k = 1/R; h = 1e-5
    y0 = yg_on_circle(R, L, 0, 0)
    de = (yg_on_circle(R, L, h, 0) - yg_on_circle(R, L, -h, 0))/(2*h)
    dp = (yg_on_circle(R, L, 0, h) - yg_on_circle(R, L, 0, -h))/(2*h)
    c0 = 1 - L**2*k**2/2; c1 = np.sqrt(1 - L**2*k**2/4)
    print(f"M2  R={R} L={L}: y_g^0={y0:.4f} (L^2k/2={L**2*k/2:.4f}) dyg/de={de:.4f} (-c0={-c0:.4f}) "
          f"dyg/dpsi={dp:.4f} (-Lc1={-L*c1:.4f}) zeta_circ(K=1)={c1/np.sqrt(2):.4f}")

# zeta_circ general K
def zeta_circ(K, L, k):
    c1 = np.sqrt(1 - L**2*k**2/4)
    return K*c1/np.sqrt(2*K + (1-K)*L**2*k**2)
print("M2  zeta_circ K=2, R=1.5, L=0.876:", round(zeta_circ(2, 0.876, 1/1.5), 4))

# ---- governor operating point: L*kappa on arcs --------------------------------
a_lat, w_max, t_L = 0.8, 1.5, 0.8
sig_e, sig_psi, sig_w = 0.04, np.deg2rad(1.0), 0.15
def Lmin(v, K=1.0):
    A = sig_w/(2*K*v)
    u2 = (-sig_psi**2 + np.sqrt(sig_psi**4 + 4*sig_e**2*A**2))/(2*sig_e**2)
    return u2**-0.5
for R in [3.0, 2.0, 1.5, 1.0, 0.5]:
    k = 1/R; v = min(2.0, np.sqrt(a_lat/k), w_max/k)
    L = min(max(v*t_L, Lmin(v)), 1.8)
    print(f"Lk  R={R}: v_cap={v:.3f} L={L:.3f} (vt_L={v*t_L:.3f}, Lmin={Lmin(v):.3f}) "
          f"L*kappa={L*k:.3f} zeta_circ(K=1)={zeta_circ(1, L, k):.3f}")

# ---- M8: Stanley virtual wheelbase damping ------------------------------------
k, ell = 1.0, 0.5
for ks in [0.0, 0.5]:
    out = []
    for v in [0.2, 0.5, 1.0, 2.0]:
        c1 = v/ell + v*k/(ks+v); c0 = v**2*k/(ell*(ks+v))
        out.append(round(c1/(2*np.sqrt(c0)), 3))
    print(f"M8  Stanley k_s={ks}: zeta at v=0.2/0.5/1/2 =", out)

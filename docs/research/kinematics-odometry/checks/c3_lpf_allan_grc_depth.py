"""Checks for sec 3.2 (depth crop), 3.3 (LPF, Allan), 4.3 (GRC) and 2.2 (WheelSlip F_N) with project config."""
import numpy as np
from scipy.optimize import brentq
rng = np.random.default_rng(3)

# ---------- LPF (IMU 100 Hz)
fs = 100.0
def mag2(alpha, f):
    W = 2*np.pi*f/fs; beta = 1-alpha
    return alpha**2/(1-2*beta*np.cos(W)+beta**2)
for fc in (20.0, 10.0):
    tau = 1/(2*np.pi*fc); a_be = (1/fs)/(tau+1/fs)
    f3 = brentq(lambda f: mag2(a_be, f)-0.5, 0.1, 49.9)
    c = np.cos(2*np.pi*fc/fs); beta = (2-c)-np.sqrt((2-c)**2-1); a = 1-beta
    f3x = brentq(lambda f: mag2(a, f)-0.5, 0.1, 49.9)
    print(f"LPF fc={fc}: backward-Euler alpha={a_be:.3f} -> -3 dB at {f3:.2f} Hz; exact alpha={a:.4f} -> -3 dB at {f3x:.3f} Hz;"
          f" noise var ratio alpha/(2-alpha)={a/(2-a):.3f} (sigma x{np.sqrt(a/(2-a)):.3f}); DC group delay beta/alpha={beta/a:.3f} samples")

# ---------- Allan confidence
for Tlog, tau in [(1800, 300), (4*3600, 10), (4*3600, 570), (4*3600, 300)]:
    M = Tlog//tau
    print(f"Allan: log {Tlog/3600:.1f} h, tau={tau} s: M={M}, rel. 1-sigma CI = {1/np.sqrt(2*(M-1)):.3f}")
print(f"gyro ARW N = sigma_g*sqrt(dt) = {2e-4*np.sqrt(0.01):.1e} rad/sqrt(s); accel VRW = {0.017*np.sqrt(0.01):.1e} m/s/sqrt(s)")

# ---------- forgetting-factor explosion (v1)
print(f"lambda=0.995 @100 Hz, 60 s unexcited: lambda^-6000 = {0.995**-6000:.2e}; lambda=0.9995: {0.9995**-6000:.1f}")

# ---------- GRC simulation with config noise
r, b, N, ss = 0.0825, 0.36, 4096, 0.01
psi_nom = r/b
sg = 2e-4
q = (1e-3*psi_nom)**2/6000
Pmax = (0.05*psi_nom)**2
delta = 2*np.pi*r/N

def seg_list():
    # (duration s, v m/s, w rad/s): mixed 60 s
    return [(10, 1.0, 0.0), (5, 0.0, 1.5), (10, 0.5, 0.0), (5, 0.0, -1.5), (10, 1.0, 0.5), (5, 0.0, 1.5), (15, 1.5, 0.0)]

def run(psi_true, T_total=60.0, segs=None, psi0=None, P0=None, seed=0, report=True):
    rg = np.random.default_rng(seed)
    segs = segs or seg_list()
    psi_hat = np.array(psi0 if psi0 is not None else [psi_nom, psi_nom], float)
    P = np.eye(2)*(P0 if P0 is not None else Pmax)
    t_conv = None; t = 0.0
    # true radii with b_true = b (scale fixed): r_i = psi_i*b
    for dur, v, w in segs:
        nwin = int(round(dur/0.1))
        for _ in range(nwin):
            vR, vL = v + w*b/2, v - w*b/2
            phR, phL = vR/(psi_true[0]*b), vL/(psi_true[1]*b)   # true wheel rates for this body motion
            # 5 encoder steps: measured wheel angle increments (quantisation approx. + multiplicative noise)
            dR = phR*0.02*(1+rg.normal(0, ss, 5)) + rg.uniform(-1, 1, 5)*0  # quantisation handled in R
            dL = phL*0.02*(1+rg.normal(0, ss, 5))
            hR, hL = dR.sum()/0.1, dL.sum()/0.1
            z = w + rg.normal(0, sg, 10).mean()
            h = np.array([hR, -hL])
            s2R = ss**2*(abs(vR)*0.02)*(abs(vR)*0.1) + delta**2/6
            s2L = ss**2*(abs(vL)*0.02)*(abs(vL)*0.1) + delta**2/6
            R = (s2R+s2L)/(b**2*0.01) + sg**2/10
            Pm = P + q*np.eye(2)
            if abs(hR) > 1.0 and abs(hL) > 1.0:
                nu = z - h@psi_hat; S = h@Pm@h + R
                if nu**2 <= 9*S:
                    K = Pm@h/S
                    step = np.clip(K*nu, -1e-3*psi_nom, 1e-3*psi_nom)
                    psi_hat = np.clip(psi_hat+step, 0.95*psi_nom, 1.05*psi_nom)
                    Pm = (np.eye(2)-np.outer(K, h))@Pm
            ev, V = np.linalg.eigh(0.5*(Pm+Pm.T)); P = V@np.diag(np.minimum(ev, Pmax))@V.T
            t += 0.1
            err = np.max(np.abs(psi_hat-psi_true)/psi_true)
            if t_conv is None and err <= 0.01:
                t_conv = t
            if t_conv is not None and err > 0.01:
                t_conv = None
    return psi_hat, P, t_conv

for pt in ([psi_nom/1.05, psi_nom/1.05], [psi_nom*1.04, psi_nom*0.97], [psi_nom*1.05, psi_nom*0.95]):
    errs = []; tcs = []
    for sd in range(20):
        ph, P, tc = run(np.array(pt), seed=sd)
        errs.append(np.max(np.abs(ph-pt)/np.array(pt))); tcs.append(tc if tc else np.inf)
    print(f"GRC true/nom = {np.array(pt)/psi_nom}: 60 s max rel err over 20 seeds = {max(errs)*100:.3f} %, median t(<=1 %) = {np.median(tcs):.1f} s, max {max(tcs):.1f} s")

# unexcited direction after convergence: 60 s straight only
ph, P, _ = run(np.array([psi_nom, psi_nom]), segs=seg_list(), seed=1)
P_before = P.copy()
ph2, P2, _ = run(np.array([psi_nom, psi_nom]), segs=[(60, 1.0, 0.0)], psi0=ph, P0=None, seed=2)
# P0=None resets; re-run with explicit P propagation instead:
def straight_growth(P, secs=60):
    for _ in range(int(secs/0.1)):
        P = P + q*np.eye(2)
        h = np.array([1.0/r*1, -1.0/r])  # 1 m/s straight
        s2 = ss**2*(0.02)*(0.1) + delta**2/6
        R = 2*s2/(b**2*0.01)
        S = h@P@h + R; K = P@h/S; P = (np.eye(2)-np.outer(K, h))@P
    return P
Pg = straight_growth(P_before)
u = np.array([1, 1])/np.sqrt(2)
print(f"unexcited sum-direction variance: before {u@P_before@u:.2e}, after 60 s straight {u@Pg@u:.2e} (Pmax {Pmax:.2e}); ratio {u@Pg@u/(u@P_before@u):.3f}")

# information content for sec 4.3
for name, v, w in (("straight 1 m/s", 1.0, 0.0), ("rotation 1.5 rad/s", 0.0, 1.5)):
    vR, vL = v+w*b/2, v-w*b/2
    h = np.array([vR/r, -vL/r])
    s2R = ss**2*(abs(vR)*0.02)*(abs(vR)*0.1) + delta**2/6; s2L = ss**2*(abs(vL)*0.02)*(abs(vL)*0.1) + delta**2/6
    R = (s2R+s2L)/(b**2*0.01)
    info = 100*(h@h)/R   # 10 s at 10 Hz along h
    d = h/np.linalg.norm(h)
    print(f"GRC info {name}: sigma along h-direction after 10 s = {1/np.sqrt(info):.2e} = {1/np.sqrt(info)/psi_nom*100:.3f} % of psi_nom")
print(f"q = {q:.2e}, Pmax = {Pmax:.2e}, max step = {1e-3*psi_nom:.2e}")

# ---------- depth crop (sensors.yaml: camera_link z=0.25 above base_link, base_link 0.18 above ground)
hcam = 0.18 + 0.25
hfov = np.deg2rad(87); vhalf = np.arctan(np.tan(hfov/2)*480/640)
Zmin_floor = hcam/np.tan(vhalf)
sig = lambda Z: np.sqrt(0.005**2 + (0.002*Z**2)**2)
Zs = np.linspace(max(Zmin_floor, 0.2), 5.0, 1000)
hz = 3*sig(Zs)*hcam/Zs
print(f"\ndepth: camera height {hcam:.2f} m, vfov/2 = {np.rad2deg(vhalf):.1f} deg, floor visible from Z = {Zmin_floor:.3f} m;"
      f" max 3-sigma height error on floor points in [{Zs[0]:.2f}, 5] m = {hz.max()*100:.2f} cm at Z={Zs[hz.argmax()]:.2f};"
      f" sigma(1)={sig(1):.4f}, sigma(5)={sig(5):.4f} (3 sigma along ray at 5 m = {3*sig(5):.3f} m)")
print(f"f_x = 320/tan(43.5 deg) = {320/np.tan(hfov/2):.1f} px")

# ---------- WheelSlip normal force per drive wheel (upper bound, casters unloaded)
m0 = 45.0 + 2*1.0 + 2*0.3
for mp in (0, 2, 10, 25):
    print(f"payload {mp:>2} kg: total {m0+mp:.1f} kg, F_N upper bound per drive wheel (g=9.8) = {(m0+mp)*9.8/2:.1f} N")

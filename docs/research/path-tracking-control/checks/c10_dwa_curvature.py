"""M5 / lemma check: minimiser over the box window of the brief's cost
((v-v*)/vmax)^2 + ((w - kappa v)/wmax)^2 vs the DWPP projection (lexicographic: stay on the line
w = kappa v if it meets V_d, maximise progress toward v*; otherwise closest point to the line)."""
import numpy as np
vmax, wmax, A, AL, dt = 2.0, 1.5, 1.0, 2.0, 0.05
def box(vc, wc):
    return (max(-0.5, vc - A*dt), min(vmax, vc + A*dt)), (max(-wmax, wc - AL*dt), min(wmax, wc + AL*dt))
def brief_argmin(vc, wc, vs, kap, n=801):
    (v0, v1), (w0, w1) = box(vc, wc); V, W = np.meshgrid(np.linspace(v0, v1, n), np.linspace(w0, w1, n))
    J = ((V - vs)/vmax)**2 + ((W - kap*V)/wmax)**2; i = np.unravel_index(np.argmin(J), J.shape); return V[i], W[i]
def dwpp(vc, wc, vs, kap):
    (v0, v1), (w0, w1) = box(vc, wc)
    # line segment inside box: v in [v0,v1] and kap*v in [w0,w1]
    lo, hi = v0, v1
    if kap > 0: lo, hi = max(lo, w0/kap), min(hi, w1/kap)
    elif kap < 0: lo, hi = max(lo, w1/kap), min(hi, w0/kap)
    elif not (w0 <= 0 <= w1): lo, hi = 1, 0
    if lo <= hi:
        v = min(max(vs, lo), hi); return v, kap*v
    V, W = np.meshgrid(np.linspace(v0, v1, 801), np.linspace(w0, w1, 801))
    d = np.abs(W - kap*V)/np.sqrt(1 + kap*kap); i = np.unravel_index(np.argmin(d - 1e-9*V), d.shape); return V[i], W[i]
cases = [("rest->R1.5 (brief test 8b)", 0.10, 0.0667, 1.10, 1/1.5),
         ("accel into R=0.5 (kappa*dv > dw)", 0.50, 1.0, 0.63, 2.0),
         ("on-line, kappa step 0->0.667 at 1.1 m/s", 1.10, 0.0, 1.10, 1/1.5),
         ("R=0.4 accel", 0.40, 1.0, 0.56, 2.5)]
for name, vc, wc, vs, kap in cases:
    vb, wb = brief_argmin(vc, wc, vs, kap); vd, wd = dwpp(vc, wc, vs, kap)
    print(f"M5 {name}: brief argmin (v,w)=({vb:.4f},{wb:.4f}) kappa_exec={wb/vb:.4f} | DWPP ({vd:.4f},{wd:.4f}) kappa_exec={wd/vd:.4f} | kappa_cmd={kap:.4f}")

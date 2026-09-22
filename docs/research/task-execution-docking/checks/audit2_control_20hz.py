"""Audit-2 check: the Park-Kuipers/Nav2 law of brief §3.5 at the 20 Hz docking-server output rate of
components.md §5.5 (audit_checks.py §10 integrated at 50 Hz). Same law incl. v_min clamp and omega-saturation
rescale; staging offsets (+-0.15 m, +-10 deg) -> pre-dock target (-0.80, 0, 0), then pre-dock -> docked (-0.50,0,0)
with v_max = 0.10 m/s. Control held constant over each 50 ms period, plant integrated at 1 kHz."""
import numpy as np
def law(r, phi, delta, kphi=2.0, kdelta=1.0, beta_=0.4, lam_=2.0, vmax=0.25, vmin=0.05, rslow=0.25, wmax=0.75, adock=None):
    kappa = -1.0/r*(kdelta*(delta-np.arctan(-kphi*phi))+(1+kphi/(1+(kphi*phi)**2))*np.sin(delta))
    v = vmax/(1+beta_*abs(kappa)**lam_); v = min(vmax*r/rslow, v)
    if adock: v = min(v, np.sqrt(2*r*adock))
    v = np.clip(v, vmin, vmax); w = kappa*v; wb = np.clip(w, -wmax, wmax)
    v = wb/kappa if kappa != 0 else v
    return v, wb
def sim(x0, target, vmax, Tc=0.05, adock=0.328, T=40, tol=0.005):
    x = np.array(x0, float); sat = 0; t = 0.0
    while t < T:
        dx, dy = target[0]-x[0], target[1]-x[1]; r = np.hypot(dx, dy)
        if r < tol: break
        los = np.arctan2(dy, dx)
        phi = np.arctan2(np.sin(target[2]-los), np.cos(target[2]-los))
        delta = np.arctan2(np.sin(x[2]-los), np.cos(x[2]-los))
        v, w = law(r, phi, delta, vmax=vmax, adock=adock)
        sat += abs(w) >= 0.75-1e-9
        for _ in range(int(Tc/0.001)):
            x += 0.001*np.array([v*np.cos(x[2]), v*np.sin(x[2]), w])
        t += Tc
    return x, r, sat, t
worst_y = worst_p = 0; worst_r = 0; sats = 0
for y0 in [-0.15, 0.0, 0.15]:
    for p0 in [-10, 0, 10]:
        xf, r, sat, t = sim([-1.5, y0, np.radians(p0)], (-0.80, 0.0, 0.0), vmax=0.25)
        worst_y = max(worst_y, abs(xf[1])); worst_p = max(worst_p, abs(np.degrees(xf[2]))); worst_r = max(worst_r, r); sats += sat
        print("staging y0=%+.2f psi0=%+3d -> pre-dock: r=%.4f y=%+.4f psi=%+.2f deg t=%.1f s sat=%d" % (y0, p0, r, xf[1], np.degrees(xf[2]), t, sat))
print("20 Hz worst: r=%.4f m |y|=%.4f m |psi|=%.2f deg, omega-saturated periods=%d" % (worst_r, worst_y, worst_p, sats))
# terminal leg with a residual pre-dock error at the gate limit (y 5 mm, psi 0.6 deg)
for y0, p0 in [(0.005, 0.6), (-0.005, -0.6), (0.0, 0.6)]:
    xf, r, sat, t = sim([-0.80, y0, np.radians(p0)], (-0.50, 0.0, 0.0), vmax=0.10, tol=0.003)
    print("terminal y0=%+.3f psi0=%+.1f -> docked: r=%.4f y=%+.4f psi=%+.3f deg t=%.1f s" % (y0, p0, r, xf[1], np.degrees(xf[2]), t))

"""2-of-2 false-alarm rate vs speed: correlation of consecutive r_v from the 80 %-overlapping IMU integral term.
Consecutive decisions k, k+1 (0.1 s apart): encoder windows disjoint (independent), IMU integral over [s-Ta, s]
averaged over the window: overlap (Ta - Tw)/Ta = 0.8 -> Cov ~ 0.8 * Var_imu (upper bound)."""
import numpy as np
from scipy.stats import chi2
rng = np.random.default_rng(11)
r, b, N, ss = 0.0825, 0.36, 4096, 0.01
dte, dti, Tw, Ta = 0.02, 0.01, 0.1, 0.5
delta = 2*np.pi*r/N; sa, sg = 0.017, 2e-4
tau = chi2.ppf(0.999, 2)
for v in (0.0, 0.3, 1.0, 2.0):
    s2 = ss**2*(v*dte)*(v*Tw) + delta**2/6
    Venc2 = 2*2*s2/(4*Tw**2); Vimu = sa**2*dti*Ta
    Sv = Venc2 + Vimu
    rho = 0.8*Vimu/Sv
    n = 40_000_000
    z1 = rng.standard_normal(n); z2 = rho*z1 + np.sqrt(1-rho**2)*rng.standard_normal(n)
    w1 = rng.standard_normal(n); w2 = rng.standard_normal(n)
    d1 = z1**2 + w1**2; d2 = z2**2 + w2**2
    p1 = np.mean(d1 > tau); p2 = np.mean((d1 > tau) & (d2 > tau))
    print(f"v={v}: IMU share of S_v = {Vimu/Sv:.2f}, rho(r_v) <= {rho:.2f}, P(d2>tau) = {p1:.2e}, P(2-of-2) = {p2:.1e} ({p2*36000:.2f} per h at 10 Hz)")

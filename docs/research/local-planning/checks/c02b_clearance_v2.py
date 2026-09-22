"""c02b: revised radii (exact covers + margins) -> centre distance d* at which min(HP,BX) bound = delta_hard.
R_hard = r_R + r_j + 0.30 (emergency_stop_distance), R_soft = r_R + r_j + 0.50 (critical_zone_distance)."""
import numpy as np
from math import sqrt
from scipy.special import ndtr
from scipy.optimize import brentq
sp, sv = 0.05, 0.2
cls = {"person (r=0.25,q=0.2)": (0.25, 0.2), "forklift circle (r=0.60,q=0.1)": (0.60, 0.1), "AMR circle (r=0.25,q=0.05)": (0.25, 0.05)}
rR = 0.25
sig = lambda t, q: sqrt(sp**2 + t*t*sv**2 + q*t**3/3)
def bmin(d, s, R):
    hp = ndtr((R-d)/s); return min(hp, (hp - ndtr((-R-d)/s))*(2*ndtr(R/s)-1))
def dstar(R, s, delta): return brentq(lambda d: bmin(d, s, R) - delta, 1e-6, R + 12*s)
def tail_time(v0, Tc=0.20, a=1.0, j=2.0, dt=1e-4):
    """hold v0 for Tc, then jerk-limited braking (accel 0 -> -a at jerk j) until v = 0 (computed, not hard-coded)."""
    t, v, acc = 0.0, v0, 0.0
    while v > 0:
        if t >= Tc: acc = max(acc - j*dt, -a); v += acc*dt
        t += dt
    return t
print("tail durations incl. T_c = 0.2 s commit (jerk-limited, a0=0):", {v: round(tail_time(v), 2) for v in (0.5, 1.0, 1.5, 2.0)},
      "| with t_r = 0.15 instead:", {v: round(tail_time(v, 0.15), 2) for v in (0.5, 1.0, 2.0)})
for name, (r, q) in cls.items():
    Rh = rR + r + 0.30
    row = {t: round(dstar(Rh, sig(t, q), 0.05), 2) for t in (0.2, 0.5, 1.0, 1.2, 1.5, 2.5)}
    print("%-32s R_hard=%.2f  d*(delta=0.05) by t:" % (name, Rh), row, " edge gap at d*: t=1.0 -> %.2f" % (row[1.0] - rR - r))
print("S3 pass offset: person at y=-0.9, tail 1.2 s: need", round(0.80 + 1.64*sig(1.2, 0.2), 2), "-> y_ref", round(-0.9 + 0.80 + 1.64*sig(1.2, 0.2), 2))
print("       person at y=-0.7: y_ref", round(-0.7 + 0.80 + 1.64*sig(1.2, 0.2), 2), "(> band 0.75 -> follow)")
print("tracker-supplied sigma_v=0.1 instead of 0.2 at 1.2 s:", round(sqrt(sp**2 + 1.44*0.01 + 0.2*1.728/3), 3), "vs", round(sig(1.2, 0.2), 3))

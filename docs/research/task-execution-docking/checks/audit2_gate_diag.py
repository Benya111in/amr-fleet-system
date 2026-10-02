"""Audit-2 check: gate outcome and diagnosis for the heading residual the APPROACH law leaves at pre-dock
(|psi| <= 0.41 deg at the 20 Hz rate, audit2_control_20hz.py), using the brief's fused sigma (0.204 deg,
x2 inflation) and the Phi-propagated (y, psi) marginal; compares the old diagnosis rule (|mu_psi| > tau_psi)
with the marginal rule (psi-marginal alone below 1 - delta)."""
import numpy as np
from scipy.stats import norm, multivariate_normal
tau = np.array([0.015, np.radians(0.6)]); delta = 0.05
sp = np.radians(0.204); L_pre = 0.80; L_dock = 0.50
# (y, psi) marginal at docked after Phi propagation: y_t ~ X - 0.5 psi, rho ~ -1 (camera X term 0.05 mm)
sx = 0.05e-3
S = np.array([[sx**2+(L_dock*sp)**2, -L_dock*sp**2], [-L_dock*sp**2, sp**2]])
def rect(mu, S):
    m = multivariate_normal(mean=mu, cov=S)
    return m.cdf(tau)-m.cdf([-tau[0], tau[1]])-m.cdf([tau[0], -tau[1]])+m.cdf(-tau)
print(" mu_psi  joint   p_psi   p_y    old rule(|mu|>tau_psi)  marginal rule")
for mpd in [0.0, 0.2, 0.3, 0.41, 0.5, 0.7]:
    mu = np.array([0.30*np.radians(mpd), np.radians(mpd)])   # y_t mean = 0.30*mu_psi (straight 0.30 m)
    j = rect(mu, S)
    pp = norm.cdf((tau[1]-mu[1])/sp)-norm.cdf((-tau[1]-mu[1])/sp)
    sy = np.sqrt(S[0, 0]); py = norm.cdf((tau[0]-mu[0])/sy)-norm.cdf((-tau[0]-mu[0])/sy)
    gate = j >= 1-delta
    old = "heading" if abs(mpd) > 0.6 else ("info" if np.degrees(sp) > 0.3 else "NONE")
    new = "info" if np.degrees(sp) > 0.3 else ("heading" if pp < 1-delta else ("lateral" if py < 1-delta else "info"))
    print(" %.2f   %.3f   %.3f   %.3f   %-8s -> %-8s          %s" % (mpd, j, pp, py, "pass" if gate else "FAIL",
          "-" if gate else old, "-" if gate else new))

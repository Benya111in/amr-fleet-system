"""Audit-2 check: E2 NEES sensitivity to a per-docking constant corner bias in the near-singular
direction of the fused (x_r, y_r, psi) posterior, and the effect of a 4th 'consider' bias state.
Reuses the PnP CRLB / h_c model of audit_checks.py (same f, s, sigma_c, l_c)."""
import numpy as np
from scipy.stats import chi2
f = 320/np.tan(np.radians(43.5)); s = 0.15; sc = 0.2; lc = 0.18; lx = 0.15
h = s/2; corners = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
def rot(ax, a):
    c, s_ = np.cos(a), np.sin(a)
    if ax == 'x': return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
    if ax == 'y': return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
R_face = np.diag([1, -1, -1.0])
def proj(p):
    R = rot('y', p[3]) @ rot('x', p[4]) @ rot('z', p[5]) @ R_face
    P = corners @ R.T + p[:3]
    return np.concatenate([f*P[:, 0]/P[:, 2], f*P[:, 1]/P[:, 2]])
def jac(fun, p, eps=1e-7):
    u0 = fun(p); J = np.zeros((u0.size, p.size))
    for k in range(p.size):
        d = np.zeros(p.size); d[k] = eps; J[:, k] = (fun(p+d)-fun(p-d))/(2*eps)
    return J
def Rc(Z):
    J = jac(proj, np.array([0, 0, Z, 0, 0, 0.0])); C = sc**2*np.linalg.inv(J.T@J); i = [0, 2, 3]
    return C[np.ix_(i, i)]
def h_c(x):
    xr, yr, psi = x[:3]
    cx_, cy_ = xr+lc*np.cos(psi), yr+lc*np.sin(psi)
    return np.array([-cx_*np.sin(psi)+cy_*np.cos(psi), -cx_*np.cos(psi)-cy_*np.sin(psi), psi])
HL = np.array([[-1, 0, 0], [0, 0, 1.0]]); RL = np.diag([0.0025**2, np.radians(0.63)**2])  # MC LiDAR values at pre-dock

def fuse(n_cam=30, n_lidar=10, Z=0.62, beta_px=None):
    """information-form fusion at a static pose; beta_px = prior sigma of a constant angular corner bias
    (px) modelled as a 4th state b with X_meas = X + Z*b (b in rad = px/f)."""
    x = np.array([-(Z+lc), 0, 0.0])
    Hc = jac(h_c, x)
    if beta_px is None:
        I = n_cam*Hc.T@np.linalg.inv(Rc(Z))@Hc + n_lidar*HL.T@np.linalg.inv(RL)@HL
        return np.linalg.inv(I), Hc
    Hc4 = np.c_[Hc, [Z, 0, 0]]; HL4 = np.c_[HL, [0, 0]]
    I = n_cam*Hc4.T@np.linalg.inv(Rc(Z))@Hc4 + n_lidar*HL4.T@np.linalg.inv(RL)@HL4
    I[3, 3] += 1/(beta_px/f)**2
    return np.linalg.inv(I), Hc

P, Hc = fuse()
w, V = np.linalg.eigh(P)
print("fused P (3-state, 30 cam + 10 LiDAR at pre-dock, no inflation):")
print("  sigma along eigvecs [mm or mrad-equivalent]:", np.round(np.sqrt(w)*1e3, 4))
print("  thinnest direction v =", np.round(V[:, 0], 3), "(x_r, y_r, psi)")
for beta in [0.02, 0.05, 0.10]:
    dX = 0.62*beta/f                      # constant X error from a common-mode corner bias
    dx = np.linalg.solve(Hc, np.array([dX, 0, 0]))   # state error it induces (all frames share it)
    nees_add = dx@np.linalg.solve(P, dx)
    print("  bias %.2f px -> dX = %.3f mm -> dy_r = %.3f mm, dpsi = %.4f deg; NEES increment = %.1f (3-DoF mean should be ~3)" % (
        beta, dX*1e3, dx[1]*1e3, np.degrees(dx[2]), nees_add))
print("E2 95%% band for N=90: [%.2f, %.2f]" % (chi2.ppf(.025, 270)/90, chi2.ppf(.975, 270)/90))
P4, _ = fuse(beta_px=0.05)
print("with 4th consider-bias state (sigma_beta = 0.05 px):")
print("  sig_x=%.3f mm sig_y=%.3f mm sig_psi=%.4f deg ; thinnest sigma %.4f mm" % (
    np.sqrt(P4[0, 0])*1e3, np.sqrt(P4[1, 1])*1e3, np.degrees(np.sqrt(P4[2, 2])), np.sqrt(np.linalg.eigvalsh(P4[:3, :3])[0])*1e3))
print("  without: sig_x=%.3f mm sig_y=%.3f mm sig_psi=%.4f deg" % (np.sqrt(P[0, 0])*1e3, np.sqrt(P[1, 1])*1e3, np.degrees(np.sqrt(P[2, 2]))))
dX = 0.62*0.05/f; dx = np.linalg.solve(Hc, np.array([dX, 0, 0]))
print("  NEES increment of a 0.05 px bias with the 4-state model: %.2f" % (dx@np.linalg.solve(P4[:3, :3], dx)))
# unit-clean statement of the thin direction: sigma of the combination y_r + (Z+l_c) psi [m]
a = np.array([0, 1, 0.62+lc]); print("sigma(y_r + 0.80 psi) = %.4f mm (3-state), %.4f mm (with beta state)" % (
    np.sqrt(a@P@a)*1e3, np.sqrt(a@P4[:3, :3]@a)*1e3))

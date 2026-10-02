"""L_min RSS values (M12), K(v) schedule vs noise budget (spec gap 1), and the
noise-limited bandwidth bound omega_n <= sqrt(v*sigma_w/sigma_e) (sigma_psi -> 0)."""
import numpy as np
sig_e, sig_psi, sig_w, tL = 0.04, np.deg2rad(1.0), 0.15, 0.8
def Lmin(v, K=1.0):
    Aq = sig_w/(2*K*v)
    u2 = (-sig_psi**2 + np.sqrt(sig_psi**4 + 4*sig_e**2*Aq**2))/(2*sig_e**2)
    return u2**-0.5
def sigma_w(v, L, K):
    return np.sqrt((2*K*v/L**2)**2*sig_e**2 + (2*K*v/L)**2*sig_psi**2)
print("M12 L_min(v) RSS K=1:", [round(Lmin(v), 3) for v in (0.2, 0.5, 1.0, 2.0)],
      " sigma_psi->0:", [round(np.sqrt(2*v*sig_e/sig_w), 3) for v in (0.2, 0.5, 1.0, 2.0)])
print("M12 sigma_psi share at v=2, L=1.6: %.3f rad/s of %.2f budget (%.0f %%)" % (2*2/1.6*sig_psi, sig_w, 100*2*2/1.6*sig_psi/sig_w))
print("M12 L_min increase from sigma_psi at v=2: %+.1f %%" % (100*(Lmin(2.0)/np.sqrt(2*2*sig_e/sig_w) - 1)))
# brief's schedule K = K0 (L/(v tL))^gamma with L = L_min(v; K=1)
for g in [0, 1, 2]:
    rows = []
    for v in [0.2, 0.5]:
        L = max(v*tL, Lmin(v)); K = min(4.0, (L/(v*tL))**g)
        rows.append(f"v={v}: L={L:.3f} K={K:.2f} zeta={np.sqrt(K/2):.2f} wn={np.sqrt(2*K)*v/L:.2f} sigma_w={sigma_w(v, L, K):.3f}")
    print(f"SG1 brief gamma={g}: " + " | ".join(rows))
# consistent schedule: L_min solved jointly with K(v) (fixed point); omega_n bound
for g in [0, 1, 2]:
    rows = []
    for v in [0.2, 0.5]:
        L = max(v*tL, Lmin(v))
        for _ in range(200):
            K = min(4.0, (L/(v*tL))**g); L = max(v*tL, Lmin(v, K))
        rows.append(f"v={v}: L={L:.3f} K={K:.2f} zeta={np.sqrt(K/2):.2f} wn={np.sqrt(2*K)*v/L:.3f} sigma_w={sigma_w(v, L, K):.3f}")
    print(f"SG1 joint gamma={g}: " + " | ".join(rows))
for v in [0.2, 0.5]:
    print(f"SG1 noise bound omega_n,max(v={v}) = sqrt(v sigma_w/sigma_e) = {np.sqrt(v*sig_w/sig_e):.3f} rad/s")
print("SG1 speed above which L=v t_L is noise-feasible (sigma_psi->0): v* = 2 sigma_e/(sigma_w t_L^2) = %.3f m/s" % (2*sig_e/(sig_w*tL**2)))

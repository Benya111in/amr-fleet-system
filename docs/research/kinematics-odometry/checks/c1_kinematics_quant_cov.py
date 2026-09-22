"""Checks for sec 2 (kinematics, quantisation, augmented covariance, drift) with PROJECT CONFIG values.
config/robot_params.yaml: r = 0.0825 m, b = 0.36 m; config/sensors.yaml wheel_encoder: 4096 tick/rev,
slip_noise_stddev = 0.01 (multiplicative per period), update_rate 50 Hz; components.md: joint_states 50 Hz.
"""
import numpy as np
rng = np.random.default_rng(1)
r, b, N, fe, ss = 0.0825, 0.36, 4096, 50.0, 0.01
dt = 1 / fe
delta = 2 * np.pi * r / N
print(f"delta = {delta:.4e} m, sigma_q^2 = delta^2/12 = {delta**2/12:.3e}, delta^2/6 = {delta**2/6:.3e}")
print(f"psi_nom = r/b = {r/b:.4f}")
w = 2.0 / r
print(f"wheel rate @2 m/s = {w:.2f} rad/s -> {w*N/(2*np.pi):.0f} tick/s, 4 h = {w*N/(2*np.pi)*4*3600:.3e} ticks")

# --- 2.1 midpoint vs exact arc
th_max = 1.5 * dt
rel = th_max**2 / 24
print(f"\n[2.1] dtheta_max = {th_max}, relative bound dth^2/24 = {rel:.3e}; per-step @2 m/s = {0.04*rel:.3e} m;"
      f" 7.2 km worst case = {7200*rel:.3f} m; 0.5 % mean-radius scale over 7.2 km = {7200*0.005:.0f} m")
worst = 0
for ds in np.linspace(1e-4, 0.046, 60):
    for dth in np.linspace(-0.035, 0.035, 71):
        if abs(dth) < 1e-12:
            continue
        u = dth / 2
        exact = ds * np.sin(u) / u  # chord length of arc
        mid = ds
        err = abs(mid - exact)
        bound = ds * dth**2 / 24
        worst = max(worst, err / bound)
print(f"[2.1] max(|e_step| / (ds*dth^2/24)) over grid = {worst:.6f} (<= 1 required)")

# --- 2.2 quantisation MA(1)
T = 200000
v = 1.0 + 0.3 * np.sin(np.arange(T) * dt * 0.7)
phi = np.cumsum(v * dt / r) + rng.uniform(0, 2 * np.pi)
n = np.round(phi * N / (2 * np.pi))
q = (n * 2 * np.pi / N - phi) * r  # position quantisation error [m]
e = np.diff(q)
print(f"\n[2.2] Var(q)/(d^2/12) = {q.var()/(delta**2/12):.3f}; Var(e)/(d^2/6) = {e.var()/(delta**2/6):.3f};"
      f" Cov(e_k,e_k+1)/(-d^2/12) = {np.mean(e[1:]*e[:-1])/(-delta**2/12):.3f}")
Tw = 5  # samples in 0.1 s window
d2 = q[2*Tw:] - 2 * q[Tw:-Tw] + q[:-2*Tw]
print(f"[2.2] second-difference quantisation var / sigma_q^2 = {d2.var()/(delta**2/12):.3f} (6 expected)")
nh = 3600 * fe
print(f"[2.2] spurious whitened random walk 1 h @50 Hz: n*d^2/6 = {nh*delta**2/6:.3e} m^2, sigma = {np.sqrt(nh*delta**2/6)*100:.2f} cm")
print(f"[2.2] heading quantisation bound: end-only delta/b = {delta/b:.2e} rad; start+end 2*delta/b = {2*delta/b:.2e} rad")
for Tn, name in [(dt, "per-step 20 ms"), (0.1, "window 0.1 s")]:
    print(f"[2.2] quantisation {name}: sigma_vx = d/(sqrt12 T) = {delta/(np.sqrt(12)*Tn):.2e} m/s,"
          f" sigma_w = d/(sqrt3 b T) = {delta/(np.sqrt(3)*b*Tn):.2e} rad/s, per wheel d/(sqrt6 T) = {delta/(np.sqrt(6)*Tn):.2e}")
for vv in (0.3, 1.0, 2.0):
    print(f"[2.2] slip (multiplicative) per-wheel velocity sigma: per-step {ss*vv:.2e}, 0.1 s window {ss*vv*np.sqrt(dt/0.1):.2e} m/s;"
          f" k_eff = ss^2 v dt = {ss**2*vv*dt:.1e} m")

# --- 2.3 augmented covariance vs Monte-Carlo (straight 20 m @1 m/s, psi error 0.1 %/wheel, slip noise)
psi = r / b
dphi = 1.0 * dt / r
steps = 1000
sig_psi = 1e-3 * psi
Spp = np.zeros((3, 3)); Spsi = np.zeros((3, 2)); Sww = sig_psi**2 * np.eye(2)
Swhite = np.zeros((3, 3))
th = 0.0
recs = {}
for k in range(steps):
    dsR = dsL = psi * b * dphi
    ds = 0.5 * (dsR + dsL); dth = (dsR - dsL) / b; ph = th + dth / 2
    Fp = np.array([[1, 0, -ds*np.sin(ph)], [0, 1, ds*np.cos(ph)], [0, 0, 1]])
    Fu = np.array([[0.5*np.cos(ph) - ds/(2*b)*np.sin(ph), 0.5*np.cos(ph) + ds/(2*b)*np.sin(ph)],
                   [0.5*np.sin(ph) + ds/(2*b)*np.cos(ph), 0.5*np.sin(ph) - ds/(2*b)*np.cos(ph)],
                   [1/b, -1/b]])
    Fpsi = Fu @ np.diag([b*dphi, b*dphi])
    Su = np.diag([ss**2*dsR**2, ss**2*dsL**2])
    Spp = Fp@Spp@Fp.T + Fp@Spsi@Fpsi.T + Fpsi@Spsi.T@Fp.T + Fpsi@Sww@Fpsi.T + Fu@Su@Fu.T
    Spsi = Fp@Spsi + Fpsi@Sww
    Swhite = Fp@Swhite@Fp.T + Fpsi@Sww@Fpsi.T + Fu@Su@Fu.T
    if k+1 in (250, 500, 1000):
        recs[k+1] = (Spp.copy(), Swhite.copy())
M = 4000
epsi = rng.normal(0, sig_psi, (M, 2))
x = np.zeros(M); y = np.zeros(M); t = np.zeros(M)
mc = {}
for k in range(steps):
    # truth moves straight; odometry error grows from psi error + multiplicative noise
    eps = rng.normal(0, ss, (M, 2))
    dsR = (psi + epsi[:, 0]) * b * dphi * (1 + eps[:, 0])
    dsL = (psi + epsi[:, 1]) * b * dphi * (1 + eps[:, 1])
    ds = 0.5*(dsR+dsL); dth = (dsR-dsL)/b; ph = t + dth/2
    x += ds*np.cos(ph); y += ds*np.sin(ph); t += dth
    if k+1 in recs:
        mc[k+1] = (np.var(t), np.var(y))
print("\n[2.3] L[m]  MC Var(th)  aug Var(th)  white Var(th) | MC Var(y)  aug Var(y)  white Var(y)")
for kk, (A, W) in recs.items():
    L = kk * dt
    print(f"      {L:4.0f}  {mc[kk][0]:.3e}  {A[2,2]:.3e}  {W[2,2]:.3e} | {mc[kk][1]:.3e}  {A[1,1]:.3e}  {W[1,1]:.3e}")

# --- 2.4 drift closed forms with config slip model (k_eff), psi exact
for vv, L in [(1.0, 20.0), (2.0, 20.0)]:
    k = ss**2 * vv * dt
    print(f"\n[2.4] v={vv}: k_eff={k:.1e} m, L={L}: sigma_th={np.sqrt(2*k*L/b**2):.4f} rad, sigma_x={np.sqrt(k*L/2)*1000:.1f} mm,"
          f" sigma_y={np.sqrt(2*k*L**3/(3*b**2)):.3f} m")
wr = 1.5 * b / 2
k = ss**2 * wr * dt
Th = 10 * 2 * np.pi
print(f"[2.4] rotation 10 rev @1.5 rad/s: wheel speed {wr:.3f} m/s, k_eff={k:.2e}, sigma_th={np.sqrt(k*Th/b):.4f} rad")
# MC check of the straight closed form @1 m/s
M = 3000; steps = int(20/(1.0*dt))
x = np.zeros(M); y = np.zeros(M); t = np.zeros(M)
for kk in range(steps):
    eps = rng.normal(0, ss, (M, 2)); d0 = 1.0*dt
    dsR = d0*(1+eps[:, 0]); dsL = d0*(1+eps[:, 1])
    ds = 0.5*(dsR+dsL); dth = (dsR-dsL)/b; ph = t+dth/2
    x += ds*np.cos(ph); y += ds*np.sin(ph); t += dth
k = ss**2*1.0*dt; L = 20
print(f"[2.4] MC @1 m/s 20 m: sigma_th {t.std():.4f} (cf {np.sqrt(2*k*L/b**2):.4f}), sigma_x {x.std()*1000:.1f} mm (cf {np.sqrt(k*L/2)*1000:.1f}),"
      f" sigma_y {y.std():.3f} (cf {np.sqrt(2*k*L**3/(3*b**2)):.3f})")
for eps_ in (0.005, 0.001):
    print(f"[2.2] systematic eps={eps_}: heading after 20 m = {20*eps_/b:.4f} rad, lateral = {400*eps_/(2*b):.3f} m")

# --- published twist covariance (per-step, 50 Hz) at 1 m/s straight
for vv in (0.3, 1.0, 2.0):
    sR2 = sL2 = ss**2*(vv*dt)**2 + delta**2/6
    Vvx = (sR2+sL2)/(4*dt**2); Vw = (sR2+sL2)/(b**2*dt**2)
    print(f"[2.3] published per-step twist v={vv}: sigma_vx={np.sqrt(Vvx):.2e} m/s (Var {Vvx:.2e}), sigma_w={np.sqrt(Vw):.3e} rad/s (Var {Vw:.2e})")

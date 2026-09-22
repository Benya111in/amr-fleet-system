"""Independent audit checks for task-execution-docking.md (revised brief).
Run: python3 audit_checks.py   (numpy/scipy only)
Each block prints the quantity the brief states and our re-derivation."""
import numpy as np
from scipy.optimize import least_squares, brentq
from scipy.stats import chi2, norm, binom, beta, multivariate_normal

rng = np.random.default_rng(7)
f = 320/np.tan(np.radians(43.5)); s = 0.15; sc = 0.2; lc = 0.18; lx = 0.15
print("f = %.1f px, VFOV = %.1f deg" % (f, np.degrees(2*np.arctan(240/f))))

# ---------------------------------------------------------------- 1. PnP CRLB (independent implementation)
h = s/2; corners = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
def rot(ax, a):
    c, s_ = np.cos(a), np.sin(a)
    if ax == 'x': return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
    if ax == 'y': return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
    return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
R_face = np.diag([1, -1, -1.0])  # marker normal pointing back at the camera
def proj(p):  # p = (X, Y, Z, theta about optical y, rx, rz)
    R = rot('y', p[3]) @ rot('x', p[4]) @ rot('z', p[5]) @ R_face
    P = corners @ R.T + p[:3]
    return np.concatenate([f*P[:, :2].T[0]/P[:, 2], f*P[:, :2].T[1]/P[:, 2]])
def jac(fun, p, eps=1e-7):
    u0 = fun(p); J = np.zeros((u0.size, p.size))
    for k in range(p.size):
        d = np.zeros(p.size); d[k] = eps; J[:, k] = (fun(p+d)-fun(p-d))/(2*eps)
    return J
def sigma_pnp(Z):
    p0 = np.array([0, 0, Z, 0, 0, 0.0]); J = jac(proj, p0)
    return sc**2*np.linalg.inv(J.T@J)

# ---------------------------------------------------------------- 2. brief's h_c(x) and its inverse -> dock frame
def h_c(x):
    xr, yr, psi = x
    cx_, cy_ = xr+lc*np.cos(psi), yr+lc*np.sin(psi)
    X = -cx_*np.sin(psi)+cy_*np.cos(psi)
    Zc = -cx_*np.cos(psi)-cy_*np.sin(psi)
    return np.array([X, Zc, psi])
print("\n== h_c sanity (brief §3.4)")
print("docked (-0.50,0,0) ->", np.round(h_c([-0.5, 0, 0]), 4), " expect X=0, Z=0.32")
print("left turn psi=+2deg at x=-0.5 ->", np.round(h_c([-0.5, 0, np.radians(2)]), 5), " expect X>0")
x = np.array([-0.8, 0.004, np.radians(0.5)]); Hc = jac(h_c, x)
print("H_c at pre-dock:\n", np.round(Hc, 4), "\n first-order X = y_r + (Z+l_c) psi -> dX/dpsi =", round(Hc[0, 2], 4), "(Z+l_c=0.80)")
print("\n== single-frame dock-frame sigma via inverse of h_c (x = h_c^-1(z), P = H^-1 R H^-T)")
print(" Z     sigZ   sigX(CRLB) sigX(closed Zsc/2f) sig_th[deg]  corr(X,th)  sig_yr[mm]  corr(yr,psi)")
for Z in [1.32, 1.0, 0.62, 0.32]:
    C = sigma_pnp(Z); idx = [0, 2, 3]; Rc = C[np.ix_(idx, idx)]
    xr = -(Z+lc); Hc = jac(h_c, np.array([xr, 0, 0.0])); Hi = np.linalg.inv(Hc)
    P = Hi@Rc@Hi.T
    print("%.2f  %5.2f  %6.3f      %6.3f            %6.3f      %+.2f       %6.2f      %+.3f" % (
        Z, np.sqrt(C[2, 2])*1e3, np.sqrt(C[0, 0])*1e3, Z*sc/(2*f)*1e3, np.degrees(np.sqrt(C[3, 3])),
        C[0, 3]/np.sqrt(C[0, 0]*C[3, 3]), np.sqrt(P[1, 1])*1e3, P[1, 2]/np.sqrt(P[1, 1]*P[2, 2])))

# MC with LM PnP at Z = 0.62, then invert h_c exactly
Z = 0.62; p0 = np.array([0, 0, Z, 0, 0, 0.0]); u0 = proj(p0); ests = []
for i in range(600):
    u = u0+rng.normal(0, sc, 8)
    r = least_squares(lambda p: proj(p)-u, p0+np.array([1e-3, 0, 5e-3, 5e-3, 0, 0]), method='lm'); ests.append(r.x)
ests = np.array(ests)
def inv_hc(z):
    return least_squares(lambda x: h_c(x)-z, np.array([-(z[1]+lc), 0, z[2]])).x
xs = np.array([inv_hc(np.array([e[0], e[2], e[3]])) for e in ests])
print("MC Z=0.62 (600 LM-PnP): sig_yr = %.1f mm, sig_psi = %.3f deg, corr = %+.3f" % (
    xs[:, 1].std()*1e3, np.degrees(xs[:, 2].std()), np.corrcoef(xs[:, 1], xs[:, 2])[0, 1]))

# ---------------------------------------------------------------- 3. LiDAR line fit: brief formula vs exact beam geometry
print("\n== LiDAR line fit (0.5 deg beams, sigma_r=0.03 along beam, plate L=0.8)")
def lidar_mc(R, L=0.8, sr=0.03, n=4000):
    half = np.arctan(L/2/R); ang = np.arange(-np.pi, np.pi, np.radians(0.5))
    ang = ang[np.abs(ang) <= half]; rng_true = R/np.cos(ang)
    al, ds = [], []
    for k in range(n):
        r = rng_true+rng.normal(0, sr, ang.size); px, py = r*np.cos(ang), r*np.sin(ang)
        A = np.c_[py, np.ones_like(py)]; (m, b), *_ = np.linalg.lstsq(A, px, rcond=None)  # x = m*y + b
        al.append(np.arctan(m)); ds.append(b/np.sqrt(1+m*m))
    return ang.size, np.degrees(np.std(al)), np.std(ds)*1e3
for name, R in [("staging", 1.35), ("pre-dock", 0.65), ("docked", 0.35)]:
    N = 2*np.arctan(0.4/R)/np.radians(0.5); sa = 0.03/0.8*np.sqrt(12/N)
    Nmc, sa_mc, sd_mc = lidar_mc(R)
    print("%-9s R=%.2f  N_formula=%.0f N_beams=%d  sig_alpha formula %.2f deg / MC %.2f deg   sig_d formula %.1f mm / MC %.1f mm" % (
        name, R, N, Nmc, np.degrees(sa), sa_mc, 0.03/np.sqrt(N)*1e3, sd_mc))

# ---------------------------------------------------------------- 4. fused estimate at pre-dock (1 s hold, x2 inflation)
print("\n== fused (y_r, psi) at pre-dock: 30 camera frames + 10 LiDAR scans, x2 variance inflation")
def cam_info_yp(Z):
    C = sigma_pnp(Z); idx = [0, 2, 3]; Rc = C[np.ix_(idx, idx)]
    Hc = jac(h_c, np.array([-(Z+lc), 0, 0.0])); return Hc.T@np.linalg.inv(Rc)@Hc
_, sa_mc_pre, sd_mc_pre = lidar_mc(0.65, n=3000)
HL = np.array([[-1, 0, 0], [0, 0, 1.0]]); RL = np.diag([(sd_mc_pre/1e3)**2, np.radians(sa_mc_pre)**2])
I = 30*cam_info_yp(0.62)+10*HL.T@np.linalg.inv(RL)@HL
P = 2*np.linalg.inv(I)
print("sig_x=%.2f mm sig_y=%.2f mm sig_psi=%.3f deg corr(y,psi)=%+.3f" % (
    np.sqrt(P[0, 0])*1e3, np.sqrt(P[1, 1])*1e3, np.degrees(np.sqrt(P[2, 2])), P[1, 2]/np.sqrt(P[1, 1]*P[2, 2])))
# terminal 0.30 m straight: F = d x_dock / d x_pre ; P_t = F P F^T + Q_t
F = np.eye(3); F[1, 2] = 0.30
Qt = np.diag([(0.01*0.30)**2, (0.01*0.30)**2*0, 0.0])  # conservative fully correlated 1 % longitudinal slip
# heading drift over 0.30 m: EKF-odom fuses gyro (0.0002 rad/s, 100 Hz) -> ARW over 3 s:
arw = 0.0002*np.sqrt(0.01*3.0); Qt[2, 2] = arw**2; Qt[1, 1] = (0.30*arw)**2
Pt = F@P@F.T+Qt
print("P_t = F P F^T + Q_t : sig_y(docked)=%.2f mm sig_psi=%.3f deg corr=%+.3f   [brief (P+Q_t, indep.): 3.1 mm]" % (
    np.sqrt(Pt[1, 1])*1e3, np.degrees(np.sqrt(Pt[2, 2])), Pt[1, 2]/np.sqrt(Pt[1, 1]*Pt[2, 2])))
Pt_naive = P+Qt
tau = np.array([0.015, np.radians(0.6)])
def rect_prob(mu, S):
    mvn = multivariate_normal(mean=mu, cov=S)
    return (mvn.cdf(tau)-mvn.cdf([-tau[0], tau[1]])-mvn.cdf([tau[0], -tau[1]])+mvn.cdf(-tau))
for mu_psi in [0.0, 0.3]:
    mu = np.array([0.0, np.radians(mu_psi)])
    S = Pt[1:, 1:]; muF = F[1:, 1:]@mu
    print("gate mu_psi=%.1f deg: joint(F-propagated)=%.4f  psi-marginal=%.4f" % (
        mu_psi, rect_prob(muF, S), norm.cdf((tau[1]-muF[1])/np.sqrt(S[1, 1]))-norm.cdf((-tau[1]-muF[1])/np.sqrt(S[1, 1]))))
# brief's example sigma_y 8 mm, 0.3 deg, rho 0.7
S = np.array([[0.008**2, 0.7*0.008*np.radians(0.3)], [0.7*0.008*np.radians(0.3), np.radians(0.3)**2]])
py = 2*norm.cdf(0.015/0.008)-1; pp = 2*norm.cdf(0.6/0.3)-1
print("brief example: joint=%.4f product=%.4f bonferroni=%.4f" % (rect_prob(np.zeros(2), S), py*pp, py+pp-1))

# ---------------------------------------------------------------- 5. NEES, E4 power, CP bound
print("\n== NEES mean 95%% interval: N=90 [%.2f, %.2f], N=100 [%.2f, %.2f]" % (
    chi2.ppf(.025, 270)/90, chi2.ppf(.975, 270)/90, chi2.ppf(.025, 300)/100, chi2.ppf(.975, 300)/100))
for n, k in [(100, 97), (300, 291)]:
    for p in [0.97, 0.99]:
        print("E4 n=%d pass>=%d  true p=%.2f -> P(pass)=%.3f" % (n, k, p, 1-binom.cdf(k-1, n, p)))
print("Clopper-Pearson 95%% two-sided lower bound at 291/300 = %.4f, at 297/300 = %.4f" % (
    beta.ppf(0.025, 291, 10), beta.ppf(0.025, 297, 4)))
k_needed = [k for k in range(280, 301) if beta.ppf(0.05, k, 300-k+1) >= 0.97]
print("min successes /300 for one-sided 95%% CP lower bound >= 0.97: %s" % (k_needed[0] if k_needed else None))

# ---------------------------------------------------------------- 6. expected tardiness closed form vs numeric
lam, sig = -12.0, 20.0; z = -lam/sig
closed = sig*(z*norm.cdf(z)+norm.pdf(z))
Cs = rng.normal(0, sig, 2_000_000); numeric = np.mean(np.maximum(Cs-lam, 0))  # C-d = -lam + noise
print("\n== E[(C-d)^+]: closed %.3f s vs MC %.3f s" % (closed, numeric))
print("LST vs EDF counterexample: A slack %d, B slack %d -> EDF picks B, LST picks A" % (300-200, 250-60))

# ---------------------------------------------------------------- 7. mass scaling from robot_params.yaml
m0 = 45.0+2*1.0+2*0.3
print("\n== unladen mass m0 = %.1f kg (robot_params.yaml: base 45 + wheels 2x1.0 + casters 2x0.3)" % m0)
for m in [0, 2, 10, 25]:
    print("  load %2d kg: a_max = %.3f m/s^2, a_dock = %.3f m/s^2  (with 45 kg: %.3f / %.3f)" % (
        m, 1.0*m0/(m0+m), 0.5*m0/(m0+m), 45/(45+m), 0.5*45/(45+m)))

# ---------------------------------------------------------------- 8. safety zones along the docking path
print("\n== footprint-edge clearance to plate (front edge = x_r + 0.30)")
for name, xr in [("staging", -1.50), ("pre-dock", -0.80), ("docked", -0.50)]:
    print("  %-8s clearance %.2f m" % (name, -(xr+0.30)))
for v in [0.10, 0.20, 0.25]:
    print("  braking d(v=%.2f) = v*0.15 + v^2/(2*1.0) = %.3f m" % (v, v*0.15+v*v/2))

# ---------------------------------------------------------------- 9. response-time budget incl. injected comm latency U(0,100) ms
mean_terms = dict(json=5, dispatch=1, assign_rpc=5, comm_latency=50, bt_wake=1, nav_accept=10, plan=60, controller=25, profiler=10, safety=10)
max_terms = dict(json=5, dispatch=1, assign_rpc=5, comm_latency=100, bt_wake=50, nav_accept=10, plan=100, controller=50, profiler=20, safety=20)
print("\n== E8 budget: mean %d ms, worst %d ms" % (sum(mean_terms.values()), sum(max_terms.values())))

# ---------------------------------------------------------------- 10. Nav2 smooth control law (humble source) incl. omega saturation rescale
def law(r, phi, delta, kphi=2.0, kdelta=1.0, beta_=0.4, lam_=2.0, vmax=0.25, vmin=0.05, rslow=0.25, wmax=0.75, adock=None):
    kappa = -1.0/r*(kdelta*(delta-np.arctan(-kphi*phi))+(1+kphi/(1+(kphi*phi)**2))*np.sin(delta))
    v = vmax/(1+beta_*abs(kappa)**lam_); v = min(vmax*r/rslow, v)
    if adock: v = min(v, np.sqrt(2*r*adock))
    v = np.clip(v, vmin, vmax); w = kappa*v; wb = np.clip(w, -wmax, wmax)
    v = wb/kappa if kappa != 0 else v
    return v, wb
def sim(x0, target, adock=0.328, dt=0.02, T=30):
    x = np.array(x0, float); vmin_viol = 0; sat = 0
    for k in range(int(T/dt)):
        dx, dy = target[0]-x[0], target[1]-x[1]; r = np.hypot(dx, dy)
        if r < 0.005: break
        los = np.arctan2(dy, dx); phi = np.arctan2(np.sin(target[2]-los), np.cos(target[2]-los))
        delta = np.arctan2(np.sin(x[2]-los), np.cos(x[2]-los))
        # Nav2 EgocentricPolarCoordinates: phi = target yaw - LOS, delta = robot yaw - LOS
        v, w = law(r, phi, delta, adock=adock)
        if abs(w) >= 0.75-1e-9: sat += 1
        if v < 0.05-1e-9: vmin_viol += 1
        x += dt*np.array([v*np.cos(x[2]), v*np.sin(x[2]), w])
    return x, r, sat, vmin_viol
print("\n== control law: staging offsets -> pre-dock target (-0.80, 0, 0)")
worst = 0
for y0 in [-0.15, 0.0, 0.15]:
    for p0 in [-10, 0, 10]:
        xf, r, sat, vv = sim([-1.5, y0, np.radians(p0)], (-0.80, 0.0, 0.0))
        worst = max(worst, abs(xf[1]))
        print("  y0=%+.2f psi0=%+3d -> final y=%+.4f psi=%+.2f deg r=%.4f  omega-sat steps=%d  v<v_min steps=%d" % (
            y0, p0, xf[1], np.degrees(xf[2]), r, sat, vv))

# ---------------------------------------------------------------- 11. alternative A: two 0.12 m markers, baseline b=0.40, yaw from depth difference
print("\n== alt A (two 0.12 m markers, b = 0.40 m)")
s_small = 0.12; hs = s_small/2
def proj_small(p):
    cs = np.array([[-hs, hs, 0], [hs, hs, 0], [hs, -hs, 0], [-hs, -hs, 0]])
    R = rot('y', p[3]) @ rot('x', p[4]) @ rot('z', p[5]) @ R_face
    P = cs @ R.T + p[:3]
    return np.concatenate([f*P[:, 0]/P[:, 2], f*P[:, 1]/P[:, 2]])
for Z in [0.45, 1.0]:
    J = jac(proj_small, np.array([0.2, 0, Z, 0, 0, 0.0])); C = sc**2*np.linalg.inv(J.T@J)
    sZ = np.sqrt(C[2, 2]); sZcf = Z*sc/(f*s_small/Z)
    print("Z=%.2f: sigZ(0.12 marker) CRLB %.2f mm / closed %.2f mm -> yaw sqrt2*sigZ/b = %.2f deg (CRLB) / %.2f deg (closed); single 0.15 marker %.2f deg" % (
        Z, sZ*1e3, sZcf*1e3, np.degrees(np.sqrt(2)*sZ/0.40), np.degrees(np.sqrt(2)*sZcf/0.40), np.degrees(np.sqrt(sigma_pnp(Z)[3, 3]))))

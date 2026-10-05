"""v3.1 audit (independent of c1-c8 code): re-derive headline numbers from the formulas in the brief,
plus the items found in the resumed audit:
  (A) headline numbers (sec 2.1-2.4, 3.2, 3.3, 4.2, 4.3, 4.6) recomputed from closed forms
  (B) EIV shrink magnitude per update (initial vs after the excited direction has converged, S -> R)
  (C) constant heading offset from the start-count quantisation -> lateral error (sec 2.3 note)
  (D) H3 exact statistics: per-window detection probability (non-central chi2_2) and 2-of-2 event detection
  (E) steady symmetric slip: channel-2 residual r_v is a high-pass of slip velocity (visible only for T_a);
      compare the v3 hysteresis (A) with a cumulative-residual hold (B); H0 flagged time fraction of (B)
All numbers use the project config: r=0.0825, b=0.36, N=4096, 50 Hz encoder, sigma_s=0.01, IMU 100 Hz,
sigma_g=2e-4, sigma_a=0.017, T_w=0.1, T_a=0.5.
"""
import numpy as np
from scipy import stats

r, b, N, dte, ss = 0.0825, 0.36, 4096, 0.02, 0.01
sg, sa, dti, Tw, Ta = 2e-4, 0.017, 0.01, 0.1, 0.5
delta = 2*np.pi*r/N; psi = r/b
tau_on, tau_off = stats.chi2.ppf(0.999, 2), stats.chi2.ppf(0.95, 2)
Pba = (sa/np.sqrt(6000))**2; Pbg = (sg/np.sqrt(6000))**2
P_conv = (1e-3*psi)**2
ok = []
def chk(name, val, ref, rtol=0.03):
    good = abs(val-ref) <= rtol*abs(ref)
    ok.append(good); print(f"[{'OK ' if good else 'BAD'}] {name}: computed {val:.4g} vs brief {ref:.4g}")

# ---------------- (A) headline numbers ----------------
dth = 1.5*dte
chk("2.1 dth^2/24", dth**2/24, 3.75e-5)
chk("2.1 per-step error @2 m/s", 2*dte*dth**2/24, 1.5e-6)
chk("2.1 7.2 km bound", 7200*dth**2/24, 0.27)
chk("2.2 delta", delta, 1.27e-4, 0.01)
chk("2.2 spurious RW sigma 1 h", np.sqrt(3600*50*delta**2/6), 0.022)
chk("2.2 heading quant bound 2delta/b", 2*delta/b, 7.0e-4)
def sig2(v, T):  # per-wheel variance of displacement over T (slip + quantisation), k_phys = 0
    return ss**2*(abs(v)*dte)*(abs(v)*T) + delta**2/6
chk("2.2 slip-term wheel speed sigma 2 m/s T_w", np.sqrt(ss**2*(2*dte)*(2*Tw))/Tw, 8.9e-3)
for v, sv, sw in ((1.0, 7.3e-3, 0.041), (2.0, 1.4e-2, 0.079)):
    s2 = sig2(v, dte)
    chk(f"2.3 sigma_vx {v} m/s", np.sqrt(2*s2/(4*dte**2)), sv)
    chk(f"2.3 sigma_w {v} m/s", np.sqrt(2*s2/(b**2*dte**2)), sw)
for v, sth, sx, sy in ((1.0, 0.025, 4.5e-3, 0.29), (2.0, 0.035, 6.3e-3, 0.41)):
    k = ss**2*v*dte; L = 20
    chk(f"2.4 sigma_th {v}", np.sqrt(2*k*L/b**2), sth); chk(f"2.4 sigma_x {v}", np.sqrt(k*L/2), sx)
    chk(f"2.4 sigma_y {v}", np.sqrt(2*k*L**3/(3*b**2)), sy)
k = ss**2*(1.5*b/2)*dte
chk("2.4 rotation 10 rev sigma_th", np.sqrt(k*20*np.pi/b), 0.0097)
# depth
hc = 0.18 + 0.25; vf = np.arctan(np.tan(np.radians(43.5))*480/640)
sig_d = lambda d: np.sqrt(0.005**2 + (0.002*d**2)**2)
Z = np.linspace(hc/np.tan(vf), 5.0, 2000)
chk("3.2 floor visible from Z", hc/np.tan(vf), 0.60)
chk("3.2 max 3sigma floor height err", np.max(3*sig_d(Z)*hc/Z), 0.013)
# LPF exact design
for fc, a_ref in ((20, 0.673), (10, 0.456)):
    c = np.cos(2*np.pi*fc/100); be = (2-c) - np.sqrt((2-c)**2-1); al = 1-be
    w = 2*np.pi*fc/100; H = al/abs(1-be*np.exp(-1j*w))
    chk(f"3.3 alpha fc={fc}", al, a_ref, 0.005); chk(f"3.3 |H(fc)|^2 fc={fc}", H**2, 0.5, 1e-6)
# SACO table (sqrt S_w, sqrt S_v) incl. converged-psi term
for v, sw_ref, sv_ref in ((0.3, 0.0058, 0.0019), (1.0, 0.018, 0.0047), (2.0, 0.036, 0.0090)):
    s2 = sig2(v, Tw); phd = v/r
    Sw = 2*s2/(b**2*Tw**2) + 2*phd**2*P_conv + sg**2/10 + Pbg
    Sv = 2*(2*s2/(4*Tw**2)) + sa**2*dti*Ta + Pba*Ta**2
    chk(f"4.2 sqrt S_w {v}", np.sqrt(Sw), sw_ref); chk(f"4.2 sqrt S_v {v}", np.sqrt(Sv), sv_ref)
# NEES band
chk("4.6 NEES 20-run band lo", stats.chi2.ppf(0.025, 60)/20, 2.02, 0.005)
chk("4.6 NEES 20-run band hi", stats.chi2.ppf(0.975, 60)/20, 4.16, 0.005)
# GRC constants
qv = (1e-3*psi)**2/6000
print(f"      4.3 q = {qv:.3e} (brief v3 wrote 8.7e-12; correct rounding 8.8e-12)")
chk("4.3 P_max", (0.05*psi)**2, 1.3e-4)
# ---------------- (B) EIV shrink per update ----------------
v = 1.0; phd = v/r; s2 = sig2(v, Tw); Sh = s2/(r**2*Tw**2); R = 2*s2/(b**2*Tw**2) + sg**2/10
Pmax = (0.05*psi)**2; h = np.array([phd, -phd])
S0 = h@(Pmax*np.eye(2))@h + R
print(f"(B) EIV shrink per update, straight 1 m/s: initial (S = h'P h + R = {S0:.3g}) {Pmax*Sh*psi/S0/psi*100:.4f} % ;"
      f" after excited direction converged (S ~ R = {R:.3g}) {Pmax*Sh*psi/R/psi*100:.3f} % of psi")
# ---------------- (C) start-count quantisation offset ----------------
L = 20.0
print(f"(C) constant heading offset from start counts: sigma = delta/(sqrt6 b) = {delta/np.sqrt(6)/b:.2e} rad"
      f" -> lateral after {L:.0f} m = {L*delta/np.sqrt(6)/b*1e3:.1f} mm (vs non-systematic sigma_y 0.29 m)")
# ---------------- (D) H3 exact statistics ----------------
def p_win(m):  # single-channel mean at m x resolution: noncentrality = m^2 tau_on, chi2 with 2 dof
    return stats.ncx2.sf(tau_on, 2, (m**2)*tau_on)
def p_2consec(p, n):  # P(at least two consecutive successes in n independent trials)
    a0, a1, done = 1.0, 0.0, 0.0  # states: last trial failed / last trial succeeded (no run yet)
    for _ in range(n):
        a0, a1, done = (a0+a1)*(1-p), a0*p, done + a1*p
    return done
print("(D) per-window P(d2 > tau_on) and event detection P(2 consecutive):")
for m in (1.0, 1.25, 1.5, 2.0):
    p = p_win(m)
    print(f"    m = {m:4.2f} x resolution: p_win = {p:.3f}; ch2 step event (visible 5 windows) {p_2consec(p, 5):.3f};"
          f" ch1 event 0.3 s (3 windows) {p_2consec(p, 3):.3f}, 0.5 s (5) {p_2consec(p, 5):.3f}")
# ---------------- (E) steady symmetric slip: high-pass residual vs cumulative hold ----------------
def simulate(rng, v0, slip_fn, Tsim, hold_cum=True, Tmax=10.0):
    """10 Hz windows; encoder 50 Hz per-step multiplicative slip noise; IMU 100 Hz white + residual bias.
    slip_fn(t): symmetric wheel over-speed [m/s] (true body speed v0 constant, a = 0)."""
    nW = int(round(Tsim/Tw)); ba_err = rng.normal(0, np.sqrt(Pba))
    t_e = np.arange(int(round(Tsim/dte)))*dte + dte
    ve = v0 + slip_fn(t_e)
    ds = (ve*dte)[:, None]*(1 + rng.normal(0, ss, (t_e.size, 2)))  # two wheels
    vbar = ds.mean(1).reshape(nW, 5).sum(1)/Tw                     # window-mean body speed from encoders
    a = rng.normal(0, sa, int(round(Tsim/dti))) + ba_err
    I = np.concatenate([[0.0], np.cumsum(a*dti)])                   # cumulative integral at 100 Hz
    Ia = I[1:].reshape(nW, 10).mean(1)                              # window-mean of I(s)
    s2 = sig2(v0, Tw); Vv = 2*s2/(4*Tw**2); Sw = 2*s2/(b**2*Tw**2) + 2*(v0/r)**2*P_conv + sg**2/10 + Pbg
    Sv = 2*Vv + sa**2*dti*Ta + Pba*Ta**2
    k = int(round(Ta/Tw)); on = False; cnt = 0; low = 0; flags = np.zeros(nW, bool); ref = None
    for i in range(k, nW):
        rv = (vbar[i]-vbar[i-k]) - (Ia[i]-Ia[i-k])
        rw = rng.normal(0, np.sqrt(Sw))                             # symmetric slip: r_w carries no signal
        d2 = rw**2/Sw + rv**2/Sv
        if not on:
            cnt = cnt+1 if d2 > tau_on else 0
            if cnt >= 2: on, low, ref = True, 0, i-k-1               # reference = window before the onset pair
        else:
            quiet = d2 < tau_off
            if hold_cum:
                dt_ref = (i-ref)*Tw
                shat = (vbar[i]-vbar[ref]) - (Ia[i]-Ia[ref])
                Ss = 2*Vv + sa**2*dti*dt_ref + Pba*dt_ref**2
                quiet = quiet and (shat**2/Ss < stats.chi2.ppf(0.95, 1) or dt_ref > Tmax)
            low = low+1 if quiet else 0
            if low >= 2: on, cnt = False, 0
        flags[i] = on
    return flags
rng = np.random.default_rng(7)
ramp = lambda t0, t1, amp: (lambda t: amp*np.clip((t-t0)/0.1, 0, 1)*(t < t1) + amp*np.clip(1-(t-t1)/0.1, 0, 1)*(t >= t1))
print("(E) steady symmetric slip 5 % at 1 m/s (0.05 m/s over-speed, onset 0.1 s ramp) from 5 s to 15 s, 50 runs:")
for hold in (False, True):
    cov = []; det = 0
    for _ in range(50):
        f = simulate(rng, 1.0, ramp(5.0, 15.0, 0.05), 25.0, hold_cum=hold)
        seg = f[int(5.3/Tw):int(15.0/Tw)]; cov.append(seg.mean()); det += f[int(5.0/Tw):int(6.0/Tw)].any()
    print(f"    {'cumulative hold (v3.1)' if hold else 'v3 hysteresis only   '}: onset detected {det}/50,"
          f" flagged fraction of slip interval median {np.median(cov):.2f} (min {np.min(cov):.2f})")
for v0 in (0.3, 1.0, 2.0):
    fr = [simulate(rng, v0, lambda t: 0*t, 600.0, hold_cum=True).mean() for _ in range(6)]
    fa = [simulate(rng, v0, lambda t: 0*t, 600.0, hold_cum=False).mean() for _ in range(6)]
    print(f"(E) H0 {v0} m/s, 1 h total: flagged time fraction v3 {np.mean(fa):.1e}, with cumulative hold {np.mean(fr):.1e}")
print(f"ALL (A) CHECKS OK: {all(ok)} ({sum(ok)}/{len(ok)})")

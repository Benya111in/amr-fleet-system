"""Checks for sec 4.2 (SACO) with project config values.
Encoder 50 Hz (joint_states), IMU 100 Hz, sigma_g = 2e-4 rad/s, sigma_a = 0.017 m/s^2 (sensors.yaml),
bias from 60 s static mean (components.md imu_filter_node bias_estimation_time default 60 s).
"""
import numpy as np
from scipy.stats import chi2
rng = np.random.default_rng(7)
r, b, N, ss = 0.0825, 0.36, 4096, 0.01
dte, dti = 0.02, 0.01
Tw, Ta = 0.1, 0.5
delta = 2*np.pi*r/N
sg, sa = 2e-4, 0.017
Pbg = (sg/np.sqrt(6000))**2
Pba = (sa/np.sqrt(6000))**2
psi = r/b
Ppsi = (1e-3*psi)**2
tau_on = chi2.ppf(0.999, 2); tau_off = chi2.ppf(0.95, 2)
print(f"tau_on(chi2_2, 0.999) = {tau_on:.3f}, tau_off(0.95) = {tau_off:.3f}, chi2_2 0.99 = {chi2.ppf(0.99,2):.3f}")

def sig2(v, T):  # per-wheel displacement variance over interval T (straight, speed v)
    return ss**2*(v*dte)*(v*T) + delta**2/6

print("\n v | sqrt(S_w) [rad/s] | asym dv [m/s] | sqrt(S_v) [m/s] | sym dv [m/s] | parts of S_v (enc, imu, bias)")
for v in (0.3, 1.0, 2.0):
    s2 = sig2(v, Tw)
    phid = v/r
    Sw = 2*s2/(b**2*Tw**2) + Ppsi*2*phid**2 + sg**2/(Tw/dti) + Pbg
    Venc = 2*s2/(4*Tw**2)
    Sv = 2*Venc + sa**2*dti*Ta + Pba*Ta**2
    print(f" {v} | {np.sqrt(Sw):.4f} | {b*np.sqrt(tau_on*Sw):.4f} | {np.sqrt(Sv):.4f} | {np.sqrt(tau_on*Sv):.4f} |"
          f" {2*Venc:.2e}, {sa**2*dti*Ta:.2e}, {Pba*Ta**2:.2e}")
# parameter term of the velocity difference for a 1 m/s change over T_a
hdv = (b/2)*np.array([1.0/r, 1.0/r])
print(f"param term for dv=1 m/s over T_a: sqrt(h^T P h) = {np.sqrt(hdv@hdv*Ppsi):.2e} m/s")

# 0.3 m/s^2 symmetric slip detectability at 2 m/s after 0.2 s
v = 2.0; s2 = sig2(v, Tw); Sv = 2*2*s2/(4*Tw**2) + sa**2*dti*Ta + Pba*Ta**2
for dv in (0.045, 0.06):
    print(f"sym slip dv={dv}: r_v^2/S_v = {dv**2/Sv:.1f}")
print(f"0.3 m/s^2 slip: time to reach detectable dv at 2 m/s = {np.sqrt(tau_on*Sv)/0.3:.2f} s (+ window/2-of-2 latency)")

# v1 acceleration channel (critique reproduction and config re-evaluation)
k_old, b_old, r_old = 1e-5, 0.35, 0.075
for v in (1.0, 2.0):
    sv_old = np.sqrt(2*k_old*v*Tw)/(2*Tw)
    print(f"v1 accel channel (k=1e-5): v={v}: sigma_vbar={sv_old:.4f}, sigma_a={np.sqrt(2)*sv_old/Tw:.3f} m/s^2")
    sv_cfg = np.sqrt(2*sig2(v, Tw)/(4*Tw**2))
    sa_cfg = np.sqrt(2)*sv_cfg/Tw
    print(f"   same channel with config noise: sigma_a={sa_cfg:.3f} m/s^2 -> (0.3/sigma_a)^2 = {(0.3/sa_cfg)**2:.1f} vs tau_on")

# --- noise-free ramp: exactness of the aligned window operator
def profile(J, amax=1.0, vmax=2.0, t_end=12.0, h=1e-4):
    t = np.arange(0, t_end, h); a = np.zeros_like(t); acc = 0.0; vel = 0.0; vs = np.zeros_like(t)
    phase = 0
    for i in range(len(t)):
        # accelerate to vmax with jerk J, then hold, then decelerate at t=8 s
        if t[i] < 8.0:
            target = amax if vel < vmax - acc**2/(2*J) else 0.0
        else:
            target = -amax if vel > acc**2/(2*J) + 1e-6 else 0.0
        acc += np.clip(target-acc, -J*h, J*h)
        vel = max(vel + acc*h, 0.0)
        a[i] = acc; vs[i] = vel
    s = np.cumsum(vs)*h
    return t, a, vs, s, h

for J in (2.0, 10.0, 1000.0):
    t, a, vv, s, h = profile(J)
    ie = np.round(np.arange(0, 12.0, dte)/h).astype(int); ii = np.round(np.arange(0, 12.0, dti)/h).astype(int)
    se = s[ie]; ai = a[ii]
    I = np.concatenate([[0], np.cumsum(0.5*(ai[1:]+ai[:-1])*dti)])  # trapezoid on IMU samples
    worst = 0; worst_off = 0
    off = 5  # 5 ms timestamp offset: IMU shifted by half a sample -> use a shifted copy
    ai_off = a[np.clip(ii - int(0.005/h), 0, None)]
    I_off = np.concatenate([[0], np.cumsum(0.5*(ai_off[1:]+ai_off[:-1])*dti)])
    ke_w, ke_a = int(Tw/dte), int(Ta/dte); ki_w, ki_a = int(Tw/dti), int(Ta/dti)
    for m in range(ke_a+ke_w, len(ie)):
        vb = (se[m]-se[m-ke_w])/Tw; vb0 = (se[m-ke_a]-se[m-ke_a-ke_w])/Tw
        j = 2*m  # IMU index of the same instant
        idx = np.arange(j-ki_w+1, j+1)  # samples inside the window, then a mean over the window
        # mean over the window of I(s)-I(s-Ta): use trapezoid-consistent mean at IMU instants in [t-Tw, t]
        idx = np.arange(j-ki_w, j+1)
        wts = np.ones(len(idx)); wts[0] = wts[-1] = 0.5; wts /= wts.sum()
        rv = (vb - vb0) - np.sum(wts*(I[idx]-I[idx-ki_a]))
        rv_off = (vb - vb0) - np.sum(wts*(I_off[idx]-I_off[idx-ki_a]))
        worst = max(worst, abs(rv)); worst_off = max(worst_off, abs(rv_off))
    print(f"ramp jerk {J:6.0f} m/s^3: max |r_v| no-slip, synchronous = {worst:.2e} m/s; with 5 ms IMU lag = {worst_off:.2e} m/s")

# --- H0 Monte-Carlo: per-decision exceedance and correlation of consecutive decisions (straight 1 m/s)
v = 1.0; D = 400000  # decisions (10 Hz) = 40000 s
ne = D*5 + 5*6; ni = 2*ne
phi_true = np.arange(ne+1)*v*dte/r + 0.3
n_tick = np.round(phi_true*N/(2*np.pi))
dsq = np.diff(n_tick)*2*np.pi/N*r                      # quantised increments (common to both wheels)
dsR = dsq*(1+rng.normal(0, ss, ne)); dsL = dsq*(1+rng.normal(0, ss, ne))
# independent quantisation phase for the left wheel
n_tickL = np.round((phi_true+0.37)*N/(2*np.pi)); dsL = np.diff(n_tickL)*2*np.pi/N*r*(1+rng.normal(0, ss, ne))
wg = rng.normal(0, sg, ni); ax = rng.normal(0, sa, ni)
sR = np.concatenate([[0], np.cumsum(dsR)]); sL = np.concatenate([[0], np.cumsum(dsL)])
I = np.concatenate([[0], np.cumsum(ax*dti)])
s2 = sig2(v, Tw); Sw = 2*s2/(b**2*Tw**2) + sg**2/10; Sv = 2*2*s2/(4*Tw**2) + sa**2*dti*Ta
ms = np.arange(30, ne, 5)
thw = ((sR[ms]-sR[ms-5]) - (sL[ms]-sL[ms-5]))/b/Tw
gy = np.array([wg[2*m-10:2*m].mean() for m in ms[:20000]])
rw = thw[:20000] - gy
vb = 0.5*((sR[ms]-sR[ms-5]) + (sL[ms]-sL[ms-5]))/Tw
vb0 = 0.5*((sR[ms-25]-sR[ms-30]) + (sL[ms-25]-sL[ms-30]))/Tw
Iw = np.array([np.mean(I[2*m-9:2*m+1]-I[2*m-59:2*m-49]) for m in ms[:20000]])
rv = (vb[:20000]-vb0[:20000]) - Iw
print(f"\nH0 (1 m/s): empirical Var(r_w)/S_w = {rw.var()/Sw:.3f}, Var(r_v)/S_v = {rv.var()/Sv:.3f}")
d2 = rw**2/Sw + rv**2/Sv
print(f"H0: P(d2>tau_on) = {np.mean(d2>tau_on):.2e} (nominal 1e-3); corr(r_v[k], r_v[k+1]) = {np.corrcoef(rv[1:], rv[:-1])[0,1]:.3f};"
      f" corr(r_v[k], r_v[k+5]) = {np.corrcoef(rv[5:], rv[:-5])[0,1]:.3f}; corr(d2[k],d2[k+1]) = {np.corrcoef(d2[1:], d2[:-1])[0,1]:.3f}")
# NEES bands
print(f"\nchi2_3 2.5/97.5 % = {chi2.ppf(0.025,3):.3f}/{chi2.ppf(0.975,3):.3f}; 20-run averaged NEES band = "
      f"[{chi2.ppf(0.025,60)/20:.2f}, {chi2.ppf(0.975,60)/20:.2f}]")

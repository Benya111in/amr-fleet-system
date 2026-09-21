"""Monte Carlo check of sec 4.A (IAG-EKF) residual statistics under the sensors.yaml noise model.
(a) sigma_rv / sigma_rw closed forms and the chi2_2 claim for s = r^T Sigma_r^-1 r (incl. turning cross term)
(b) the residual-based R estimator: doc formula R_ww = C_rw - sg^2/20 vs corrected 10*(C_rw - sg^2/20)
(c) estimator spread: overlapping 0.2 s windows (W = 50) vs per-sample residuals (W = 50)
Run: python3 iag_residual_mc.py
"""
import math
import numpy as np
from scipy.stats import chi2

rng = np.random.default_rng(3)
ks, b = 0.01, 0.36
dte, dti = 0.02, 0.01
sg, sa = 2e-4, 0.017
sba = 0.017 / math.sqrt(1000)       # residual accel bias after 10 s calibration
NW = 10                              # encoder samples per 0.2 s window
NI = 20                              # IMU samples per window

def simulate(v, w, n_enc, bias_a=0.0):
    dsR = (v + w * b / 2) * dte
    dsL = (v - w * b / 2) * dte
    nR = rng.normal(0, ks * abs(dsR), n_enc)
    nL = rng.normal(0, ks * abs(dsL), n_enc)
    v_enc = ((dsR + nR) + (dsL + nL)) / (2 * dte)
    w_enc = ((dsR + nR) - (dsL + nL)) / (b * dte)
    g = w + rng.normal(0, sg, 2 * n_enc)
    a = 0.0 + bias_a + rng.normal(0, sa, 2 * n_enc)
    return v_enc, w_enc, g, a, dsR, dsL

def residuals(v_enc, w_enc, g, a):
    # window ending at encoder sample t: encoder t-9..t, IMU samples 2(t-9)..2t+1, r_v uses v(t) - v(t-10)
    T = len(v_enc)
    ts = np.arange(NW, T)
    rw = np.array([w_enc[t - NW + 1:t + 1].mean() - g[2 * (t - NW + 1):2 * t + 2].mean() for t in ts])
    rv = np.array([v_enc[t] - v_enc[t - NW] - a[2 * (t - NW + 1):2 * t + 2].sum() * dti for t in ts])
    return rw, rv

def sigma_r(dsR, dsL):
    vR, vL = (ks * dsR) ** 2, (ks * dsL) ** 2
    var_v1 = (vR + vL) / (4 * dte ** 2)
    var_w1 = (vR + vL) / (b ** 2 * dte ** 2)
    cov1 = (vR - vL) / (2 * b * dte ** 2)
    S = np.array([[var_w1 / NW + sg ** 2 / NI, cov1 / NW],
                  [cov1 / NW, 2 * var_v1 + NI * sa ** 2 * dti ** 2 + (sba * NI * dti) ** 2]])
    return S, var_v1, var_w1

print("(a) closed forms and chi2_2 calibration (bias residual included as random constant per run)")
for (v, w) in ((2.0, 0.0), (1.0, 1.5), (0.5, 1.39), (0.0, 1.5)):
    S, var_v1, var_w1 = sigma_r((v + w * b / 2) * dte, (v - w * b / 2) * dte)
    s_full, s_diag = [], []
    for run in range(40):
        ba = rng.normal(0, sba)
        v_enc, w_enc, g, a, dsR, dsL = simulate(v, w, 3000, bias_a=ba)
        rw, rv = residuals(v_enc, w_enc, g, a)
        R = np.stack([rw, rv], 1)
        Si = np.linalg.inv(S)
        s_full.append(np.einsum("ij,jk,ik->i", R, Si, R))
        s_diag.append(rw ** 2 / S[0, 0] + rv ** 2 / S[1, 1])
    s_full = np.concatenate(s_full); s_diag = np.concatenate(s_diag)
    corr = S[0, 1] / math.sqrt(S[0, 0] * S[1, 1])
    print(f" v={v} w={w}: sigma_rw={math.sqrt(S[0,0]):.4f} rad/s, sigma_rv={math.sqrt(S[1,1]):.4f} m/s, corr={corr:+.3f}; "
          f"mean s={s_full.mean():.3f} (chi2_2: 2), P(s>13.8)={np.mean(s_full>13.8):.4f} (0.001); diag-only P={np.mean(s_diag>13.8):.4f}")
    print(f"    gate thresholds (sqrt(13.8) sigma): |r_v| >= {math.sqrt(13.8)*math.sqrt(S[1,1]):.3f} m/s, |r_w| >= {math.sqrt(13.8)*math.sqrt(S[0,0]):.3f} rad/s")

print("\n(b)/(c) residual-based R_ww estimator, W = 50, v = 1 m/s straight")
v, w = 1.0, 0.0
S, var_v1, var_w1 = sigma_r(v * dte, v * dte)
print(f" true per-sample R_ww = {var_w1:.3e}, R_vv = {var_v1:.3e}")
est_doc, est_fix, est_ps, est_vv = [], [], [], []
for run in range(2000):
    v_enc, w_enc, g, a, dsR, dsL = simulate(v, w, 50 + NW)
    rw, rv = residuals(v_enc, w_enc, g, a)
    rw = rw[-50:]; rv = rv[-50:]
    C = rw.var(ddof=1)
    est_doc.append(C - sg ** 2 / NI)
    est_fix.append(NW * (C - sg ** 2 / NI))
    e = w_enc[-50:] - 0.5 * (g[-100::2] + g[-99::2])       # per-sample residual (2 gyro samples per encoder sample)
    est_ps.append(e.var(ddof=1) - sg ** 2 / 2)
    dv = np.diff(v_enc[-51:]) - (a[-100::2] + a[-99::2]) * dti   # 1-step velocity-difference residual
    est_vv.append(0.5 * (dv.var(ddof=1) - 2 * sa ** 2 * dti ** 2))
for name, e, tru in (("doc  C_rw - sg^2/20 (windowed)", est_doc, var_w1), ("fix  10(C_rw - sg^2/20) (windowed)", est_fix, var_w1),
                     ("per-sample e_w, W=50", est_ps, var_w1), ("per-sample 1-step dv -> R_vv, W=50", est_vv, var_v1)):
    e = np.array(e)
    print(f" {name:38s}: mean/true = {e.mean()/tru:.3f}, relative sd = {e.std()/tru:.2f}")
print(f" iid W=50 theory sqrt(2/49) = {math.sqrt(2/49):.2f}")

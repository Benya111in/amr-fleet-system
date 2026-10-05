"""Resumed audit (2026-09-22): checks for items found after the first audit pass.

1. imu_filter_node LPF (brief sec 7: 2nd-order Butterworth 20 Hz @ 100 Hz) group delay.
2. Slip-residual timing bias: windowed r_v bias = delta*(a(t)-a(t-Tw)) (zero under constant a),
   r_omega bias = delta*omega_dot; compare with sigma_r at operating points incl. the floor.
3. Constant-R baseline (repo ekf.yaml comment: wheel twist var 1e-4) vs sec 2.1 encoder model.
4. Pose-graph chi2/dof denominator with gauge fixing.
5. Heading lag of a delayed gyro during a turn -> odom-frame position error v*tau*dTheta.
"""
import numpy as np
from scipy import signal, stats

fs, fc = 100.0, 20.0
b, a = signal.butter(2, fc, fs=fs)
w, gd = signal.group_delay((b, a), w=np.array([0.01, 1.0, 2.0, 5.0]), fs=fs)
print("== 1 LPF group delay (samples -> ms) @", w, "Hz:", np.round(gd, 3), "->", np.round(gd / fs * 1e3, 2), "ms")
print("   analog approx sqrt2/(2 pi fc) =", round(np.sqrt(2) / (2 * np.pi * fc) * 1e3, 2), "ms")
tau = gd[0] / fs

# noise variance reduction of the LPF on white noise (for R of imu/data)
imp = signal.lfilter(b, a, np.r_[1.0, np.zeros(999)])
print("   white-noise variance gain of LPF:", round(float(np.sum(imp ** 2)), 3))

# residual sigmas (from iag_residual_mc.out, sec 4.A)
ks, bw, dt = 0.01, 0.36, 0.02
def sig_rw(v, om, floor=0.01):
    dsR = (v + om * bw / 2) * dt
    dsL = (v - om * bw / 2) * dt
    s = ks * np.sqrt(dsR ** 2 + dsL ** 2) / (bw * dt)
    s = max(s, floor)
    return np.sqrt(s ** 2 / 10 + (2e-4) ** 2 / 20)

print("\n== 2 timing bias vs residual sigma (Tw = 0.2 s, gate 13.8)")
jerk, Tw, omdot = 2.0, 0.2, 2.0
for d_ms in (2.0, 5.0, tau * 1e3):
    d = d_ms / 1e3
    print(f" delta = {d_ms:.1f} ms")
    # r_v: constant accel -> 0; jerk-limited ramp -> delta*jerk*Tw; step 1 m/s^2 -> delta*1
    for lab, da in (("const a", 0.0), ("jerk 2 m/s^3 ramp", jerk * Tw), ("step 1 m/s^2", 1.0)):
        print(f"   r_v bias ({lab}): {d*da*1e3:.2f} mm/s  -> /sigma_rv(floor 0.0141) = {d*da/0.0141:.2f}")
    for v, om in ((0.0, 0.75), (0.0, 1.5), (1.0, 1.0), (2.0, 0.0)):
        s = sig_rw(v, om)
        bias = d * omdot
        lam = (bias / s) ** 2
        pfa = 1 - stats.ncx2.cdf(13.8, 2, lam) if lam > 0 else 1 - stats.chi2.cdf(13.8, 2)
        print(f"   r_w v={v} w={om}: sigma {s:.4f} rad/s, bias {bias:.4f} ({bias/s:.1f} sigma), "
              f"P(s>13.8 | ramp) = {pfa:.3f}")
print("   max delta for bias <= 1 sigma at floor-limited sigma_rw:",
      round(sig_rw(0, 0.75) / omdot * 1e3, 2), "ms")

print("\n== 3 constant-R baseline vs encoder model")
for v in (0.5, 1.0, 2.0):
    sv2 = ks ** 2 * v ** 2 / 2
    sw2 = 2 * ks ** 2 * v ** 2 / bw ** 2
    print(f"   v={v}: sigma_v^2 {sv2:.2e} ({sv2/1e-4:.1f}x of 1e-4), sigma_w^2 {sw2:.2e} ({sw2/1e-4:.1f}x of 1e-4)")

print("\n== 4 pose-graph dof: residual dim 3|E|, free params 3(|V|-1) -> dof = 3(|E|-|V|+1)")
V, E = 300, 330
print(f"   e.g. |V|={V}, |E|={E}: doc 3|E|-3|V| = {3*E-3*V}, correct = {3*(E-V+1)}")

print("\n== 5 odom-frame position error from gyro delay during a turn: v*tau*dTheta")
for v, dth in ((1.0, np.pi / 2), (2.0, np.pi / 2), (0.5, np.pi)):
    print(f"   v={v}, dTheta={dth:.2f} rad: {v*tau*dth*100:.2f} cm (tau {tau*1e3:.1f} ms)")

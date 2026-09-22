"""Audit-2 check (alt T, Sec 2.6.2): torque-level wheel PI on M_v dv/dt = g u - b_v v (g = 1/r), zeta = 1,
omega_c = 6.  (a) 2-DOF b = 0.5 without feedforward (v2.1 text): S-curve 0->1 m/s ramp lag (analytic a/omega_c)
vs the v2.1 acceptance 'ramp tracking error <= 0.03 m/s'.  (b) acceleration feedforward u_ff = (Mhat a_r + bhat r)/g
+ PI on (r - y) (b = 1, model-following with ideal model y_m = r): matched and 25 kg payload unknown (Mhat 48.6
vs M 73.6) and known.  100 Hz controller, torque saturation 6 N m/wheel (12 N m total)."""
import numpy as np
r_w = 0.0825; g = 1/r_w; wc = 6.0
def scurve(v1, a=1.0, j=2.0, t0=0.1):
    tj = a/j; ap = a; tc = v1/a - tj
    def f(t):
        t = t - t0
        if t <= 0: return 0.0, 0.0
        if t < tj: return j*t*t/2, j*t
        v_a = j*tj*tj/2
        if t < tj + tc: return v_a + ap*(t - tj), ap
        t3 = t - tj - tc
        if t3 < tj: return v_a + ap*tc + ap*t3 - j*t3*t3/2, ap - j*t3
        return v1, 0.0
    return f
def run(M, Mhat, ff, b, T=3.0, dt=1e-4):
    Kp = 2*wc*Mhat/g; Ki = wc*wc*Mhat/g; f = scurve(1.0); y = I = u = 0.0; err = 0.0; nc = 100
    for k in range(int(T/dt)):
        r, a_r = f(k*dt)
        if k % nc == 0:
            I += Ki*(r - y)*0.01
            u = (Mhat*a_r/g if ff else 0.0) + Kp*(b*r - y) + I
            u = np.clip(u, -12.0, 12.0)
        y += dt*g*u/M; err = max(err, abs(r - y))
    return err
print(f"(a) b=0.5, no FF, matched: max|r-y| {run(48.6, 48.6, False, 0.5):.3f} m/s (analytic a/omega_c = {1/wc:.3f})")
print(f"(b) FF + b=1, matched: {run(48.6, 48.6, True, 1.0):.3f} m/s")
print(f"(b) FF + b=1, payload 25 kg unknown (Mhat 48.6, M 73.6): {run(73.6, 48.6, True, 1.0):.3f} m/s")
print(f"(b) FF + b=1, payload known via payload/mass: {run(73.6, 73.6, True, 1.0):.3f} m/s")

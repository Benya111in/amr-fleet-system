"""Audit-2 check: contract base design (Sec 2.6.1) = feedforward r + 2-DOF PI (b) at 50 Hz on the
DiffDrive velocity servo P(s) = exp(-Td s)/(Tp s + 1). Checks the acceptance criteria written in the
brief (0.15 m/s step overshoot <= 1 %, ramp tracking error <= 0.03 m/s at a = 1 m/s^2) for b in {0.5, 1},
the analytic steady ramp error a*Kp*(1-b)/Ki, and the phase lag the closed velocity loop adds at the PP
outer-loop bandwidth wn = sqrt(2)/t_L (M11 for the default design) and its effect on the PP overshoot."""
import numpy as np

def sim(Td, Tp, Kp, Ki, b, ref, T=4.0, dt=1e-4, fc=50.0):
    n = int(T/dt); nc = int(round(1/(fc*dt))); nd = int(round(Td/dt))
    y = 0.0; I = 0.0; u = 0.0; ybuf = [0.0]*(nd + 1); ys = np.zeros(n); rs = np.zeros(n)
    for k in range(n):
        t = k*dt; r = ref(t)
        if k % nc == 0:
            ym = ybuf[0]                                # delayed measurement (EKF + transport)
            I += Ki*(r - ym)/fc
            u = r + Kp*(b*r - ym) + I
        y += dt*(u - y)/Tp                              # first-order servo
        ybuf.append(y); ybuf.pop(0); ys[k] = y; rs[k] = r
    return rs, ys

step = lambda t: 0.15 if t > 0.1 else 0.0
ramp = lambda t: min(max(t - 0.1, 0.0)*1.0, 1.5)           # a = 1 m/s^2 ramp (S-curve constant-accel phase)
print("Analytic steady ramp error a*Kp*(1-b)/Ki (a=1): b=0.5 ->", 1.0*0.4*0.5/4.0, "m/s ; b=1 -> 0")
for Td, Tp in [(0.04, 0.05), (0.06, 0.10)]:
    for Kp, Ki in [(0.3, 3.0), (0.4, 4.0), (0.5, 5.0)]:
        for b in [0.5, 1.0]:
            r, y = sim(Td, Tp, Kp, Ki, b, step)
            os_ = 100*(y.max() - 0.15)/0.15
            r2, y2 = sim(Td, Tp, Kp, Ki, b, ramp, T=1.6)
            k0 = int(1.0/1e-4); k1 = int(1.55/1e-4)          # late part of the ramp (0.9-1.45 s after start)
            err = np.max(np.abs(r2[k0:k1] - y2[k0:k1]))
            print(f"Td={Td} Tp={Tp} Kp={Kp} Ki={Ki} b={b}: step 0.15 overshoot {os_:5.2f} %, ramp error (late) {err:.3f} m/s")

# phase lag of the closed velocity loop at the PP bandwidth (feedforward + PI, b), and PP overshoot with it
wn = np.sqrt(2)/0.8
def T_ref(s, Td, Tp, Kp, Ki, b):
    P = np.exp(-s*Td)/(Tp*s + 1); C = Kp + Ki/s
    return P*(1 + Kp*b + Ki/s)/(1 + P*C)
for Td, Tp in [(0.04, 0.05), (0.06, 0.10)]:
    for b in [0.5, 1.0]:
        H = T_ref(1j*wn, Td, Tp, 0.4, 4.0, b)
        print(f"lag Td={Td} Tp={Tp} b={b}: |T|={abs(H):.3f} phase {np.degrees(np.angle(H)):.1f} deg at wn={wn:.3f}")

def pp_overshoot(Td, Tp, Kp, Ki, b, v=1.0, L=0.8, T=12.0, dt=1e-4, fc=50.0):
    # straight-line PP (K=1): w_cmd = -(2v/L^2)(e + L psi), yaw-rate loop = feedforward + PI(b) on the servo
    nc = int(round(1/(fc*dt))); nd = int(round(Td/dt))
    e, psi, w, I, u = 0.2, 0.0, 0.0, 0.0, 0.0; buf = [0.0]*(nd + 1); emin = 0.0; wc = 0.0
    for k in range(int(T/dt)):
        if k % 1000 == 0 or k == 0:                     # PP at 10 Hz? no: controller 20 Hz
            pass
        if k % 500 == 0:
            wc = -(2*v/L**2)*(e + L*psi)                # controller_server 20 Hz
        if k % nc == 0:
            wm = buf[0]; I += Ki*(wc - wm)/fc; u = wc + Kp*(b*wc - wm) + I
        w += dt*(u - w)/Tp; buf.append(w); buf.pop(0)
        e += dt*v*np.sin(psi); psi += dt*w; emin = min(emin, e)
    return -100*emin/0.2
print("PP overshoot ideal (analytic 4.32 %)")
for Td, Tp in [(0.04, 0.05), (0.06, 0.10)]:
    for b in [0.5, 1.0]:
        print(f"PP overshoot with base inner loop Td={Td} Tp={Tp} b={b}: {pp_overshoot(Td, Tp, 0.4, 4.0, b):.2f} %")

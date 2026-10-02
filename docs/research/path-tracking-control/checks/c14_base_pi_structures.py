"""Audit-2 check (follow-up of c13): base design velocity loop structures on the DiffDrive servo
P = exp(-Ta s) * 1/(Tp s + 1), measurement delay Tm (EKF + transport), controller 50 Hz ZOH.
  A  (v2.1 text)      u = r + Kp(b r - y_m) + Ki int(r - y_m),  b = 0.5
  B  model-following  u = r + Kp(r_m - y_m) + Ki int(r_m - y_m), r_m = Phat r (1st order Tp_hat + delay Td_hat)
  C  B + lag FF       u = r + Tp_hat*rdot + PI(r_m - y_m),        r_m = r delayed Td_hat (reference = S-curve (v, a))
Metrics: 0.15 m/s step overshoot, S-curve 0->1 m/s (a 1, j 2) end overshoot, max |y - r_m| (model-following
error seen by the loop), max |y - r| (absolute lag), disturbance: servo gain drop 10 % (effort/slip) steady error,
and PP (K=1, straight, t_L=0.8, v=1) initial-offset overshoot with the same structure on the yaw-rate axis."""
import numpy as np
dt = 1e-4; fc = 50.0; nc = int(round(1/(fc*dt)))

def scurve(v1, a=1.0, j=2.0, t0=0.1):
    tj = a/j if v1 >= a*a/j else np.sqrt(v1/j); ap = j*tj; tc = max(v1/ap - tj, 0.0)
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

def run(struct, ref, Ta, Tm, Tp, Tp_hat=0.075, Td_hat=0.05, Kp=0.4, Ki=4.0, b=0.5, gain=1.0, T=4.0):
    n = int(T/dt); na = int(round(Ta/dt)); nm = int(round(Tm/dt)); nh = int(round(Td_hat/dt))
    y = 0.0; I = 0.0; u = 0.0; ua = [0.0]*(na + 1); yb = [0.0]*(nm + 1); rb = [0.0]*(nh + 1)
    rmod = 0.0; out = np.zeros((n, 3))
    for k in range(n):
        t = k*dt; r, rd = ref(t)
        rb.append(r); rb.pop(0)
        if struct == "B": rmod += dt*(rb[0] - rmod)/Tp_hat      # Phat r: delay then 1st order
        else: rmod = rb[0]
        if k % nc == 0:
            ym = yb[0]
            if struct == "A":
                I += Ki*(r - ym)/fc; u = r + Kp*(b*r - ym) + I
            elif struct == "B":
                I += Ki*(rmod - ym)/fc; u = r + Kp*(rmod - ym) + I
            else:
                I += Ki*(rmod - ym)/fc; u = r + Tp_hat*rd + Kp*(rmod - ym) + I
        ua.append(u); ua.pop(0)
        y += dt*(gain*ua[0] - y)/Tp
        yb.append(y); yb.pop(0)
        out[k] = (r, y, rmod)
    return out

def stepref(t): return (0.15 if t > 0.1 else 0.0), 0.0
cases = [(0.02, 0.02, 0.05), (0.03, 0.03, 0.10), (0.02, 0.02, 0.10), (0.03, 0.03, 0.05)]
for s in ["A", "B", "C"]:
    for Ta, Tm, Tp in cases:
        o = run(s, stepref, Ta, Tm, Tp); os_ = 100*(o[:, 1].max() - 0.15)/0.15
        f = scurve(1.0); o2 = run(s, lambda t: f(t), Ta, Tm, Tp)
        os2 = 100*(o2[:, 1].max() - 1.0)
        em = np.abs(o2[:, 1] - o2[:, 2]).max(); ea = np.abs(o2[:, 1] - o2[:, 0]).max()
        o3 = run(s, lambda t: f(t), Ta, Tm, Tp, gain=0.9, T=6.0); ess = 1.0 - o3[-1, 1]
        print(f"{s} Ta={Ta} Tm={Tm} Tp={Tp}: step0.15 OS {os_:5.2f} % | S-curve 0->1 OS {os2:5.2f} %, "
              f"max|y-r_m| {em:.3f}, max|y-r| {ea:.3f} m/s | 10 % gain drop: steady err {ess:.4f}")

def pp_os(struct, Ta, Tm, Tp, v=1.0, L=0.8, T=12.0):
    na = int(round(Ta/dt)); nm = int(round(Tm/dt)); nh = int(round(0.05/dt))
    e, psi, w, I, u = 0.2, 0.0, 0.0, 0.0, 0.0; ua = [0.0]*(na + 1); yb = [0.0]*(nm + 1); rb = [0.0]*(nh + 1)
    wc = 0.0; wc_prev = 0.0; rd = 0.0; rmod = 0.0; emin = 0.0
    for k in range(int(T/dt)):
        if k % 500 == 0:
            wc_prev, wc = wc, -(2*v/L**2)*(e + L*psi); rd = (wc - wc_prev)/0.05
        rb.append(wc); rb.pop(0)
        if struct == "B": rmod += dt*(rb[0] - rmod)/0.075
        else: rmod = rb[0]
        if k % nc == 0:
            wm = yb[0]
            if struct == "A": I += 4.0*(wc - wm)/fc; u = wc + 0.4*(0.5*wc - wm) + I
            elif struct == "B": I += 4.0*(rmod - wm)/fc; u = wc + 0.4*(rmod - wm) + I
            elif struct == "C": I += 4.0*(rmod - wm)/fc; u = wc + 0.075*rd + 0.4*(rmod - wm) + I
            else: u = wc
        ua.append(u); ua.pop(0)
        w += dt*(ua[0] - w)/Tp; yb.append(w); yb.pop(0)
        e += dt*v*np.sin(psi); psi += dt*w; emin = min(emin, e)
    return -100*emin/0.2
print("PP overshoot, ideal inner loop (20 Hz ZOH only):", round(pp_os("ideal", 0.0, 0.0, 1e-4), 2), "%")
for s in ["A", "B", "C"]:
    for Ta, Tm, Tp in [(0.02, 0.02, 0.05), (0.03, 0.03, 0.10)]:
        print(f"PP overshoot with structure {s}, Ta={Ta} Tm={Tm} Tp={Tp}: {pp_os(s, Ta, Tm, Tp):.2f} %")

"""Audit-2 check (follow-up of c13/c14): proposed fix for the base design (Sec 2.6.1):
  D  u = u_ff + Kp(y_hat - y_m) + Ki int(y_hat - y_m),  u_ff = r + Tp_hat*a_r,  y_hat = Phat(s) u_ff
     (Phat = identified exp(-Td_hat s)/(Tp_hat s + 1), run inside velocity_profiler_node at 50 Hz;
      a_r = S-curve filter acceleration, 0 for the identification step).
  vs A (v2.1 text: u = r + Kp(0.5 r - y_m) + Ki int(r - y_m)).
Plant: actuation delay Ta, servo Tp, measurement delay Tm (Td = Ta + Tm), controller 50 Hz ZOH.
Model: matched (identified) and mismatched (Tp_hat x0.7/x1.3, Td_hat +-0.01 s).
Metrics: 0.15 m/s step overshoot; S-curve 0->1 m/s (a 1, j 2) overshoot; max |y_hat_true - y| where
y_hat_true is the model-following error seen at the plant output; max |y - r(t - Td)| (lag beyond pure delay);
10 % servo gain drop (effort saturation / slip) steady error; PP straight-line overshoot on the yaw axis."""
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

class Loop:
    def __init__(s, struct, Ta, Tm, Tp, Tp_hat, Td_hat, Kp=0.4, Ki=4.0, gain=1.0):
        s.__dict__.update(locals()); s.na = int(round(Ta/dt)); s.nm = int(round(Tm/dt)); s.nh = int(round(Td_hat/dt))
        s.y = 0.0; s.I = 0.0; s.u = 0.0; s.ua = [0.0]*(s.na + 1); s.yb = [0.0]*(s.nm + 1)
        s.ub = [0.0]*(s.nh + 1); s.yhat = 0.0; s.k = 0; s.uff = 0.0
    def tick(s, r, ar):
        if s.k % nc == 0:
            ym = s.yb[0]
            if s.struct == "A":
                s.I += s.Ki*(r - ym)/fc; s.u = r + s.Kp*(0.5*r - ym) + s.I
            else:
                s.uff = r + s.Tp_hat*ar
                s.I += s.Ki*(s.yhat - ym)/fc; s.u = s.uff + s.Kp*(s.yhat - ym) + s.I
        if s.struct == "D":                               # model driven by u_ff (held, like the plant input)
            s.ub.append(s.uff); s.ub.pop(0); s.yhat += dt*(s.ub[0] - s.yhat)/s.Tp_hat
        s.ua.append(s.u); s.ua.pop(0); s.y += dt*(s.gain*s.ua[0] - s.y)/s.Tp
        s.yb.append(s.y); s.yb.pop(0); s.k += 1
        return s.y

def run(struct, ref, Ta, Tm, Tp, Tp_hat, Td_hat, gain=1.0, T=4.0):
    lp = Loop(struct, Ta, Tm, Tp, Tp_hat, Td_hat, gain=gain); n = int(T/dt); out = np.zeros((n, 2))
    for k in range(n):
        r, ar = ref(k*dt); out[k] = (r, lp.tick(r, ar))
    return out

def stepref(t): return (0.15 if t > 0.1 else 0.0), 0.0
f = scurve(1.0)
plants = [(0.02, 0.02, 0.05), (0.03, 0.03, 0.10), (0.02, 0.02, 0.10), (0.03, 0.03, 0.05)]
worst = {}
for Ta, Tm, Tp in plants:
    Td = Ta + Tm
    for label, Tph, Tdh in [("matched", Tp, Td), ("Tp_hat x0.7", 0.7*Tp, Td), ("Tp_hat x1.3", 1.3*Tp, Td),
                            ("Td_hat -0.01", Tp, Td - 0.01), ("Td_hat +0.01", Tp, Td + 0.01)]:
        for s in (["A"] if label == "matched" else []) + ["D"]:
            o = run(s, stepref, Ta, Tm, Tp, Tph, Tdh); os1 = 100*(o[:, 1].max() - 0.15)/0.15
            o2 = run(s, lambda t: f(t), Ta, Tm, Tp, Tph, Tdh); os2 = 100*(o2[:, 1].max() - 1.0)
            k_d = int(round(Td/dt)); rdel = np.r_[np.zeros(k_d), o2[:-k_d, 0]]
            lag = np.abs(o2[:, 1] - rdel).max()
            o3 = run(s, lambda t: f(t), Ta, Tm, Tp, Tph, Tdh, gain=0.9, T=6.0); ess = 1.0 - o3[-1, 1]
            key = (s, label == "matched")
            w = worst.setdefault(key, [0, 0, 0]); w[0] = max(w[0], os1); w[1] = max(w[1], os2); w[2] = max(w[2], lag)
            print(f"{s} plant Ta={Ta} Tm={Tm} Tp={Tp} model {label:12s}: step0.15 OS {os1:5.2f} % | S-curve OS {os2:5.2f} % | "
                  f"max|y - r(t-Td)| {lag:.3f} m/s | gain -10 %: steady err {ess:+.4f}")
for (s, m), w in worst.items():
    print(f"WORST {s} {'matched' if m else 'mismatched'}: step OS {w[0]:.2f} %, S-curve OS {w[1]:.2f} %, lag beyond delay {w[2]:.3f} m/s")

def pp_os(struct, Ta, Tm, Tp, Tp_hat, Td_hat, v=1.0, L=0.8, T=12.0):
    lp = Loop(struct, Ta, Tm, Tp, Tp_hat, Td_hat) if struct != "ideal" else None
    e, psi, w, emin, wc, wprev = 0.2, 0.0, 0.0, 0.0, 0.0, 0.0
    for k in range(int(T/dt)):
        if k % 500 == 0: wprev, wc = wc, -(2*v/L**2)*(e + L*psi)
        w = wc if lp is None else lp.tick(wc, 0.0)        # yaw axis: controller output is a 20 Hz staircase, a_r = 0
        e += dt*v*np.sin(psi); psi += dt*w; emin = min(emin, e)
    return -100*emin/0.2
print("PP overshoot ideal inner loop (20 Hz ZOH):", round(pp_os("ideal", 0, 0, 1, 1, 0), 2), "%")
for Ta, Tm, Tp in [(0.02, 0.02, 0.05), (0.03, 0.03, 0.10)]:
    for s in ["A", "D"]:
        print(f"PP overshoot {s}, plant Ta={Ta} Tm={Tm} Tp={Tp} (matched model): {pp_os(s, Ta, Tm, Tp, Tp, Ta + Tm):.2f} %")

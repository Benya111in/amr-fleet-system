"""Audit checks: general S-curve decel distance d(v,vt) (M4), piecewise D_stop/v_stop (M7),
start-up rise time (M6), E-stop distances with project config (M10)."""
import numpy as np
a, j = 1.0, 2.0

def scurve_sim(v0, vt, dt=1e-5):
    """Time-optimal symmetric jerk-limited decel from (v0, a=0) to (vt, a=0); returns distance."""
    dv = v0 - vt
    if dv >= a*a/j:
        tj = a/j; tc = dv/a - a/j
    else:
        tj = np.sqrt(dv/j); tc = 0.0
    T = 2*tj + tc; n = int(round(T/dt)); v = v0; acc = 0.0; s = 0.0
    for i in range(n):
        t = i*dt
        jj = -j if t < tj else (0.0 if t < tj+tc else j)
        v_new = v + acc*dt + 0.5*jj*dt*dt
        s += v*dt + 0.5*acc*dt*dt + jj*dt**3/6
        acc += jj*dt; v = v_new
    return s, v

def d_closed(v, vt):
    dv = v - vt
    return (v*v - vt*vt)/(2*a) + (v+vt)*a/(2*j) if dv >= a*a/j else (v+vt)*np.sqrt(dv/j)
def d_reviewer(v, vt):
    return (v*v - vt*vt)/(2*a) + (v-vt)*a/(2*j)
def dinv(vt, D):
    v = -a*a/(2*j) + np.sqrt(a**4/(4*j*j) + vt*vt - vt*a*a/j + 2*a*D)
    if v - vt >= a*a/j: return v
    lo, hi = vt, vt + a*a/j
    for _ in range(200):
        mid = 0.5*(lo+hi)
        if d_closed(mid, vt) > D: hi = mid
        else: lo = mid
    return 0.5*(lo+hi)

for v0, vt in [(2.0, 1.10), (2.0, 1.8), (2.0, 0.0), (0.5, 0.0), (0.2, 0.0), (1.1, 0.0)]:
    s, vend = scurve_sim(v0, vt)
    print(f"M4  {v0}->{vt}: numeric {s:.4f} m, closed {d_closed(v0, vt):.4f}, reviewer {d_reviewer(v0, vt):.4f}, "
          f"trapezoid {(v0**2-vt**2)/(2*a):.4f}")

# inverse round trip over grid
err = 0.0
for v in np.linspace(0.05, 2.0, 60):
    for vt in np.linspace(0.0, v - 1e-3, 30):
        err = max(err, abs(dinv(vt, d_closed(v, vt)) - v))
print("M4  max |dinv(vt, d(v,vt)) - v| over grid =", err)

# M7 piecewise D_stop / v_stop
def Dstop(v): return v*v/(2*a) + v*a/(2*j) if v >= a*a/j else v**1.5/np.sqrt(j)
def vstop(D): return -a*a/(2*j) + np.sqrt(a**4/(4*j*j) + 2*a*D) if D >= a**3/j**2 else (D*np.sqrt(j))**(2/3)
print("M7  Dstop(0.2) piecewise", round(Dstop(0.2), 4), " old closed form", 0.2**2/2 + 0.2/4,
      " over-estimate %", round(100*((0.2**2/2 + 0.2/4)/Dstop(0.2) - 1), 1))
print("M7  threshold a^3/j^2 =", a**3/j**2, " Dstop(0.5) =", Dstop(0.5))
print("M7  max round-trip err", max(abs(vstop(Dstop(v)) - v) for v in np.linspace(0.01, 2, 400)))
print("M7  Dstop(2.0) =", Dstop(2.0))

# M6 start-up rise time to 0.02 m/s
th = 0.02
print("M6  pure jerk rise to 0.02 m/s:", round(1000*np.sqrt(2*th/j), 1), "ms")
a0 = 0.3; t = (-a0 + np.sqrt(a0*a0 + 2*j*th))/j
print("M6  with a0=0.3 step:", round(1000*t, 1), "ms;  first 50 Hz profiler sample v =", j*0.02**2/2, "m/s (>0)")

# M10 E-stop distance: project config model vs torque-limited
m_total = 45.0 + 2*1.0 + 2*0.3          # robot_params.yaml
r = 0.0825; tau_max = 6.0               # tau_max: brief's assumption (not in config)
for label, acc, lat in [("config a=1.0, tau=0.15", 1.0, 0.15),
                        ("brief  a_tau(70kg)=%.2f, tau=0.05" % (2*tau_max/(r*70)), 2*tau_max/(r*70), 0.05),
                        ("torque a_tau(72.6kg)=%.2f, tau=0.15" % (2*tau_max/(r*(m_total+25))), 2*tau_max/(r*(m_total+25)), 0.15),
                        ("torque a_tau(47.6kg)=%.2f, tau=0.15" % (2*tau_max/(r*m_total)), 2*tau_max/(r*m_total), 0.15)]:
    DE = [v*lat + v*v/(2*acc) for v in (2.0, 1.0, 0.5, 0.2)]
    vmax03 = -lat*acc + np.sqrt((lat*acc)**2 + 2*acc*0.3)
    print(f"M10 {label}: D_E(2/1/0.5/0.2) = {[round(x, 3) for x in DE]}, max v stopping within 0.3 m = {vmax03:.3f}")

# DiffDrive limiter set to 1.2 * a_max (Sec 2.6.1) -> actual E-stop decel 1.2 m/s^2 with config latency
acc, lat = 1.2, 0.15
DE = [v*lat + v*v/(2*acc) for v in (2.0, 1.0, 0.5, 0.2)]
print("M10 DiffDrive limiter 1.2 a_max, tau=0.15: D_E =", [round(x, 3) for x in DE],
      " max v within 0.3 m =", round(-lat*acc + np.sqrt((lat*acc)**2 + 2*acc*0.3), 3))

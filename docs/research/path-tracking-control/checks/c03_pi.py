"""Audit checks: PI zero overshoot (M3), 2-DOF setpoint weighting, gains with project-config masses,
payload scheduling percentages (M11), inner-loop phase lag and its effect on the PP loop,
encoder quantisation / slip-noise ripple (spec gap 4), contract-compliant body-level PI margins."""
import numpy as np
from scipy import signal

def step_overshoot(num, den, T=6.0):
    t = np.linspace(0, T, 60001); _, y = signal.step((num, den), T=t)
    return 100*(y.max() - 1), y
w = 6.0
for z in [1.0, 0.707]:
    for b in [1.0, 0.5, 0.3]:
        os_, _ = step_overshoot([2*z*w*b, w*w], [1, 2*z*w, w*w])
        print(f"M3  zeta={z} b={b}: overshoot {os_:.2f} %")

# gains with project config masses
r = 0.0825; g = 1/r
m_base, m_wheels, m_casters = 45.0, 2*1.0, 2*0.3
Jw = 0.0034                                   # wheel_inertia iyy (axle) from robot_params.yaml
m0 = m_base + m_wheels + m_casters
for m in [45.0, m0, m0+25]:
    Mv = m + 2*Jw/r**2
    Kp = 2*1.0*w*Mv/g; Ki = w*w*Mv/g
    print(f"M3  m={m:.1f} kg: M_v={Mv:.2f} kg, Kp={Kp:.1f} N s, Ki={Ki:.1f} N")
Mv0, Mv1 = m0 + 2*Jw/r**2, m0 + 25 + 2*Jw/r**2
print(f"M11 fixed gains {Mv0:.1f}->{Mv1:.1f} kg: omega_c {100*(np.sqrt(Mv0/Mv1)-1):.1f} %, zeta {100*(np.sqrt(Mv0/Mv1)-1):.1f} %, "
      f"zeta*omega_c {100*(Mv0/Mv1-1):.1f} %   (brief 45->70: {100*(np.sqrt(45/70)-1):.1f} / {100*(45/70-1):.1f} %)")
# identification step: torque and acceleration demand with b=0.5, zeta=1 (first-order response)
for dv in [0.2, 0.15]:
    acc0 = w*dv; tau_tot = Mv0*acc0*r
    print(f"M3  ID step {dv} m/s (b=0.5): initial accel {acc0:.2f} m/s^2, total torque {tau_tot:.2f} N m ({tau_tot/2:.2f}/wheel)")
# 0.5 m/s 1-DOF step torque demand
print(f"M3  0.5 m/s 1-DOF step: Kp*0.5 = {2*w*Mv0*r*0.5:.1f} N m total")

# inner-loop phase lag at outer-loop bandwidth
wn = np.sqrt(2)/0.8
for b in [1.0, 0.5]:
    _, H = signal.freqs([2*w*b, w*w], [1, 2*w, w*w], worN=[wn])
    ph = np.degrees(np.angle(H[0]))
    print(f"M11 b={b}: phase at {wn:.3f} rad/s = {ph:.2f} deg, equiv delay {1000*np.radians(-ph)/wn:.0f} ms")

# PP straight-line loop with inner lag: exact 1st-order lag (b=0.5 -> 6/(s+6)) and b=1 inner loop
v_over_L = 1/0.8
def pp_poles(extra):
    return np.roots(extra)
# b=0.5: (tau s + 1) s^2 + (2v/L) s + 2v^2/L^2
tau = 1/w
p = np.roots([tau, 1, 2*v_over_L, 2*v_over_L**2])
dom = p[np.argmin(np.abs(p.real))]
print("M11 PP+lag(b=0.5) poles", np.round(p, 3), " dominant zeta", round(-dom.real/abs(dom), 3))
# overshoot of e(t) for initial condition e0 (psi0=0): simulate 3rd-order systems
def pp_ic_overshoot(inner_num, inner_den):
    # state: e, psi, inner state; w_cmd = -(2v/L^2)(e + L psi); psi_dot = w_act; e_dot = v psi
    v, L = 1.0, 0.8
    A_in, B_in, C_in, D_in = signal.tf2ss(inner_num, inner_den)
    n = A_in.shape[0]
    dt = 1e-4; x = np.zeros(n); e = 1.0; psi = 0.0; emin = 0
    for k in range(int(20/dt)):
        wc = -(2*v/L**2)*(e + L*psi)
        wa = (C_in @ x + D_in[:, 0]*wc).item()
        x = x + dt*(A_in @ x + B_in[:, 0]*wc)
        e += dt*v*psi; psi += dt*wa; emin = min(emin, e)
    return -100*emin
print("M11 PP overshoot, ideal inner loop:", round(pp_ic_overshoot([1.0], [1.0]), 2), "%")
print("M11 PP overshoot, b=1 inner loop  :", round(pp_ic_overshoot([2*w, w*w], [1, 2*w, w*w]), 2), "%")
print("M11 PP overshoot, b=0.5 inner loop:", round(pp_ic_overshoot([w*w*0.5*2/2*1.0*0+w], [1, w]), 2), "%")

# spec gap 4: encoder quantisation and slip noise (sensors.yaml: 4096 ticks, slip 1 % per 20 ms step)
q = 2*np.pi/4096*r
for T in [0.01, 0.02, 0.03]:
    print(f"SG4 tick-difference resolution over {int(T*1000)} ms: {q/T:.4f} m/s (uniform rms {q/T/np.sqrt(12):.4f})")
for v in [0.5, 2.0]:
    sw = 0.01*v                   # per-wheel velocity noise from multiplicative slip on displacement per step
    print(f"SG4 slip noise at v={v}: per-wheel sigma {sw:.4f} m/s, body v sigma {sw/np.sqrt(2):.4f} m/s, "
          f"torque-level Kp*sigma = {2*w*Mv0*r*sw/np.sqrt(2):.2f} N m")

# contract-compliant body-level 2-DOF PI in velocity_profiler_node (50 Hz) on DiffDrive velocity servo
# plant P(s) = exp(-Td s)/(Tp s + 1); controller C(s) = Kp + Ki/s (feedback part), feedforward u_ff = r
for Td, Tp in [(0.04, 0.05), (0.06, 0.10)]:
    for Kp, Ki in [(0.3, 3.0), (0.4, 4.0), (0.5, 5.0)]:
        ws = np.logspace(-2, 3, 200000)
        C = Kp + Ki/(1j*ws); P = np.exp(-1j*ws*Td)/(1j*ws*Tp + 1); Lw = C*P
        i = np.argmin(np.abs(np.abs(Lw) - 1)); pm = 180 + np.degrees(np.angle(Lw[i]))
        ph = np.unwrap(np.angle(Lw)); k180 = np.argmin(np.abs(ph + np.pi)); gm = -20*np.log10(np.abs(Lw[k180]))
        print(f"PIc Td={Td} Tp={Tp} Kp={Kp} Ki={Ki}: crossover {ws[i]:.2f} rad/s, PM {pm:.1f} deg, GM {gm:.1f} dB")

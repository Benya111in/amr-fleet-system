import numpy as np
from math import *

# ---------- 1. CTE sign convention ----------
t = np.array([1.0,0.0,0.0]); d = np.array([0.0,1.0,0.0])  # robot to the LEFT of path
print("cross(d,t).z =", np.cross(d,t)[2], " cross(t,d).z =", np.cross(t,d)[2], " d.n(left normal) =", d@np.array([-t[1],t[0],0]))

# ---------- 2. Circle linearization of y_g ----------
def yg_on_circle(R, L, e, psi):
    # path: circle radius R centered at (0,R), robot nominal at origin heading +x (CCW circle, kappa=1/R>0)
    # left normal at origin is (0,1); robot at p = (0,e), heading psi
    p = np.array([0.0, e]); th = psi
    # lookahead point: intersection of circle |q-c|=R with |q-p|=L, ahead of robot
    c = np.array([0.0, R])
    # solve: parametrize q = c + R(sin a, -cos a), a from 0 (origin) increasing CCW
    f = lambda a: np.linalg.norm(c + R*np.array([sin(a), -cos(a)]) - p) - L
    # bisection a in (0, pi)
    lo, hi = 1e-6, pi-1e-6
    for _ in range(200):
        mid = 0.5*(lo+hi)
        if f(mid) < 0: lo = mid
        else: hi = mid
    q = c + R*np.array([sin(lo), -cos(lo)])
    dq = q - p
    # robot frame
    xg = cos(th)*dq[0] + sin(th)*dq[1]
    yg = -sin(th)*dq[0] + cos(th)*dq[1]
    return xg, yg

for R, L in [(1.5,1.6),(1.5,0.9),(1.5,0.4),(1e6,1.6)]:
    h=1e-5
    yg0 = yg_on_circle(R,L,0,0)[1]
    dye = (yg_on_circle(R,L,h,0)[1]-yg_on_circle(R,L,-h,0)[1])/(2*h)
    dyp = (yg_on_circle(R,L,0,h)[1]-yg_on_circle(R,L,0,-h)[1])/(2*h)
    k=1/R
    print(f"R={R} L={L}: yg0={yg0:.4f} (L^2k/2={L*L*k/2:.4f}) dyg/de={dye:.4f} (pred -cos(phi0)={-(1-L*L*k*k/2):.4f}) dyg/dpsi={dyp:.4f} (pred -L*sqrt(1-L^2k^2/4)={-L*sqrt(1-L*L*k*k/4):.4f})")

# damping on circle for K=1
for R,L in [(1.5,1.6),(1.5,0.9)]:
    k=1/R; print(f"R={R} L={L}: zeta_circle(K=1)= {sqrt(1-L*L*k*k/4)/sqrt(2):.3f}")

# ---------- 3. PI step response with/without setpoint weighting ----------
from scipy import signal
def pi_step(zeta, wc, b, T=5.0):
    # closed loop from r to y: plant g/(M s) with C = Kp + Ki/s, b_v = 0. Normalize: char poly s^2 + 2 zeta wc s + wc^2
    # y = [ (b*2*zeta*wc) s + wc^2 ] / (s^2 + 2 zeta wc s + wc^2) r   (setpoint weight b on proportional path)
    num = [b*2*zeta*wc, wc**2]; den = [1, 2*zeta*wc, wc**2]
    t = np.linspace(0,T,20001); _, y = signal.step((num,den), T=t)
    return y.max()-1
for zeta in [1.0, 0.707]:
    for b in [1.0, 0.5, 0.3, 0.0]:
        print(f"zeta={zeta} b={b}: overshoot = {100*pi_step(zeta,6.0,b):.1f} %")

# ---------- 4. D_stop piecewise ----------
a=1.0; j=2.0
def Dstop(v):
    if v >= a*a/j: return v*v/(2*a) + v*a/(2*j)
    return v**1.5/sqrt(j)
def vstop(D):
    Dc = a**3/(2*j*j)*2  # threshold: D at v=a^2/j -> v=0.5: 0.125+0.125=0.25
    Dc = Dstop(a*a/j)
    if D >= Dc: return -a*a/(2*j) + sqrt(a**4/(4*j*j) + 2*a*D)
    return (D*sqrt(j))**(2/3)
for v in [0.1,0.2,0.5,1.0,2.0]:
    D=Dstop(v); print(f"v={v}: Dstop={D:.4f}  closedform={v*v/(2*a)+v*a/(2*j):.4f}  vstop(D)={vstop(D):.4f}")
print("threshold D at v=a^2/j:", Dstop(a*a/j))

# ---------- 5. general S-curve decel distance v -> v_t ----------
def d_gen(v, vt):
    dv = v - vt
    if dv <= 0: return 0.0
    if dv >= a*a/j:
        T = dv/a + a/j
        # distance = v*T - area under decel profile... compute numerically
    # numeric integration of 7-seg (3-seg) profile
    dt=1e-4; vv=v; acc=0.0; s=0.0; tt=0
    # decel S-curve: jerk -j until acc=-a (or halfway), hold, jerk +j
    if dv >= a*a/j:
        tj=a/j; tc=dv/a - a/j
        phases=[(-j,tj),(0,tc),(j,tj)]
    else:
        tj=sqrt(dv/j); phases=[(-j,tj),(j,tj)]
    for jj,dur in phases:
        n=int(round(dur/dt))
        for _ in range(n):
            s += vv*dt + 0.5*acc*dt*dt
            vv += acc*dt + 0.5*jj*dt*dt
            acc += jj*dt
    return s, vv
for v,vt in [(2.0,1.10),(2.0,0.0),(1.1,0.0),(0.5,0.0),(0.2,0.0),(2.0,1.8)]:
    s,vend = d_gen(v,vt); dv=v-vt
    cf = (v*v-vt*vt)/(2*a) + dv*a/(2*j) if dv>=a*a/j else (v+vt)*sqrt(dv/j)
    print(f"v={v}->{vt}: numeric d={s:.4f} vend={vend:.4f}  closedform={cf:.4f}")

# ---------- 6. jerk rise time ----------
for vth in [0.02,0.005]:
    print(f"rise to {vth} m/s with j=2: {sqrt(2*vth/j)*1000:.0f} ms")
# ---------- 7. stanley damping ----------
k=1.0; l=0.5
for v in [0.2,0.5,1.0,2.0]:
    wn=sqrt(v*k/l); z=(v/l+k)/(2*wn); print(f"stanley v={v}: wn={wn:.2f} zeta={z:.3f}")
# ---------- 8. bandwidth vs mass ----------
print("wc ratio 45->70:", sqrt(45/70), " real part ratio:", 45/70)
# ---------- 9. E-stop distances ----------
for v in [2.0,0.5,0.2]:
    print(f"v={v}: a=1 -> {v*v/2:.3f} m ; a=2.1 -> {v*v/(2*2.1):.3f} m; a=1 + 50ms latency -> {v*v/2+0.05*v:.3f}")
# torque-limited accel: 2 wheels x 6 Nm / r=0.0825 / 70kg
print("a_torque(70kg)=", 2*6/0.0825/70, " a_torque(45kg)=", 2*6/0.0825/45)
# ---------- 10. sagitta ----------
for L,R in [(1.6,1.5),(0.88,1.5),(0.9,1.5)]:
    print(f"L={L} R={R}: sagitta L^2/(8R) = {L*L/(8*R):.3f}")
# ---------- 11. inner-loop phase lag at outer wn ----------
tL=0.8; wn_outer=sqrt(2)/tL
wc=6.0; zeta=1.0
s=1j*wn_outer
G=(2*zeta*wc*s+wc**2)/(s*s+2*zeta*wc*s+wc**2)
print(f"outer wn={wn_outer:.2f} rad/s = {wn_outer/2/pi:.2f} Hz; inner-loop phase at wn: {degrees(np.angle(G)):.1f} deg (b=1); ")
G0=(wc**2)/(s*s+2*zeta*wc*s+wc**2); print(f" with b=0 (prefilter-like): {degrees(np.angle(G0)):.1f} deg")
# ---------- 12. K(v) schedule for constant wn when L_min active ----------
tL=0.8; sig_e=0.04; sig_w=0.15; K0=1.0
for v in [0.2,0.5,1.0,2.0]:
    Lmin=sqrt(2*K0*v*sig_e/sig_w); L=max(v*tL,Lmin); Kv=K0*(L/(v*tL))**2
    print(f"v={v}: L_min={Lmin:.3f} L={L:.3f} K(v)={Kv:.2f} wn={sqrt(2*Kv)*v/L:.3f}")

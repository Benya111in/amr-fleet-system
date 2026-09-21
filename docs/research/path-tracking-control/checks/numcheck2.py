from math import *
import numpy as np
a=1.0; j=2.0
def d_gen(v,vt):
    dv=v-vt
    if dv<=0: return 0.0
    if dv>=a*a/j: return (v*v-vt*vt)/(2*a)+(v+vt)*a/(2*j)
    return (v+vt)*sqrt(dv/j)
def d_inv(D,vt):
    # long branch: v^2 + (a^2/j) v - (vt^2 - vt a^2/j + 2aD) = 0
    v=-a*a/(2*j)+sqrt(a**4/(4*j*j)+vt*vt-vt*a*a/j+2*a*D)
    if v-vt>=a*a/j: return v
    # short branch: (v+vt) sqrt((v-vt)/j) = D  -> solve numerically (monotone in v)
    lo,hi=vt,vt+a*a/j
    for _ in range(100):
        m=0.5*(lo+hi)
        if d_gen(m,vt)<D: lo=m
        else: hi=m
    return 0.5*(lo+hi)
for v,vt in [(2.0,1.1),(2.0,1.8),(1.1,0.0),(0.4,0.2),(0.6,0.5)]:
    D=d_gen(v,vt); print(f"v={v}->{vt}: d={D:.4f} inv={d_inv(D,vt):.4f}")
# Backward pass example on a 2.0 cap then R=1.5 curve: how far before the curve must decel start?
print("decel dist 2.0->1.10:", d_gen(2.0,1.10), " (v_stop-based wrong bound would be 0)")
# inner-loop phase lag with b=0.5, zeta=1, wc=6 at outer wn=1.77
for b in [1.0,0.5,0.0]:
    s=1j*sqrt(2)/0.8; wc=6.0; z=1.0
    G=(b*2*z*wc*s+wc**2)/(s*s+2*z*wc*s+wc**2)
    print(f"b={b}: |G|={abs(G):.3f} phase={degrees(np.angle(G)):.1f} deg -> equiv delay {(-np.angle(G))/abs(s)*1000:.0f} ms")
# effect of pure delay tau on PP damping (Pade-1): char eq s^2 + (2v/L) s e^{-s tau} + (2v^2/L^2) e^{-s tau} = 0
def roots_with_delay(v,L,tau):
    # Pade(1,1): e^{-s tau} ~ (1 - s tau/2)/(1 + s tau/2)
    # (s^2)(1+s tau/2) + (2v/L s + 2v^2/L^2)(1 - s tau/2) = 0
    p=np.polynomial.polynomial
    A=np.array([0,0,1.0]); # s^2
    P1=np.array([1.0,tau/2]); P2=np.array([1.0,-tau/2])
    B=np.array([2*v*v/L/L, 2*v/L])
    poly=p.polyadd(p.polymul(A,P1), p.polymul(B,P2))
    r=np.roots(poly[::-1]); return r
for tau in [0.0,0.05,0.16]:
    r=roots_with_delay(2.0,1.6,tau); dom=r[np.argsort(-r.real)][:2]
    zeta=-dom[0].real/abs(dom[0]); print(f"tau={tau}: dominant roots {dom[0]:.3f}  zeta_eff={zeta:.3f}")
# step torque for 0.2 m/s step with b=0.5
Kp=45.5; print("initial torque total (b=0.5, step 0.2):", 0.5*Kp*0.2, "N.m ; per wheel", 0.5*Kp*0.2/2)
# S-curve 0->2 m/s time with j exemption at start? (no)
# jerk metric: IMU noise check - sigma_a differentiated: with 5 Hz LP then diff, noise-floor estimate
sig_a=0.05 # m/s^2 guess placeholder
print("done")

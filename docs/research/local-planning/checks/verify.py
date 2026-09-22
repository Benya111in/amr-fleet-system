import numpy as np
from math import erf, sqrt, cos, sin, log, pi
rng=np.random.default_rng(0)
Phi=lambda z:0.5*(1+erf(z/sqrt(2)))
def bound(mu,R,sig): return Phi((R-mu)/sig)
def mc(mu,R,sig,n=400000):
    x=rng.normal(mu,sig,n); y=rng.normal(0,sig,n); return np.mean(x*x+y*y<=R*R)
def exact_iso(mu,R,sig):
    # noncentral chi2 (2 dof) via numerical integration in polar coords
    r=np.linspace(0,R,2001); 
    from scipy.special import i0e
    # pdf of radius: r/s^2 exp(-(r^2+mu^2)/(2s^2)) I0(r mu/s^2)
    s2=sig*sig; f=r/s2*np.exp(-(r*r+mu*mu)/(2*s2))*np.exp(r*mu/s2)*i0e(r*mu/s2)
    return np.trapezoid(f,r)
print("== MC vs bound vs exact ==")
for mu,R,sig in [(1.0,0.6,0.66),(1.0,0.66,0.15),(1.0,0.6,0.33),(1.5,1.0,0.33),(0.9,0.6,0.15),(1.2,0.6,0.5),(2.0,0.6,0.83)]:
    print(f"mu={mu} R={R} sig={sig}: MC={mc(mu,R,sig):.4f} exact={exact_iso(mu,R,sig):.4f} bound={bound(mu,R,sig):.4f}")
print("== sigma(t) per class (sp=0.05, sv=0.2) ==")
for name,q in [("person",0.2),("forklift",0.1),("amr",0.05)]:
    print(name,[f"t={t}: {sqrt(0.05**2+t*t*0.2**2+q*t**3/3):.3f}" for t in (0.5,1.0,1.5,2.0,3.0)])
print("== clearance R_j+1.28 sigma(t) (z_0.10=1.2816) ==")
z=1.2816
for name,q,rj in [("person",0.2,0.30),("forklift(2-circle r=0.70)",0.1,0.70),("forklift(1-circle r=1.2)",0.1,1.2),("amr",0.05,0.36)]:
    Rj=0.30+rj
    print(name,"R_j=",Rj,[f"t={t}: {Rj+z*sqrt(0.05**2+t*t*0.2**2+q*t**3/3):.2f}" for t in (0.5,1.0,2.0)])
print("== arc vs chord max deviation (v=1, tau=2) ==")
def dev(v,w,tau):
    ts=np.linspace(0,tau,2001)
    xa=(v/w)*np.sin(w*ts); ya=(v/w)*(1-np.cos(w*ts))
    veff=v*np.sin(w*tau/2)/(w*tau/2); ang=w*tau/2
    xc=veff*np.cos(ang)*ts; yc=veff*np.sin(ang)*ts
    d=np.hypot(xa-xc,ya-yc); sag=(v/w)*(1-cos(w*tau/2)); return d.max(),sag
for w in (0.3,0.5,1.0,1.5):
    for tau in (1.0,2.0):
        m,s=dev(1.0,w,tau); print(f"w={w} tau={tau}: max|arc-chord|={m:.3f} sagitta={s:.3f}")
print("== J_risk product saturation ==")
print("1-0.98^250=",1-0.98**250,"1-0.995^500=",1-0.995**500)
print("== return time: 45deg bang-bang alpha=2, w_peak ==")
alpha=2.0; th=pi/4; t_turn=2*sqrt(th/alpha); print("t per 45deg heading change",t_turn,"peak w",sqrt(th*alpha))
print("lateral 0.8 m at v=1 45deg:",0.8/(1*sin(pi/4)))
print("== Wilcoxon min p n=5,8,30 ==",[2/2**n for n in (5,8,30)])
print("== T_sim clip ==",[min(max(v/1.0+0.5,1.5),2.5) for v in (0.5,1.0,1.5,2.0)])
print("== jerk-limited window: dv per cycle from a_prev=0:",0.1*0.05, " cycles to reach a=1:",1.0/0.1)
print("== S-curve stop distance from 2 m/s, j=2,a=1: t_ramp=0.5s ==")
# ramp: a goes 0->-1 over 0.5s; v drop in ramp = 0.25; then const decel
v0=2.0; j=2.0; a=1.0; tr=a/j; dv_r=0.5*j*tr*tr; d_r=v0*tr-j*tr**3/6; v1=v0-dv_r; d2=v1*v1/(2*a); print("ramp dv",dv_r,"d_ramp",d_r,"d_const",d2,"total",d_r+d2,"vs no-jerk",v0*v0/2)

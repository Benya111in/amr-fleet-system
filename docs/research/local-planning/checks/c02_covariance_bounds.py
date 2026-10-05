"""c02: CV covariance growth per class, collision-probability bounds (half-plane HP, principal-axis
box bound BX, min of both) vs exact/MC, clearance radii for delta_hard, J_risk saturation."""
import numpy as np
from math import sqrt, erf
from scipy.special import ndtr, i0e
from scipy.optimize import brentq
rng = np.random.default_rng(1)
sp, sv = 0.05, 0.2
cls = {"person": (0.30, 0.2), "forklift(circle of 2-circle cover)": (0.70, 0.1), "amr": (0.36, 0.05)}
rR = 0.30   # robot cover circle 0.25 + 0.05 margin
sig = lambda t,q: sqrt(sp**2 + t*t*sv**2 + q*t**3/3)
print("== sigma(t) [m], sp=0.05, sv=0.2")
for n,(r,q) in cls.items(): print("  %-36s"%n, {t: round(sig(t,q),3) for t in (0.2,0.5,1.0,1.5,2.0,2.5,3.0)})
print("  (original example used q=0.1 -> sigma(2)=%.3f; person q=0.2 -> %.3f)"%(sig(2,0.1), sig(2,0.2)))
def hp(mu, S, R):
    m = np.linalg.norm(mu); a = mu/m; s = sqrt(a@S@a); return ndtr((R-m)/s)
def bx(mu, S, R):
    lam, E = np.linalg.eigh(S); p = 1.0
    for i in range(2):
        mi = E[:,i]@mu; si = sqrt(lam[i]); p *= ndtr((R-mi)/si) - ndtr((-R-mi)/si)
    return p
def exact_iso(m, s, R, n=4001):
    r = np.linspace(0, R, n); s2 = s*s
    f = r/s2*np.exp(-(r-m)**2/(2*s2))*i0e(r*m/s2)
    return np.trapezoid(f, r)
def mc(mu, S, R, n=400000):
    x = rng.multivariate_normal(mu, S, n); return np.mean((x**2).sum(1) <= R*R)
print("== isotropic: exact vs HP vs BX vs min")
for m,R,s in [(1.0,0.6,0.66),(1.0,0.6,0.83),(1.0,0.66,0.15),(1.2,0.6,0.33),(1.5,1.0,0.28),(2.0,0.6,0.83),(0.9,0.6,0.15)]:
    S=np.eye(2)*s*s; mu=np.array([m,0.]); e=exact_iso(m,s,R); h=hp(mu,S,R); b=bx(mu,S,R)
    print("  |mu|=%.2f R=%.2f s=%.2f exact=%.4f HP=%.4f BX=%.4f min=%.4f  gapHP=%.3f gapMin=%.3f"%(m,R,s,e,h,b,min(h,b),h-e,min(h,b)-e))
print("== random anisotropic check (MC 4e5): bound >= MC ?")
viol=0; gaps=[]; ratios=[]
for k in range(60):
    R = rng.uniform(0.5,1.1); m = rng.uniform(0.3,3.0); ang = rng.uniform(0,np.pi)
    s1, s2 = rng.uniform(0.05,0.9), rng.uniform(0.05,0.9); c,s=np.cos(ang),np.sin(ang); Rot=np.array([[c,-s],[s,c]])
    S = Rot@np.diag([s1*s1,s2*s2])@Rot.T; mu=np.array([m,0.])
    p = mc(mu,S,R); h=hp(mu,S,R); b=bx(mu,S,R); u=min(h,b)
    se = sqrt(max(p*(1-p),1e-12)/4e5)
    if u < p - 4*se: viol+=1
    if p>1e-3: gaps.append(u-p); ratios.append(u/p)
print("  violations:",viol,"/60 ; gap(min-MC) median %.3f max %.3f ; ratio median %.2f max %.2f"%(np.median(gaps),np.max(gaps),np.median(ratios),np.max(ratios)))
print("  HP-only gap on same set would be larger; e.g. (1.0,0.6,0.66): 0.272 vs exact 0.132")
# clearance radius d*(t): centre distance where min(HP,BX)=delta (isotropic)
def dstar(R, s, delta):
    f = lambda d: min(hp(np.array([d,0.]),np.eye(2)*s*s,R), bx(np.array([d,0.]),np.eye(2)*s*s,R)) - delta
    return brentq(f, 1e-3, R+10*s)
def dstar_hp(R, s, delta): return R - s*float(__import__('scipy').stats.norm.ppf(delta))
print("== centre-distance clearance needed so that bound < delta (isotropic)")
for delta in (0.10, 0.05):
    print("  delta =",delta)
    for n,(r,q) in cls.items():
        R=rR+r
        print("   %-36s R=%.2f"%(n,R), {t: (round(dstar_hp(R,sig(t,q),delta),2), round(dstar(R,sig(t,q),delta),2)) for t in (0.2,0.5,1.0,2.0)}, "(HP-only, min-bound)")
print("== J_risk saturation")
print("  old 1-prod over 250 terms P=0.02:", 1-0.98**250, "; 500 terms P=0.005:", 1-0.995**500)
print("  new 1-prod_j(1-max_k P): one track max 0.02 ->", 0.02, "; three tracks max (0.02,0.05,0.01) ->", 1-(0.98*0.95*0.99))

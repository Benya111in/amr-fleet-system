import numpy as np, itertools, math
rng = np.random.default_rng(0)

# ---------- 1. Cluster covariance R_cl (corrected) for a person at 5 m ----------
sig_r=0.03; dphi=math.radians(0.5); r=5.0
Lpar=0.25; Lperp=0.4  # person: ~0.25 m depth visible face, 0.4 m wide
n = max(3,int(Lperp/(r*dphi)))
s_par2 = sig_r**2/n + Lpar**2/12
s_perp2 = (r*dphi)**2/(12*n) + Lperp**2/12
print(f"[R_cl person@5m] n={n} sigma_par={math.sqrt(s_par2):.3f} m sigma_perp={math.sqrt(s_perp2):.3f} m")
Lpar_b=0.4; Lperp_b=0.5; nb=max(3,int(Lperp_b/(r*dphi)))
print(f"[R_cl box@5m] n={nb} sigma_par={math.sqrt(sig_r**2/nb+Lpar_b**2/12):.3f} sigma_perp={math.sqrt((r*dphi)**2/(12*nb)+Lperp_b**2/12):.3f}")

# ---------- 2. CV-KF confirmation latency (velocity chi2 test), Monte Carlo ----------
def sim_latency(q, sig_par, sig_perp, v=1.0, dt=0.1, N=2000, consec=3, thr=9.21, P0v=4.0, T=3.0):
    F=np.eye(4); F[0,2]=dt; F[1,3]=dt
    Q=q*np.array([[dt**3/3,0,dt**2/2,0],[0,dt**3/3,0,dt**2/2],[dt**2/2,0,dt,0],[0,dt**2/2,0,dt]])
    H=np.zeros((2,4)); H[0,0]=1; H[1,1]=1
    R=np.diag([sig_par**2, sig_perp**2])
    steps=int(T/dt); lat=[]
    for _ in range(N):
        x=np.array([0,0,v,0.0]); z0=x[:2]+rng.multivariate_normal([0,0],R)
        xh=np.array([z0[0],z0[1],0,0.0]); P=np.diag([R[0,0],R[1,1],P0v,P0v])
        cnt=0; done=None
        for k in range(1,steps):
            x=F@x; z=x[:2]+rng.multivariate_normal([0,0],R)
            xh=F@xh; P=F@P@F.T+Q
            S=H@P@H.T+R; K=P@H.T@np.linalg.inv(S); nu=z-H@xh
            xh=xh+K@nu; I=np.eye(4); P=(I-K@H)@P@(I-K@H).T+K@R@K.T
            Pvv=P[2:,2:]; Tstat=xh[2:]@np.linalg.solve(Pvv,xh[2:])
            cnt = cnt+1 if Tstat>thr else 0
            if cnt>=consec: done=k*dt; break
        lat.append(done if done is not None else np.nan)
    lat=np.array(lat); ok=~np.isnan(lat)
    return np.nanmedian(lat), np.nanpercentile(lat,90), ok.mean()

for q in (0.5, 0.1, 0.02):
    for v in (1.0, 0.3, 1.5):
        m,p90,ok=sim_latency(q, math.sqrt(s_par2), math.sqrt(s_perp2), v=v, N=600)
        print(f"[latency CV-KF] q={q} v={v}: median={m:.2f}s p90={p90:.2f}s conf_rate={ok:.2f}")
# false alarm of the 3-consecutive test for a truly static object
def static_fa(q,sig_par,sig_perp,N=600,dt=0.1,T=5.0,thr=9.21,consec=3):
    m,p90,ok=sim_latency(q,sig_par,sig_perp,v=0.0,N=N,T=T)
    return ok
print(f"[static false-dyn rate over 5 s, q=0.5] {static_fa(0.5,math.sqrt(s_par2),math.sqrt(s_perp2)):.3f}")
print(f"[static false-dyn rate over 5 s, q=0.1] {static_fa(0.1,math.sqrt(s_par2),math.sqrt(s_perp2)):.3f}")

# ---------- 3. Free-space fast path: expected displacement needed ----------
# A cluster centroid must have moved by > k * sigma of the free-space evidence: with 1.0 m/s, in 0.3 s it moves 0.3 m
print("[free-space] 1.0 m/s * 0.3 s = 0.30 m displacement; person depth 0.25 m -> cluster fully in previously-free cells after ~0.3 s")
print("[free-space] 0.3 m/s * 0.3 s = 0.09 m  -> free-space path NOT decisive at 0.3 m/s; falls back to velocity test")

# ---------- 4. v_max gate: mean vs max-over-tau ----------
vmax=dict(box=0.0,person=2.0,sign=0.0,forklift=3.0,amr=2.0)
p=dict(box=0.6,person=0.4,sign=0,forklift=0,amr=0)
mean=sum(p[c]*vmax[c] for c in p); mx=max(vmax[c] for c in p if p[c]>0.05)
print(f"[vmax gate] mean={mean:.2f} m/s, max_over_tau={mx:.2f} m/s")
p=dict(box=0.95,person=0.0125,sign=0.0125,forklift=0.0125,amr=0.0125)
mean=sum(p[c]*vmax[c] for c in p); mx=max(vmax[c] for c in p if p[c]>0.05)
print(f"[vmax gate, p_box=0.95 cap] mean={mean:.3f} m/s, max_over_tau(tau=0.05)={mx:.2f} m/s")

# ---------- 5. MPFS truth table with re-derived weights ----------
def logit(x): return math.log(x/(1-x))
def sig(x): return 1/(1+math.exp(-x))
pdyn=dict(box=0.05,person=0.90,sign=0.02,forklift=0.80,amr=0.90)
def pdyn_post(p): return sum(p[c]*pdyn[c] for c in p)
uniform={c:0.2 for c in pdyn}
print(f"[MPFS] p_dyn(uniform posterior)={pdyn_post(uniform):.3f}")
def score(rho_map, rho_free, vel_pass, p, w):
    w1,w2,w3=w
    l_map = -w1*max(0.0, rho_map-0.5)*2   # negative-only: 0 at rho<=0.5, -w1 at rho=1
    l_free = w2*rho_free                   # rho_free counts only cells OBSERVED free in last K scans (unknown = 0)
    l_vel = w3*(1 if vel_pass else 0)
    pd=pdyn_post(p)
    return sig(l_map+l_free+l_vel+logit(pd))
cases = [
 ("mapped shelf, no class",            0.95, 0.0, False, uniform),
 ("un-mapped static box, no class",     0.0,  0.0, False, uniform),
 ("un-mapped static box, class box .8", 0.0,  0.0, False, dict(box=0.8,person=0.05,sign=0.05,forklift=0.05,amr=0.05)),
 ("standing person, class person .8",   0.0,  0.0, False, dict(box=0.05,person=0.8,sign=0.05,forklift=0.05,amr=0.05)),
 ("walking person near shelf, no class",0.6,  0.8, True,  uniform),
 ("walking person near shelf, class .8",0.6,  0.8, True,  dict(box=0.05,person=0.8,sign=0.05,forklift=0.05,amr=0.05)),
 ("walking person open floor, no class",0.0,  0.9, True,  uniform),
 ("pushed box 1.0 m/s, class box .8",   0.0,  0.9, True,  dict(box=0.8,person=0.05,sign=0.05,forklift=0.05,amr=0.05)),
 ("moving box, free-space only (early)",0.0,  0.9, False, dict(box=0.8,person=0.05,sign=0.05,forklift=0.05,amr=0.05)),
 ("walking person, vel only (occluded free)",0.0,0.0,True, dict(box=0.05,person=0.8,sign=0.05,forklift=0.05,amr=0.05)),
]
for w in [(4,3,2),(2,3,3),(2,4,3)]:
    print(f"--- weights w={w}")
    for name,rm,rf,vp,p in cases:
        print(f"   {name:42s} P(dyn)={score(rm,rf,vp,p,w):.3f}")

# ---------- 6. Hungarian: row-offset invariance and padded assignment ----------
def brute(C):
    n,m=C.shape; best=None
    for perm in itertools.permutations(range(m), n):
        s=sum(C[i,perm[i]] for i in range(n))
        if best is None or s<best[0]: best=(s,perm)
    return best
C=rng.uniform(0,9,(4,4)); c=rng.uniform(0.1,1,4)
A1=brute(C); A2=brute(C-np.log(c)[:,None])
print(f"[hungarian row-offset] same assignment: {A1[1]==A2[1]}")
# padded: track i either assigned or 'miss' at cost gamma; confidence changes miss vs assign decision
gamma=9.21
def padded(C, c_track, s_det, lam):
    n,m=C.shape
    big=1e6
    # rows: tracks (n) + birth rows (m); cols: dets (m) + miss cols (n)
    M=np.full((n+m, m+n), big)
    M[:n,:m]=C - lam*np.log(c_track)[:,None] - lam*np.log(s_det)[None,:]
    for i in range(n): M[i, m+i]=gamma
    for j in range(m): M[n+j, j]=gamma
    M[n:, m:]=0.0
    return M
C2=np.array([[8.5, 30.],[30., 30.]]); ct=np.array([0.9,0.3]); sd=np.array([0.9,0.9])
M=padded(C2,ct,sd,1.0); best=brute(M)
print(f"[padded] assignment (rows: t0,t1,b0,b1 -> cols d0,d1,miss0,miss1): {best[1]} cost={best[0]:.2f}")
C2b=np.array([[8.5, 30.],[30., 30.]]); ct=np.array([0.3,0.3])
M=padded(C2b,ct,sd,1.0); best=brute(M)
print(f"[padded, low track conf] assignment: {best[1]} cost={best[0]:.2f}  (d2=8.5 vs gamma=9.21: low-conf track -> miss preferred? )")

# ---------- 7. IMM spread-of-means in association covariance ----------
zhat=[np.array([0,0.]),np.array([0.3,0.])]; S=[np.eye(2)*0.05, np.eye(2)*0.05]; cbar=[0.5,0.5]
zbar=sum(c*z for c,z in zip(cbar,zhat))
S_naive=sum(c*s for c,s in zip(cbar,S))
S_full=sum(c*(s+np.outer(z-zbar,z-zbar)) for c,s,z in zip(cbar,S,zhat))
print(f"[IMM S] naive diag={np.diag(S_naive)}, with spread diag={np.diag(S_full)}")

# ---------- 8. NEES band ----------
for N in (100,1000,10000):
    print(f"[NEES 2-DoF mean band N={N}] 2 +/- {1.96*math.sqrt(4/N):.3f}")

# ---------- 9. Ego-speed term double count ----------
print(f"[ego term] beta*v^2*dt^2 = {0.1*4*0.01:.4f} m^2 -> sigma={math.sqrt(0.004)*100:.1f} cm; TF interpolation residual at 10 Hz/2 m/s with 1 kHz-equivalent per-beam interpolation ~ mm")

# ---------- 10. Slope-variance formula for velocity from n positions ----------
for n in (3,5,8,10):
    sig=0.065; dt=0.1
    var_v = 12*sig**2/(dt**2*n*(n**2-1))
    print(f"[LS slope var] n={n}: sigma_v={math.sqrt(var_v):.3f} m/s -> 1.0 m/s gives T~{(1.0/math.sqrt(var_v))**2:.1f} (1-DoF), 0.3 m/s gives {(0.3/math.sqrt(var_v))**2:.1f}")

# ---------- 11. GPU budget ----------
print(f"[GPU] per-image budget 1000/150={1000/150:.1f} ms; per-batch(5) budget at 30 Hz = {1000/30:.1f} ms")
# ---------- 12. Surface-vs-centre bias ----------
print("[bias] box 0.5x0.4x0.3: half-depth 0.15-0.25 m; forklift ~1 m; person ~0.10-0.15 m")
# ---------- 13. sync: 30 Hz dets vs 15 Hz depth slop 0.03 ----------
print("[sync] depth period 66.7 ms, slop 30 ms -> every other 30-Hz detection has no depth within slop -> 15 Hz effective")

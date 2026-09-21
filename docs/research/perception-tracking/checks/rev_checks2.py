import math, numpy as np
sig=lambda x:1/(1+math.exp(-x)); logit=lambda p:math.log(p/(1-p))
pd=[0.05,0.90,0.02,0.80,0.90]; pu=sum(pd)/5
print(f"[MPFS brief formula] uniform class p_dyn={pu:.3f}")
for rho_map in (0.0,0.3,0.5):
    l=4*(0.5-rho_map)+0+0+logit(pu); print(f"  unmapped static, rho_map={rho_map}, rho_free=0, T fail: P(dyn)={sig(l):.3f}")
l=4*(0.5-0.0)+0+0+logit(0.8*0.05+0.2*0.5); print(f"  unmapped static box p(box)=.8: P(dyn)={sig(l):.3f}")
# single-frame projection error at 5 m
fx=fy=615.0  # 640x480, 87deg HFOV -> fx = 320/tan(43.5deg)
print(f"[fx for 87deg HFOV @640] {320/math.tan(math.radians(43.5)):.1f} px")
Z=5.0; sZ=0.01+0.002*Z*Z; w=80; su=0.1*w
sX=Z/fx*su; print(f"[3D @5m] sigma_Z={sZ*100:.1f} cm, sigma_X(u-jitter {su:.0f}px)={sX*100:.1f} cm, RMS 2D={math.hypot(sX,sZ)*100:.1f} cm (target 10 cm), + surface bias box 15-25 cm")
Z=3.0; sZ=0.01+0.002*Z*Z; sX=Z/fx*su; print(f"[3D @3m] sigma_Z={sZ*100:.1f} cm, sigma_X={sX*100:.1f} cm, RMS={math.hypot(sX,sZ)*100:.1f} cm")
# class posterior saturation
M=np.full((5,5),0.025); np.fill_diagonal(M,0.9); eps=0.1; s=0.8
p=np.full(5,0.2)
for k in range(1,6):
    lik=eps+(1-eps)*M[1,:]*s; p=p*lik; p/=p.sum(); print(f"[class Bayes] after {k} consistent 'person' hits: p(person)={p[1]:.3f}")
# GPU budget
print(f"[GPU] 150 img/s -> per-image {1000/150:.2f} ms; batch-5 at 30 Hz -> per-batch {1000/30:.1f} ms (brief says per-batch 6.7 ms)")
# lifecycle min latency
print("[lifecycle] tentative->confirmed needs 3 hits (>=0.3 s incl. first); velocity chi2 3 consecutive after that -> >=0.5-0.6 s minimum even with perfect data")
# vmax gate with mean vs max
print(f"[vmax] p=(box .6, person .4): mean={0.6*0+0.4*2:.2f}; person at 1.5 m/s moves {1.5*0.1:.2f} m/step; gate radius = vmax*dt + 3*sigma = {0.8*0.1:.2f}+3*0.116={0.08+3*0.116:.2f} m -> passes only thanks to the 3-sigma term")
# ego term
print(f"[ego] beta*v^2*dt^2 at 2 m/s = {0.1*4*0.01:.4f} m^2 -> {math.sqrt(0.004)*100:.1f} cm added std per axis; at 1 m/s {math.sqrt(0.001)*100:.1f} cm")
# Hungarian row-offset invariance proof-check on random rectangular
import itertools
rng=np.random.default_rng(1); same=0; tot=0
for _ in range(200):
    n,k=[int(v) for v in rng.integers(2,5,size=2)]; C=rng.uniform(0,9,(n,k)); c=rng.uniform(0.1,1,n); sd=rng.uniform(0.1,1,k)
    def best(Cm):
        if n<=k:
            return min(itertools.permutations(range(k),n), key=lambda pm:sum(Cm[i,pm[i]] for i in range(n)))
        else:
            return min(itertools.permutations(range(n),k), key=lambda pm:sum(Cm[pm[j],j] for j in range(k)))
    a=best(C); b=best(C-np.log(c)[:,None]-np.log(sd)[None,:])
    tot+=1; same+= (a==b)
print(f"[hungarian] rectangular random tests where -log c_i -log s_j changes assignment: {tot-same}/{tot} (nonzero only when n!=k, i.e. via which rows/cols are left unassigned)")

import numpy as np
def clamp(x,lo,hi): return max(lo,min(hi,x))
def run(v_ref_seq, dt=0.01, a_max=1.0, a_dec=1.0, j_max=2.0, v0=0.0, a0=0.0):
    v,a=v0,a0; V=[];A=[];J=[]
    for v_ref in v_ref_seq:
        sigma=(v_ref-v)-a*abs(a)/(2*j_max)
        delta=j_max*dt*(abs(a)+j_max*dt/2)
        if abs(sigma)>delta: j=j_max*np.sign(sigma)
        else:
            j=clamp(sigma/delta*j_max,-j_max,j_max)
            j=clamp(j,(-a_max-a)/dt,(a_max-a)/dt)
            if abs(v_ref-v)<j_max*dt*dt:
                a=0.0; v=v_ref; V.append(v);A.append(a);J.append(0.0); continue
        a=clamp(a+j*dt,-a_dec,a_max); v=v+a*dt
        V.append(v);A.append(a);J.append(j)
    return np.array(V),np.array(A),np.array(J)
dt=0.01
# test 1: 0 -> 2 m/s step
V,A,J=run([2.0]*600)
t_reach=np.argmax(np.abs(V-2.0)<1e-3)*dt
print("0->2: reach", t_reach,"s; max|a|",A.max(),"max|j|",np.abs(J).max(), " final a",A[-1]," final v",V[-1])
# chatter: sign changes of j after reaching
after=J[int(t_reach/dt)+5:]
sc=np.sum(np.diff(np.sign(after[after!=0]))!=0); print("jerk sign changes after reach:",sc)
# test 2: random step sequence
rng=np.random.default_rng(0); seq=np.repeat(rng.uniform(0,2,20),150)
V,A,J=run(seq)
print("random: max|a|",np.abs(A).max()," max|j|",np.abs(J).max()," max |dv/dt| ",np.abs(np.diff(V)/dt).max()," max|da/dt|",np.abs(np.diff(A)/dt).max())
err=np.abs(V-seq); 
# settled error at end of each hold
ends=[abs(V[(k+1)*150-1]-seq[(k+1)*150-1]) for k in range(20)]
print("end-of-hold errors max:",max(ends))
# overshoot check
ov=0
for k in range(1,20):
    seg=V[k*150:(k+1)*150]; tgt=seq[k*150]; prev=seq[k*150-1]
    if tgt>prev: ov=max(ov,seg.max()-tgt)
    else: ov=max(ov,tgt-seg.min())
print("max overshoot:",ov)
# test 3: startup exemption a0=0.3
V,A,J=run([2.0]*600,a0=0.3)
print("startup a0=0.3: t to 0.02 m/s:",np.argmax(V>0.02)*dt,"s")
# test 4: small step 0.1 (short branch)
V,A,J=run([0.1]*200); print("0->0.1: reach",np.argmax(np.abs(V-0.1)<1e-4)*dt," expected 2*sqrt(0.1/2)=",2*np.sqrt(0.05))

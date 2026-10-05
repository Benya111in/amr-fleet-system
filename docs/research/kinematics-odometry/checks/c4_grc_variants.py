"""GRC safeguard variants: v2 as written (step clip + gate) deadlocks; test (a) no clip, (b) Joseph-consistent clip,
and slow persistent asymmetric slip absorption after convergence."""
import numpy as np
r, b, N, ss = 0.0825, 0.36, 4096, 0.01
psi_nom = r/b; sg = 2e-4
q = (1e-3*psi_nom)**2/6000; Pmax = (0.05*psi_nom)**2; delta = 2*np.pi*r/N
segs0 = [(10, 1.0, 0.0), (5, 0.0, 1.5), (10, 0.5, 0.0), (5, 0.0, -1.5), (10, 1.0, 0.5), (5, 0.0, 1.5), (15, 1.5, 0.0)]

def run(psi_true, mode, segs=segs0, psi0=None, P=None, seed=0, slipL=0.0):
    rg = np.random.default_rng(seed)
    psi_hat = np.array(psi0 if psi0 is not None else [psi_nom]*2, float)
    P = np.eye(2)*Pmax if P is None else P.copy()
    t = 0.0; tc = None; nrej = 0; hist = []
    for dur, v, w in segs:
        for _ in range(int(round(dur/0.1))):
            vR, vL = v + w*b/2, v - w*b/2
            phR, phL = vR/(psi_true[0]*b), vL/(psi_true[1]*b)*(1+slipL)   # slipL: left wheel spins faster than rolling
            hR = (phR*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1
            hL = (phL*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1
            z = w + rg.normal(0, sg, 10).mean()
            h = np.array([hR, -hL])
            s2R = ss**2*(abs(vR)*0.02)*(abs(vR)*0.1) + delta**2/6
            s2L = ss**2*(abs(vL)*0.02)*(abs(vL)*0.1) + delta**2/6
            R = (s2R+s2L)/(b**2*0.01) + sg**2/10
            Pm = P + q*np.eye(2)
            if abs(hR) > 1.0 and abs(hL) > 1.0:
                nu = z - h@psi_hat; S = h@Pm@h + R
                if nu**2 <= 9*S:
                    K = Pm@h/S; step = K*nu
                    if mode == "v2clip":
                        step = np.clip(step, -1e-3*psi_nom, 1e-3*psi_nom); Pm = (np.eye(2)-np.outer(K, h))@Pm
                    elif mode == "joseph":
                        c = min(1.0, 1e-3*psi_nom/max(np.max(np.abs(step)), 1e-30)); Ke = c*K; step = Ke*nu
                        A = np.eye(2)-np.outer(Ke, h); Pm = A@Pm@A.T + np.outer(Ke, Ke)*R
                    else:  # "noclip"
                        Pm = (np.eye(2)-np.outer(K, h))@Pm
                    psi_hat = np.clip(psi_hat+step, 0.95*psi_nom, 1.05*psi_nom)
                else:
                    nrej += 1
            ev, V = np.linalg.eigh(0.5*(Pm+Pm.T)); P = V@np.diag(np.minimum(ev, Pmax))@V.T
            t += 0.1
            err = np.max(np.abs(psi_hat-psi_true)/psi_true)
            if tc is None and err <= 0.01: tc = t
            elif tc is not None and err > 0.01: tc = None
            hist.append(psi_hat.copy())
    return psi_hat, P, tc, nrej, np.array(hist)

cases = ([psi_nom/1.05]*2, [psi_nom*1.04, psi_nom*0.97], [psi_nom*1.05, psi_nom*0.95], [psi_nom*0.96, psi_nom*1.03])
for mode in ("v2clip", "joseph", "noclip"):
    for pt in cases:
        pt = np.array(pt); errs = []; tcs = []; rej = []
        for sd in range(20):
            ph, P, tc, nr, _ = run(pt, mode, seed=sd)
            errs.append(np.max(np.abs(ph-pt)/pt)); tcs.append(tc if tc else np.inf); rej.append(nr)
        print(f"{mode:7s} true/nom={np.round(pt/psi_nom,3)}: 60 s max err {max(errs)*100:6.3f} %, t(<=1 %) median {np.median(tcs):5.1f} s max {max(tcs):5.1f} s, gate rejections median {int(np.median(rej))}")

# slow persistent asymmetric slip after convergence (noclip): left wheel over-rotates by 0.5 % / 2 % for 120 s straight 1 m/s
pt = np.array([psi_nom]*2)
ph, P, _, _, _ = run(pt, "noclip", seed=5)
for sl in (0.005, 0.02):
    ph2, P2, _, nr, hist = run(pt, "noclip", segs=[(120, 1.0, 0.0)], psi0=ph, P=P, seed=6, slipL=sl)
    drift = (hist[-1]-ph)/psi_nom*100
    print(f"persistent left-wheel slip {sl*100:.1f} % for 120 s: psi_hat change {np.round(drift,3)} % of nominal, gate rejections {nr}/1200")

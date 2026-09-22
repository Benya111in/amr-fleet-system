"""GRC two-mode variant (commissioning: bias-compensated KF, no gate/CUSUM, >= 60 s excited; tracking: 3-sigma gate + CUSUM freeze) WITH errors-in-variables bias compensation (+P Sigma_n psi / S): no per-update clip, 3-sigma innovation gate, P eigen-clip, physical bounds,
two-sided CUSUM on ALL excited normalised innovations (kappa=0.5, h=12), armed once lambda_max(P) <= (0.2 % psi)^2;
alarm -> freeze updates (latched, diagnostic). Checks: convergence, slow-slip absorption, false freezes over 4 h."""
import numpy as np
r, b, N, ss = 0.0825, 0.36, 4096, 0.01
psi_nom = r/b; sg = 2e-4
q = (1e-3*psi_nom)**2/6000; Pmax = (0.05*psi_nom)**2; delta = 2*np.pi*r/N
Parm = (2e-3*psi_nom)**2; KAP, HC = 0.5, 12.0
segs0 = [(10, 1.0, 0.0), (5, 0.0, 1.5), (10, 0.5, 0.0), (5, 0.0, -1.5), (10, 1.0, 0.5), (5, 0.0, 1.5), (15, 1.5, 0.0)]

class GRC:
    def __init__(s, psi0=None):
        s.psi = np.array(psi0 if psi0 is not None else [psi_nom]*2, float); s.P = np.eye(2)*Pmax
        s.gp = s.gm = 0.0; s.armed = False; s.t_exc = 0.0; s.frozen = False; s.t_alarm = None
    def step(s, h, z, R, t, Sn=None):
        Pm = s.P + q*np.eye(2)
        nu = z - h@s.psi; S = h@Pm@h + R; nn = nu/np.sqrt(S)
        s.t_exc += 0.1
        if not s.armed and s.t_exc >= 60.0 and np.linalg.eigvalsh(Pm).max() <= Parm: s.armed = True
        if s.armed and not s.frozen:
            s.gp = max(0.0, s.gp + nn - KAP); s.gm = max(0.0, s.gm - nn - KAP)
            if max(s.gp, s.gm) > HC: s.frozen = True; s.t_alarm = t
        if not s.frozen and (not s.armed or nu**2 <= 9*S):
            K = Pm@h/S; bc = (Pm@(Sn@s.psi))/S if Sn is not None else 0.0; s.psi = np.clip(s.psi + K*nu + bc, 0.95*psi_nom, 1.05*psi_nom); Pm = (np.eye(2)-np.outer(K, h))@Pm
        ev, V = np.linalg.eigh(0.5*(Pm+Pm.T)); s.P = V@np.diag(np.minimum(ev, Pmax))@V.T

def drive(g, psi_true, segs, rg, t0=0.0, slipL=0.0, track=None):
    t = t0; tc = None
    for dur, v, w in segs:
        for _ in range(int(round(dur/0.1))):
            vR, vL = v + w*b/2, v - w*b/2
            phR, phL = vR/(psi_true[0]*b), vL/(psi_true[1]*b)*(1+slipL)
            hR = (phR*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1; hL = (phL*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1
            z = w + rg.normal(0, sg, 10).mean()
            s2R = ss**2*(abs(vR)*0.02)*(abs(vR)*0.1) + delta**2/6; s2L = ss**2*(abs(vL)*0.02)*(abs(vL)*0.1) + delta**2/6
            R = (s2R+s2L)/(b**2*0.01) + sg**2/10
            Sn = np.diag([s2R, s2L])/(r**2*0.01)
            if abs(hR) > 1.0 and abs(hL) > 1.0: g.step(np.array([hR, -hL]), z, R, t, Sn)
            t += 0.1
            if track is not None:
                err = np.max(np.abs(g.psi-psi_true)/psi_true)
                if tc is None and err <= 0.01: tc = t
                elif tc is not None and err > 0.01: tc = None
    return t, tc

cases = ([psi_nom/1.05]*2, [psi_nom*1.04, psi_nom*0.97], [psi_nom*1.05, psi_nom*0.95], [psi_nom*0.96, psi_nom*1.03])
for pt in cases:
    pt = np.array(pt); errs = []; tcs = []; fz = 0
    for sd in range(20):
        g = GRC(); _, tc = drive(g, pt, segs0, np.random.default_rng(sd), track=True)
        errs.append(np.max(np.abs(g.psi-pt)/pt)); tcs.append(tc if tc else np.inf); fz += g.frozen
    print(f"true/nom={np.round(pt/psi_nom,3)}: 60 s max err {max(errs)*100:.3f} %, t(<=1 %) median {np.median(tcs):.1f} s max {max(tcs):.1f} s, frozen {fz}/20")

for sl in (0.005, 0.02):
    ch = []; fr = 0; lat = []
    for sd in range(10):
        rg = np.random.default_rng(100+sd); g = GRC(); t, _ = drive(g, np.array([psi_nom]*2), segs0, rg)
        p0 = g.psi.copy(); drive(g, np.array([psi_nom]*2), [(120, 1.0, 0.0)], rg, t0=t, slipL=sl)
        ch.append(np.max(np.abs(g.psi-p0))/psi_nom*100); fr += g.frozen
        if g.frozen: lat.append(g.t_alarm - t)
    print(f"persistent left slip {sl*100:.1f} % 120 s: max |psi change| {max(ch):.3f} % of nominal, frozen {fr}/10, alarm latency median {np.median(lat) if lat else float('nan'):.1f} s")

# 4 h no-slip endurance: repeated mixed pattern; count false freezes and final drift
fz = 0; drifts = []
for sd in range(5):
    rg = np.random.default_rng(200+sd); g = GRC(); t, _ = drive(g, np.array([psi_nom]*2), segs0, rg)
    p0 = g.psi.copy()
    drive(g, np.array([psi_nom]*2), segs0*int(4*3600/60), rg, t0=t)
    fz += g.frozen; drifts.append(np.max(np.abs(g.psi-p0))/psi_nom*100)
print(f"4 h no-slip: false freezes {fz}/5, max psi drift {max(drifts):.3f} % of nominal")

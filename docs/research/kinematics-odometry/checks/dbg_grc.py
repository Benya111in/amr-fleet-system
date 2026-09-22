import numpy as np, importlib.util, sys
spec = importlib.util.spec_from_file_location("c5", "c5_grc_cusum.py")
src = open("c5_grc_cusum.py").read().split("cases = (")[0]
exec(src)
pt = np.array([psi_nom*1.04, psi_nom*0.97]); rg = np.random.default_rng(0); g = GRC()
t = 0.0; log = []
for dur, v, w in segs0:
    for _ in range(int(round(dur/0.1))):
        vR, vL = v + w*b/2, v - w*b/2
        phR, phL = vR/(pt[0]*b), vL/(pt[1]*b)
        hR = (phR*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1; hL = (phL*0.02*(1+rg.normal(0, ss, 5))).sum()/0.1
        z = w + rg.normal(0, sg, 10).mean()
        s2R = ss**2*(abs(vR)*0.02)*(abs(vR)*0.1) + delta**2/6; s2L = ss**2*(abs(vL)*0.02)*(abs(vL)*0.1) + delta**2/6
        R = (s2R+s2L)/(b**2*0.01) + sg**2/10
        if abs(hR) > 1.0 and abs(hL) > 1.0:
            h = np.array([hR, -hL]); Pm = g.P + q*np.eye(2); nu = z - h@g.psi; S = h@Pm@h + R
            was = g.armed; g.step(h, z, R, t)
            if (not was and g.armed) or (g.frozen and g.t_alarm == t) or int(t*10) % 50 == 0:
                e = (g.psi-pt)/pt*100
                print(f"t={t:5.1f} seg=({v},{w}) err%={np.round(e,3)} sqrtEigP%={np.round(np.sqrt(np.linalg.eigvalsh(g.P))/psi_nom*100,3)} nn={nu/np.sqrt(S):6.2f} armed={g.armed} frozen={g.frozen} gp={g.gp:.1f} gm={g.gm:.1f}")
        t += 0.1

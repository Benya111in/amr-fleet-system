"""c01: dynamic window, horizons, cell-access counts, J_vel range, stopping distances,
admissible speed (robot_params.yaml clearance formula), return-manoeuvre time."""
import numpy as np
from math import sqrt, pi, sin, cos
a_v, alpha_w, dt_c, j_max = 1.0, 2.0, 0.05, 2.0      # robot_params.yaml limits
v_min, v_max, w_max = -0.5, 2.0, 1.5
t_react = 0.15                                        # safety.reaction_latency
print("== V_d half-widths: dv =", a_v*dt_c, "m/s, dw =", alpha_w*dt_c, "rad/s")
print("   resolution Nv=11 ->", 2*a_v*dt_c/10, "m/s ; Nw=31 ->", round(2*alpha_w*dt_c/30,5), "rad/s")
print("   jerk-limited accel change per cycle j*dt_c =", j_max*dt_c, "m/s^2")
# horizons
Tsim = lambda v: min(max(abs(v)/a_v+0.5,1.5),2.5)
print("== T_sim(v) =", {v: Tsim(v) for v in (0,0.5,1.0,1.5,2.0)}, "-> N_s max =", round(Tsim(2.0)/0.1))
print("   old clip upper 3.0 unreachable: v/a+0.5 at v=2 ->", 2/a_v+0.5)
M, Ns, P = 343, 25, 40
print("== static cell reads worst case M*N_s*P =", M*Ns*P, "(old 30 poses:", M*30*P, ")")
# J_vel
for name,f in [("old (vmax-v)/vmax", lambda v:(v_max-v)/v_max), ("new (vmax-v)/(vmax-vmin)", lambda v:(v_max-v)/(v_max-v_min))]:
    vs=np.linspace(v_min,v_max,251); print("== J_vel", name, "range [%.3f, %.3f]"%(f(vs).min(), f(vs).max()))
# stopping distance: no jerk vs jerk-limited (a0 = current accel, worst case a0=+a when accelerating)
def stop_jerk(v0, a0=0.0, a=a_v, j=j_max, dt=1e-4):
    v, acc, s, t = v0, a0, 0.0, 0.0
    while v > 0:
        acc = max(acc - j*dt, -a)          # ramp decel with jerk limit
        v += acc*dt; s += max(v,0)*dt; t += dt
    return s, t
print("== stopping (after reaction t_r=0.15 s): v, d_nojerk, d_jerk(a0=0), d_jerk(a0=+1), t_stop(a0=0)")
for v in (0.2,0.5,1.0,1.5,2.0):
    dn = v*t_react + v*v/(2*a_v); sj,tj = stop_jerk(v); sj1,_ = stop_jerk(v,1.0)
    print("   %.1f  %.3f  %.3f  %.3f  %.2f s" % (v, dn, v*t_react+sj, v*t_react+sj1, t_react+tj))
# clearance speed limit from robot_params.yaml (no jerk) and a jerk-aware inverse
vlim = lambda D: -a_v*t_react + sqrt((a_v*t_react)**2 + 2*a_v*max(D-0.30,0))
print("== config v_max(D):", {D: round(vlim(D),3) for D in (0.5,1.0,2.6)})
def vlim_jerk(D):
    lo,hi=0.0,2.0
    for _ in range(40):
        m=(lo+hi)/2; s,_=stop_jerk(m); d=m*t_react+s+0.30
        lo,hi=(m,hi) if d<=D else (lo,m)
    return lo
print("   jerk-aware v_max(D):", {D: round(vlim_jerk(D),3) for D in (0.5,1.0,2.6,3.1)})
# return manoeuvre: lateral offset e0 -> |e|<0.10 with v const, alpha_w, w_max (min time over heading psi)
def return_time(e0, v, dt=1e-3):
    best=None
    for psi in np.radians(np.arange(5,91,1)):
        # bang-bang heading up to psi, cruise, bang-bang back to 0 so that heading 0 when |e|<=0.1
        tw = sqrt(psi/alpha_w); wpk = alpha_w*tw
        if wpk > w_max: continue   # (peak w would exceed limit; skip for simplicity)
        # simulate turn-in
        y, th, t = e0, 0.0, 0.0
        def turn(th0, sign, y, t):
            th=th0
            for k in range(int(round(2*tw/dt))):
                tt=k*dt; w = sign*(alpha_w*tt if tt<tw else alpha_w*(2*tw-tt))
                th += w*dt; y += v*sin(th)*dt; t += dt
            return th,y,t
        th,y,t = turn(0.0,-1,y,t)          # heading toward path (negative)
        _,dy_out,_ = turn(-psi,+1,0.0,0.0) # lateral change during turn-out (negative)
        # cruise until y + dy_out <= 0.05 (land in the 0.1 band centre)
        target = 0.05 - dy_out
        if y <= target: tc=0.0
        else: tc = (y-target)/(v*sin(psi))
        tot = t + tc + 2*tw
        if best is None or tot<best[0]: best=(tot, np.degrees(psi), wpk)
    return best
for v in (0.5,1.0,1.5):
    for e0 in (0.8,1.0):
        b=return_time(e0,v); print("== return e0=%.1f m, v=%.1f: t_min=%.2f s at psi=%d deg, w_peak=%.2f"%(e0,v,b[0],b[1],b[2]))
print("== critique estimate check: 45deg bang-bang each way", 2*sqrt((pi/4)/alpha_w), "s, peak w", sqrt(pi/4*alpha_w))

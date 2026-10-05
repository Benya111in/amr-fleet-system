"""Audit-2 independent check of Prop. 3 / unit test (3b): closed-loop PP (= CC-PP with K=1 on a circle)
on a CCW circle of radius R, exact geometry (look-ahead point = intersection of the circle of radius L
around the robot with the path circle, forward branch), continuous-time kinematics, small initial offset.
zeta is extracted from the first overshoot  OS = exp(-zeta*pi/sqrt(1-zeta^2))  and compared with
zeta_circ = K c1 / sqrt(2K + (1-K) L^2 kappa^2), c1 = sqrt(1 - L^2 kappa^2/4). Also K=2 CC-PP."""
import numpy as np

def lookahead_on_circle(p, R, L, phi_robot):
    # path circle centred at origin radius R, robot at p; intersection points of |x|=R and |x-p|=L
    d = np.linalg.norm(p); a = (R*R - L*L + d*d)/(2*d); h = np.sqrt(max(R*R - a*a, 0.0))
    u = p/d; n = np.array([-u[1], u[0]]); base = a*u
    c1, c2 = base + h*n, base - h*n
    # forward (CCW) branch: larger polar angle relative to robot's polar angle
    def ahead(c): return np.mod(np.arctan2(c[1], c[0]) - phi_robot, 2*np.pi)
    return c1 if ahead(c1) < ahead(c2) else c2

def run(R, L, v, K, e0, T=20.0, dt=1e-4):
    kap = 1/R
    # robot on the circle at polar angle 0, left offset e0 (left of CCW travel = towards the centre)
    x = np.array([R - e0, 0.0]); th = np.pi/2          # tangent heading at polar angle 0, psi0 = 0
    es = []
    for k in range(int(T/dt)):
        phi = np.arctan2(x[1], x[0]); rad = np.linalg.norm(x); e = R - rad
        G = lookahead_on_circle(x, R, L, phi); rel = G - x
        yg = -np.sin(th)*rel[0] + np.cos(th)*rel[1]
        # Frenet reference chord from the projection P(s_r) = R*(cos phi, sin phi): y_g^0 = L^2 kappa / 2
        yg0 = L*L*kap/2
        w = v*(kap + K*2*(yg - yg0)/L**2)
        x = x + dt*v*np.array([np.cos(th), np.sin(th)]); th += dt*w
        es.append(e)
    es = np.array(es)
    first_os = -es.min()/e0 if e0 > 0 else -es.max()/e0
    return first_os

for R, L, K in [(1.5, 0.876, 1.0), (1.5, 1.6, 1.0), (1.5, 0.9, 1.0), (1.5, 0.876, 2.0), (1e6, 0.876, 1.0)]:
    kap = 1/R; c1 = np.sqrt(1 - L*L*kap*kap/4); z_th = K*c1/np.sqrt(2*K + (1 - K)*L*L*kap*kap)
    for e0 in [0.01, -0.01]:
        os_ = run(R, L, 1.095, K, e0)
        if os_ > 1e-6:
            q = np.log(os_); z = -q/np.sqrt(np.pi**2 + q*q)
        else:
            z = float("nan")
        print(f"R={R:g} L={L} K={K} e0={e0:+.2f}: first overshoot {100*os_:.3f} % -> zeta {z:.3f} | Prop.3 zeta_circ {z_th:.3f}")

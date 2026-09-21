"""Unit test (3)/(3b) values by nonlinear kinematic simulation of PP on a straight line and a circle,
with and without the b_w=1 inner loop (omega_c=6)."""
import numpy as np
def sim_straight(L, v, e0, K=1.0, inner=None, T=12.0, dt=1e-3):
    x, y, th = 0.0, e0, 0.0; w_state = np.zeros(2); w_act = 0.0; ys = []
    for k in range(int(T/dt)):
        # lookahead on line y=0 at distance L from robot (forward intersection)
        dx = np.sqrt(max(L*L - y*y, 0.0)); gx, gy = x + dx, 0.0
        rx, ry = gx - x, gy - y; yg = -np.sin(th)*rx + np.cos(th)*ry
        w_cmd = v*K*2*yg/L**2
        if inner is None: w_act = w_cmd
        else:   # 2nd-order inner loop (2 z w s + w^2)/(s^2 + 2 z w s + w^2), z=1, b=1
            wc = 6.0; x1, x2 = w_state
            dx1 = x2; dx2 = -wc*wc*x1 - 2*wc*x2 + w_cmd
            w_act = wc*wc*x1 + 2*wc*x2; w_state = w_state + dt*np.array([dx1, dx2])
        x += v*np.cos(th)*dt; y += v*np.sin(th)*dt; th += w_act*dt; ys.append(y)
    ys = np.array(ys); return -100*ys.min()/e0
for (L, v, e0) in [(1.2, 1.5, 0.2), (0.4, 0.5, 0.2), (1.0, 1.25, 0.2)]:
    print(f"UT3 straight L={L} v={v} e0={e0}: overshoot K=1 {sim_straight(L, v, e0):.2f} %, K=2 {sim_straight(L, v, e0, K=2):.3f} %, "
          f"K=1+inner(b=1) {sim_straight(L, v, e0, inner=True):.2f} %")

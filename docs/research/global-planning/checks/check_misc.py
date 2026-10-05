"""Remaining numeric checks for the global-planning brief."""
import math
import random
import numpy as np

# --- §2.2 Theta*: per-LOS-cell traversal cost at c=0
print("Theta* 2*(26/252)^2 =", round(2 * (26 / 252) ** 2, 4), "; unknown -> 26+0.9*253 =", 26 + 0.9 * 253,
      "; LOS safe iff 26+0.9c<=252 -> c<=", math.floor((252 - 26) / 0.9))

# --- §2.2 NavFn kappa-equivalent
print("NavFn kappa-eq 0.8*252/50 =", round(0.8 * 252 / 50, 3))

# --- §2.3 octile excess
th = np.linspace(0, math.pi / 4, 100001)
f = np.cos(th) + (math.sqrt(2) - 1) * np.sin(th)
print("octile max excess = %.4f at %.2f deg" % (f.max(), math.degrees(th[f.argmax()])))

# --- §2.6 smoother: in-place Gauss-Seidel sweep (Nav2 smoother.cpp) vs Jacobi
def sweep_radius(wd, ws, n=60, gs=True, iters=4000):
    x = np.zeros(n); x[1:-1] = np.random.RandomState(0).randn(n - 2)
    # error dynamics: iterate with x fixed at 0 target -> fixed point y=0 (Dirichlet ends 0)
    y = np.random.RandomState(1).randn(n); y[0] = y[-1] = 0
    xz = np.zeros(n)
    norms = []
    for _ in range(iters):
        if gs:
            for i in range(1, n - 1):
                y[i] += wd * (xz[i] - y[i]) + ws * (y[i - 1] + y[i + 1] - 2 * y[i])
        else:
            yn = y.copy()
            for i in range(1, n - 1):
                yn[i] = y[i] + wd * (xz[i] - y[i]) + ws * (y[i - 1] + y[i + 1] - 2 * y[i])
            y = yn
        nr = np.linalg.norm(y)
        norms.append(nr)
        if nr > 1e12 or nr < 1e-14:
            break
    k = len(norms)
    return (norms[-1] / norms[max(0, k - 50)]) ** (1 / min(50, k - 1)) if k > 1 else 0

for wd, ws in ((0.2, 0.3), (0.2, 0.5), (0.2, 0.85), (0.2, 0.95)):
    print(f"smoother (w_d,w_s)=({wd},{ws}) omega={wd+2*ws:.2f} jacobi_bound={wd+4*ws:.2f}: "
          f"GS rate={sweep_radius(wd, ws):.4f}  Jacobi rate={sweep_radius(wd, ws, gs=False):.4f}")

# --- §4.2 tube cell count: rasterised Chebyshev tube around a straight segment vs (2L rho + 4 rho^2)/r^2
r = 0.05
def tube_cells(L, rho):
    n = int(round(L / r)); k = int(round(rho / r))
    xs = set()
    for i in range(-k, n + k + 1):
        for j in range(-k, k + 1):
            xs.add((i, j))
    return len(xs)
for L, rho in ((6, 1), (6, 3), (6, 6), (6, 12), (15, 1), (15, 3)):
    print(f"tube L={L} rho={rho}: raster={tube_cells(L, rho)}  formula={(2*L*rho+4*rho**2)/r**2:.0f}")
N = 1200 * 800
worst = (2 * 6 * 1 + 4) / r**2 + (2 * 6 * 3 + 36) / r**2 + N
print(f"worst cells (L=6): {worst:.3g}; at 150-300 ns: {worst*150e-9*1e3:.0f}-{worst*300e-9*1e3:.0f} ms")
print(f"FULL alone: {N*150e-9*1e3:.0f}-{N*300e-9*1e3:.0f} ms ; tube budget 40 ms + FULL max = {40+N*300e-9*1e3:.0f} ms")

# --- §4.3(c) curvature: turning-angle estimate vs Menger, incl. unequal steps
random.seed(0)
viol = 0
for _ in range(200000):
    a, b = random.uniform(0.01, 1), random.uniform(0.01, 1)
    dth = random.uniform(1e-4, math.pi - 1e-4)
    c = math.sqrt(a * a + b * b + 2 * a * b * math.cos(dth))
    menger = 2 * math.sin(dth) / c
    turn = dth / ((a + b) / 2)
    if turn < menger - 1e-12:
        viol += 1
print("turning-angle >= Menger violations (random unequal steps):", viol)
h = 0.05
for deg in (10, 45, 90):
    d = math.radians(deg)
    print(f"  equal step h, dth={deg}: Menger={2*math.sin(d/2)/h:.3f}  turning={d/h:.3f}")

# --- §4.3(d) rotation and jerk terms with robot_params limits
W, ALPHA, V, AA, J = 1.5, 2.0, 2.0, 1.0, 2.0
def rot_time(th):
    th_acc = W * W / ALPHA  # angle used to accelerate+decelerate
    if th >= th_acc:
        return th / W + W / ALPHA
    return 2 * math.sqrt(th / ALPHA)
for deg in (45, 90, 180):
    th = math.radians(deg)
    print(f"rotate {deg} deg: |dth|/w_max={th/W:.3f} s  trapezoid(alpha=2)={rot_time(th):.3f} s")
# jerk-limited rest-to-rest vs trapezoid, long move
for L in (5, 20, 40):
    t_trap = L / V + V / AA
    t_jerk = L / V + V / AA + AA / J
    print(f"L={L} m: trapezoid {t_trap:.2f} s, S-curve(j=2) {t_jerk:.2f} s (+{(t_jerk/t_trap-1)*100:.1f} %)")

# --- §6 costmap update size
print("raytrace cells/scan:", 720 * int(12 / r), " inflation window cells:", round((24 + 2 * 1.2) ** 2 / r**2))

# --- §5.3 e2e latency budget: costmap period + BT period + plugin
for bt in (1.0, 5.0):
    print(f"BT {bt} Hz: e2e worst = 200 (costmap 5 Hz) + {1000/bt:.0f} (BT) + 400 (deadline) = {200+1000/bt+400:.0f} ms;"
          f" typical-worst with FULL p95 100 ms = {200+1000/bt+100:.0f} ms")

"""§4.2 tube: L-inf tube allows Euclidean deviation up to rho*sqrt(2); Euclidean (disk-stamped) tube keeps <= rho.
Cell counts of the Euclidean tube vs (2 L rho + pi rho^2)/r^2 and the L-inf upper bound (2 L rho + 4 rho^2)/r^2."""
import math
import numpy as np

r = 0.05


def path_cells(L, angle_deg):
    n = int(round(L / r))
    a = math.radians(angle_deg)
    return np.array([(round(k * math.cos(a)), round(k * math.sin(a))) for k in range(n + 1)])


def tube(P, rho, metric):
    k = int(math.ceil(rho / r))
    cells = set()
    for (x, y) in P:
        for i in range(-k, k + 1):
            for j in range(-k, k + 1):
                d = max(abs(i), abs(j)) if metric == "inf" else math.hypot(i, j)
                if d * r <= rho + 1e-9:
                    cells.add((x + i, y + j))
    return cells


def max_euclid_dev(cells, P):
    P = P.astype(float)
    worst = 0.0
    for (x, y) in cells:
        worst = max(worst, np.min(np.hypot(P[:, 0] - x, P[:, 1] - y)))
    return worst * r


for ang in (0, 45):
    P = path_cells(6.0, ang)
    for rho in (1.0, 3.0):
        ti, te = tube(P, rho, "inf"), tube(P, rho, "euc")
        dev_i = max_euclid_dev(ti, P) if rho == 1.0 else float("nan")
        dev_e = max_euclid_dev(te, P) if rho == 1.0 else float("nan")
        print(f"L=6 angle={ang:2d} rho={rho}: Linf cells={len(ti):6d} (max euclid dev {dev_i:.2f} m) | "
              f"Euclid cells={len(te):6d} (max dev {dev_e:.2f} m) | (2Lr+pi r^2)/r^2={(12*rho+math.pi*rho**2)/r**2:.0f} "
              f"(2Lr+4r^2)/r^2={(12*rho+4*rho**2)/r**2:.0f}")

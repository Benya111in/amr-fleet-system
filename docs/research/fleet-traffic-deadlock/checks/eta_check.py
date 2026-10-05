"""§3.2 ETA closed forms vs numerical integration of a jerk-limited (S-curve) profile.

Config values (config/robot_params.yaml limits): a = 1.0 m/s^2, j = 2.0 m/s^3,
omega_max = 1.5 rad/s, alpha_max = 2.0 rad/s^2.  velocity_profiler_node applies the jerk
limit (components.md §3.3), so the ETA model must include it.
"""
import math

A, J = 1.0, 2.0
W, ALPHA = 1.5, 2.0
DT = 1e-4


def scurve(v0, v1, a=A, j=J):
    """Simulate a jerk-limited velocity change v0 -> v1; return (time, distance)."""
    dv = abs(v1 - v0)
    sgn = 1.0 if v1 >= v0 else -1.0
    if dv >= a * a / j:
        t1 = a / j                   # jerk ramp
        t2 = dv / a - a / j          # constant accel
    else:
        t1 = math.sqrt(dv / j)
        t2 = 0.0
    T = 2 * t1 + t2
    t, v, acc, s = 0.0, v0, 0.0, 0.0
    while t < T - 1e-12:
        h = min(DT, T - t)
        if t < t1:
            jj = j
        elif t < t1 + t2:
            jj = 0.0
        else:
            jj = -j
        acc_new = acc + jj * h
        v_new = v + sgn * (acc + acc_new) / 2 * h
        s += (v + v_new) / 2 * h
        v, acc, t = v_new, acc_new, t + h
    return T, s, v


def closed_T(dv, a=A, j=J):
    return dv / a + a / j if dv >= a * a / j else 2 * math.sqrt(dv / j)


def main():
    vc, vj = 1.5, 0.5
    # 1) stop-and-go excess (decel to 0 + accel back), with and without jerk
    T, s, vend = scurve(vc, 0.0)
    excess_stop_sim = 2 * T - 2 * s / vc
    print(f"S-curve decel {vc}->0: T={T:.4f}s s={s:.4f}m (vend={vend:.2e}); "
          f"stop excess sim={excess_stop_sim:.4f}s closed v/a+a/j={vc/A + A/J:.4f}s "
          f"(trapezoid-only v/a={vc/A:.4f}s)")
    assert abs(excess_stop_sim - (vc / A + A / J)) < 1e-3
    # 2) junction transition vc->vj->vc excess
    T, s, _ = scurve(vc, vj)
    excess_j_sim = 2 * T - 2 * s / vc
    closed = closed_T(vc - vj) * (vc - vj) / vc
    print(f"junction {vc}->{vj}->{vc}: excess sim={excess_j_sim:.4f}s closed "
          f"(dv/a+a/j)*dv/vc={closed:.4f}s (trapezoid-only (vc-vj)^2/(a vc)={(vc-vj)**2/(A*vc):.4f}s)")
    assert abs(excess_j_sim - closed) < 1e-3
    # 3) small dv branch (dv < a^2/j = 0.5)
    T, s, _ = scurve(1.5, 1.2)
    ex = 2 * T - 2 * s / 1.5
    cl = closed_T(0.3) * 0.3 / 1.5
    print(f"junction 1.5->1.2->1.5 (dv<a^2/j): excess sim={ex:.4f} closed={cl:.4f}")
    assert abs(ex - cl) < 1e-3
    # 4) rotation: trapezoid in omega with alpha limit
    def trot(th):
        return th / W + W / ALPHA if th >= W * W / ALPHA else 2 * math.sqrt(th / ALPHA)
    for deg in (90, 180):
        th = math.radians(deg)
        print(f"rotation {deg} deg: |dth|/w={th/W:.3f}s  with alpha: {trot(th):.3f}s "
              f"(underestimate {100*(1-(th/W)/trot(th)):.0f} %)")
    # 5) worked examples in the brief
    print(f"10 m, cruise 1.5, no stop: {10/1.5:.3f}s ; old per-segment stop model "
          f"{10/1.5 + vc/A:.3f}s (+{100*(vc/A)/(10/1.5):.0f} %)")
    old = 20 / 1.5 + vc / A + math.radians(90) / W
    new = 20 / 1.5 + (vc / A + A / J) + trot(math.radians(90))
    print(f"20 m from rest + end stop + 90 deg: brief(rev2)={old:.2f}s  "
          f"with jerk+alpha={new:.2f}s  (rev2 under by {100*(new-old)/new:.1f} %)")
    # 6) minimum stop-to-stop distance to reach vc with jerk
    Tr = closed_T(vc)
    print(f"min stop-to-stop length to reach vc={vc}: vc*(vc/a+a/j)={vc*Tr:.2f} m "
          f"(trapezoid-only vc^2/a={vc*vc/A:.2f} m)")
    # 7) hold-vertex braking distance from v_h = 0.5 m/s
    T, s, _ = scurve(0.5, 0.0)
    print(f"brake 0.5->0 with jerk: {s:.3f} m (trapezoid v^2/2a={0.25/2:.3f} m)")


if __name__ == "__main__":
    main()


def audit_2026_09_22():
    """One-sided speed-limit change v1 -> v2 at a vertex (e.g. main lane 1.5 -> junction/aisle 0.5):
    the transition lies in the faster edge, excess = T_dv * dv / (2 v_fast)."""
    for v1, v2 in ((1.5, 0.5), (2.0, 1.5), (1.0, 0.5)):
        T, s, _ = scurve(v1, v2)
        ex_sim = T - s / max(v1, v2)
        cl = closed_T(abs(v1 - v2)) * abs(v1 - v2) / (2 * max(v1, v2))
        print(f"one-sided {v1}->{v2}: excess sim={ex_sim:.4f}s closed T_dv*dv/(2 v_fast)={cl:.4f}s")
        assert abs(ex_sim - cl) < 1e-3
    # worked example: rotation happens in place at the START (no interior stop)
    ex_start = 20 / 1.5 + (1.5 / A + A / J) + (math.radians(90) / W + W / ALPHA)
    ex_mid = ex_start + (1.5 / A + A / J)
    print(f"20 m, 90 deg in-place turn at start: {ex_start:.2f}s ; same turn at an interior vertex "
          f"(extra stop): {ex_mid:.2f}s")


if __name__ == "__main__":
    audit_2026_09_22()

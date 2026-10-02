"""Relation between C1 speed map and the project safety gate (config/robot_params.yaml safety.*).

safety_node: D = shortest distance footprint polygon -> obstacle (distance_reference: footprint_edge)
  D <= 0.3 : stop ; D <= 0.5 : v <= 0.2 ; D <= 1.0 : v <= 0.5 ;
  continuous: v_max(D) = -a t + sqrt((a t)^2 + 2 a (D - 0.3)), t = 0.15, a = 1.0 (applied D >= 1.0,
  zone cap wins if lower).
Geometry: for centre distance d_c, D in [d_c - r_circ, d_c - r_ins]  (footprint contains inscribed disk,
is contained in circumscribed disk).  So v_gate(D) <= v_gate(d_c - r_ins) if v_gate is non-decreasing.
"""
import math
from check_speedmap import d_lo, d_hi, v_lim, achievable_costs, VMAX

A, T, DSTOP = 1.0, 0.15, 0.30


def v_cont(D):
    if D <= DSTOP:
        return 0.0
    return min(VMAX, -A * T + math.sqrt((A * T) ** 2 + 2 * A * (D - DSTOP)))


def v_gate(D):
    if D <= 0.3:
        return 0.0
    if D <= 0.5:
        return min(0.2, v_cont(D))
    if D < 1.0:
        return min(0.5, v_cont(D))
    return v_cont(D)


if __name__ == "__main__":
    r = 0.20
    print("robot_params examples: v_cont(2.6)=%.3f v_cont(1.0)=%.3f v_cont(0.5)=%.3f" % (v_cont(2.6), v_cont(1.0), v_cont(0.5)))
    # monotonicity of v_gate
    Ds = [i * 0.001 for i in range(0, 4000)]
    mono = all(v_gate(Ds[k + 1]) >= v_gate(Ds[k]) - 1e-12 for k in range(len(Ds) - 1))
    print("v_gate non-decreasing on [0,4] m:", mono)
    for R, name in ((1.2, "A"), (2.2, "B")):
        worst = None
        for c, _ in achievable_costs(r, R).items():
            D_up = min(d_hi(c, r), R) - r      # largest possible edge distance for this cell (lateral)
            margin = v_lim(c, r) - v_gate(D_up)
            if worst is None or margin < worst[0]:
                worst = (margin, c, v_lim(c, r), v_gate(D_up))
        print(f"config {name}: min over achievable c of v_lim(c) - v_gate(d_hi-r_ins) = {worst[0]:.3f} (c={worst[1]}, v_lim={worst[2]:.2f}, v_gate={worst[3]:.2f})")
    print("\nbrake rule vs safety gate at lateral edge distance D:")
    for D in (0.1, 0.3, 0.5, 0.8, 1.0, 1.3, 2.0, 2.6):
        print(f"  D={D:.1f}: sqrt(2aD)={math.sqrt(2*A*D):.2f}  v_gate={v_gate(D):.2f}")
    # aisles, robot centred, walls both sides
    for w in (0.6, 1.2, 2.4, 3.0, 4.0):
        D = w / 2 - r
        print(f"  aisle {w:.1f} m centred: D={D:.2f} -> v_gate={v_gate(D):.2f} m/s")
    # T_pred bias if gate ignored in a 3 m aisle (v_lim says 2.0 at c=0)
    vg = v_gate(3.0 / 2 - r)
    print(f"\n3 m aisle, 20 m straight: time at v_lim=2.0: {20/2.0:.1f} s ; at gate {vg:.2f}: {20/vg:.1f} s ; under-prediction {(1-2.0**-1/vg**-1)*100:.0f} %")

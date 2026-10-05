"""C1 speed map checks (global-planning brief §2.2, §4.1).

Nav2 Humble inflation: c = floor(252*exp(-s*(d - r_ins))) for d > r_ins, d = hypot(i,j)*res
(cached_distances_ = hypot(i, j) in cells; see inflation_layer.cpp L369 / .hpp computeCost).
"""
import math

RES = 0.05
S = 2.0
A = 1.0
VMAX, VFLOOR = 2.0, 0.2


def cost(d, r_ins, s=S, R=1.2):
    if d == 0:
        return 254
    if d <= r_ins + 1e-12:
        return 253
    if d > R + 1e-12:
        return 0
    return int(252 * math.exp(-s * (d - r_ins)))


def d_lo(c, r_ins, s=S):
    return r_ins - math.log((c + 1) / 252) / s


def d_hi(c, r_ins, s=S):
    return r_ins - math.log(c / 252) / s


def v_lim(c, r_ins, s=S):
    if c == 0:
        return VMAX
    if c == 255:
        return VFLOOR
    dfree = d_lo(c, r_ins, s) - r_ins
    return min(max(math.sqrt(2 * A * max(dfree, 0.0)), VFLOOR), VMAX)


def achievable_costs(r_ins, R, s=S):
    out = {}
    n = int(R / RES) + 2
    for i in range(n):
        for j in range(i, n):
            d = math.hypot(i, j) * RES
            c = cost(d, r_ins, s, R)
            if 1 <= c <= 252:
                out.setdefault(c, d)
                out[c] = min(out[c], d)
    return out


if __name__ == "__main__":
    r = 0.20
    print("quantisation width ln((c+1)/c)/s at s=2:")
    for c in (1, 34, 186, 228):
        print(f"  c={c}: {math.log((c+1)/c)/S:.4f} m")

    print("\ntable (s=2, r_ins=0.20):  c | d_free (lo,hi] | v_lim | m=vmax/v | Smac k=1")
    for c in (34, 50, 100, 186, 206, 228, 240, 245, 248):
        lo, hi = d_lo(c, r) - r, d_hi(c, r) - r
        v = v_lim(c, r)
        print(f"  {c:3d} | ({lo:.3f}, {hi:.3f}] | {v:.2f} | {VMAX/v:.2f} | {1+c/252:.2f}")

    print("\ncost at d=1.2 (R_infl boundary):", cost(1.2, r))
    print("corridor centre d=0.30:", cost(0.30, r), " d=0.25:", cost(0.25, r))
    ach = achievable_costs(r, 1.2)
    hi_band = sorted(c for c in ach if c > 228)
    print("achievable c in (228,252] (grid distances hypot(i,j)*0.05):",
          [(c, round(ach[c], 4)) for c in hi_band])
    floor_cs = sorted(c for c in ach if v_lim(c, r) <= VFLOOR + 1e-12)
    print("achievable c where v_lim is clamped to v_floor:", floor_cs)

    # boundary discontinuity, config A
    t_in = 1 / v_lim(34, r)
    print(f"\nconfig A: s/m inside c=34 = {t_in:.4f}, outside = {1/VMAX:.4f}, drop = {(t_in-0.5)/t_in*100:.1f} %")
    # ΔT for radial excursion 1.2 -> r_circ and back (continuous rule sqrt(2(d-r)))
    rc = math.hypot(0.3, 0.2)
    one_way = (math.sqrt(2 * (1.2 - r)) - math.sqrt(2 * (rc - r))) - 0.5 * (1.2 - rc)
    print(f"  dT excursion to r_circ={rc:.3f}: {2*one_way:.3f} s  -> equivalent length {2*one_way*VMAX:.2f} m")

    # config B
    RB = r + VMAX**2 / (2 * A)
    cB = cost(RB, r, R=RB)
    print(f"\nconfig B: R_infl={RB:.2f}, boundary c={cB}, v_lim={v_lim(cB, r):.3f} (jump {(1 - v_lim(cB, r)/VMAX)*100:.1f} %)")

    # footprint_padding default 0.01 -> r_ins 0.21
    r2 = 0.21
    c2 = cost(0.30, r2)
    print(f"\nfootprint_padding=0.01 -> r_ins=0.21: corridor centre c={c2}, d_free_lo={d_lo(c2, r2)-r2:.3f}, v_lim={v_lim(c2, r2):.2f}")
    print(f"  r_circ padded = {math.hypot(0.31, 0.21):.4f}")

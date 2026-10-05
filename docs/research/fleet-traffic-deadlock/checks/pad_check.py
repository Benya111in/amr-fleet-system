"""§3.3 reservation geometry: pads, junction windows, blocks/headway, hold vertices.

Config (config/robot_params.yaml): L=0.60, W=0.40, a=1.0, j=2.0, v_max=2.0,
safety.emergency_stop_distance=0.30 (footprint_edge), safety.reaction_latency=0.15,
clearance limiter v_max(D) = -a t + sqrt((a t)^2 + 2 a (D - 0.3)).
Latency: fleet.yaml comm_latency_ms [0,100] (multi_robot.md §6) -> tau_c <= 0.1 s.
"""
import math

L, W = 0.60, 0.40
A, J, VMAX = 1.0, 2.0, 2.0
D_SAFE, T_REACT = 0.30, 0.15
EPS_LOC, TOL_XY = 0.08, 0.10
TAU_C, F_T, TAU_CTRL = 0.10, 10.0, 0.05


def main():
    # --- rev2 as written: delta_s applied on BOTH window ends -> separation 2*delta_s
    ds_rev2 = L + D_SAFE + VMAX * (TAU_C + TAU_CTRL) + EPS_LOC
    print(f"rev2 delta_s = {ds_rev2:.2f} m per window end -> ref-point separation "
          f"2*delta_s = {2*ds_rev2:.2f} m (body gap {2*ds_rev2 - L:.2f} m)")
    # --- corrected: total handoff separation Delta_r, split as Delta_r/2 per window end
    s_pad = L + D_SAFE + EPS_LOC
    tau_proto = 2 * TAU_C + 1 / F_T + TAU_CTRL
    print(f"s_pad = L+d_safe+eps = {s_pad:.2f} m ; tau_proto = 2tau_c+1/f_t+tau_ctrl = {tau_proto:.2f} s")
    for v in (0.5, 1.0, 1.5, 2.0):
        Dr = s_pad / v + tau_proto
        print(f"  v_r={v:.1f}: Delta_r={Dr:.3f}s  delta_t=Delta_r/2={Dr/2:.3f}s  "
              f"spatial sep={s_pad + v*tau_proto:.3f} m (critique req. 1.28 m at v_max) "
              f"| rev2 total 2*1.28/v={2*ds_rev2/v:.2f}s")
        # nose-to-tail margin actually left after body: Delta_r - L/v
        assert Dr - L / v > 0
    # Theorem 1 strictness with tau(exit)=clear time = t_out + (L/2+eps)/v:
    # need 2*delta_t = mu*Delta_r > (L/2+eps)/v  for all v -> mu > sup_v ratio
    ratios = [((L / 2 + EPS_LOC) / v) / (s_pad / v + tau_proto) for v in (0.05, 0.5, 1.0, 2.0)]
    print(f"Thm1 strictness: mu must exceed max ratio {max(ratios):.3f} (v->0 limit "
          f"{(L/2+EPS_LOC)/s_pad:.3f}); S4 sweep mu in {{0.5,1.0,1.5}} OK")
    # --- junction windows (ref-point traverse length l_j; body inside the pads)
    lj = 3.0
    for vj, rot in ((1.0, 0.0), (0.5, math.radians(90) / 1.5 + 1.5 / 2.0)):
        Dr = s_pad / vj + tau_proto
        win = rot + lj / vj + Dr
        occ = rot + (lj + L) / vj
        print(f"junction l_j={lj} v_j={vj} rot={rot:.2f}s: window={win:.2f}s  physical occupancy "
              f"(l_j+L)/v_j(+rot)={occ:.2f}s  margin={win-occ:.2f}s ; rev2 formula "
              f"{rot + (lj+L)/vj + 2*ds_rev2/vj:.2f}s")
    # --- headway / block length vs safety_node clearance limiter
    def d_hw(v):
        return D_SAFE + v * T_REACT + v * v / (2 * A)

    def vlim(D):
        return -A * T_REACT + math.sqrt((A * T_REACT) ** 2 + 2 * A * (D - D_SAFE))
    for v in (1.5, 2.0):
        print(f"d_hw({v}) = {d_hw(v):.3f} m ; safety limiter at D=d_hw -> {vlim(d_hw(v)):.3f} m/s")
    print(f"rev2 block rule l_b >= L + d_hw(1.5) = {L + d_hw(1.5):.2f} m ; corrected l_b >= d_hw(v_max) = "
          f"{d_hw(VMAX):.2f} m ; adopted 3.0 m")
    # --- hold vertex placement with jerk-limited braking from v_h=0.5
    vh = 0.5
    s_brake = vh * (vh / A + A / J) / 2 if vh >= A * A / J else vh * math.sqrt(vh / J)
    d_hold = s_brake + L / 2 + TOL_XY
    d_after = L / 2 + EPS_LOC + TOL_XY
    print(f"s_brake(0.5, jerk) = {s_brake:.3f} m -> d_hold >= {d_hold:.3f} m (rev2 used "
          f"{vh*vh/(2*A) + L/2 + TOL_XY:.3f}, adopted 0.6 -> too small) ; adopt 0.7")
    print(f"d_after >= L/2+eps+tol = {d_after:.2f} m")
    need = L + D_SAFE + 2 * (TOL_XY + EPS_LOC)
    print(f"two robots stopped on either side of one boundary: need d_after+d_hold >= {need:.2f} m ; "
          f"0.48+0.7={0.48+0.7:.2f} (fail), 0.6+0.7={1.3:.2f} (ok)")
    # gap between two robots stopped at consecutive hold vertices in 3 m blocks
    lb = 3.0
    gap = (lb - 0.7) + 0.7 - L - 2 * (TOL_XY + EPS_LOC)
    print(f"stopped at hold vertices of consecutive 3 m blocks: min body gap {gap:.2f} m")
    # --- wide corridor lanes
    wc = 3.0
    lane_sep = wc / 2
    print(f"3 m corridor, 2 lanes: lateral body gap {lane_sep - W:.2f} m ; wall gap {wc/4 - W/2:.2f} m "
          f"(warning zone 1.0 m, critical 0.5 m)")
    print(f"narrow aisle width W+0.2={W+0.2:.2f} m: side gap {0.1:.2f} m < e-stop {D_SAFE} m "
          f"(if zones are omnidirectional)")
    r_circ = math.hypot(L, W) / 2
    print(f"circumscribed radius {r_circ:.3f} m -> wait-for lateral bound W/2+r+m = {W/2 + r_circ + 0.2:.2f} m")


if __name__ == "__main__":
    main()


def audit_2026_09_22():
    """Audit additions: (1) junction atomic grant edge exit(p,r+) -> enter(i,J) keeps tau monotone;
    (2) approach section length and grant lead needed for a hold-free handoff."""
    s_pad = L + D_SAFE + EPS_LOC
    tau_proto = 2 * TAU_C + 1 / F_T + TAU_CTRL
    clr = L / 2 + EPS_LOC
    worst = 0.0
    for mu in (0.39, 0.5, 1.0, 1.5):
        for vJ in (0.5, 1.0, 1.5):
            for vr in (0.5, 1.0, 1.5, 2.0):
                dJ = mu * (s_pad / vJ + tau_proto) / 2
                dr = mu * (s_pad / vr + tau_proto) / 2
                # tau(enter(i,J)) - tau(exit(p,r+)) >= dJ + dr - clr/vr
                gap = dJ + dr - clr / vr
                if mu >= 0.5:
                    assert gap > 0, (mu, vJ, vr, gap)
                worst = min(worst, gap) if mu == 0.39 else worst
    print(f"junction atomic edge exit(p,r+)->enter(i,J): tau gap = d_J + d_r+ - (L/2+eps)/v_r+ > 0 for all "
          f"mu in {{0.5,1,1.5}}, v in {{0.5..2.0}} (holds whenever mu > 0.39)")

    def s_change(v0, v1):
        dv = abs(v0 - v1)
        T = dv / A + A / J if dv >= A * A / J else 2 * math.sqrt(dv / J)
        return (v0 + v1) / 2 * T
    vh, d_hold = 0.5, 0.7
    for vr in (1.0, 1.5, 2.0):
        s_app = s_change(vr, vh)
        lead_d = d_hold + s_app
        # grant for the follower arrives (L/2 + d_safe) before its reference point reaches the boundary
        have = L / 2 + D_SAFE
        extra = (lead_d - have) / vr
        print(f"v_r={vr}: approach section s({vr}->{vh})={s_app:.2f} m ; grant needed {lead_d:.2f} m "
              f"({lead_d/vr:.2f} s) before boundary for no slow-down ; tight handoff (Delta_r) gives "
              f"{have:.2f} m -> extra lead {extra:.2f} s")


if __name__ == "__main__":
    audit_2026_09_22()

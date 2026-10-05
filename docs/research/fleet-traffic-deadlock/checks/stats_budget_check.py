"""§6 statistics, §5.5 battery feasibility, §5.4 detection bound, §7.5 response-time budget."""
import math


def wilson(k, n, z=1.959964):
    p = k / n
    den = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - h) / den, (c + h) / den


def main():
    for k in (20, 19, 18):
        lo, hi = wilson(k, 20)
        print(f"Wilson 95% {k}/20: [{lo:.3f}, {hi:.3f}]")
    for lam in (30, 60, 120):
        for win in (10, 30):
            N = lam * win / 60
            print(f"lambda={lam}/h window={win} min: E[N]={N:.0f}, CV=1/sqrt(N)={1/math.sqrt(N):.2f}")
    # battery: full charge ~2 h continuous motion, 0->100 % in 30 min, b_low 25 %, b_min 15 %
    drain_move = 100 / 7200          # %/s while moving (upper bound: always moving)
    t_low = (100 - 25) / drain_move / 3600
    chg = 100 / 1800
    t_chg = (100 - 25) / chg / 60
    duty = (t_chg / 60) / (t_low + t_chg / 60)
    print(f"battery: 100->25 % in {t_low:.2f} h of motion; 25->100 % charge {t_chg:.1f} min; "
          f"worst-case charging duty {duty*100:.0f} % -> {5*duty:.2f} robots charging on average (3 chargers)")
    print(f"4 h run: >= {math.floor(4 / (t_low + t_chg/60))} charge cycles per always-moving robot")
    # detection bound: T_b + input decimation/tick + comm delay
    for Tb, ft, frs in ((2.0, 10, 10), (2.0, 2, 2), (5.0, 2, 2)):
        print(f"detection bound T_b={Tb} tick={1/ft:.1f}s input period={1/frs:.1f}s tau_c=0.1 -> "
              f"{Tb + 1/ft + 1/frs + 0.1:.1f} s")
    # response time budget (mean), comm U(0,100) -> mean 50 ms (multi_robot.md §6)
    stages = {"JSON+HRA": 3, "SIPP+grant": 5, "DDS": 5, "comm delay mean": 50,
              "NTP accept/BT tick": 15, "team A* first prefix (hyp.)": 50, "first cmd_vel mean (20 Hz)": 25,
              "velocity_profiler+safety (50 Hz) mean": 10}
    print("mean response budget:", sum(stages.values()), "ms ;", stages)
    worst = dict(stages)
    worst.update({"comm delay mean": 100, "first cmd_vel mean (20 Hz)": 50,
                  "velocity_profiler+safety (50 Hz) mean": 20})
    print("worst-case (100 ms delay):", sum(worst.values()), "ms")


if __name__ == "__main__":
    main()


def audit_2026_09_22():
    """Response-time budget with TWO delayed paths (assign_task and traffic/grant, each U(0,100) ms,
    both needed before motion) -> E[max(U1,U2)] = 2/3 * 100 ms."""
    import random
    rng = random.Random(0)
    mc = sum(max(rng.uniform(0, 100), rng.uniform(0, 100)) for _ in range(200000)) / 200000
    print(f"E[max(U1,U2)]: analytic {200/3:.1f} ms, Monte Carlo {mc:.1f} ms")
    stages = {"JSON+HRA": 3, "SIPP+grant": 5, "DDS": 5, "comm delay E[max]": 200 / 3,
              "NTP accept/BT tick": 15, "team A* first prefix (hyp.)": 50, "first cmd_vel mean (20 Hz)": 25,
              "velocity_profiler+safety (50 Hz) mean": 10}
    print(f"mean response budget with two delayed paths: {sum(stages.values()):.1f} ms (brief: ~180 ms)")
    # constant-load LinearBattery (Fortress 6.18: power_load only, no power_draining_topic/start_on_motion)
    t_cycle_min = 90 + 22.5
    print(f"battery constant load: every robot 100->25 % in 90 min regardless of motion; cycle {t_cycle_min} min; "
          f"charges in 240 min: {int((240 - 90) // t_cycle_min) + 1}; mean concurrent charging "
          f"{5 * 22.5 / t_cycle_min:.2f} of 3 chargers")


if __name__ == "__main__":
    audit_2026_09_22()

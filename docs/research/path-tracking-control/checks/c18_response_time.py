"""Audit-2 check (M6/SG6/RR8): spec 10장 table defines response time as
'작업 명령 발행 timestamp - 로봇 첫 움직임 timestamp'; sequences.md operationalises it as first cmd_vel != 0.
Rise of a jerk-limited start (j = 2 m/s^3, a0 = 0) and with the optional start-up step a0 = 0.3 m/s^2 to
several 'first motion' detection thresholds, plus the average contract budget (sequences.md §1)."""
import numpy as np
j = 2.0
def t_v(th, a0=0.0):           # v(t) = a0 t + j t^2/2
    return (-a0 + np.sqrt(a0*a0 + 2*j*th))/j
def t_x(th, a0=0.0):           # x(t) = a0 t^2/2 + j t^3/6 (bisection)
    lo, hi = 0.0, 2.0
    for _ in range(100):
        m = 0.5*(lo + hi)
        if a0*m*m/2 + j*m**3/6 < th: lo = m
        else: hi = m
    return hi
for th in [0.005, 0.01, 0.02]:
    print(f"GT v > {th} m/s: jerk-limited {1000*t_v(th):.0f} ms, with a0=0.3 {1000*t_v(th, 0.3):.0f} ms")
for th in [0.001, 0.005]:
    print(f"GT displacement > {1000*th:.0f} mm: jerk-limited {1000*t_x(th):.0f} ms, with a0=0.3 {1000*t_x(th, 0.3):.0f} ms")
avg = 20 + 50 + 10 + 50 + 50      # fleet + comm (uniform 0-100, mean 50) + BT + A* + controller first cycle
hops = 10 + 10                    # velocity_profiler_node + safety_node 50 Hz hops, mean
act = 20                          # DiffDrive actuation / bridge (assumed, part of Ta in c15)
print(f"contract budget (mean): {avg} ms + profiler/safety hops {hops} ms = {avg + hops} ms (first cmd_vel != 0)")
for th in [0.005, 0.02]:
    print(f"physical first motion (GT v > {th}): >= {avg + hops + act + 1000*t_v(th):.0f} ms mean "
          f"(event-driven hops: {avg + act + 1000*t_v(th):.0f} ms; with a0=0.3: {avg + act + 1000*t_v(th, 0.3):.0f} ms)")

#!/usr/bin/env python3
"""C++ 원본 출력(stdin)과 파이썬 기준 구현을 비교한다 — crosscheck_cpp.sh 가 부른다."""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amr_rl_contract import geometry as G  # noqa: E402
from amr_rl_contract import kinematics as K  # noqa: E402
from amr_rl_contract import mask as M  # noqa: E402
from amr_rl_contract import profiler as P  # noqa: E402
from amr_rl_contract import safety as S  # noqa: E402


def close(a, b, tol=1e-9):
    """절대·상대 1e-9."""
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def main():
    """줄마다 비교, 불일치 수를 종류별로 센다."""
    bad = {'vo': 0, 'dwa': 0, 'prof': 0, 'saf': 0}
    n = {'vo': 0, 'dwa': 0, 'prof': 0, 'saf': 0}
    prof = P.ProfilerReference()
    for line in sys.stdin:
        f = line.split()
        kind, x = f[0], [float(v) for v in f[1:]]
        n[kind] += 1
        ok = True
        if kind == 'vo':
            v, w, cx, cy, px, py, wx, wy, R, tau, t = x
            c = G.chord_velocity(0.0, v, w, 2.0)
            tp = G.vo_time((px, py), (wx, wy), R, tau)
            ok = close(c[0], cx) and close(c[1], cy) and (
                (t < 0 and math.isinf(tp)) or (t >= 0 and close(tp, t)))
        elif kind == 'dwa':
            v, w, T, n_poses, lx, ly, lth, vc, wc, vb, wb = x
            n_k = max(1, int(math.ceil(M.sim_time(v) / 0.1 - 1e-9)))
            ex, ey, eth = G.integrate_arc(v, w, n_k * 0.1)
            fb = K.fallback_command(vc, wc)
            ok = (close(M.sim_time(v), T) and int(n_poses) == n_k + 1 and close(ex, lx)
                  and close(ey, ly) and close(eth, lth) and close(fb[0], vb) and close(fb[1], wb))
        elif kind == 'prof':
            vt, wt, vr, wr, ar = x
            pv, pw = prof.step(vt, wt, 0.02)
            ok = close(pv, vr) and close(pw, wr) and close(prof.a_ref, ar)
        elif kind == 'saf':
            d, v, w, lim, u, zone = x
            ok = (close(S.clearance_speed_limit(d), lim) and close(S.max_point_speed(v, w), u)
                  and S.classify_zone(d) == int(zone))
        if not ok:
            bad[kind] += 1
            if bad[kind] <= 3:
                print('불일치', line.strip())
    for k in n:
        print(f'{k}: {n[k]} 건, 불일치 {bad[k]}')
    return 1 if any(bad.values()) else 0


if __name__ == '__main__':
    sys.exit(main())

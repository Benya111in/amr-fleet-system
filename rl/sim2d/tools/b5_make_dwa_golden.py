#!/usr/bin/env python3
"""
B5 2D-DWA golden 생성 — 바인딩 없이 코어를 직접 부른 값을 tests/golden_dwa2d.json 에 쓴다.

tests/test_dwa2d.py 의 합성 사례를 독립 실행 파일 dwa_binding/dwa_golden_main.cpp 로 계산한다.
코어나 배포 설정이 바뀌면 다시 만든다.
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D = os.path.dirname(HERE)
sys.path.insert(0, SIM2D)
sys.path.insert(0, os.path.join(SIM2D, 'tests'))

import test_dwa2d as T  # noqa: E402

from amr_sim2d import dwa2d  # noqa: E402


def main():
    m = dwa2d.module()
    conf = dwa2d.load_config()
    out = {}
    for kind in T.CASES:
        cells, res, ox, oy, poses, st = T._grid_case(kind)
        g = T._inflate(m, cells, res, conf)
        circ = m.inflation_cost(math.hypot(0.3, 0.2), conf['costmap']['inscribed_radius'],
                                conf['costmap']['cost_scaling_factor'])
        d = T._run_main(g, res, ox, oy, circ, poses, st)
        out[kind] = {k: ('%.17g' % float(v) if k in ('v', 'w', 'best_cost', 'd_goal')
                         else str(int(v))) for k, v in d.items()}
    with open(T.GOLDEN, 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()

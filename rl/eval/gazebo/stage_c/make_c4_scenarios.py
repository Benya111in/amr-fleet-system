#!/usr/bin/env python3
"""
C4 동적 장애물 스트레스 시나리오 — 시나리오 08 의 왕복 구간을 gzbench 실행기 형식으로.

결과 전에 커밋한다.

    make_c4_scenarios.py --planner-dir <amr_core_py 디렉터리> --out <scenarios_c4_s08.json> \
        [--trials 30]

시나리오 08 (`tests/integration/test_08_dynamic_obstacles.py`): A(−4, −5, −0.46) ↔ B(4, −9, 2.68) 왕복,
경로가 작업자 횡단선(worker_crossing, y = −7)과 지게차·셔틀 구역을 지난다. 여기서는:
  - 시행 k 가 짝수면 A → B, 홀수면 B → A (08 과 같은 교대). 시작 자세 = 08 의 점 (yaw 포함, 실행기가 순간 이동).
  - 목표 xy = 08 의 다른 점, 목표 yaw = 팀 A* 경로 끝 접선 (scenarios_v1 과 같은 규칙 — 08 은 다른 점의 yaw(되돌아보는 방향)
    를 줘서 목표에서 제자리 반 바퀴가 든다. 그 차이를 결과에 적는다).
  - 경로 길이 L = 팀 A* (scenarios_v1 과 같은 계획기·비용), 시간 상한 min(120, 20 + 3L) s sim (계약 §8; 08 은 90 s).
  - 실행기의 집합 검사 때문에 set 은 'main' 이다 (기록의 scenario_set 이름 gz_static_v1_main 은 물려받은 이름일 뿐 —
    C4 행은 run_tag 로 가른다). 끝 접선 직진 여유가 0.40 m 미만이면 실패한다 (그러면 08 점이 좁은 목표라는 뜻).
"""

import argparse
import datetime as dt
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GZ = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(GZ, '..', '..', '..'))
sys.path.insert(0, GZ)

import make_scenarios as MS  # noqa: E402

A = (-4.0, -5.0, -0.46)
B = (4.0, -9.0, 2.68)


def plan_leg(planner, gm, cost, world, s, g):
    """구간 하나 → 시나리오 필드 dict."""
    yaw = math.atan2(g[1] - s[1], g[0] - s[0])
    st, poses, _c, _e = planner.plan(cost, gm.res, gm.ox, gm.oy, float(s[0]), float(s[1]),
                                     float(g[0]), float(g[1]), yaw)
    if st != 'success':
        raise RuntimeError(f'계획 실패 {s} → {g}: {st}')
    p = np.asarray(poses)[:, :2]
    L = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    yaw_g = MS.tangent(p, True)
    straight = world.straight_approach(g[0], g[1], yaw_g)
    if straight < MS.STRAIGHT_MAIN:
        raise RuntimeError(f'목표 {g} 직진 여유 {straight:.3f} < {MS.STRAIGHT_MAIN}')
    return {
        'start': [s[0], s[1], s[2]],
        'goal': [g[0], g[1], round(yaw_g, 5)],
        'path_length_m': round(L, 4),
        'time_limit_s': round(min(120.0, 20.0 + 3.0 * L), 3),
        'goal_straight_m': round(straight, 4) if math.isfinite(straight) else None,
        'goal_point_clear_m': round(world.point_clearance(g[0], g[1]), 4),
        'goal_footprint_clear_m': round(world.footprint_clearance(g[0], g[1], yaw_g), 4),
        'start_footprint_clear_m': round(world.footprint_clearance(s[0], s[1], s[2]), 4),
        'path_end_gap_m': round(float(np.hypot(p[-1, 0] - g[0], p[-1, 1] - g[1])), 4),
        'path_ref': [[round(float(a), 3), round(float(b), 3)] for a, b in MS.resample(p)],
    }


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--planner-dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--trials', type=int, default=30)
    a = ap.parse_args()
    from amr_sim2d.gridmap import GridMap
    planner = MS.load_planner(a.planner_dir)
    gm = GridMap.from_yaml(os.path.join(REPO, 'maps', 'warehouse.yaml'))
    cost = gm.cost_grid()
    world = MS.world_geom.StaticWorld.from_sdf(
        os.path.join(REPO, 'src', 'amr_simulation', 'worlds', 'warehouse.sdf'),
        os.path.join(REPO, 'src', 'amr_simulation', 'models'))
    legs = {'AB': plan_leg(planner, gm, cost, world, A, B),
            'BA': plan_leg(planner, gm, cost, world, B, A)}
    scen = []
    for k in range(a.trials):
        leg = 'AB' if k % 2 == 0 else 'BA'
        sc = dict(legs[leg])
        sc.update({'id': f'c4_s08_t{k:02d}_{leg}', 'set': 'main', 'scenario_seed': 3000 + k,
                   'leg': leg})
        scen.append(sc)
    d = {'version': 'c4_s08_v1',
         'generated': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
         'generator': 'rl/eval/gazebo/stage_c/make_c4_scenarios.py',
         'source': 'tests/integration/test_08_dynamic_obstacles.py POINTS (A, B)',
         'rules': {'main_straight_min_m': MS.STRAIGHT_MAIN,
                   'time_limit': 'min(120, 20 + 3 L) s sim'},
         'world': ('src/amr_simulation/worlds/warehouse.sdf '
                   '(원본 — actor 6, forklift_main, shuttle_amr)'),
         'exec_order': [s['id'] for s in scen], 'smoke': [scen[0]['id'], scen[1]['id']],
         'scenarios': scen}
    with open(a.out, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=1)
    keys = ('goal', 'path_length_m', 'time_limit_s', 'goal_straight_m', 'path_end_gap_m')
    print(json.dumps({k: {kk: v[kk] for kk in keys} for k, v in legs.items()},
                     ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())

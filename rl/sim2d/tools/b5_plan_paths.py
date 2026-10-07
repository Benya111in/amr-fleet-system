#!/usr/bin/env python3
"""
B5 경로 — 평가 집합 행의 (시작, 목표, 목표 yaw) 를 팀 A* 코어 + 평활기로 다시 계획한다.

계획기 = dwa_binding/amr_core_py.plan (nav2_params.yaml AStar 설정: κ 2.0, 허용 0.25 m, 미지 252, 대각 모서리 금지,
숏컷·Gauss–Seidel 평활 0.05 m 간격) — Gazebo 의 Nav2 AStarPlanner 와 같은 코어.

- 계획 지도 = Isaac 기하의 0.05 m 점유 격자 (warehouse.sdf 사각형, pad 0.025 — Isaac 이 평가 경로 풀을 만든 격자와 같다,
  crosseval.export_world(res 0.05, pad 0.025) 가 그 격자를 다시 만든다는 것을 여기서 확인한다) + 전역 팽창 0.20/1.2/2.0
  (GridMap.cost_grid, 2D 경로 풀과 같은 함수). 배포 전역 코스트맵 해상도 0.05 m 와 같다.
- 시작 = Isaac 경로 첫 점, 목표 = 끝 점, 목표 yaw = 끝 접선 (끝 점과 0.25 m 앞 점, eval_sets.end_tangent_pose — Gazebo
  시나리오 v1 의 목표 yaw 와 같은 규칙).
- 이유: 과제 지시 ("경로 = 2D 의 A* 경로 (Gazebo 와 같은 계획기 코어)") 와 배포 충실도. Isaac 풀 경로는 Dijkstra +
  숏컷 근사(평활 없음)다. 배포 BT 는 정적 세계에서 경로가 유효한 한 다시 계획하지 않으므로 (navigate_to_pose.xml
  IsPathValid) 에피소드 동안 이 경로 하나를 쓴다. 두 팔에 같은 경로를 준다.
출력: --out npz (set, pid, status, n_pts, flat [M,3] (x, y, yaw), length, isaac_length, goal_yaw,
start, goal) + json 요약.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D = os.path.dirname(HERE)
RL = os.path.dirname(SIM2D)
sys.path.insert(0, SIM2D)
sys.path.insert(0, os.path.join(RL, 'isaac'))

from amr_sim2d.binding import core_module  # noqa: E402
from amr_sim2d.crosseval import export_world  # noqa: E402
from amr_sim2d.gridmap import GridMap  # noqa: E402
from amr_isaac import eval_sets as ES  # noqa: E402

SETS = ('main_3_15', 'stress_3_15', 'main_3_30', 'stress_3_30')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--eval-set', default=os.path.join(RL, 'isaac', 'eval_sets',
                                                       'b1_eval_sets_v1.json'))
    ap.add_argument('--pool', required=True)
    ap.add_argument('--geom', required=True)
    ap.add_argument('--world-dir', required=True, help='계획 지도 yaml 을 쓸 곳')
    ap.add_argument('--pool-rows', default='', help='스모크용: 집합 대신 풀 행 "lo:hi"')
    ap.add_argument('--threads', type=int, default=8)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    z = np.load(a.pool)
    pts, offs, length = z['pts'], z['offs'], z['length']
    if a.pool_rows:
        lo, hi = (int(x) for x in a.pool_rows.split(':'))
        rows = [('pool', i) for i in range(lo, hi)]
        digest = ES.pool_digest(pts, offs, length)
    else:
        doc = ES.load_sets(a.eval_set)
        digest = ES.check_pool(doc, pts, offs, length)
        rows = [(s, int(p)) for s in SETS for p in doc['sets'][s]]
    yml = export_world(a.geom, a.world_dir, res=0.05, pad=0.025, name='plan_isaac_occ_r050_p025')
    gm = GridMap.from_yaml(yml)
    g = np.load(a.geom)
    same = bool(np.array_equal(gm.solid.astype(bool), np.asarray(g['occ'], dtype=bool)))
    if not same:
        raise RuntimeError('다시 만든 0.05 m 격자가 Isaac occ 와 다르다')
    cost = gm.cost_grid()
    m = core_module()

    def plan(r):
        _, p = r
        P = pts[offs[p]:offs[p + 1]]
        gx, gy, gyaw = ES.end_tangent_pose(P)
        st, poses, c, e = m.plan(cost, gm.res, gm.ox, gm.oy, float(P[0, 0]), float(P[0, 1]),
                                 float(gx), float(gy), float(gyaw))
        return st, np.asarray(poses), float(gyaw), P
    with ThreadPoolExecutor(max_workers=a.threads) as ex:
        res = list(ex.map(plan, rows))
    status = np.array([r[0] for r in res])
    flat, npts, lens, ilens, gyaws, starts, goals, endpos = [], [], [], [], [], [], [], []
    for st, poses, gyaw, P in res:
        if st != 'success' or len(poses) < 2:
            poses = np.zeros((0, 3))
        flat.append(poses)
        npts.append(len(poses))
        seg = np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1) if len(poses) > 1 else [0]
        lens.append(float(np.sum(seg)))
        ilens.append(float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))))
        gyaws.append(gyaw)
        starts.append(P[0])
        goals.append(P[-1])
        endpos.append(poses[-1, :2] if len(poses) else [np.nan, np.nan])
    flat = np.concatenate(flat)
    ok = status == 'success'
    lens, ilens = np.array(lens), np.array(ilens)
    endpos, goals = np.array(endpos), np.array(goals)
    end_err = np.linalg.norm(endpos - goals, axis=1)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.savez_compressed(a.out, set=np.array([s for s, _ in rows]),
                        pid=np.array([p for _, p in rows]), status=status, n_pts=np.array(npts),
                        flat=flat, length=lens, isaac_length=ilens, goal_yaw=np.array(gyaws),
                        start=np.array(starts), goal=goals)
    summ = {'rows': len(rows), 'plan_success': int(ok.sum()), 'status_counts':
            {k: int((status == k).sum()) for k in np.unique(status)},
            'pool_digest': digest, 'plan_map': yml, 'plan_map_equals_isaac_occ': same,
            'length_ratio_q': [float(np.quantile(lens[ok] / ilens[ok], q))
                               for q in (0.01, 0.5, 0.99)],
            'end_point_err_max_m': float(np.nanmax(end_err[ok])) if ok.any() else None,
            'end_point_err_gt_0.01': int((end_err[ok] > 0.01).sum()),
            'flat_sha256': hashlib.sha256(np.ascontiguousarray(flat).tobytes()).hexdigest(),
            'by_set': {s: int(ok[np.array([r[0] for r in rows]) == s].sum()) for s in
                       dict.fromkeys(r[0] for r in rows)}}
    with open(a.out.replace('.npz', '.json'), 'w') as f:
        json.dump(summ, f, indent=1, ensure_ascii=False)
    print(json.dumps(summ, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()

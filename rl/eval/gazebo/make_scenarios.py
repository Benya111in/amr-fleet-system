#!/usr/bin/env python3
"""
Gazebo 정적 시나리오 집합 v1 생성 (보너스 1 A5 — 결과를 보기 전에 커밋한다).

    python3 rl/eval/gazebo/make_scenarios.py --planner-dir <amr_core_py 디렉터리> \
        --out rl/eval/gazebo/scenarios_v1.json

규칙 (rl/eval/gazebo/README.md §1, 계약 §10, 완성 계획 §6):
  - 지도: maps/warehouse.yaml (AMCL·전역 코스트맵과 같은 지도, map 프레임 = 월드). 점유 ∪ 미지 = 막힘.
  - 정적 기하: src/amr_simulation/worlds/warehouse.sdf 의 충돌 상자 (amr_rl_bench.world_geom —
    Isaac 과 같은 사각형).
  - 경로: 팀 A* 코어 (amr_core_py.plan — astar.cpp + path_smoother.cpp, κ 2.0, 대각 모서리 금지,
    허용 0.25 m) 를 전역 코스트맵 정적 팽창 (0.20/1.2/2.0, rl/sim2d gridmap.cost_grid 와 같다) 위에서.
    길이 L ∈ [3, 30] m.
    자기 근접(호길이 차 > 3 m 인 두 점이 1 m 안) 경로 제외 (계약 §14.5).
  - 시작: 막힘 경계와 ≥ 0.5 + 외접 반경 0.361 m (지도 EDT 와 충돌 상자 점 여유 둘 다) — 어떤 yaw 에서도 풋프린트가
    정적 장애물과 ≥ 0.5 m (계약 §10). yaw = 경로 시작 접선 + U[−0.5, 0.5] rad.
  - 목표: 막힘 경계와 ≥ 0.40 m (지도 EDT 와 충돌 상자 점 여유 둘 다, 계약 §10·2D·Isaac 와 같은 goal_clear).
    yaw = 경로 끝 접선 (마지막 점과 0.25 m 앞 점, A3 정의). 끝 접선 자세의 풋프린트가 상자와 겹치면 제외.
  - 집합: 끝 접선 자세의 **직진 접근 거리**(A3 straight_approach) ≥ 0.40 m 이면 주 집합 `main`, 아니면 좁은 목표
    스트레스 집합 `stress` (완성 계획 §6 2026-10-06 21:05 결정 + 21:55 정정 — 둘 다 같은 비중으로 보고).
  - 표집: 시작·목표 셀을 균일 추출(직선거리 [2.4, 30] m), 계획 성공·길이·자기 근접·여유 조건을 통과한 것을 생성 순서대로
    두 집합에 넣는다 (집합마다 n 개가 차면 그 집합은 더 받지 않는다). 시드 고정 — 다시 돌리면 같은 파일이 나온다.
  - 실행 순서: 두 집합을 번갈아 섞은 고정 순서 `exec_order` (단계 C 는 이 순서로 돌고 순번을 exec_index 로 남긴다),
    스모크 부분집합 `smoke` = 실행 순서의 처음 주 10 + 스트레스 10.
  - 시간 상한: min(120, 20 + 3·L) s (sim, 계약 §8).
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import datetime as dt
import hashlib
import importlib
import json
import math
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'rl', 'sim2d'))
sys.path.insert(0, os.path.join(REPO, 'src', 'amr_rl_bench'))

from amr_rl_bench import world_geom  # noqa: E402

VERSION = 'gz_static_v1'
L_MIN, L_MAX = 3.0, 30.0
START_CLEAR = 0.5 + world_geom.CIRCUMRADIUS
GOAL_CLEAR = 0.40
STRAIGHT_MAIN = 0.40
YAW_JITTER = 0.5
SELF_GAP, SELF_ARC = 1.0, 3.0
TANGENT_PTS = 5                # 0.05 m 간격 × 5 = 0.25 m (A3 끝 접선과 같다)


def sha256(path):
    """파일 해시 — SHA-256 앞 16 자."""
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def load_planner(planner_dir):
    """amr_core_py 를 planner_dir 에서 읽고 amr_sim2d.binding 이 그것을 쓰게 한다 (rl/sim2d 에 빌드하지 않는다)."""
    sys.path.insert(0, planner_dir)
    mod = importlib.import_module('amr_core_py')
    from amr_sim2d import binding
    binding._MOD = mod
    return mod


def self_proximity_ok(p, stride=5):
    """호길이 차 > 3 m 인 점 쌍이 1 m 안에 들어오지 않는가 (rl/sim2d paths.self_proximity_ok 와 같다)."""
    q = np.asarray(p)[::stride]
    if len(q) < 3:
        return True
    seg = np.linalg.norm(np.diff(q, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    d = np.linalg.norm(q[:, None, :] - q[None, :, :], axis=-1)
    far = np.abs(s[:, None] - s[None, :]) > SELF_ARC
    return not bool(np.any(far & (d < SELF_GAP)))


def tangent(p, at_end):
    """경로 끝(또는 시작) 접선 방향 [rad] — 0.25 m 떨어진 점과의 방향."""
    if at_end:
        a, b = p[max(0, len(p) - 1 - TANGENT_PTS)], p[-1]
    else:
        a, b = p[0], p[min(len(p) - 1, TANGENT_PTS)]
    return math.atan2(b[1] - a[1], b[0] - a[0])


def resample(p, step=0.25):
    """경로를 step 간격으로 (끝점 포함) — 참고용으로만 파일에 싣는다."""
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    q = np.arange(0.0, s[-1], step).tolist() + [s[-1]]
    return np.stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])], 1)


def generate(args):
    """시나리오 집합을 만든다 → dict."""
    from amr_sim2d.gridmap import GridMap, GLOBAL_INFLATION
    planner = load_planner(args.planner_dir)
    map_yaml = os.path.join(REPO, 'maps', 'warehouse.yaml')
    sdf = os.path.join(REPO, 'src', 'amr_simulation', 'worlds', 'warehouse.sdf')
    models = os.path.join(REPO, 'src', 'amr_simulation', 'models')
    gm = GridMap.from_yaml(map_yaml)
    cost = gm.cost_grid()
    world = world_geom.StaticWorld.from_sdf(sdf, models)
    edt = gm.edt
    sy, sx = np.where(edt >= START_CLEAR)
    gy, gx = np.where(edt >= GOAL_CLEAR)
    rng = np.random.default_rng(args.seed)
    sets = {'main': [], 'stress': []}
    stats = {'tries': 0, 'start_geom': 0, 'goal_geom': 0, 'plan_fail': 0, 'length': 0,
             'self_prox': 0, 'goal_footprint': 0, 'start_footprint': 0, 'valid_main': 0,
             'valid_stress': 0}
    want = {'main': args.n_main, 'stress': args.n_stress}

    def plan(sg):
        s, g = sg
        yaw = math.atan2(g[1] - s[1], g[0] - s[0])
        st, poses, _c, _e = planner.plan(cost, gm.res, gm.ox, gm.oy, float(s[0]), float(s[1]),
                                         float(g[0]), float(g[1]), yaw)
        return s, g, st, poses

    while (len(sets['main']) < want['main'] or len(sets['stress']) < want['stress']) \
            and stats['tries'] < args.max_tries:
        cand = []
        for _ in range(64):
            i = rng.integers(0, len(sx))
            s = np.array(gm.cell_center(sx[i], sy[i]), dtype=np.float64)
            for _k in range(50):
                j = rng.integers(0, len(gx))
                g = np.array(gm.cell_center(gx[j], gy[j]), dtype=np.float64)
                if 0.8 * L_MIN <= float(np.hypot(*(g - s))) <= L_MAX:
                    cand.append((s, g))
                    break
        # 지터는 후보마다 미리 뽑는다 — 거름 결과와 무관하게 난수열이 같게
        jit = rng.uniform(-YAW_JITTER, YAW_JITTER, len(cand))
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            res = list(ex.map(plan, cand))
        for (s, g, st, poses), dj in zip(res, jit):
            stats['tries'] += 1
            if world.point_clearance(*s) < START_CLEAR:
                stats['start_geom'] += 1
                continue
            if world.point_clearance(*g) < GOAL_CLEAR:
                stats['goal_geom'] += 1
                continue
            if st != 'success' or len(poses) < TANGENT_PTS + 2:
                stats['plan_fail'] += 1
                continue
            p = np.asarray(poses)[:, :2]
            L = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
            if not (L_MIN <= L <= L_MAX):
                stats['length'] += 1
                continue
            if not self_proximity_ok(p):
                stats['self_prox'] += 1
                continue
            yaw_g = tangent(p, True)
            gx_, gy_ = float(p[-1, 0]), float(p[-1, 1])
            fp_goal = world.footprint_clearance(gx_, gy_, yaw_g)
            if fp_goal <= 0.0:
                stats['goal_footprint'] += 1
                continue
            yaw_s = math.remainder(tangent(p, False) + float(dj), 2 * math.pi)
            fp_start = world.footprint_clearance(float(s[0]), float(s[1]), yaw_s)
            if fp_start < 0.5:
                stats['start_footprint'] += 1
                continue
            straight = world.straight_approach(gx_, gy_, yaw_g)
            which = 'main' if straight >= STRAIGHT_MAIN else 'stress'
            stats['valid_' + which] += 1       # 조건을 다 통과한 후보 (집합이 차서 버린 것 포함) — 좁은 목표 비율
            if len(sets[which]) >= want[which]:
                continue
            sets[which].append({
                'start': [round(float(s[0]), 4), round(float(s[1]), 4), round(yaw_s, 5)],
                'goal': [round(gx_, 4), round(gy_, 4), round(yaw_g, 5)],
                'path_length_m': round(L, 4),
                'time_limit_s': round(min(120.0, 20.0 + 3.0 * L), 3),
                'goal_straight_m': round(straight, 4) if math.isfinite(straight) else None,
                'goal_point_clear_m': round(world.point_clearance(gx_, gy_), 4),
                'goal_footprint_clear_m': round(fp_goal, 4),
                'start_footprint_clear_m': round(fp_start, 4),
                'goal_map_edt_m': round(float(edt[gm.world_to_cell(gx_, gy_)[::-1]]), 4),
                'path_ref': [[round(float(a), 3), round(float(b), 3)] for a, b in resample(p)],
            })
    for which, prefix in (('main', 'm'), ('stress', 's')):
        for k, sc in enumerate(sets[which]):
            sc['id'] = f'{VERSION}_{prefix}{k:03d}'
            sc['set'] = which
            sc['scenario_seed'] = (0 if which == 'main' else 1000) + k
    order_rng = np.random.default_rng(args.seed + 1)
    m_ids = [sc['id'] for sc in sets['main']]
    s_ids = [sc['id'] for sc in sets['stress']]
    m_perm = [m_ids[i] for i in order_rng.permutation(len(m_ids))]
    s_perm = [s_ids[i] for i in order_rng.permutation(len(s_ids))]
    order = []
    for k in range(max(len(m_perm), len(s_perm))):
        pair = [x[k] for x in (m_perm, s_perm) if k < len(x)]
        if len(pair) == 2 and order_rng.random() < 0.5:
            pair.reverse()
        order += pair
    smoke = m_perm[:10] + s_perm[:10]
    smoke = [i for i in order if i in set(smoke)]
    commit = subprocess.run(['git', '-C', REPO, 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    return {
        'version': VERSION,
        'generated': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
        'generator': 'rl/eval/gazebo/make_scenarios.py', 'generator_commit': commit,
        'seed': args.seed,
        'rules': {'path_length_m': [L_MIN, L_MAX], 'start_clear_m': START_CLEAR,
                  'goal_clear_m': GOAL_CLEAR, 'main_straight_min_m': STRAIGHT_MAIN,
                  'start_yaw_jitter_rad': YAW_JITTER, 'self_proximity': [SELF_GAP, SELF_ARC],
                  'tangent_m': 0.05 * TANGENT_PTS, 'inflation': GLOBAL_INFLATION,
                  'planner': 'amr_core_py.plan (astar.cpp + path_smoother.cpp), κ 2.0, tol 0.25',
                  'time_limit': 'min(120, 20 + 3 L) s sim'},
        'inputs': {'map_yaml': 'maps/warehouse.yaml',
                   'map_pgm_sha256': sha256(os.path.join(REPO, 'maps', 'warehouse.pgm')),
                   'world_sdf': 'src/amr_simulation/worlds/warehouse.sdf',
                   'world_sdf_sha256': sha256(sdf), 'n_static_rects': int(world.obb.shape[0])},
        'stats': stats,
        'counts': {k: len(v) for k, v in sets.items()},
        'exec_order': order,
        'smoke': smoke,
        'scenarios': sets['main'] + sets['stress'],
    }


def main():
    """명령행."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--planner-dir', required=True, help='amr_core_py*.so 가 있는 디렉터리')
    ap.add_argument('--out', default=os.path.join(HERE, 'scenarios_v1.json'))
    ap.add_argument('--seed', type=int, default=20261006)
    ap.add_argument('--n-main', type=int, default=200)
    ap.add_argument('--n-stress', type=int, default=200)
    ap.add_argument('--max-tries', type=int, default=200000)
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()
    out = generate(args)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=None, separators=(',', ':'))
        f.write('\n')
    print(json.dumps({'counts': out['counts'], 'stats': out['stats'], 'out': args.out},
                     ensure_ascii=False))


if __name__ == '__main__':
    main()

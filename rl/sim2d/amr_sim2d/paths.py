"""
A* 경로 풀 — 리셋마다 계획하는 대신 미리 계획한 (시작, 목표, 경로) 를 뽑는다 (contract.md §10).

계획기 = 팀 A* 코어 그대로 (dwa_binding/amr_core_py: astar.cpp + path_smoother.cpp, 비용 κ 2.0, 대각 모서리 금지,
허용 오차 0.25 m, 정적 팽창 0.20/1.2/2.0). 결정적이므로 같은 (시작, 목표) 의 경로는 리셋 때 계획한 것과 같다.

거르기:
  - 시작: 자유 셀, EDT ≥ 0.5 + 외접 반경 0.361 (풋프린트가 어떤 yaw 에서도 정적 장애물과 ≥ 0.5 m, §10)
  - 목표: 자유 셀, EDT ≥ goal_clear (기본 0.40)
  - 경로 길이 L ∈ [3, 30] m (§10), 계획 성공
  - 자기 근접: 호길이 차 > 3 m 인 두 점이 1 m 안 → 제외 (§14.5 전역 투영 불연속 방지)
캐시: /home/CAPYI_AI/runs/r2-sim2d/cache/paths_<지도>_<설정 해시>.npz
"""

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os

import numpy as np

from ._contract import C
from .binding import core_module

CACHE_DIR = os.environ.get('SIM2D_CACHE', '/runs/cache' if os.path.isdir('/runs')
                           else '/home/CAPYI_AI/runs/r2-sim2d/cache')


def self_proximity_ok(path, min_gap=1.0, min_arc=3.0, stride=5):
    """호길이 차 > min_arc 인 점 쌍이 min_gap 안에 들어오지 않는가."""
    p = np.asarray(path)[::stride]
    if len(p) < 3:
        return True
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=-1)
    far = np.abs(s[:, None] - s[None, :]) > min_arc
    return not bool(np.any(far & (d < min_gap)))


def build_pool(gm, n_paths=4096, seed=0, l_range=(3.0, 30.0), goal_clear=0.40,
               start_clear=0.5 + C.ROBOT_RADIUS, workers=4, max_tries_factor=6, log=print):
    """경로 풀 생성 → dict(start [n,2], goal [n,2], paths list[[k,2]], length [n])."""
    m = core_module()
    cost = gm.cost_grid()
    edt = gm.edt
    sy, sx = np.where(edt >= start_clear)
    gy, gx = np.where(edt >= goal_clear)
    rng = np.random.default_rng(seed)
    out = {'start': [], 'goal': [], 'paths': [], 'length': []}
    tries = 0
    stats = {'plan_fail': 0, 'length': 0, 'self_prox': 0}
    batch = max(64, workers * 32)
    while len(out['paths']) < n_paths and tries < n_paths * max_tries_factor:
        si = rng.integers(0, len(sx), batch)
        s_xy = np.stack(gm.cell_center(sx[si], sy[si]), 1)
        # 목표: 시작에서 직선 거리 [0.8·L_min, L_max] 인 자유 셀 (거절 표본)
        cand = []
        for k in range(batch):
            for _ in range(50):
                j = rng.integers(0, len(gx))
                g = np.array(gm.cell_center(gx[j], gy[j]))
                dd = np.linalg.norm(g - s_xy[k])
                if 0.8 * l_range[0] <= dd <= l_range[1]:
                    cand.append((s_xy[k], g))
                    break
        tries += len(cand)

        def plan(sg):
            s, g = sg
            yaw = math.atan2(g[1] - s[1], g[0] - s[0])
            st, poses, _c, _e = m.plan(cost, gm.res, gm.ox, gm.oy, float(s[0]), float(s[1]),
                                       float(g[0]), float(g[1]), yaw)
            return s, g, st, poses
        with ThreadPoolExecutor(max_workers=workers) as ex:
            res = list(ex.map(plan, cand))
        for s, g, st, poses in res:
            if st != 'success' or len(poses) < 2:
                stats['plan_fail'] += 1
                continue
            p = poses[:, :2]
            L = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
            if not (l_range[0] <= L <= l_range[1]):
                stats['length'] += 1
                continue
            if not self_proximity_ok(p):
                stats['self_prox'] += 1
                continue
            out['start'].append(s)
            out['goal'].append(p[-1])
            out['paths'].append(p.astype(np.float64))
            out['length'].append(L)
            if len(out['paths']) >= n_paths:
                break
        log(f'[paths] {gm.name}: {len(out["paths"])}/{n_paths} (시도 {tries}, 거름 {stats})')
    out['start'] = np.array(out['start'])
    out['goal'] = np.array(out['goal'])
    out['length'] = np.array(out['length'])
    out['stats'] = stats
    return out


def load_or_build(gm, n_paths=4096, seed=0, l_range=(3.0, 30.0), goal_clear=0.40, log=print,
                  **kw):
    """캐시가 있으면 읽고 없으면 만든다."""
    key = json.dumps({'map': gm.name, 'H': gm.H, 'W': gm.W, 'solid': int(gm.solid.sum()),
                      'n': n_paths, 'seed': seed, 'l': list(l_range), 'gc': goal_clear,
                      'v': 2}, sort_keys=True)
    h = hashlib.sha1(key.encode()).hexdigest()[:10]
    path = os.path.join(CACHE_DIR, f'paths_{gm.name}_{h}.npz')
    if os.path.exists(path):
        z = np.load(path, allow_pickle=False)
        lens = z['n_pts']
        flat = z['flat']
        offs = np.concatenate([[0], np.cumsum(lens)])
        paths = [flat[offs[i]:offs[i + 1]] for i in range(len(lens))]
        return {'start': z['start'], 'goal': z['goal'], 'paths': paths, 'length': z['length'],
                'cache': path}
    pool = build_pool(gm, n_paths, seed, l_range, goal_clear, log=log, **kw)
    os.makedirs(CACHE_DIR, exist_ok=True)
    np.savez_compressed(path, start=pool['start'], goal=pool['goal'], length=pool['length'],
                        n_pts=np.array([len(p) for p in pool['paths']]),
                        flat=np.concatenate(pool['paths']), key=key)
    pool['cache'] = path
    return pool

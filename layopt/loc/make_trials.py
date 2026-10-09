"""
E2 시행 목록 만들기.

    # 파일럿 (SLAM 지도, 묶음 없음)
    python3 -m layopt.loc.make_trials smoke --out <file>
    # 실험 1 (기준 묶음 + E0 자유 공간 표본 → 5분위 층화)
    python3 -m layopt.loc.make_trials exp1 --bundle <dir> --out layopt/loc/exp1_trials.json

시행 = (자세, k) 한 번. neighbors 는 가까운 순 이웃 자세 목록이고 시행은 앞에서 k 개를 쓴다 (E0 과 같다).
"""

import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Dict

import numpy as np

from layopt.loc import e0, mapio

SCHEMA = 'layopt-e2-trials/1'


def _trial(order: int, pose_id: str, kind: str, pose, k: int, neighbors, extra=None) -> Dict:
    t = {'trial_id': f'{pose_id.replace(":", "_")}_k{k}', 'order': order, 'pose_id': pose_id,
         'kind': kind, 'x': round(float(pose[0]), 3), 'y': round(float(pose[1]), 3),
         'yaw': round(float(pose[2]), 6), 'k': int(k),
         'neighbors': [[round(float(v), 6) for v in n] for n in neighbors]}
    if extra:
        t.update(extra)
    return t


def smoke() -> Dict:
    """SLAM 지도 파일럿: 대기 슬롯 3곳 × k, 자유 공간 2곳."""
    lay = e0.slam_waiting_layout()
    dps = {d['id']: d for d in lay['dwell_points']}
    rows = []
    for pid, k in [('wait:amr_03', 2), ('wait:amr_05', 0), ('wait:amr_01', 4),
                   ('wait:amr_02', 2)]:
        d = dps[pid]
        rows.append((pid, 'waiting', (d['x'], d['y'], d['yaw']), k, d['neighbors']))
    m = mapio.load_map(mapio.REPO / 'maps' / 'warehouse.yaml')
    for pid, pose, k in [('free:p0', (12.0, 6.0, math.pi / 2), 2),
                         ('free:p1', (-12.0, -6.0, 0.0), 0)]:
        rows.append((pid, 'free', pose, k, e0.free_space_neighbors(m, pose)))
    trials = [_trial(i, pid, kind, pose, k, nbs) for i, (pid, kind, pose, k, nbs)
              in enumerate(rows)]
    return {'schema': SCHEMA, 'name': 'pilot-slam', 'map_kind': 'slam',
            'world': str(mapio.REPO / 'src' / 'amr_simulation' / 'worlds' / 'warehouse.sdf'),
            'map': str(mapio.REPO / 'maps' / 'warehouse.yaml'),
            'created': dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'trials': trials}


# ------------------------------------------------------------------ 실험 1
EXP1_SEED = 20261010
EXP1_TARGET = 240          # 시행 목표 (prereg §1: 최소 150, 목표 240)
EXP1_POOL = 400            # 자유 공간 후보 자세 수 (E0 k=0 마진으로 5분위)
K_LEVELS = (0, 2, 4)


def exp1(bundle: Path, seed: int = EXP1_SEED, target: int = EXP1_TARGET,
         pool_n: int = EXP1_POOL) -> Dict:
    """
    실험 1 시행 목록 (사전 등록 — 결과를 보기 전에 커밋한다).

    자세: 묶음 layout.json 의 체류 지점 전부(waiting · dock · charger · queue, 이웃 = layout 의 neighbors),
    그리고 자유 공간 자세를 E0 k=0 마진 5분위에서 같은 수씩 (이웃 = e0.free_space_neighbors, 4 자리 모두
    비어야 후보). 각 자세 × k ∈ {0, 2, 4} 한 번씩, 순서는 시드로 섞는다. 시행마다 같은 자세·k·이웃의 E0
    값(margin, inlier, inlier_raw, occluded_frac, second_dist)을 함께 적는다 (분석이 이것과 잇는다).
    """
    rng = np.random.default_rng(seed)
    layout = json.loads((bundle / 'layout.json').read_text(encoding='utf-8'))
    m = mapio.load_map(bundle / 'map.yaml')
    field = mapio.distance_field(m)
    poses = []        # (pose_id, kind, pose, neighbors, extra)
    for dp in layout['dwell_points']:
        poses.append((dp['id'], dp['kind'], (dp['x'], dp['y'], dp['yaw']),
                      dp.get('neighbors', []), {}))
    n_free = max(int(math.ceil(target / len(K_LEVELS))) - len(poses), 5)
    per_q = int(math.ceil(n_free / 5))
    # 자유 공간 후보: 4 이웃 자리가 다 비고, 체류 지점에서 1.5 m 밖
    pool = []
    dwell_xy = np.array([[p[2][0], p[2][1]] for p in poses]) if poses else np.zeros((0, 2))
    for pose in e0.sample_free_poses(field, pool_n, seed, bounds=layout.get('bounds')):
        if dwell_xy.size and np.min(np.hypot(dwell_xy[:, 0] - pose[0],
                                             dwell_xy[:, 1] - pose[1])) < 1.5:
            continue
        nbs = e0.free_space_neighbors(m, pose)
        if len(nbs) < 4:
            continue
        r = e0.evaluate_pose(field, m.occ, pose, ())
        if r['margin'] is None:
            continue
        pool.append((pose, nbs, r['margin']))
    margins = np.array([q[2] for q in pool])
    edges = np.quantile(margins, [0.2, 0.4, 0.6, 0.8])
    quint = np.searchsorted(edges, margins, side='right')
    for q in range(5):
        idx = np.nonzero(quint == q)[0]
        pick = rng.choice(idx, size=min(per_q, idx.size), replace=False)
        for j, i in enumerate(sorted(pick.tolist())):
            pose, nbs, mg = pool[i]
            poses.append((f'free:q{q}_{j:02d}', 'free', pose, nbs,
                          {'quintile': q, 'pool_margin_k0': mg}))
    rows = []
    for pid, kind, pose, nbs, extra in poses:
        for k in K_LEVELS:
            used = nbs[:k]
            r = e0.evaluate_pose(field, m.occ, pose, used)
            e0v = {key: r[key] for key in ('margin', 'inlier', 'inlier_raw', 'occluded_frac',
                                           'rho_true', 'second', 'second_rho', 'second_dist',
                                           'truth_rank')}
            rows.append((pid, kind, pose, k, nbs, {**extra, 'k_used': len(used), 'e0': e0v}))
    order = rng.permutation(len(rows))
    trials = []
    for o, i in enumerate(order.tolist()):
        pid, kind, pose, k, nbs, extra = rows[i]
        trials.append(_trial(o, pid, kind, pose, k, nbs, extra))
    trials.sort(key=lambda t: t['order'])
    return {'schema': SCHEMA, 'name': 'exp1', 'map_kind': 'bundle-raster',
            'spec_id': layout.get('spec_id', bundle.name), 'bundle': str(bundle),
            'world': str(bundle / 'world.sdf'), 'map': str(bundle / 'map.yaml'),
            'map_sha256': m.pgm_sha256,
            'world_sha256': hashlib.sha256((bundle / 'world.sdf').read_bytes()).hexdigest(),
            'seed': seed, 'target': target, 'pool_n': pool_n, 'pool_kept': len(pool),
            'quintile_edges': [round(float(v), 4) for v in edges],
            'n_dwell_poses': sum(1 for p in poses if p[1] != 'free'),
            'n_free_poses': sum(1 for p in poses if p[1] == 'free'),
            'k_levels': list(K_LEVELS),
            'e0_params': {'beams': 720, 'noise': 0.03, 'scan_seed': 0, 'inlier_dist': 0.2},
            'created': dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'trials': trials}


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('smoke')
    s.add_argument('--out', required=True)
    x = sub.add_parser('exp1')
    x.add_argument('--bundle', required=True)
    x.add_argument('--out', required=True)
    x.add_argument('--seed', type=int, default=EXP1_SEED)
    x.add_argument('--target', type=int, default=EXP1_TARGET)
    x.add_argument('--pool', type=int, default=EXP1_POOL)
    a = ap.parse_args(argv)
    if a.cmd == 'smoke':
        spec = smoke()
    else:
        spec = exp1(Path(a.bundle), a.seed, a.target, a.pool)
    out = Path(a.out)
    if out.exists():
        print(f'이미 있다: {out}', file=sys.stderr)
        return 3
    out.write_text(json.dumps(spec, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'{out}: {len(spec["trials"])} trials')
    return 0


if __name__ == '__main__':
    sys.exit(main())

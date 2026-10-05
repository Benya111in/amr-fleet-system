#!/usr/bin/env python3
"""
창고 기하·경로 풀 캐시를 만든다 (Isaac 불필요, numpy·scipy).

    python3 rl/isaac/scripts/build_assets.py [--out /home/CAPYI_AI/runs/r1-isaac/assets]
        [--goals 256] [--starts 16] [--seed 0] [--workers 4]

산출 (커밋하지 않는다): <out>/warehouse_<sha>.npz (메시 정점·삼각형, 2D 사각형, 점유 격자),
<out>/paths_<sha>_s<seed>.npz (경로 풀). sha = warehouse.sdf + 이 모듈 소스의 해시 앞 10 자리.
"""

import argparse
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from amr_isaac import paths as PA  # noqa: E402
from amr_isaac.assets import geometry_sha  # noqa: E402
from amr_isaac import warehouse_geom as W  # noqa: E402


def _job(args):
    occ, origin, n_goals, starts, seed = args
    return PA.build_pool(occ, origin, n_goals=n_goals, starts_per_goal=starts, seed=seed,
                         log=None)


def main():
    """캐시를 만든다 (있으면 건너뛴다)."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/home/CAPYI_AI/runs/r1-isaac/assets')
    ap.add_argument('--goals', type=int, default=256)
    ap.add_argument('--starts', type=int, default=16)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--workers', type=int, default=4)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    sha = geometry_sha()
    geo_p = os.path.join(a.out, f'warehouse_{sha}.npz')
    boxes = W.load_boxes()
    obb = W.obstacles_2d(boxes)
    occ, origin, res = W.rasterize(obb)
    if not os.path.exists(geo_p):
        v, t = W.triangle_mesh(boxes)
        np.savez_compressed(geo_p, verts=v, tris=t, obb=obb, occ=occ,
                            origin=np.asarray(origin), res=res,
                            names=np.asarray([b.name for b in boxes]))
    print(f'geometry {geo_p}: boxes {len(boxes)}, 2d {len(obb)}, occ {occ.shape}')
    pool_p = os.path.join(a.out, f'paths_{sha}_s{a.seed}.npz')
    if os.path.exists(pool_p):
        print(f'pool exists {pool_p}')
        return
    t0 = time.time()
    per = a.goals // a.workers
    jobs = [(occ, origin, per, a.starts, a.seed * 1000 + k) for k in range(a.workers)]
    with Pool(a.workers) as p:
        parts = p.map(_job, jobs)
    pts, offs, lens, meta = [], [0], [], []
    for r in parts:
        base = offs[-1]
        pts.append(r['pts'])
        offs += list(base + r['offs'][1:])
        lens.append(r['length'])
        meta.append(r['meta'])
    lens = np.concatenate(lens)
    np.savez_compressed(pool_p, pts=np.concatenate(pts), offs=np.asarray(offs, dtype=np.int64),
                        length=lens, meta=np.concatenate(meta), origin=np.asarray(origin),
                        res=res)
    info = {'paths': int(len(lens)), 'L_mean': float(lens.mean()), 'L_min': float(lens.min()),
            'L_max': float(lens.max()), 'max_points': int(np.max(np.diff(offs))),
            'seconds': round(time.time() - t0, 1), 'sha': sha}
    with open(pool_p.replace('.npz', '.json'), 'w') as f:
        json.dump(info, f, indent=1)
    print(json.dumps(info))


if __name__ == '__main__':
    main()

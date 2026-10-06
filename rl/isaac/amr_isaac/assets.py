"""
창고 기하·경로 풀 캐시 (생성물은 커밋하지 않는다 — 기본 위치 $AMR_ISAAC_RUNS/assets).

AMR_ISAAC_RUNS 기본값은 /home/CAPYI_AI/runs/r7-isaac12 (1차 물결 캐시는 /home/CAPYI_AI/runs/r1-isaac/assets).

캐시 키 = warehouse.sdf·모델 SDF·변환기·경로 모듈 소스의 sha256 앞 10 자리. 소스가 바뀌면 새로 만든다.
"""

import hashlib
import json
import os
import time

import numpy as np

from . import paths as PA
from . import warehouse_geom as W

RUNS = os.environ.get('AMR_ISAAC_RUNS', '/home/CAPYI_AI/runs/r7-isaac12')
DEFAULT_DIR = os.path.join(RUNS, 'assets')


def geometry_sha():
    """창고 SDF·모델·변환기·경로 모듈 소스의 해시."""
    sdf, models = W.default_paths()
    h = hashlib.sha256()
    for p in [sdf, W.__file__, PA.__file__] + sorted(
            os.path.join(models, m, 'model.sdf') for m in os.listdir(models)):
        if os.path.exists(p):
            with open(p, 'rb') as f:
                h.update(f.read())
    return h.hexdigest()[:10]


def load_geometry(out_dir=DEFAULT_DIR):
    """기하 캐시 (없으면 만든다) → dict(verts, tris, obb, occ, origin, res, sha)."""
    os.makedirs(out_dir, exist_ok=True)
    sha = geometry_sha()
    p = os.path.join(out_dir, f'warehouse_{sha}.npz')
    if not os.path.exists(p):
        boxes = W.load_boxes()
        obb = W.obstacles_2d(boxes)
        occ, origin, res = W.rasterize(obb)
        v, t = W.triangle_mesh(boxes)
        tmp = p + f'.{os.getpid()}.tmp.npz'
        np.savez_compressed(tmp, verts=v, tris=t, obb=obb, occ=occ, origin=np.asarray(origin),
                            res=res, names=np.asarray([b.name for b in boxes]))
        os.replace(tmp, p)
    d = dict(np.load(p))
    d['sha'] = sha
    d['path'] = p
    return d


def load_pool(seed=0, out_dir=DEFAULT_DIR, n_goals=512, starts=16, log=print):
    """경로 풀 캐시 (없으면 한 프로세스로 만든다 — 수 분) → dict(pts, offs, length, …)."""
    geo = load_geometry(out_dir)
    p = os.path.join(out_dir, f'paths_{geo["sha"]}_s{seed}.npz')
    if not os.path.exists(p):
        t0 = time.time()
        pool = PA.build_pool(geo['occ'], tuple(geo['origin']), n_goals=n_goals,
                             starts_per_goal=starts, seed=seed * 1000, log=log)
        tmp = p + f'.{os.getpid()}.tmp.npz'
        np.savez_compressed(tmp, **pool)
        os.replace(tmp, p)
        with open(p.replace('.npz', '.json'), 'w') as f:
            json.dump({'paths': int(len(pool['length'])), 'seconds': round(time.time() - t0, 1),
                       'sha': geo['sha']}, f)
    d = dict(np.load(p))
    d['path'] = p
    return d

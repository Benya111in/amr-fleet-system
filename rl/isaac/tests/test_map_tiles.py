"""
타일 지도 env 의 순수 부분 시험.

env 별 충돌 판정이 기본 판정과 비트 단위로 같은지, 관측이 타일 이동에 불변인지,
eval_mapgen.py 가 eval_unbiased.py 에서 옮긴 함수가 같은지.
"""

import ast
import math
import os

import numpy as np
import torch

from amr_isaac import contract_torch as CT
from amr_isaac import env_math as EM
from amr_isaac import map_tiles as MT
from amr_isaac import mapgen as MG

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.join(HERE, '..', 'scripts')
F64 = torch.float64


def _poses_near(obb, n, rng):
    """사각형 경계 근처 자세 (판정이 갈리는 곳)."""
    i = rng.integers(0, obb.shape[0], n)
    ang = rng.uniform(-math.pi, math.pi, n)
    r = np.hypot(obb[i, 2], obb[i, 3]) + rng.uniform(-0.2, 0.6, n)
    return np.stack([obb[i, 0] + r * np.cos(ang), obb[i, 1] + r * np.sin(ang),
                     rng.uniform(-math.pi, math.pi, n)], 1)


def test_per_env_hits_equal_shared():
    nom = MG.nominal_layout()['obb']
    rng = np.random.default_rng(0)
    pose = torch.tensor(_poses_near(nom, 6000, rng), dtype=F64)
    obb = torch.tensor(nom, dtype=F64)
    ref = EM.footprint_hits(pose, obb)
    tiles = MT.padded_tile_rects([nom, np.concatenate([nom, nom[:7]])], 'cpu')
    got = MT.footprint_hits_per_env(pose, tiles[torch.zeros(6000, dtype=torch.long)])
    assert torch.equal(ref, got)
    assert 300 < int(ref.sum()) < 5700          # 경계 근처라 양쪽이 다 나온다


def test_per_env_hits_use_own_tile():
    a = np.array([[0.0, 0.0, 0.5, 0.5, 0.0]])
    b = np.array([[10.0, 0.0, 0.5, 0.5, 0.0], [20.0, 0.0, 0.5, 0.5, 0.0]])
    tiles = MT.padded_tile_rects([a, b], 'cpu')
    pose = torch.tensor([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [10.0, 0.0, 0.3], [10.0, 0.0, 0.3]],
                        dtype=F64)
    tid = torch.tensor([0, 1, 0, 1])
    hit = MT.footprint_hits_per_env(pose, tiles[tid])
    assert hit.tolist() == [True, False, False, True]
    assert MT.pad_is_far(np.concatenate([a, b]))


def test_move_and_collide_per_env_equal():
    nom = MG.nominal_layout()['obb']
    rng = np.random.default_rng(1)
    n = 3000
    pose = torch.tensor(_poses_near(nom, n, rng), dtype=F64)
    v = torch.tensor(rng.uniform(-1.0, 1.0, n), dtype=F64)
    w = torch.tensor(rng.uniform(-2.0, 2.0, n), dtype=F64)
    col = torch.tensor(rng.random(n) < 0.1)
    ref = EM.move_and_collide(pose, v, w, col, torch.tensor(nom, dtype=F64), 0.02)
    tiles = MT.padded_tile_rects([nom], 'cpu')
    got = MT.move_and_collide_per_env(pose, v, w, col, tiles[torch.zeros(n, dtype=torch.long)],
                                      0.02)
    for a, b in zip(ref, got):
        assert torch.equal(a, b)


def _state(rng, B=6, P=640):
    pts = np.cumsum(rng.normal(0, 0.05, (B, P, 2)), axis=1) + rng.uniform(-10, 10, (B, 1, 2))
    z = lambda: torch.zeros(B, dtype=F64)  # noqa: E731
    scan = torch.tensor(rng.uniform(0.2, 20.0, (B, 720)), dtype=F64)
    return CT.BatchState(
        pose=torch.tensor(np.concatenate([pts[:, 3] + rng.normal(0, 0.2, (B, 2)),
                                          rng.uniform(-3, 3, (B, 1))], 1), dtype=F64),
        path_pts=torch.tensor(pts, dtype=F64), path_n=torch.full((B,), 400, dtype=torch.long),
        scan=scan, v_meas=z() + 0.4, w_meas=z() + 0.1, a_v_meas=z(), a_w_meas=z(),
        v_cmd=z() + 0.5, w_cmd=z(), prev_actions=torch.zeros(B, 3, 2, dtype=F64),
        v_cap=z() + 1.0, scan_age=z(), track_age=z(),
        tracks=torch.zeros(B, 8, 6, dtype=F64), track_id=torch.zeros(B, 8, dtype=torch.long),
        zone=torch.zeros(B, dtype=torch.long), speed_limit=torch.full((B,), math.inf, dtype=F64),
        proximity_stop=torch.zeros(B, dtype=torch.bool),
        min_ttc=torch.full((B,), math.inf, dtype=F64), in_dev=torch.zeros(B, dtype=torch.bool),
        dev_peak=z(), dev_t=z())


def test_observation_translation_invariant():
    rng = np.random.default_rng(2)
    st = _state(rng)
    o0 = CT.build_observation(st)
    for off in ((-264.0, -138.0), (198.0, 138.0)):
        sh = CT.BatchState(**{k: getattr(st, k).clone() if getattr(st, k) is not None else None
                              for k in st.__dataclass_fields__})
        sh.pose[:, 0] += off[0]
        sh.pose[:, 1] += off[1]
        sh.path_pts[..., 0] += off[0]
        sh.path_pts[..., 1] += off[1]
        o1 = CT.build_observation(sh)
        assert float((o0 - o1).abs().max()) <= 1e-5


def _funcs(path):
    tree = ast.parse(open(path, encoding='utf-8').read())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef):
            if node.body and isinstance(node.body[0], ast.Expr) and \
                    isinstance(getattr(node.body[0], 'value', None), ast.Constant):
                node.body = node.body[1:]          # 독스트링은 비교하지 않는다
            out[node.name] = ast.dump(node)
    return out


def test_copied_eval_functions_unchanged():
    a = _funcs(os.path.join(SCRIPTS, 'eval_unbiased.py'))
    b = _funcs(os.path.join(SCRIPTS, 'eval_mapgen.py'))
    for name in ('summarize', 'interval_block', 'run_first_episodes'):
        assert a[name] == b[name], name

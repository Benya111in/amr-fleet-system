"""충돌 판정과 스윕 접근 거리 — 독립 기준(일반 볼록 다각형 SAT, C++ 원본 바인딩)과 대조."""

import math

import numpy as np
import pytest
import torch

from amr_sim2d import geometry2d as G
from amr_sim2d._contract import C
from amr_sim2d.gridmap import GridMap, stack_maps

from amr_rl_contract import geometry as RG

DEVICES = ['cpu'] + (['cuda'] if torch.cuda.is_available() else [])


def rect_poly(x, y, th, x0, x1, y0, y1):
    c, s = math.cos(th), math.sin(th)
    loc = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return np.array([(x + c * a - s * b, y + s * a + c * b) for a, b in loc])


def poly_overlap(p, q, eps=1e-9):
    """일반 볼록 다각형 SAT (경계 접촉 = 겹침) — 시험용 독립 기준."""
    for poly in (p, q):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            nrm = np.array([-e[1], e[0]])
            a, b = p @ nrm, q @ nrm
            if a.max() < b.min() - eps or b.max() < a.min() - eps:
                return False
    return True


def cluster_map():
    gm = GridMap(np.zeros((60, 60), dtype=np.uint8), 0.05, (0.0, 0.0))
    for cx, cy in ((30, 30), (31, 30), (30, 31), (20, 40), (41, 22)):
        gm.solid[cy, cx] = 1
    return gm


@pytest.mark.parametrize('dev', DEVICES)
def test_rect_vs_grid_matches_polygon_sat(dev):
    gm = cluster_map()
    cells = np.argwhere(gm.solid)               # (my, mx)
    rng = np.random.default_rng(0)
    n = 3000
    xy = rng.uniform(0.6, 2.4, (n, 2))
    th = rng.uniform(-math.pi, math.pi, n)
    grid, meta, _, _ = stack_maps([gm], dev)
    pose = torch.tensor(np.c_[xy, th], dtype=torch.float64, device=dev)
    got = G.rect_hits_grid(grid, meta, torch.zeros(n, dtype=torch.long, device=dev), pose).cpu()
    exp = []
    for i in range(n):
        rp = rect_poly(xy[i, 0], xy[i, 1], th[i], -C.HALF_L, C.HALF_L, -C.HALF_W, C.HALF_W)
        hit = any(poly_overlap(rp, rect_poly(0, 0, 0, mx * 0.05, (mx + 1) * 0.05,
                                             my * 0.05, (my + 1) * 0.05)) for my, mx in cells)
        exp.append(hit)
    exp = torch.tensor(exp)
    assert 200 < int(exp.sum()) < n - 200          # 양쪽 사례가 충분하다
    assert torch.equal(got, exp)


@pytest.mark.parametrize('dev', DEVICES)
def test_rect_vs_grid_boundary_contact(dev):
    gm = cluster_map()                             # 셀 (30,30): x ∈ [1.50, 1.55]
    grid, meta, _, _ = stack_maps([gm], dev)
    y = 30 * 0.05 + 0.025
    pose = torch.tensor([[1.50 - C.HALF_L, y, 0.0], [1.50 - C.HALF_L - 1e-4, y, 0.0]],
                        dtype=torch.float64, device=dev)
    got = G.rect_hits_grid(grid, meta, torch.zeros(2, dtype=torch.long, device=dev), pose)
    assert got.tolist() == [True, False]          # 경계 접촉 = 충돌 (contract §8)


@pytest.mark.parametrize('dev', DEVICES)
def test_rect_hits_out_of_map(dev):
    gm = GridMap(np.zeros((40, 40), dtype=np.uint8), 0.05, (0.0, 0.0))
    grid, meta, _, _ = stack_maps([gm], dev)
    pose = torch.tensor([[0.25, 1.0, 0.0], [0.35, 1.0, 0.0]], dtype=torch.float64, device=dev)
    got = G.rect_hits_grid(grid, meta, torch.zeros(2, dtype=torch.long, device=dev), pose)
    assert got.tolist() == [True, False]


@pytest.mark.parametrize('dev', DEVICES)
def test_rect_vs_obstacles(dev):
    rng = np.random.default_rng(1)
    n = 4000
    pose = np.c_[rng.uniform(-1.5, 1.5, (n, 2)), rng.uniform(-math.pi, math.pi, n)]
    kind = rng.integers(0, 2, n)
    yaw = rng.uniform(-math.pi, math.pi, n)
    boxes = np.array([(-1.2, 2.0, -0.64, 0.64), (-0.3, 0.3, -0.2, 0.2)])[rng.integers(0, 2, n)]

    def t(v, dt=torch.float64):
        return torch.tensor(np.asarray(v), dtype=dt, device=dev)
    ob = {'xy': t(np.zeros((n, 1, 2))), 'kind': t(kind[:, None], torch.long),
          'r': t(np.full((n, 1), 0.3)), 'box': t(boxes[:, None, :]), 'yaw': t(yaw[:, None]),
          'active': t(np.ones((n, 1), bool), torch.bool)}
    got = G.rect_hits_obstacles(t(pose), ob).cpu().numpy()
    for i in range(n):
        if kind[i] == 0:
            p = RG.to_robot_frame(tuple(pose[i]), [(0.0, 0.0)])
            exp = RG.footprint_clearance(p)[0] <= 0.3
        else:
            exp = poly_overlap(rect_poly(*pose[i], -C.HALF_L, C.HALF_L, -C.HALF_W, C.HALF_W),
                               rect_poly(0, 0, yaw[i], *boxes[i]))
        assert got[i] == exp, i


def test_swept_approach_matches_cpp():
    """safety_gate.cpp SweptFootprint (sweptRegion 기본 매개변수) 와 점별로 같은가."""
    from amr_sim2d.binding import core_module
    m = core_module()
    rng = np.random.default_rng(2)
    rnd = np.c_[rng.uniform(-0.5, 2.0, 60), rng.uniform(-1.5, 1.5, 60)]
    motions = [(0.0, 0.0), (0.005, 0.0), (0.0, 0.03), (0.8, 0.0), (-0.4, 0.0), (0.0, 1.2),
               (0.0, -0.6)] + [tuple(x) for x in rnd]
    pts = rng.uniform(-3.0, 3.0, (400, 2))
    pts = np.r_[pts, [[0.3, 0.0], [0.31, 0.19], [0.0, 0.21], [-0.31, 0.0], [0.2, 0.1]]]
    for v, w in motions:
        exp = m.swept_approach(pts, v, w)
        n = len(pts)
        px = torch.tensor(pts[:, 0:1], dtype=torch.float64)
        py = torch.tensor(pts[:, 1:2], dtype=torch.float64)
        got = G.swept_approach(px, py, torch.ones(n, 1, dtype=torch.bool),
                               torch.full((n,), v, dtype=torch.float64),
                               torch.full((n,), w, dtype=torch.float64)).numpy()
        fin = np.isfinite(exp)
        assert np.array_equal(fin, np.isfinite(got)), (v, w)
        assert np.allclose(got[fin], exp[fin], atol=1e-9), (v, w)


def test_safety_params_match_robot_params():
    """swept_approach 의 매개변수 = robot_params.yaml safety.* (safety_gate.hpp 기본값과 같아야 C++ 대조가 유효)."""
    import yaml
    from amr_sim2d._contract import REPO_DIR
    with open(f'{REPO_DIR}/config/robot_params.yaml', encoding='utf-8') as f:
        txt = f.read()
    d = yaml.safe_load(txt)
    flat = {}

    def walk(x):
        if isinstance(x, dict):
            for k, v in x.items():
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    flat.setdefault(k, v)
                walk(v)
    walk(d)
    for k in ('emergency_stop_distance', 'warning_zone_distance', 'reaction_latency'):
        assert flat[k] == pytest.approx(G.SAFETY[k]), k

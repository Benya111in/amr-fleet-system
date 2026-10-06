"""
레이캐스트 정확도 — 해석해가 있는 단순 격자·원·사각형, torch DDA 와 Triton DDA 의 일치.

격자 레이캐스트는 셀 경계까지의 정확한 거리를 내야 한다 (DDA 진입 매개변수). 허용오차는 float32 누적 오차 몫이다.
"""

import math

import numpy as np
import pytest
import torch

from amr_sim2d import raycast as RC
from amr_sim2d._contract import C
from amr_sim2d.gridmap import GridMap, map_rect_world, stack_maps

DEVICES = ['cpu'] + (['cuda'] if torch.cuda.is_available() else [])
IMPLS = [('torch', d) for d in DEVICES] + (
    [('triton', 'cuda')] if torch.cuda.is_available() and RC.HAVE_TRITON else [])


def box_map(size=10.0, res=0.05, wall=0.2):
    """size×size m 방, 원점 (0,0), 두께 wall 의 벽."""
    n = int(round(size / res))
    gm = GridMap(np.zeros((n, n), dtype=np.uint8), res, (0.0, 0.0), name='box')
    map_rect_world(gm, 0, size, 0, wall)
    map_rect_world(gm, 0, size, size - wall, size)
    map_rect_world(gm, 0, wall, 0, size)
    map_rect_world(gm, size - wall, size, 0, size)
    return gm


def _cast(impl, dev, gm, x0, y0, ang, max_m=25.0):
    grid, meta, _, res = stack_maps([gm], dev)
    x0 = torch.as_tensor(x0, dtype=torch.float64, device=dev).reshape(-1, 1)
    y0 = torch.as_tensor(y0, dtype=torch.float64, device=dev).reshape(-1, 1)
    ang = torch.as_tensor(ang, dtype=torch.float64, device=dev).reshape(x0.shape[0], -1)
    dx, dy = torch.cos(ang).float(), torch.sin(ang).float()
    gx = ((x0 - gm.ox) / res).float().expand_as(dx)
    gy = ((y0 - gm.oy) / res).float().expand_as(dx)
    midx = torch.zeros(x0.shape[0], dtype=torch.long, device=dev)
    fn = RC.raycast_grid_triton if impl == 'triton' else RC.raycast_grid_torch
    return (fn(grid, midx, gx, gy, dx, dy, max_m / res) * res).double().cpu().numpy()


@pytest.mark.parametrize('impl,dev', IMPLS)
def test_axis_aligned_walls_exact(impl, dev):
    gm = box_map()
    # 안쪽 벽면: x = 0.2, 9.8, y = 0.2, 9.8
    x0, y0 = 3.017, 4.511
    r = _cast(impl, dev, gm, [x0], [y0], [[0.0, math.pi / 2, math.pi, -math.pi / 2]])[0]
    assert r == pytest.approx([9.8 - x0, 9.8 - y0, x0 - 0.2, y0 - 0.2], abs=2e-5)


@pytest.mark.parametrize('impl,dev', IMPLS)
def test_oblique_rays_hit_plane(impl, dev):
    gm = box_map()
    x0, y0 = 2.0, 5.0
    ang = np.deg2rad(np.linspace(-30, 30, 41))   # 7.8·tan30° = 4.5 → 오른쪽 벽면 안 (y 0.5…9.5)
    r = _cast(impl, dev, gm, [x0], [y0], ang[None])[0]
    expect = (9.8 - x0) / np.cos(ang)        # 오른쪽 벽면 (맞은 y 가 0.2…9.8 안)
    assert np.allclose(r, expect, atol=5e-5)


@pytest.mark.parametrize('impl,dev', IMPLS)
def test_start_inside_and_out_of_map(impl, dev):
    gm = box_map()
    r = _cast(impl, dev, gm, [0.1], [5.0], [[0.0]])[0]
    assert r[0] == 0.0                     # 막힌 셀 안에서 시작 → 0
    gm2 = GridMap(np.zeros((40, 40), dtype=np.uint8), 0.05, (0.0, 0.0))
    r = _cast(impl, dev, gm2, [0.5], [1.0], [[math.pi]])[0]
    assert r[0] == pytest.approx(0.5, abs=1e-5)   # 지도 밖 = 막힘 (atOrLethal)


@pytest.mark.parametrize('impl,dev', IMPLS)
def test_max_range_no_return(impl, dev):
    gm = box_map(size=60.0)
    r = _cast(impl, dev, gm, [1.0], [30.0], [[0.0, math.pi]], max_m=25.0)[0]
    assert math.isinf(r[0]) and r[1] == pytest.approx(0.8, abs=1e-5)


def test_triton_matches_torch_on_team_map():
    if not (torch.cuda.is_available() and RC.HAVE_TRITON):
        pytest.skip('CUDA/Triton 없음')
    gm = GridMap.from_yaml()
    rng = np.random.default_rng(0)
    ys, xs = np.where(gm.edt > 0.3)
    sel = rng.integers(0, len(xs), 256)
    wx, wy = gm.cell_center(xs[sel], ys[sel])
    pose = torch.tensor(np.stack([wx, wy, rng.uniform(-math.pi, math.pi, 256)], 1), device='cuda')
    grid, meta, _, _ = stack_maps([gm], 'cuda')
    midx = torch.zeros(256, dtype=torch.long, device='cuda')
    a = RC.cast_scan(grid, meta, midx, pose, use_triton=True)
    b = RC.cast_scan(grid, meta, midx, pose, use_triton=False)
    assert torch.equal(torch.isfinite(a), torch.isfinite(b))
    fin = torch.isfinite(a)
    assert float((a[fin] - b[fin]).abs().max()) < 1e-3


def _obs(dev, xy, kind, r=0.0, box=(0, 0, 0, 0), yaw=0.0):
    def t(v, dt=torch.float64):
        return torch.tensor(v, dtype=dt, device=dev)
    return {'xy': t([[xy]]), 'kind': t([[kind]], torch.long), 'r': t([[r]]),
            'box': t([[box]]), 'yaw': t([[yaw]]), 'active': t([[True]], torch.bool)}


@pytest.mark.parametrize('dev', DEVICES)
def test_circle_obstacle_analytic(dev):
    ob = _obs(dev, (5.0, 0.0), 0, r=0.3)
    ang = torch.tensor([[0.0, 0.05, math.asin(0.3 / 5.0) + 1e-3]], device=dev)
    r = RC.raycast_obstacles(torch.zeros(1, 1, device=dev), torch.zeros(1, 1, device=dev),
                             torch.cos(ang), torch.sin(ang), ob)[0].cpu().numpy()
    b = 5.0 * math.cos(0.05)
    assert r[0] == pytest.approx(4.7, abs=1e-5)
    assert r[1] == pytest.approx(b - math.sqrt(0.09 - (5.0 * math.sin(0.05)) ** 2), abs=1e-5)
    assert math.isinf(r[2])                         # 접선 바깥은 놓친다


@pytest.mark.parametrize('dev', DEVICES)
def test_rotated_box_against_marching(dev):
    box = (-1.2, 2.0, -0.64, 0.64)
    yaw = 0.7
    ob = _obs(dev, (4.0, 1.0), 1, box=box, yaw=yaw)
    ang = torch.linspace(-0.6, 0.9, 61, device=dev)[None]
    r = RC.raycast_obstacles(torch.zeros(1, 1, device=dev), torch.zeros(1, 1, device=dev),
                             torch.cos(ang), torch.sin(ang), ob)[0].cpu().numpy()
    # 독립 기준: 0.5 mm 간격 행진으로 첫 내부점
    s = np.arange(0, 10, 0.0005)
    c, sn = math.cos(yaw), math.sin(yaw)
    for a, ri in zip(ang[0].cpu().numpy(), r):
        px, py = s * math.cos(a) - 4.0, s * math.sin(a) - 1.0
        lx, ly = c * px + sn * py, -sn * px + c * py
        inside = (lx >= box[0]) & (lx <= box[1]) & (ly >= box[2]) & (ly <= box[3])
        if inside.any():
            assert ri == pytest.approx(s[np.argmax(inside)], abs=1e-3)
        else:
            assert math.isinf(ri)


@pytest.mark.parametrize('dev', DEVICES)
def test_full_scan_in_box_room(dev):
    """방 한가운데 로봇: 720 빔 = 내벽까지 해석 거리 (LiDAR 원점 (0.15, 0)), 잡음 없음."""
    gm = box_map()
    grid, meta, _, _ = stack_maps([gm], dev)
    th = 0.3
    pose = torch.tensor([[5.0, 5.0, th]], dtype=torch.float64, device=dev)
    r = RC.cast_scan(grid, meta, torch.zeros(1, dtype=torch.long, device=dev), pose,
                     use_triton=(dev == 'cuda' and RC.HAVE_TRITON))[0].double().cpu().numpy()
    ox, oy = 5.0 + C.LIDAR_X * math.cos(th), 5.0 + C.LIDAR_X * math.sin(th)
    ang = th + C.LIDAR_ANGLE_MIN + np.arange(720) * C.LIDAR_ANGLE_INC
    dx, dy = np.cos(ang), np.sin(ang)
    with np.errstate(divide='ignore'):
        tx = np.where(dx > 0, (9.8 - ox) / dx, np.where(dx < 0, (0.2 - ox) / dx, np.inf))
        ty = np.where(dy > 0, (9.8 - oy) / dy, np.where(dy < 0, (0.2 - oy) / dy, np.inf))
    assert np.allclose(r, np.minimum(tx, ty), atol=5e-5)

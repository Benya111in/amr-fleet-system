"""스캔-지도 점-대-면 정합 (scan_registration) 테스트: 합성 방·통로 + 광선 투사."""

import math

from amr_localization import scan_registration as sr
from amr_localization.global_seed import base_beams
from amr_localization.scan_map_match import compose
from grid_sim import make_room, raycast
import numpy as np
import pytest
from scipy import ndimage

OFFSET = (0.15, 0.0, 0.0)


def slam_like(grid: np.ndarray):
    """
    채운 격자 → SLAM(Karto) 같은 띠 지도: 관측면 셀(자유와 닿은 점유 셀) ±1 셀 띠만 점유, 물체 속은 미지.

    반환 (occupied, free) — 행 0 = 최하단 y.
    """
    occ = grid >= 65
    free = ~occ
    surface = occ & ndimage.binary_dilation(free, structure=np.ones((3, 3), bool))
    band = ndimage.binary_dilation(surface, structure=np.ones((3, 3), bool))
    inside = occ & ~band
    return band, free & ~band & ~inside


def scan_at(grid, pose, noise=0.03, seed=0):
    ranges, amin, inc = raycast(grid, 0.05, compose(pose, OFFSET), noise=noise, seed=seed)
    return base_beams(ranges, amin, inc, 25.0, 360, OFFSET)


@pytest.fixture(scope='module')
def room():
    grid, res = make_room()
    occ, free = slam_like(grid)
    return grid, sr.SurfaceMap(occ, free, res, 0.0, 0.0)


def test_surface_points_have_free_side_normals(room):
    grid, surface = room
    assert len(surface) > 500
    assert np.allclose(np.linalg.norm(surface.normals, axis=1), 1.0)
    # 서쪽 외벽(x ≈ 0.1) 면 점의 법선은 +x (방 안쪽 = 자유 공간)
    west = ((surface.points[:, 0] < 0.3) & (surface.points[:, 1] > 2.0)
            & (surface.points[:, 1] < 12))
    assert west.sum() > 20
    assert np.mean(surface.normals[west, 0]) > 0.95
    assert surface.planar[west].mean() > 0.9
    # 띠 중심선: 띠(3 셀) 한가운데 = 관측면 셀 중심 (x = 0.075)
    assert np.median(surface.points[west, 0]) == pytest.approx(0.075, abs=0.013)


@pytest.mark.parametrize('pose, perturb', [
    ((6.0, 6.0, 0.3), (0.12, -0.08, math.radians(3.0))),
    ((14.5, 3.3, -2.0), (-0.1, 0.1, math.radians(-2.5))),
    ((10.0, 11.5, 1.2), (0.05, 0.15, math.radians(1.0))),
])
def test_registration_converges_to_truth(room, pose, perturb):
    grid, surface = room
    beams = scan_at(grid, pose, seed=3)
    init = (pose[0] + perturb[0], pose[1] + perturb[1], pose[2] + perturb[2])
    res = sr.register(surface, beams, init)
    assert res.ok, res.reason
    # 띠 중심 규약(관측면 셀 중심 = 면 뒤 반 셀) 이 사방 벽에서 상쇄되므로 참값과 1.5 cm 안
    assert math.hypot(res.pose[0] - pose[0], res.pose[1] - pose[1]) < 0.015
    assert abs(math.atan2(math.sin(res.pose[2] - pose[2]),
                          math.cos(res.pose[2] - pose[2]))) < math.radians(0.3)
    assert res.inlier_ratio > 0.9 and res.rms_residual < 0.05
    assert res.iterations < 15
    evals = np.linalg.eigvalsh(res.covariance)
    assert np.all(evals > 0)


def test_hessian_covariance_matches_noise_monte_carlo(room):
    # κ = 1 이면 Σ = s²(JᵀWJ)⁻¹ 는 독립 빔 잡음의 실제 자세 분산과 같은 크기 (±배 2): 공식 검증
    grid, surface = room
    params = sr.RegistrationParams(covariance_scale=1.0, min_sigma_xy=0.0, min_sigma_yaw=0.0)
    pose = (6.0, 6.0, 0.3)
    poses, covs = [], []
    for seed in range(30):
        res = sr.register(surface, scan_at(grid, pose, seed=100 + seed), pose, params)
        assert res.ok
        poses.append(res.pose)
        covs.append(res.covariance)
    emp = np.cov(np.array(poses).T)
    model = np.mean(covs, axis=0)
    for i in range(3):
        assert 0.3 < emp[i, i] / model[i, i] < 3.0, (i, emp[i, i], model[i, i])


def test_corridor_is_weak_along_axis_only():
    # 끝벽이 사거리 밖인 긴 통로: 가로 방향만 구속 → 공분산이 통로 방향(x)으로만 크다
    res_m = 0.05
    grid = np.zeros((int(4.0 / res_m), int(120.0 / res_m)), dtype=np.int8)
    grid[:2, :] = 100
    grid[-2:, :] = 100
    occ, free = slam_like(grid)
    surface = sr.SurfaceMap(occ, free, res_m, 0.0, 0.0)
    pose = (60.0, 2.0, 0.0)
    beams = scan_at(grid, pose, seed=4)
    res = sr.register(surface, beams, (60.3, 2.05, 0.02))
    assert res.ok, res.reason
    assert abs(res.pose[1] - pose[1]) < 0.015
    # 약한 방향(x)은 격자 이산화의 가짜 기울기만큼 떠다니지만 보고 공분산 안이다 (EKF 가 알아서 무시)
    assert abs(res.pose[0] - 60.3) < 2.0 * math.sqrt(res.covariance[0, 0])
    assert res.covariance[0, 0] > 100 * res.covariance[1, 1]


def test_rejects_far_initial_and_sparse_scans(room):
    grid, surface = room
    beams = scan_at(grid, (6.0, 6.0, 0.3), seed=5)
    far = sr.register(surface, beams, (8.0, 4.0, 1.5))    # 납치 규모 오차: 다른 분지
    assert not far.ok
    few = sr.register(surface, beams[:20], (6.0, 6.0, 0.3))
    assert not few.ok and few.reason == 'too few'
    empty = sr.SurfaceMap(np.zeros((10, 10), bool), np.ones((10, 10), bool), 0.05, 0.0, 0.0)
    assert len(empty) == 0 and not sr.register(empty, beams, (0.2, 0.2, 0.0)).ok


def test_robust_to_unmapped_obstacles(room):
    # 지도에 없는 사람·지게차: 빔 20 % 를 가까운 가짜 끝점으로 → 대응 거리·Huber 로 버틴다
    grid, surface = room
    pose = (6.0, 6.0, 0.3)
    beams = scan_at(grid, pose, seed=6)
    rng = np.random.default_rng(1)
    k = len(beams) // 5
    beams[:k] = beams[:k] * rng.uniform(0.3, 0.7, size=(k, 1))
    res = sr.register(surface, beams, (6.05, 5.95, 0.32))
    assert res.ok, res.reason
    assert math.hypot(res.pose[0] - pose[0], res.pose[1] - pose[1]) < 0.02


def test_from_occupancy_grid_and_surface_offset():
    grid, res = make_room()
    occ, free = slam_like(grid)
    data = np.where(occ, 100, np.where(free, 0, -1)).astype(np.int16).ravel()
    base = sr.SurfaceMap.from_occupancy_grid(data, grid.shape[1], grid.shape[0], res, 0.0, 0.0)
    moved = sr.SurfaceMap.from_occupancy_grid(
        data, grid.shape[1], grid.shape[0], res, 0.0, 0.0,
        params=sr.RegistrationParams(surface_offset=0.02))
    assert len(base) == len(moved)
    shift = np.einsum('ij,ij->i', moved.points - base.points, base.normals)
    assert np.allclose(shift, 0.02)

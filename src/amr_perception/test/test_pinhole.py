"""Pinhole 역투영·깊이 통계·공분산 전파·3D 변환 도우미 테스트 (해석해 / 몬테카를로 대조)."""

import math

from amr_perception import pinhole
from amr_perception.pinhole import CameraIntrinsics, LocalizationParams, OffsetModel
from amr_perception.transforms import matrix_to_quat, prefixed_frame, quat_to_matrix, \
    rpy_to_matrix, Transform, yaw_of
import numpy as np
import pytest

K = CameraIntrinsics.from_fov(640, 480, 1.518436)   # sensors.yaml: HFOV 87°


def test_intrinsics_from_fov_and_camera_info():
    assert K.fx == pytest.approx(337.2, abs=0.1)
    assert K.cx == pytest.approx(319.5)
    k2 = CameraIntrinsics.from_k([300.0, 0, 320.0, 0, 310.0, 240.0, 0, 0, 1], 640, 480)
    assert (k2.fx, k2.fy, k2.cx, k2.cy, k2.width) == (300.0, 310.0, 320.0, 240.0, 640)
    assert k2.valid() and not CameraIntrinsics(0, 0, 0, 0).valid()
    assert np.allclose(k2.matrix(), [[300, 0, 320], [0, 310, 240], [0, 0, 1]])


@pytest.mark.parametrize('point', [(0.0, 0.0, 1.0), (0.5, -0.3, 2.0), (-1.2, 0.4, 7.5)])
def test_projection_round_trip(point):
    u, v = pinhole.project(point, K)
    back = pinhole.back_project(u, v, point[2], K)
    assert np.allclose(back, point, atol=1e-12)


def test_projection_analytic_pixels_and_behind_camera():
    # 광축 위 점은 주점, X = Z·(u - cx)/fx 의 정의 그대로
    assert pinhole.project((0.0, 0.0, 3.0), K) == pytest.approx((K.cx, K.cy))
    u, v = pinhole.project((1.0, 0.5, 2.0), K)
    assert u == pytest.approx(K.fx * 0.5 + K.cx)
    assert v == pytest.approx(K.fy * 0.25 + K.cy)
    with pytest.raises(ValueError):
        pinhole.project((0.0, 0.0, -1.0), K)


def test_depth_sigma_model():
    # sensors.yaml: σ(d) = √(0.005² + (0.002·d²)²) → 1 m 0.0054, 3 m 0.0187, 10 m 0.2
    assert pinhole.depth_sigma(1.0, 0.005, 0.002) == pytest.approx(math.hypot(0.005, 0.002))
    assert pinhole.depth_sigma(3.0, 0.005, 0.002) == pytest.approx(0.01868, abs=1e-5)
    assert pinhole.depth_sigma(10.0, 0.005, 0.002) == pytest.approx(0.2, rel=1e-3)


def test_roi_bounds_clip_to_image():
    assert pinhole.roi_bounds(320, 240, 100, 60, 0.5, 640, 480) == (295, 345, 225, 255)
    u0, u1, v0, v1 = pinhole.roi_bounds(5, 470, 100, 100, 0.5, 640, 480)
    assert u0 == 0 and v1 == 480 and u1 > u0 and v1 > v0


def test_robust_depth_rejects_background_and_invalid_pixels():
    roi = np.full((20, 20), 3.0)
    roi[:7, :] = 8.0            # 35 % 배경 (bbox 가장자리에 걸린 벽)
    roi[0, 0] = np.nan
    roi[0, 1] = np.inf
    est = pinhole.robust_depth(roi, 0.2, 10.0, 0.3)
    assert est.z == pytest.approx(3.0)
    assert est.n_valid == 398
    assert est.n_inliers == 260
    assert est.valid_fraction == pytest.approx(398 / 400)
    # 유효 픽셀 부족 → None
    sparse = np.full((10, 10), np.nan)
    sparse[:2, :] = 2.0
    assert pinhole.robust_depth(sparse, 0.2, 10.0, 0.3) is None
    assert pinhole.robust_depth(np.zeros((0, 0)), 0.2, 10.0, 0.3) is None
    # 거리 밖 픽셀은 무효
    assert pinhole.robust_depth(np.full((4, 4), 12.0), 0.2, 10.0, 0.3) is None


def test_robust_depth_noise_and_median_variance_mc():
    rng = np.random.default_rng(3)
    z0 = 4.0
    sigma = 0.002 * z0 ** 2
    m = 400
    est = [pinhole.robust_depth(np.full((20, 20), z0), 0.2, 10.0, 0.3, 0.002, rng).z
           for _ in range(2000)]
    # 중앙값 분산 ≈ (π/2)σ²/m (정규 표본 중앙값의 점근 분산)
    assert np.mean(est) == pytest.approx(z0, abs=5e-4)
    assert np.var(est) == pytest.approx(pinhole.median_depth_variance(sigma, m), rel=0.15)
    assert pinhole.median_depth_variance(0.1, 0) == pytest.approx(0.5 * math.pi * 0.01)


def test_offset_model_and_center_offset():
    table = {'box_small': (0.136, 0.026), 'box_medium': (0.25, 0.034),
             'box_large': (0.306, 0.039), 'person': (0.12, 0.04)}
    om = OffsetModel(table, (0.0, 0.05))
    assert om.lookup('box', 0.52) == (0.25, 0.034)      # 가로 0.52 m → 중형
    assert om.lookup('box', 0.28) == (0.136, 0.026)
    assert om.lookup('box', 0.0) == (0.0, 0.05)         # 폭 모름 + 'box' 키 없음 → 기본
    assert om.lookup('person') == (0.12, 0.04)
    assert om.lookup('unknown') == (0.0, 0.05)
    assert OffsetModel({'box': (0.2, 0.1)}).lookup('box', 0.5) == (0.2, 0.1)
    p = np.array([0.0, 0.0, 2.0])
    assert np.allclose(pinhole.apply_center_offset(p, 0.25), [0.0, 0.0, 2.25])
    q = np.array([1.0, 0.0, 1.0])
    out = pinhole.apply_center_offset(q, 0.1)
    assert np.linalg.norm(out) == pytest.approx(math.sqrt(2) + 0.1)
    assert np.allclose(pinhole.apply_center_offset(np.zeros(3), 0.3), 0.0)


def test_jacobian_matches_numeric_derivative():
    u, v, z = 450.0, 120.0, 3.2
    j = pinhole.backprojection_jacobian(u, v, z, K)
    eps = 1e-6
    num = np.zeros((3, 3))
    for i, d in enumerate(np.eye(3)):
        hi = pinhole.back_project(u + eps * d[0], v + eps * d[1], z + eps * d[2], K)
        lo = pinhole.back_project(u - eps * d[0], v - eps * d[1], z - eps * d[2], K)
        num[:, i] = (hi - lo) / (2 * eps)
    assert np.allclose(j, num, atol=1e-7)


def test_optical_covariance_matches_monte_carlo():
    rng = np.random.default_rng(0)
    u, v, z = 500.0, 300.0, 2.5
    su, sv, sz = 3.0, 2.0, 0.03
    cov = pinhole.optical_covariance(u, v, z, K, su, sv, sz)
    samples = np.array([pinhole.back_project(u + su * a, v + sv * b, z + sz * c, K)
                        for a, b, c in rng.standard_normal((20000, 3))])
    assert np.allclose(np.cov(samples.T), cov, rtol=0.1, atol=2e-6)
    # 가로 σ_X ≈ κ·L: bbox 폭 w 픽셀 = fx·L/Z 이므로 (Z/fx)·κw = κL (거리 무관)
    L, kappa = 0.5, 0.05
    for zz in (1.0, 4.0, 8.0):
        w = K.fx * L / zz
        c = pinhole.optical_covariance(K.cx, K.cy, zz, K, kappa * w, kappa * w, 0.0)
        assert math.sqrt(c[0, 0]) == pytest.approx(kappa * L)


def test_pose_uncertainty_covariance_matches_monte_carlo():
    rng = np.random.default_rng(1)
    robot = np.array([2.0, 1.0])
    yaw = 0.3
    rel = np.array([3.0, -0.5])
    c, s = math.cos(yaw), math.sin(yaw)
    p = robot + np.array([[c, -s], [s, c]]) @ rel
    pose_cov = np.array([[0.0025, 0.0005, 0.0], [0.0005, 0.0016, 0.0002], [0.0, 0.0002, 1e-4]])
    lin = pinhole.pose_uncertainty_covariance(p, robot, pose_cov)
    d = rng.multivariate_normal(np.zeros(3), pose_cov, 40000)
    pts = []
    for dx, dy, dth in d:
        c2, s2 = math.cos(yaw + dth), math.sin(yaw + dth)
        pts.append(robot + np.array([dx, dy]) + np.array([[c2, -s2], [s2, c2]]) @ rel)
    assert np.allclose(np.cov(np.array(pts).T), lin[:2, :2], rtol=0.08, atol=2e-5)
    assert np.all(lin[2, :] == 0.0)


def test_localize_bbox_on_synthetic_depth():
    depth = np.full((480, 640), 8.0)
    depth[170:230, 360:440] = 3.0          # 전면 평판 (bbox 영역)
    params = LocalizationParams(add_quadratic_noise=False)
    om = OffsetModel({'sign': (0.02, 0.02)})
    loc = pinhole.localize_bbox(depth, (400.0, 200.0), (80.0, 60.0), 'sign', K, params, om)
    surface = pinhole.back_project(400.0, 200.0, 3.0, K)
    assert np.allclose(loc.surface, surface)
    assert np.linalg.norm(loc.center) == pytest.approx(np.linalg.norm(surface) + 0.02)
    assert loc.offset == (0.02, 0.02)
    assert loc.depth.n_inliers == 40 * 30
    assert np.all(np.linalg.eigvalsh(loc.covariance) > 0.0)
    # 공분산 깊이 성분 ≥ σ_δ² (오프셋 불확실성)
    assert loc.covariance[2, 2] >= 0.02 ** 2
    # 깊이 없음 / bbox 가 이미지 밖
    assert pinhole.localize_bbox(np.full((480, 640), np.nan), (400.0, 200.0), (80.0, 60.0),
                                 'sign', K, params, om) is None
    assert pinhole.localize_bbox(depth, (-100.0, -100.0), (10.0, 10.0), 'sign', K, params,
                                 om) is None
    # 잡음 가산 경로 (rng) + 상관 픽셀 가정 → 분산이 커진다
    rng = np.random.default_rng(5)
    noisy = LocalizationParams(iid_pixels=False)
    loc2 = pinhole.localize_bbox(depth, (400.0, 200.0), (80.0, 60.0), 'sign', K, noisy, om, rng)
    assert abs(loc2.surface[2] - 3.0) < 0.01
    assert loc2.covariance[2, 2] > loc.covariance[2, 2]


def test_transforms_round_trip():
    r = rpy_to_matrix(0.1, -0.2, 0.7)
    q = matrix_to_quat(r)
    assert np.allclose(quat_to_matrix(q), r)
    assert q[3] >= 0.0
    assert yaw_of(rpy_to_matrix(0.0, 0.0, 1.2)) == pytest.approx(1.2)
    # 광학 회전 (-π/2, 0, -π/2): 광학 z(전방) = 본체 x, 광학 x(우) = 본체 −y
    opt = rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    assert np.allclose(opt @ [0, 0, 1], [1, 0, 0], atol=1e-12)
    assert np.allclose(opt @ [1, 0, 0], [0, -1, 0], atol=1e-12)
    for rr in (rpy_to_matrix(math.pi, 0.0, 0.0), rpy_to_matrix(0.0, math.pi, 0.0),
               rpy_to_matrix(0.0, 0.0, math.pi), np.diag([-1.0, -1.0, 1.0])):
        assert np.allclose(quat_to_matrix(matrix_to_quat(rr)), rr, atol=1e-9)
    assert np.allclose(quat_to_matrix((0, 0, 0, 0)), np.eye(3))
    a = Transform(rpy_to_matrix(0, 0, 0.5), np.array([1.0, 2.0, 0.0]))
    b = Transform.from_quat((0.3, 0.0, 0.2), matrix_to_quat(rpy_to_matrix(0.1, 0.0, 0.0)))
    p = np.array([0.4, -0.2, 1.0])
    assert np.allclose(a.compose(b).apply(p), a.apply(b.apply(p)))
    assert np.allclose(a.inverse().apply(a.apply(p)), p)
    assert np.allclose(Transform.identity().apply(p), p)
    cov = np.diag([1.0, 2.0, 3.0])
    assert np.allclose(np.trace(a.rotate_covariance(cov)), 6.0)
    assert np.allclose(quat_to_matrix(a.quaternion()), a.rotation)
    assert prefixed_frame('amr_02/', 'base_link') == 'amr_02/base_link'
    assert prefixed_frame('amr_02/', 'map') == 'map'
    assert prefixed_frame('', 'odom') == 'odom'
    assert prefixed_frame('amr_02/', 'amr_02/odom') == 'amr_02/odom'

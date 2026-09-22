"""
ArUco 자세 정확도 테스트: 알려진 자세로 마커를 렌더링(projectPoints·원근 와핑)해 다시 추정한다.

도킹 요구(명세 4.8: 위치 2 cm, 각도 1°)의 각도는 바닥 위 로봇의 방향(yaw) 오차이므로 마커 법선의
수평 성분 각 차(heading_error)로 잰다. 최종 접근 구간(0.32–0.5 m) 은 1 cm / 0.5° 이내,
접근 구간(0.75–1.0 m) 은 1 cm / 2° 이내. 원거리(≥ 1.25 m) 는 평면 자세 모호성 때문에
각도가 불안정하다는 사실 자체를 검사한다 (ambiguity_angle 로 보고).
"""

import math

from amr_perception import aruco
from amr_perception.pinhole import CameraIntrinsics
import numpy as np
import pytest

K = CameraIntrinsics.from_fov(640, 480, 1.518436).matrix()
UP = np.array([0.0, -1.0, 0.0])   # 수평 장착 카메라의 광학 프레임 연직 위


def _errors(dist, yaws, lats, vert=0.0, seed=0, noise=2.0):
    est = aruco.ArucoPoseEstimator('DICT_4X4_50', 0.18)
    rng = np.random.default_rng(seed)
    out = []
    for yaw in yaws:
        for lat in lats:
            r, t = aruco.look_at_marker_pose(dist, math.radians(yaw), 0.0, lat, vert)
            img = aruco.render_marker_image(3, 0.18, K, r, t, noise_std=noise, rng=rng,
                                            supersample=3)
            dets = est.detect(img, K, up_hint=UP)
            assert len(dets) == 1
            d = dets[0]
            assert d.marker_id == 3
            pe, ae = aruco.pose_error(d.rotation @ aruco.CV_FROM_MODEL.T, d.translation, r, t)
            he = abs(aruco.heading_error(d.rotation, r @ aruco.CV_FROM_MODEL, UP))
            out.append((pe, math.degrees(he), d, math.degrees(ae)))
    return out


@pytest.mark.parametrize('dist', [0.32, 0.5])
def test_final_approach_accuracy(dist):
    errs = _errors(dist, [-20, -8, 0, 8, 20], [-0.08, 0.0, 0.08])
    assert max(e[0] for e in errs) < 0.01
    assert max(e[1] for e in errs) < 0.5
    assert max(e[3] for e in errs) < 1.0          # 롤·피치 포함 전체 회전각


@pytest.mark.parametrize('dist', [0.75, 1.0])
def test_approach_accuracy(dist):
    errs = _errors(dist, [-20, 0, 20], [-0.1, 0.1], seed=1)
    assert max(e[0] for e in errs) < 0.01
    assert max(e[1] for e in errs) < 2.0


def test_upright_prior_resolves_off_axis_ambiguity():
    # 마커가 광축보다 10 cm 아래: 두 IPPE 해 중 연직 사전정보에 맞는 해 → 1.5 m 에서도 방향 < 2°
    errs = _errors(1.5, [-20, 0, 20], [-0.1, 0.1], vert=0.10, seed=2)
    assert max(e[1] for e in errs) < 2.0
    assert max(e[0] for e in errs) < 0.02


def test_ambiguity_reported_at_long_range():
    errs = _errors(2.0, [15], [0.0], seed=3)
    d = errs[0][2]
    assert d.ambiguity_angle > 0.0
    assert d.covariance is not None
    # 모호하면 회전 분산에 (각/2)² 가 더해진다
    assert d.covariance[3, 3] >= (0.5 * d.ambiguity_angle) ** 2
    assert d.side_px < 35.0 and d.distance == pytest.approx(2.0, abs=0.1)


def test_heading_error_sign_and_projection():
    up = np.array([0.0, 0.0, 1.0])
    r0 = np.eye(3)
    c, s = math.cos(0.1), math.sin(0.1)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    assert aruco.heading_error(r0, rz, up) == pytest.approx(0.1)
    assert aruco.heading_error(rz, r0, up) == pytest.approx(-0.1)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])   # 롤만 다름 → 방향 오차 0
    assert aruco.heading_error(rx, r0, up) == pytest.approx(0.0, abs=1e-12)


def test_model_frame_convention_frontal():
    # 정면 마커: 모델 x(면 바깥 법선) 는 카메라 쪽(−z_cam), z(위) 는 −y_cam
    r, t = aruco.look_at_marker_pose(0.5)
    img = aruco.render_marker_image(0, 0.18, K, r, t)
    # 완전 정면은 IPPE 두 해가 가장 비슷한 경우 → 연직 사전정보로 고른다
    d = aruco.ArucoPoseEstimator().detect(img, K, up_hint=UP)[0]
    assert np.allclose(d.rotation[:, 0], [0, 0, -1], atol=0.02)
    assert np.allclose(d.rotation[:, 2], [0, -1, 0], atol=0.02)
    assert d.reprojection_rms < 0.5


def test_filters_and_errors():
    r, t = aruco.look_at_marker_pose(0.6)
    img = aruco.render_marker_image(5, 0.18, K, r, t)
    est = aruco.ArucoPoseEstimator()
    assert est.detect(img, K, ids=[1, 2]) == []                # id 필터
    assert len(est.detect(np.dstack([img] * 3), K, ids=[5])) == 1  # BGR 입력
    assert est.detect(np.full((480, 640), 128, np.uint8), K) == []
    far = aruco.ArucoPoseEstimator(min_side_px=200.0)           # 너무 작게 보임 → 무시
    assert far.detect(img, K) == []
    with pytest.raises(ValueError):
        aruco.dictionary_id('DICT_NOPE')
    with pytest.raises(ValueError):
        aruco.render_marker_image(0, 0.18, K, np.eye(3), np.array([0.0, 0.0, -1.0]))
    assert aruco.rotation_angle(np.eye(3)) == pytest.approx(0.0)
    assert aruco.ArucoPoseEstimator._covariance(np.zeros((8, 15)), 0.1) is None

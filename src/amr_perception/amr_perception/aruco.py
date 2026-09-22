"""
ArUco 도킹 마커 검출과 자세 추정 (OpenCV, ROS 비의존).

검출: cv2.aruco.ArucoDetector (DICT_4X4_50, 서브픽셀 코너 정제).
자세: 4 코너 ↔ 마커 평면 점으로 solvePnPGeneric(IPPE_SQUARE, 평면 정사각형 전용 해석해) 의 두 해.
      원거리·작은 마커에서는 두 해의 재투영 오차가 거의 같다 (평면 자세 모호성: 법선이 시선에 대해
      뒤집힌 해). 오차 차가 ambiguity_px 이내면 "마커는 수직으로 서 있다" 는 사전정보(up_hint = 광학
      프레임에서 본 연직 위)에 더 맞는 해를 고르고 → LM 정제. 마커가 카메라 광축 높이에 있으면 두 해가
      모두 수직이라 사전정보로도 못 가른다 → 두 해 사이 각을 ambiguity_angle 로 보고해 공분산을 키운다.
공분산: 재투영 자코비안 J (cv2.projectPoints) 로 Σ = σ_px²·(JᵀJ)⁻¹, σ_px = 재투영 RMS (하한 0.1 px),
       + 모호할 때 회전 분산에 (ambiguity_angle/2)² 가산.
프레임 규약
  - OpenCV 마커 프레임(cv): 원점 = 마커 중심, x = 보는 사람 기준 오른쪽, y = 위, z = 면 바깥(카메라 쪽).
  - 출력 마커 프레임(model): Gazebo dock_marker 모델과 같은 REP-103 규약 — x = 면 바깥 법선
    (로봇 쪽), y = x_cv, z = 위(y_cv). R_cv→model 의 열 = [z_cv, x_cv, y_cv].
    정면으로 마주 보면 base_link 기준 마커 yaw = π.
합성 렌더러(render_marker_image)는 알려진 자세의 마커를 투영·와핑해 정확도 시험에 쓴다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 마커 모델 프레임 축을 OpenCV 마커 프레임으로 표현한 행렬 (열 = x_M, y_M, z_M)
CV_FROM_MODEL = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
DUPLICATE_ANGLE = math.radians(0.5)   # 후보 해가 이보다 가까우면 같은 해로 본다


def heading_error(r_est: np.ndarray, r_true: np.ndarray, up: np.ndarray) -> float:
    """
    도킹에 쓰이는 방향 오차 [rad]: 마커 법선(모델 x 축)을 연직(up)에 수직인 평면에 사영한 각 차.

    로봇은 바닥 위에서 yaw 만 바꾸므로 명세 도킹 "각도 오차" 는 이 값이다 (롤·피치 오차 제외).
    """
    u = np.asarray(up, dtype=float) / np.linalg.norm(up)

    def flat(v):
        f = v - (v @ u) * u
        return f / max(float(np.linalg.norm(f)), 1e-12)

    a, b = flat(np.asarray(r_est)[:, 0]), flat(np.asarray(r_true)[:, 0])
    return math.atan2(float(np.cross(a, b) @ u), float(a @ b))


def dictionary_id(name: str) -> int:
    """'DICT_4X4_50' → cv2.aruco 상수."""
    value = getattr(cv2.aruco, name, None)
    if not isinstance(value, int):
        raise ValueError(f'알 수 없는 ArUco 사전: {name}')
    return value


def marker_object_points(marker_size: float) -> np.ndarray:
    """IPPE_SQUARE 순서의 마커 코너 (cv 마커 프레임): 좌상, 우상, 우하, 좌하."""
    h = 0.5 * marker_size
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)


@dataclass
class MarkerDetection:
    """한 마커의 검출·자세 (카메라 광학 프레임)."""

    marker_id: int
    corners: np.ndarray            # (4, 2) 픽셀
    rotation: np.ndarray           # R_cam←model (3×3)
    translation: np.ndarray        # 마커 중심 [m]
    reprojection_rms: float        # [px]
    covariance: Optional[np.ndarray]  # 6×6 (위치 xyz, 회전 벡터) — 광학 프레임, 실패 시 None
    side_px: float                 # 화면상 평균 변 길이 [px]
    ambiguity_angle: float = 0.0   # 재투영 오차가 비슷한 다른 IPPE 해와의 회전각 [rad] (0 = 모호하지 않음)

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.translation))


def rotation_angle(r: np.ndarray) -> float:
    """회전 행렬의 회전각 [rad]."""
    c = (np.trace(r) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, c)))


class ArucoPoseEstimator:
    """ArUco 검출 + 단일 마커 자세 추정."""

    def __init__(self, dictionary: str = 'DICT_4X4_50', marker_size: float = 0.18,
                 min_side_px: float = 12.0, sigma_floor_px: float = 0.1,
                 ambiguity_px: float = 1.0):
        self.marker_size = float(marker_size)
        self.min_side_px = float(min_side_px)
        self.sigma_floor_px = float(sigma_floor_px)
        self.ambiguity_px = float(ambiguity_px)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id(dictionary))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.01
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.object_points = marker_object_points(self.marker_size)

    def detect(self, image: np.ndarray, camera_matrix: np.ndarray,
               dist_coeffs: Optional[np.ndarray] = None,
               ids: Optional[Sequence[int]] = None,
               up_hint: Optional[np.ndarray] = None) -> List[MarkerDetection]:
        """
        이미지(BGR 또는 흑백) 에서 마커들을 찾고 자세를 푼다.

        ids 가 있으면 그 id 만. up_hint: 광학 프레임에서 본 연직 위 방향 (없으면 사전정보 미사용).
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, found, _ = self.detector.detectMarkers(gray)
        out: List[MarkerDetection] = []
        if found is None:
            return out
        k = np.asarray(camera_matrix, dtype=np.float64)
        d = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)
        for c, mid in zip(corners, found.flatten()):
            if ids is not None and len(ids) > 0 and int(mid) not in ids:
                continue
            det = self.estimate(c.reshape(4, 2).astype(np.float64), int(mid), k, d, up_hint)
            if det is not None:
                out.append(det)
        return out

    def estimate(self, img_pts: np.ndarray, marker_id: int, k: np.ndarray, d: np.ndarray,
                 up_hint: Optional[np.ndarray] = None) -> Optional[MarkerDetection]:
        """코너 4점 → 자세 (IPPE_SQUARE 두 해 중 선택 + LM 정제) 와 공분산."""
        side = float(np.mean(np.linalg.norm(img_pts - np.roll(img_pts, 1, axis=0), axis=1)))
        if side < self.min_side_px:
            return None
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(self.object_points, img_pts, k, d,
                                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        # SQPnP(전역 최적) 해를 후보에 더한다: 광축 위 완전 정면 마커에서 IPPE 가 수치적으로
        # 퇴화(재투영 오차 수 px, 심하면 뒤집힌 해)하는 경우를 보완한다
        ok, rv_sq, tv_sq = cv2.solvePnP(self.object_points, img_pts, k, d,
                                        flags=cv2.SOLVEPNP_SQPNP)
        rvecs, tvecs = list(rvecs), list(tvecs)
        errs = [float(e) for e in np.asarray(errs, dtype=float).reshape(-1)]
        if ok:
            proj, _ = cv2.projectPoints(self.object_points, rv_sq, tv_sq, k, d)
            rvecs.append(rv_sq)
            tvecs.append(tv_sq)
            errs.append(self._rms(proj, img_pts))
        n = len(rvecs)
        if n == 0:
            return None
        errs = np.asarray(errs, dtype=float)
        best = int(np.argmin(errs))
        cands = [i for i in range(n) if errs[i] <= errs[best] + self.ambiguity_px]
        if up_hint is not None and len(cands) > 1:
            up = np.asarray(up_hint, dtype=float).reshape(3)
            up = up / max(float(np.linalg.norm(up)), 1e-12)
            best = max(cands, key=lambda i: float(cv2.Rodrigues(rvecs[i])[0][:, 1] @ up))
        ambiguity = 0.0
        r_best = cv2.Rodrigues(rvecs[best])[0]
        for i in cands:
            angle = rotation_angle(cv2.Rodrigues(rvecs[i])[0] @ r_best.T)
            if angle > DUPLICATE_ANGLE:  # 같은 해(SQPnP ≈ IPPE)는 모호성이 아니다
                ambiguity = max(ambiguity, angle)
        rvec, tvec = cv2.solvePnPRefineLM(self.object_points, img_pts, k, d, rvecs[best],
                                          tvecs[best])
        proj, jac = cv2.projectPoints(self.object_points, rvec, tvec, k, d)
        rms = self._rms(proj, img_pts)
        cov = self._covariance(jac, max(rms, self.sigma_floor_px))
        if cov is not None and ambiguity > 0.0:
            cov[3:, 3:] += np.eye(3) * (0.5 * ambiguity) ** 2
        r_cv, _ = cv2.Rodrigues(rvec)
        return MarkerDetection(marker_id, img_pts, r_cv @ CV_FROM_MODEL, tvec.reshape(3), rms,
                               cov, side, ambiguity)

    @staticmethod
    def _rms(proj: np.ndarray, img_pts: np.ndarray) -> float:
        resid = np.asarray(proj).reshape(4, 2) - img_pts
        return float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))

    @staticmethod
    def _covariance(jac: np.ndarray, sigma_px: float) -> Optional[np.ndarray]:
        """(rvec, tvec) 자코비안 → (위치, 회전) 순서 6×6 공분산."""
        j = np.asarray(jac)[:, :6]            # 열: rvec(3), tvec(3)
        info = j.T @ j
        try:
            cov_rt = np.linalg.inv(info) * sigma_px ** 2
        except np.linalg.LinAlgError:
            return None
        perm = [3, 4, 5, 0, 1, 2]              # → (tvec, rvec)
        return cov_rt[np.ix_(perm, perm)]


def pose_error(r_est: np.ndarray, t_est: np.ndarray, r_true: np.ndarray,
               t_true: np.ndarray) -> Tuple[float, float]:
    """(위치 오차 [m], 회전 오차 [rad])."""
    return (float(np.linalg.norm(np.asarray(t_est) - np.asarray(t_true))),
            rotation_angle(np.asarray(r_est) @ np.asarray(r_true).T))


def look_at_marker_pose(distance: float, yaw: float = 0.0, pitch: float = 0.0,
                        lateral: float = 0.0, vertical: float = 0.0) -> Tuple[np.ndarray,
                                                                              np.ndarray]:
    """
    시험용 마커 자세 (카메라 광학 프레임): 광축 위 distance 에 정면을 향하게 두고 yaw/pitch 만큼 기울인다.

    반환 (R_cam←cv, t). 정면 자세의 cv 마커 축: x_cv = +x_cam, y_cv = −y_cam, z_cv = −z_cam.
    """
    base = np.diag([1.0, -1.0, -1.0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r_yaw = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])   # 카메라 y 축(수직) 회전
    r_pitch = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])
    r = r_yaw @ r_pitch @ base
    return r, np.array([lateral, vertical, distance])


def render_marker_image(marker_id: int, marker_size: float, camera_matrix: np.ndarray,
                        r_cam_cv: np.ndarray, t_cam: np.ndarray,
                        image_size: Tuple[int, int] = (640, 480),
                        dictionary: str = 'DICT_4X4_50', plate_scale: float = 300.0 / 180.0,
                        supersample: int = 4, background: int = 110, noise_std: float = 0.0,
                        rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """
    알려진 자세의 마커 판을 원근 와핑으로 렌더링한 흑백 이미지.

    판 = 마커 + 흰 여백 (plate_scale = 판 한 변 / 마커 한 변, Gazebo 모델 0.30/0.18).
    supersample 배로 그린 뒤 INTER_AREA 로 줄여 가장자리 안티앨리어싱을 흉내 낸다.
    """
    w, h = image_size
    ss = max(int(supersample), 1)
    k = np.asarray(camera_matrix, dtype=np.float64)
    k_ss = k.copy()
    k_ss[0, 0] *= ss
    k_ss[1, 1] *= ss
    k_ss[0, 2] = (k[0, 2] + 0.5) * ss - 0.5
    k_ss[1, 2] = (k[1, 2] + 0.5) * ss - 0.5
    cells = 6
    cell_px = 40
    tex_marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(dictionary_id(dictionary)), marker_id,
        cells * cell_px, borderBits=1)
    plate_px = int(round(cells * cell_px * plate_scale))
    margin = (plate_px - cells * cell_px) // 2
    tex = np.full((plate_px, plate_px), 255, np.uint8)
    tex[margin:margin + cells * cell_px, margin:margin + cells * cell_px] = tex_marker
    plate = marker_size * plate_px / (cells * cell_px)
    hp = 0.5 * plate
    obj = np.array([[-hp, hp, 0.0], [hp, hp, 0.0], [hp, -hp, 0.0], [-hp, -hp, 0.0]])
    cam = (np.asarray(r_cam_cv) @ obj.T).T + np.asarray(t_cam).reshape(1, 3)
    if np.any(cam[:, 2] <= 0.05):
        raise ValueError('마커 판이 카메라 뒤에 있다')
    uv = (k_ss @ cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    src = np.array([[-0.5, -0.5], [plate_px - 0.5, -0.5], [plate_px - 0.5, plate_px - 0.5],
                    [-0.5, plate_px - 0.5]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(src, uv.astype(np.float32))
    size = (w * ss, h * ss)
    warped = cv2.warpPerspective(tex, homography, size, flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(tex, 255), homography, size, flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    alpha = mask.astype(np.float32) / 255.0
    canvas = warped.astype(np.float32) * alpha + float(background) * (1.0 - alpha)
    img = cv2.resize(canvas, (w, h), interpolation=cv2.INTER_AREA)
    if noise_std > 0.0:
        gen = rng if rng is not None else np.random.default_rng(0)
        img = img + gen.normal(0.0, noise_std, img.shape)
    return np.clip(np.round(img), 0, 255).astype(np.uint8)

"""
Pinhole 카메라 모델 2D→3D 변환과 불확실성 전파 (numpy, ROS 비의존).

광학 프레임(x 우, y 하, z 전방, REP-103 optical) 에서

    u = fx·X/Z + cx,  v = fy·Y/Z + cy          (투영)
    X = (u − cx)·Z/fx,  Y = (v − cy)·Z/fy      (역투영, Z = 깊이 이미지 값 = Z-depth)

대표 깊이: bbox 중앙 roi_frac×roi_frac 영역의 유효 픽셀 → (선택) 픽셀별 N(0, (k·Z²)²) 가산
(sensors.yaml: 시뮬레이터는 noise_base 만 넣으므로 거리 제곱 항은 여기서 더한다) → 중앙값 →
MAD 기반 인라이어만 남겨 다시 중앙값 ("인라이어 중앙값", 배경·가장자리 픽셀 제거).

표면→중심 보정: 깊이는 보이는 표면이므로 광선을 따라 클래스별 μ_δ 만큼 물체 중심 쪽으로 민다.
공분산: ξ = (u, v, Z) 의 대각 공분산을 자코비안 J = ∂(X,Y,Z)/∂(u,v,Z) 로 전파 Σ = J Σ_ξ Jᵀ.
유도 전체는 docs/algorithms/perception.md §2–§3.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

MAD_TO_SIGMA = 1.4826  # 정규분포에서 σ = 1.4826 · MAD


@dataclass
class CameraIntrinsics:
    """Pinhole 내부 파라미터 (왜곡 없음 — Gazebo 카메라는 왜곡 0)."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0

    @staticmethod
    def from_k(k: Sequence[float], width: int = 0, height: int = 0) -> 'CameraIntrinsics':
        """sensor_msgs/CameraInfo.k (행 우선 3×3) 에서."""
        return CameraIntrinsics(float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                                int(width), int(height))

    @staticmethod
    def from_fov(width: int, height: int, hfov: float) -> 'CameraIntrinsics':
        """수평 화각으로부터 (정사각 픽셀). Gazebo 카메라 규약: fx = (W/2) / tan(hfov/2)."""
        fx = 0.5 * width / math.tan(0.5 * hfov)
        return CameraIntrinsics(fx, fx, 0.5 * width - 0.5, 0.5 * height - 0.5, width, height)

    def valid(self) -> bool:
        return self.fx > 0.0 and self.fy > 0.0

    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


def project(point: Sequence[float], k: CameraIntrinsics) -> Tuple[float, float]:
    """광학 프레임 점 → 픽셀 (u, v). Z <= 0 이면 ValueError."""
    x, y, z = (float(v) for v in point)
    if z <= 0.0:
        raise ValueError('카메라 뒤의 점은 투영할 수 없다 (Z <= 0)')
    return k.fx * x / z + k.cx, k.fy * y / z + k.cy


def back_project(u: float, v: float, z: float, k: CameraIntrinsics) -> np.ndarray:
    """픽셀 (u, v) + Z-depth → 광학 프레임 점 (X, Y, Z)."""
    return np.array([(u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z])


def depth_sigma(d: float, noise_base: float, noise_quadratic_coeff: float) -> float:
    """합성 깊이 잡음 σ(d) = √(σ_base² + (k·d²)²) (sensors.yaml depth_camera)."""
    return math.hypot(noise_base, noise_quadratic_coeff * d * d)


def roi_bounds(cx: float, cy: float, w: float, h: float, frac: float,
               width: int, height: int) -> Tuple[int, int, int, int]:
    """바운딩 박스 중심 frac×frac 영역의 정수 픽셀 범위 [u0, u1) × [v0, v1) (이미지 안으로 자름)."""
    hw = 0.5 * max(w * frac, 1.0)
    hh = 0.5 * max(h * frac, 1.0)
    u0 = int(max(0, math.floor(cx - hw)))
    u1 = int(min(width, math.ceil(cx + hw)))
    v0 = int(max(0, math.floor(cy - hh)))
    v1 = int(min(height, math.ceil(cy + hh)))
    return u0, u1, v0, v1


@dataclass
class DepthEstimate:
    """ROI 대표 깊이."""

    z: float                # 인라이어 중앙값 [m]
    n_valid: int            # 유효 픽셀 수
    n_inliers: int          # 인라이어 수 (m)
    valid_fraction: float   # 유효 픽셀 / ROI 픽셀
    spread: float           # 인라이어 MAD·1.4826 [m] (표면 굴곡 + 잡음의 실측 척도)


def robust_depth(roi: np.ndarray, range_min: float, range_max: float, min_valid_fraction: float,
                 noise_quadratic_coeff: float = 0.0, rng: Optional[np.random.Generator] = None,
                 mad_k: float = 3.0, min_gate: float = 0.05) -> Optional[DepthEstimate]:
    """
    ROI 깊이 픽셀의 인라이어 중앙값.

    1) 유한·[range_min, range_max] 픽셀만 (유효 비율 < min_valid_fraction 이면 None)
    2) rng 가 있으면 픽셀별 N(0, (k·Z²)²) 가산 (명세 노이즈 모델)
    3) m₀ = median, 게이트 g = max(mad_k·1.4826·MAD, min_gate) → |Z − m₀| <= g 인 인라이어의 중앙값
    """
    z = np.asarray(roi, dtype=np.float64).ravel()
    total = z.size
    if total == 0:
        return None
    z = z[np.isfinite(z)]
    z = z[(z >= range_min) & (z <= range_max)]
    if z.size == 0 or z.size < min_valid_fraction * total:
        return None
    if rng is not None and noise_quadratic_coeff > 0.0:
        z = z + noise_quadratic_coeff * z * z * rng.standard_normal(z.size)
    m0 = float(np.median(z))
    mad = float(np.median(np.abs(z - m0)))
    gate = max(mad_k * MAD_TO_SIGMA * mad, min_gate)
    inl = z[np.abs(z - m0) <= gate]
    zi = float(np.median(inl))
    spread = MAD_TO_SIGMA * float(np.median(np.abs(inl - zi)))
    return DepthEstimate(zi, int(z.size), int(inl.size), z.size / total, spread)


def median_depth_variance(sigma: float, m_eff: int) -> float:
    """독립 픽셀 m 개 중앙값의 분산 ≈ (π/2)·σ²/m (정규 표본 중앙값의 점근 분산)."""
    return 0.5 * math.pi * sigma * sigma / max(int(m_eff), 1)


@dataclass
class OffsetModel:
    """클래스별 표면→중심 광선 방향 오프셋 (평균 μ_δ, 표준편차 σ_δ) [m]."""

    table: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    default: Tuple[float, float] = (0.0, 0.05)

    def lookup(self, class_name: str, width_m: float = 0.0) -> Tuple[float, float]:
        """
        클래스(와 box 의 추정 가로폭)로 (μ_δ, σ_δ).

        box 는 bbox 폭 L̂ = Z·w/fx 에 가장 가까운 소/중/대 (box_small/medium/large, 가로 0.30/0.50/
        0.60 m) 를 고른다. 해당 키가 없으면 'box', 그것도 없으면 default.
        """
        if class_name == 'box' and width_m > 0.0:
            sizes = {'box_small': 0.30, 'box_medium': 0.50, 'box_large': 0.60}
            present = {kk: vv for kk, vv in sizes.items() if kk in self.table}
            if present:
                best = min(present, key=lambda kk: abs(present[kk] - width_m))
                return self.table[best]
        return self.table.get(class_name, self.default)


def apply_center_offset(p_surface: np.ndarray, mu: float) -> np.ndarray:
    """보이는 표면 점을 광선 방향으로 μ 만큼 밀어 물체 중심으로: P_c = P_s + μ·P_s/|P_s|."""
    n = float(np.linalg.norm(p_surface))
    if n < 1e-9 or mu == 0.0:
        return np.asarray(p_surface, dtype=float)
    return np.asarray(p_surface, dtype=float) * (1.0 + mu / n)


def backprojection_jacobian(u: float, v: float, z: float, k: CameraIntrinsics) -> np.ndarray:
    """J = ∂(X, Y, Z)/∂(u, v, Z)."""
    return np.array([
        [z / k.fx, 0.0, (u - k.cx) / k.fx],
        [0.0, z / k.fy, (v - k.cy) / k.fy],
        [0.0, 0.0, 1.0],
    ])


def optical_covariance(u: float, v: float, z: float, k: CameraIntrinsics, sigma_u: float,
                       sigma_v: float, sigma_z: float) -> np.ndarray:
    """광학 프레임 위치 공분산 Σ = J diag(σ_u², σ_v², σ_Z²) Jᵀ."""
    j = backprojection_jacobian(u, v, z, k)
    return j @ np.diag([sigma_u ** 2, sigma_v ** 2, sigma_z ** 2]) @ j.T


def pose_uncertainty_covariance(p_map: Sequence[float], robot_xy: Sequence[float],
                                pose_cov_xyyaw: np.ndarray) -> np.ndarray:
    """
    로봇 자세 오차 (x, y, yaw) 가 map 물체 위치에 주는 3×3 공분산 (xy 성분만).

    선형화: δp = [I₂ g] δ(x, y, θ),  g = [−(p_y − y_b), p_x − x_b]ᵀ (base 원점 피벗, 교차항 포함).
    """
    gx = -(float(p_map[1]) - float(robot_xy[1]))
    gy = float(p_map[0]) - float(robot_xy[0])
    a = np.array([[1.0, 0.0, gx], [0.0, 1.0, gy]])
    out = np.zeros((3, 3))
    out[:2, :2] = a @ np.asarray(pose_cov_xyyaw, dtype=float) @ a.T
    return out


@dataclass
class LocalizationParams:
    """object_localizer_node 의 수치 파라미터 (config/perception.yaml)."""

    roi_frac: float = 0.5
    min_valid_fraction: float = 0.3
    range_min: float = 0.20
    range_max: float = 10.0
    noise_base: float = 0.005
    noise_quadratic_coeff: float = 0.002
    add_quadratic_noise: bool = True
    iid_pixels: bool = True
    kappa: float = 0.05
    sigma_skew_px: float = 0.0
    mad_k: float = 3.0


@dataclass
class LocalizedPoint:
    """한 검출의 광학 프레임 3D 결과."""

    center: np.ndarray          # 표면→중심 보정 후 [m]
    surface: np.ndarray         # 보이는 표면 점 [m]
    covariance: np.ndarray      # 3×3 [m²]
    depth: DepthEstimate
    offset: Tuple[float, float]


def localize_bbox(depth_image: np.ndarray, bbox_center: Tuple[float, float],
                  bbox_size: Tuple[float, float], class_name: str, k: CameraIntrinsics,
                  params: LocalizationParams, offsets: OffsetModel,
                  rng: Optional[np.random.Generator] = None) -> Optional[LocalizedPoint]:
    """
    바운딩 박스 + 깊이 이미지 → 광학 프레임 물체 중심과 공분산 (docs/algorithms/perception.md §2).

    depth_image 는 [m] 단위 2차원 배열 (32FC1, 또는 16UC1 을 1e-3 배 한 것).
    """
    h, w = depth_image.shape[:2]
    cx, cy = bbox_center
    bw, bh = bbox_size
    u0, u1, v0, v1 = roi_bounds(cx, cy, bw, bh, params.roi_frac, w, h)
    if u1 <= u0 or v1 <= v0:
        return None
    est = robust_depth(depth_image[v0:v1, u0:u1], params.range_min, params.range_max,
                       params.min_valid_fraction,
                       params.noise_quadratic_coeff if params.add_quadratic_noise else 0.0,
                       rng if params.add_quadratic_noise else None, params.mad_k)
    if est is None:
        return None
    surface = back_project(cx, cy, est.z, k)
    width_m = est.z * bw / k.fx
    mu, sigma_delta = offsets.lookup(class_name, width_m)
    center = apply_center_offset(surface, mu)
    # 깊이 분산: 픽셀 잡음 중앙값 분산 + 오프셋 불확실성
    sigma_d = depth_sigma(est.z, params.noise_base, params.noise_quadratic_coeff)
    m_eff = est.n_inliers if params.iid_pixels else 1
    var_z = median_depth_variance(sigma_d, m_eff) + sigma_delta ** 2
    sigma_px = math.hypot(params.kappa * bw, params.sigma_skew_px)
    sigma_py = math.hypot(params.kappa * bh, params.sigma_skew_px)
    # 공분산은 중심 깊이 Z_c 에서 전파 (광선 방향 오프셋이 Z 를 바꾸므로)
    cov = optical_covariance(cx, cy, float(center[2]), k, sigma_px, sigma_py, math.sqrt(var_z))
    return LocalizedPoint(center, surface, cov, est, (mu, sigma_delta))

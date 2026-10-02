r"""
정적 지도에 대한 스캔 정합 (점-대-면 Gauss–Newton) — map EKF 의 자세 측정 (명세 4.3 정확도, rclpy 비의존).

AMCL 파티클 평균은 σ_hit 0.08 m 의 넓은 우도와 유한한 파티클 수 때문에 지도 대비 수 cm 흔들린다.
정지·저속에서 스캔 수백 빔을 지도 면에 최소제곱으로 맞추면 지도 대비 자세를 mm 수준으로 정할 수 있다.
이 모듈이 그 측정과 공분산(헤시안 역)을 만들고, scan_matcher_node 가 map EKF 에 pose1 로 넣는다
(AMCL 은 전역 일관성·납치 감지를 계속 맡는다). 유도와 검증: docs/algorithms/slam.md §4.3, ekf.md §3.

지도 면 (맵당 1 회, SurfaceMap)
  SLAM(Karto) 격자는 면을 2.8~3.2 셀 두께의 점유 띠로 그린다. 띠의 한 층만 쓰면(가장 안쪽 셀, 또는 AMCL 처럼
  "가장 가까운 점유 셀") 띠 안에서 비용이 평평해져 정합이 띠 폭(±5 cm) 안에서 떠다닌다. 그래서 자유 셀과 닿은
  점유 셀(관측된 면)마다 반지름 r_c 안 점유 셀의 중심(= 띠 중심선)을 면 점 q_j 로, 그 이웃의 주성분(PCA)
  최소 고유벡터를 법선 n_j (자유 공간 쪽) 로 둔다. 선택 surface_offset 만큼 n_j 방향으로 옮긴다 (띠 중심과 실제 면의
  규약 차 보정, 기본 0).

정합 (스캔마다)
  자세 x = (t_x, t_y, θ), base 프레임 끝점 b_i, p_i(x) = R(θ) b_i + t.
  대응: p_i 에서 가장 가까운 면 점 j(i) (≤ max_correspondence).  잔차 r_i = n_jᵀ (p_i − q_j)  [m]
  야코비안 J_i = [n_x, n_y, n_jᵀ R'(θ) b_i]   (R' = dR/dθ)
  로버스트 가중 w_i = Huber(r_i; k = huber_k·σ_r)  → (Σ w JᵀJ) δ = −Σ w Jᵀ r,  x ← x + δ,  대응 재탐색 반복
  공분산 Σ_x = κ · s² (Σ w JᵀJ)⁻¹,  s² = Σ w r² / (Σ w − 3)  (+ 하한 σ_xy_min, σ_θ_min)
  κ (covariance_scale): 빔 잔차는 지도 격자 양자화·띠 모양을 공유해 독립이 아니다 → 헤시안 역은 과신 →
  Gazebo NEES 로 정한 팽창 (slam.md §6.3).
  통로처럼 한 방향 구속이 약하면 헤시안이 그 방향으로 작아져 Σ 가 그 방향으로만 커진다 (EKF 가 알아서 덜 믿음).
품질 판정: 인라이어 비율 ≥ min_inlier_ratio, 반복 수렴, |보정| ≤ max_correction (그 밖이면 측정 없음).
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

Pose = Tuple[float, float, float]


@dataclass
class RegistrationParams:
    """정합 파라미터 (config/scan_matcher.yaml 에 의미·근거)."""

    centroid_radius: float = 0.12        # [m] r_c: 띠 중심선 = 이 반지름 안 점유 셀 중심의 평균
    normal_radius: float = 0.20          # [m] 법선 PCA 이웃 반지름
    min_planarity: float = 3.0           # λ_max/λ_min 이 이보다 작으면(모서리·기둥 끝) 점-대-점 잔차
    surface_offset: float = 0.0          # [m] 면 점을 자유 공간 쪽 법선으로 옮기는 양
    max_correspondence: float = 0.25     # [m] 대응 거리 상한 (지도에 없는 동적 장애물 제외)
    range_noise: float = 0.03            # [m] σ_r (sensors.yaml lidar.noise_stddev) — Huber 척도
    huber_k: float = 2.0                 # Huber 문턱 = huber_k · σ_r
    max_iterations: int = 15
    converge_translation: float = 1e-4   # [m] 걸음·잔차 RMS 변화 문턱
    converge_rotation: float = 1e-5      # [rad]
    min_inlier_ratio: float = 0.6        # 유효 빔 중 대응을 찾은 비율 하한
    min_points: int = 60
    max_correction: float = 0.3          # [m] 초기 추정에서 이보다 멀리 가면 실패 (다른 분지)
    max_rotation_correction: float = 0.1  # [rad]
    covariance_scale: float = 10.0       # κ
    min_sigma_xy: float = 0.005          # [m]
    min_sigma_yaw: float = 0.002         # [rad]


@dataclass
class RegistrationResult:
    """정합 결과."""

    ok: bool
    pose: Pose
    covariance: np.ndarray               # 3×3 (x, y, θ)
    iterations: int
    inlier_ratio: float
    rms_residual: float                  # [m] 인라이어 가중 RMS
    points: int
    reason: str = ''


def _rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class SurfaceMap:
    """점유격자 → 띠 중심선 면 점 + 법선 (KD-트리)."""

    def __init__(self, occupied: np.ndarray, free: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float,
                 params: Optional[RegistrationParams] = None) -> None:
        """occupied/free: bool 격자 (행 0 = 최하단 y, nav_msgs/OccupancyGrid 와 같은 배치)."""
        self.params = params or RegistrationParams()
        p = self.params
        res = resolution
        # 관측된 면: 자유 셀과 (8-이웃으로) 닿은 점유 셀
        near_free = ndimage.binary_dilation(free, structure=np.ones((3, 3), bool))
        boundary = occupied & near_free
        occ_rc = np.argwhere(occupied)
        occ_xy = np.stack([origin_x + (occ_rc[:, 1] + 0.5) * res,
                           origin_y + (occ_rc[:, 0] + 0.5) * res], 1)
        bnd_rc = np.argwhere(boundary)
        bnd_xy = np.stack([origin_x + (bnd_rc[:, 1] + 0.5) * res,
                           origin_y + (bnd_rc[:, 0] + 0.5) * res], 1)
        free_rc = np.argwhere(free & ndimage.binary_dilation(occupied, iterations=3))
        free_xy = np.stack([origin_x + (free_rc[:, 1] + 0.5) * res,
                            origin_y + (free_rc[:, 0] + 0.5) * res], 1)
        self.points = np.zeros((0, 2))
        self.normals = np.zeros((0, 2))
        self.planar = np.zeros(0, dtype=bool)
        if len(bnd_xy) == 0:
            self.tree = None
            return
        occ_tree = cKDTree(occ_xy)
        free_tree = cKDTree(free_xy) if len(free_xy) else None
        pts, nrm, planar = [], [], []
        cent_nb = occ_tree.query_ball_point(bnd_xy, p.centroid_radius)
        norm_nb = occ_tree.query_ball_point(bnd_xy, p.normal_radius)
        free_nb = (free_tree.query_ball_point(bnd_xy, p.normal_radius)
                   if free_tree is not None else [[] for _ in range(len(bnd_xy))])
        for k in range(len(bnd_xy)):
            c = occ_xy[cent_nb[k]].mean(axis=0)
            nb = occ_xy[norm_nb[k]]
            if len(nb) >= 3:
                cov = np.cov((nb - nb.mean(axis=0)).T)
                evals, evecs = np.linalg.eigh(cov)
                n = evecs[:, 0]
                ratio = evals[1] / max(evals[0], 1e-12)
            else:
                n = np.array([1.0, 0.0])
                ratio = 0.0
            fr = free_xy[free_nb[k]] if len(free_nb[k]) else np.zeros((0, 2))
            if len(fr):
                if float(n @ (fr.mean(axis=0) - c)) < 0.0:
                    n = -n
            pts.append(c)
            nrm.append(n)
            planar.append(ratio >= p.min_planarity)
        pts_arr = np.array(pts)
        # 같은 띠 중심선 점이 여러 경계 셀에서 겹쳐 나오므로 반 셀 격자로 한 번 솎는다
        key = np.floor(pts_arr / (0.5 * res)).astype(np.int64)
        _, uniq = np.unique(key, axis=0, return_index=True)
        self.normals = np.array(nrm)[uniq]
        self.points = pts_arr[uniq] + p.surface_offset * self.normals
        self.planar = np.array(planar)[uniq]
        self.tree = cKDTree(self.points)

    def __len__(self) -> int:
        """면 점 수."""
        return len(self.points)

    @classmethod
    def from_occupancy_grid(cls, data, width: int, height: int, resolution: float,
                            origin_x: float, origin_y: float, occupied_thresh: int = 65,
                            free_thresh: int = 25,
                            params: Optional[RegistrationParams] = None) -> 'SurfaceMap':
        """nav_msgs/OccupancyGrid 데이터 (−1 미지, 0~100) 에서 (원점 yaw 0 가정)."""
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls(grid >= occupied_thresh, (grid >= 0) & (grid <= free_thresh), resolution,
                   origin_x, origin_y, params)


def register(surface: SurfaceMap, beams: np.ndarray, initial: Pose,
             params: Optional[RegistrationParams] = None) -> RegistrationResult:
    """
    base_footprint 프레임 끝점 beams (N, 2) 를 initial 자세에서 시작해 지도 면에 맞춘다.

    반환 pose 는 map 프레임 base_footprint 자세, covariance 는 (x, y, θ) 3×3.
    """
    p = params or surface.params
    eye = np.eye(3) * 1e6
    if surface.tree is None or len(beams) < p.min_points:
        return RegistrationResult(False, initial, eye, 0, 0.0, math.nan, len(beams), 'too few')
    x = np.array(initial, dtype=float)
    k_huber = p.huber_k * p.range_noise
    n_total = len(beams)
    hessian = np.eye(3)
    resid = np.zeros(0)
    w = np.zeros(0)
    it = 0
    converged = False
    inliers = 0
    for it in range(1, p.max_iterations + 1):
        rot = _rot(x[2])
        world = beams @ rot.T + x[:2]
        dist, idx = surface.tree.query(world, distance_upper_bound=p.max_correspondence)
        valid = np.isfinite(dist)
        inliers = int(valid.sum())
        if inliers < p.min_points:
            return RegistrationResult(False, tuple(x), eye, it, inliers / n_total, math.nan,
                                      n_total, 'too few correspondences')
        pw = world[valid]
        q = surface.points[idx[valid]]
        n = surface.normals[idx[valid]]
        planar = surface.planar[idx[valid]]
        drot = np.array([[-math.sin(x[2]), -math.cos(x[2])],
                         [math.cos(x[2]), -math.sin(x[2])]])
        db = beams[valid] @ drot.T                  # ∂p/∂θ
        diff = pw - q
        # 평면 점: 점-대-면 (1 잔차), 비평면 점: 점-대-점 (x, y 2 잔차)
        rp = np.einsum('ij,ij->i', n[planar], diff[planar])
        jp = np.column_stack([n[planar], np.einsum('ij,ij->i', n[planar], db[planar])])
        dq = diff[~planar]
        jx = np.column_stack([np.ones(len(dq)), np.zeros(len(dq)), db[~planar, 0]])
        jy = np.column_stack([np.zeros(len(dq)), np.ones(len(dq)), db[~planar, 1]])
        resid = np.concatenate([rp, dq[:, 0], dq[:, 1]])
        jac = np.vstack([jp, jx, jy])
        a = np.abs(resid)
        w = np.where(a <= k_huber, 1.0, k_huber / np.maximum(a, 1e-12))
        hessian = (jac * w[:, None]).T @ jac
        grad = (jac * w[:, None]).T @ resid
        # 작은 감쇠(LM): 통로처럼 한 방향 구속이 (거의) 없으면 그 방향 걸음을 억누른다
        damping = 1e-9 + 1e-4 * float(np.trace(hessian)) / 3.0
        try:
            delta = -np.linalg.solve(hessian + damping * np.eye(3), grad)
        except np.linalg.LinAlgError:
            return RegistrationResult(False, tuple(x), eye, it, inliers / n_total, math.nan,
                                      n_total, 'singular')
        x += delta
        x[2] = math.atan2(math.sin(x[2]), math.cos(x[2]))
        # 수렴: 잘 구속된 고유 방향(λ > 1e-3 λ_max)에서 한 걸음이 잔차를 바꾸는 RMS 양 √(Σ λ_i δ_i² / Σw) 가
        # converge_translation 미만, 또는 걸음 자체가 문턱 미만. 통로의 약한 방향은 격자 이산화로 생긴 가짜
        # 기울기에 떠다니지만 공분산이 그 방향으로 크므로(EKF 가 무시) 수렴 판정에서 뺀다
        evals, evecs = np.linalg.eigh(hessian)
        strong = evals > 1e-3 * max(float(evals.max()), 1e-12)
        proj = evecs.T @ delta
        change = math.sqrt(float(np.sum(evals[strong] * proj[strong] ** 2))
                           / max(float(w.sum()), 1e-9))
        if change < p.converge_translation or (
                math.hypot(delta[0], delta[1]) < p.converge_translation
                and abs(delta[2]) < p.converge_rotation):
            converged = True
            break
    dof = max(float(w.sum()) - 3.0, 1.0)
    s2 = float(np.sum(w * resid ** 2)) / dof
    try:
        cov = p.covariance_scale * s2 * np.linalg.inv(hessian)
    except np.linalg.LinAlgError:
        cov = eye
    cov = 0.5 * (cov + cov.T)
    cov[0, 0] = max(cov[0, 0], p.min_sigma_xy ** 2)
    cov[1, 1] = max(cov[1, 1], p.min_sigma_xy ** 2)
    cov[2, 2] = max(cov[2, 2], p.min_sigma_yaw ** 2)
    ratio = inliers / n_total
    rms = math.sqrt(float(np.sum(w * resid ** 2) / max(float(w.sum()), 1e-9)))
    corr = math.hypot(x[0] - initial[0], x[1] - initial[1])
    drot_total = abs(math.atan2(math.sin(x[2] - initial[2]), math.cos(x[2] - initial[2])))
    reason = ''
    if not converged:
        reason = 'not converged'
    elif ratio < p.min_inlier_ratio:
        reason = f'inlier ratio {ratio:.2f}'
    elif corr > p.max_correction or drot_total > p.max_rotation_correction:
        reason = f'correction {corr:.2f} m / {math.degrees(drot_total):.1f} deg'
    return RegistrationResult(reason == '', (float(x[0]), float(x[1]), float(x[2])), cov, it,
                              ratio, rms, n_total, reason)

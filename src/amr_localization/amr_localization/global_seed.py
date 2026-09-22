"""
전역 자세 가설 탐색 (납치 복구 시드, rclpy 비의존).

AMCL 균일 재초기화는 60 × 40 m 창고에서 파티클이 정답 근처(σ_hit 규모의 분지)에 떨어질 확률이 낮다.
그래서 현재 스캔 하나로 자유 공간 전체를 거리장 점수로 훑어 상위 가설을 만들고, 가장 좋은 가설부터
AMCL initialpose 로 넣어 회전으로 검증한다 (research brief state-estimation §4.C SEED 단계의 기준 구현).
  coarse : 자유 셀(점유까지 clearance 이상) 격자 step_xy × 헤딩 step_yaw, 빔 coarse_beams 개,
           점수 m(h) = mean(min(d(끝점), d_max))  — 캡 평균 거리 (작을수록 좋음)
  NMS    : 상위 가설을 거리 nms_xy / 각 nms_yaw 안에서 하나만 남김
  fine   : 각 후보 주변 ±step_xy, ±step_yaw 를 fine 격자로, 빔 fine_beams 개, trim 비율 절사 평균
           (동적 장애물 가림 빔을 버림)
복잡도: coarse 가설 수 × 빔 수 조회 (60 × 40 m, 0.5 m × 5°, 90 빔 ≈ 6×10⁷ 조회).
가설 위치를 셀 중심에 두면 빔 끝점 셀 = 중심 셀 + (헤딩별) 정수 오프셋이 정확히 성립하므로
(floor(c + ½ + o) = c + floor(½ + o)), 조회를 '평탄 인덱스 덧셈 + take' 한 번으로 한다 (GridScorer).
테두리를 최대 빔 길이만큼 두른 거리장 사본을 쓰므로 맵 밖 끝점도 분기 없이 max_distance 가 된다.
"""

from dataclasses import dataclass, field as dc_field
import math
from typing import List, Optional, Sequence, Tuple

from amr_localization.scan_map_match import DistanceField, scan_endpoints
import numpy as np


@dataclass
class SeedParams:
    """탐색 파라미터."""

    step_xy: float = 0.5            # [m] coarse 위치 격자
    step_yaw: float = math.radians(5.0)
    clearance: float = 0.3          # [m] 로봇이 있을 수 있는 셀: 점유까지 거리 ≥ clearance
    coarse_beams: int = 90
    fine_beams: int = 180
    d_max: float = 1.0              # [m] 캡
    top_k: int = 50
    nms_xy: float = 1.0             # [m]
    nms_yaw: float = math.radians(20.0)
    fine_step_xy: float = 0.1
    fine_step_yaw: float = math.radians(1.0)
    trim: float = 0.8               # fine 점수에 쓰는 하위 빔 비율
    rank_inlier_dist: float = 0.2   # [m] 최종 순위: 전체 빔 인라이어 비율 (절사 없음)
    max_candidates: int = 5


@dataclass
class Hypothesis:
    """자세 가설 (map 프레임 base_footprint)."""

    x: float
    y: float
    yaw: float
    score: float                    # [m] 절사 평균 거리 (작을수록 좋음)
    ratio: float = 0.0              # 전체 빔 인라이어 비율 (클수록 좋음, 최종 순위 기준)
    cell: Optional[Tuple[int, int]] = dc_field(default=None, repr=False, compare=False)  # (행, 열)


def candidate_cells(field: DistanceField, free: np.ndarray, step: float,
                    clearance: float) -> Tuple[np.ndarray, np.ndarray]:
    """자유 공간 격자 셀 (행, 열): 알려진 자유 셀이고 점유까지 거리 ≥ clearance."""
    spec = field.spec
    stride = max(int(round(step / spec.resolution)), 1)
    rows = np.arange(stride // 2, spec.height, stride)
    cols = np.arange(stride // 2, spec.width, stride)
    rr, cc = np.meshgrid(rows, cols, indexing='ij')
    ok = free[rr, cc] & (field.field[rr, cc] >= clearance)
    return rr[ok], cc[ok]


def cell_centers(spec, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
    """셀 (행, 열) → 셀 중심 map 좌표 (N, 2)."""
    lx = (np.asarray(cols) + 0.5) * spec.resolution
    ly = (np.asarray(rows) + 0.5) * spec.resolution
    c, s = math.cos(spec.origin_yaw), math.sin(spec.origin_yaw)
    return np.stack([spec.origin_x + c * lx - s * ly, spec.origin_y + s * lx + c * ly], 1)


def candidate_positions(field: DistanceField, free: np.ndarray, step: float,
                        clearance: float) -> np.ndarray:
    """자유 공간 격자 점 (N, 2) map 좌표 (candidate_cells 의 셀 중심)."""
    rows, cols = candidate_cells(field, free, step, clearance)
    return cell_centers(field.spec, rows, cols)


class GridScorer:
    """
    셀 중심 가설의 정수 오프셋 조회기.

    격자 좌표 g = R(−ψ₀)(p − o)/res (ψ₀ = 맵 원점 yaw) 에서 셀 중심 가설은 g = (c + ½, r + ½) 이고
    빔 끝점은 g + R(yaw − ψ₀) b / res 이므로 끝점 셀 = (c, r) + floor(½ + R(yaw − ψ₀) b / res) —
    오프셋이 가설 위치와 무관한 정수라 헤딩마다 한 번만 계산하면 된다.
    """

    def __init__(self, field: DistanceField, max_beam_length: float) -> None:
        """max_beam_length [m]: 테두리 폭 (이보다 긴 빔 끝점은 없어야 한다)."""
        spec = field.spec
        self.spec = spec
        self.pad = int(math.ceil(max_beam_length / spec.resolution)) + 2
        height, width = spec.height + 2 * self.pad, spec.width + 2 * self.pad
        grid = np.full((height, width), field.max_distance, dtype=np.float32)
        grid[self.pad:self.pad + spec.height, self.pad:self.pad + spec.width] = field.field
        self.flat = grid.ravel()
        self.stride = width

    def base_index(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        """셀 (행, 열) → 평탄 인덱스."""
        return (np.asarray(rows, dtype=np.int64) + self.pad) * self.stride + (
            np.asarray(cols, dtype=np.int64) + self.pad)

    def beam_offsets(self, beams: np.ndarray, yaw: float) -> np.ndarray:
        """base_footprint 끝점 (B, 2) 를 헤딩 yaw 로 돌린 평탄 인덱스 오프셋 (B,)."""
        a = yaw - self.spec.origin_yaw
        c, s = math.cos(a), math.sin(a)
        inv = 1.0 / self.spec.resolution
        dc = np.floor(0.5 + (c * beams[:, 0] - s * beams[:, 1]) * inv).astype(np.int64)
        dr = np.floor(0.5 + (s * beams[:, 0] + c * beams[:, 1]) * inv).astype(np.int64)
        return dr * self.stride + dc

    def scores(self, base: np.ndarray, offsets: np.ndarray, d_max: float,
               trim: float = 1.0) -> np.ndarray:
        """가설 (N,) × 빔 (B,) 점수: 하위 trim 비율 빔의 min(d, d_max) 평균."""
        d = np.minimum(self.flat[base[:, None] + offsets[None, :]], np.float32(d_max))
        if trim < 1.0:
            k = max(int(round(trim * d.shape[1])), 1)
            d = np.partition(d, k - 1, axis=1)[:, :k]
        return d.mean(axis=1, dtype=np.float64)


def _beams(ranges, angle_min, angle_increment, range_max, n, offset):
    """base_footprint 프레임 끝점 (센서 오프셋 적용)."""
    sx, sy = scan_endpoints(ranges, angle_min, angle_increment, range_max, n)
    c, s = math.cos(offset[2]), math.sin(offset[2])
    return np.stack([offset[0] + c * sx - s * sy, offset[1] + s * sx + c * sy], 1)


def score_poses(field: DistanceField, beams: np.ndarray, xs: np.ndarray, ys: np.ndarray,
                yaws: np.ndarray, d_max: float, trim: float = 1.0) -> np.ndarray:
    """가설들 (같은 길이 배열) 의 점수: 하위 trim 비율 빔의 min(d, d_max) 평균."""
    c, s = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
    ex = xs[:, None] + c * beams[None, :, 0] - s * beams[None, :, 1]
    ey = ys[:, None] + s * beams[None, :, 0] + c * beams[None, :, 1]
    d = np.minimum(field.lookup(ex.ravel(), ey.ravel()).reshape(ex.shape), d_max)
    if trim < 1.0:
        k = max(int(round(trim * d.shape[1])), 1)
        d = np.partition(d, k - 1, axis=1)[:, :k]
    return d.mean(axis=1)


def inlier_ratio(field: DistanceField, beams: np.ndarray, pose: Tuple[float, float, float],
                 inlier_dist: float) -> float:
    """가설 하나의 인라이어 비율 (끝점이 점유 셀에서 inlier_dist 이내인 빔 비율, 절사 없음)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    ex = pose[0] + c * beams[:, 0] - s * beams[:, 1]
    ey = pose[1] + s * beams[:, 0] + c * beams[:, 1]
    return float(np.mean(field.lookup(ex, ey) <= inlier_dist))


def nms(hyps: Sequence[Hypothesis], dist: float, ang: float, limit: int,
        key=lambda h: h.score) -> List[Hypothesis]:
    """정렬 키(key) 순으로 훑으며 이미 남긴 가설과 가까운 가설을 제거."""
    kept: List[Hypothesis] = []
    for h in sorted(hyps, key=key):
        if all(math.hypot(h.x - k.x, h.y - k.y) > dist
               or abs(math.atan2(math.sin(h.yaw - k.yaw), math.cos(h.yaw - k.yaw))) > ang
               for k in kept):
            kept.append(h)
            if len(kept) >= limit:
                break
    return kept


def pose_close(a: Tuple[float, float, float], b: Tuple[float, float, float], dist: float,
               ang: float) -> bool:
    """두 자세가 위치 dist, 헤딩 ang 안인지."""
    dyaw = math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= dist and abs(dyaw) <= ang


def alternative_poses(hyps: Sequence[Hypothesis], current: Tuple[float, float, float],
                      motion: Tuple[float, float, float], dist: float = 1.0,
                      ang: float = math.radians(20.0)) -> List[Tuple[float, float, float]]:
    """
    현재 추정과 다른 가설(별칭)들의 지금 자세.

    가설은 탐색 시각의 자세이므로 그 뒤 odom 상대 이동 motion (탐색 시각 base 프레임에서 본 현재 base) 을
    합성한다. 현재 추정(AMCL) 에서 dist / ang 안의 가설은 같은 가설로 보고 뺀다.
    """
    out = []
    for h in hyps:
        c, s = math.cos(h.yaw), math.sin(h.yaw)
        pose = (h.x + c * motion[0] - s * motion[1], h.y + s * motion[0] + c * motion[1],
                math.atan2(math.sin(h.yaw + motion[2]), math.cos(h.yaw + motion[2])))
        if not pose_close(pose, current, dist, ang):
            out.append(pose)
    return out


def refined_ratio(field: DistanceField, beams: np.ndarray, pose: Tuple[float, float, float],
                  inlier_dist: float, radius: float = 0.1, step: float = 0.025,
                  yaw_range: float = math.radians(2.0),
                  yaw_step: float = math.radians(0.5)) -> Tuple[float, Tuple[float, float, float]]:
    """
    자세 pose 주변 ±radius, ±yaw_range 격자에서 인라이어 비율의 최댓값과 그 자세.

    가설(탐색 격자 0.1 m × 1°)과 수렴한 AMCL 자세는 양자화가 달라 그대로 비교하면 격자에 걸친 쪽이 불리하다
    (대칭 통로 시험: 같은 별칭인데 ρ 0.99 vs 0.94). 양쪽을 같은 국소 탐색으로 올린 뒤 비교한다.
    """
    if len(beams) == 0:
        return 0.0, pose
    k = int(round(radius / step))
    dx, dy = [a.ravel() * step for a in np.meshgrid(np.arange(-k, k + 1), np.arange(-k, k + 1))]
    ky = int(round(yaw_range / yaw_step))
    best = (-1.0, pose)
    for j in range(-ky, ky + 1):
        yaw = pose[2] + j * yaw_step
        c, s = math.cos(yaw), math.sin(yaw)
        bx = c * beams[:, 0] - s * beams[:, 1]
        by = s * beams[:, 0] + c * beams[:, 1]
        ex = (pose[0] + dx)[:, None] + bx[None, :]
        ey = (pose[1] + dy)[:, None] + by[None, :]
        ratio = np.mean(field.lookup(ex.ravel(), ey.ravel()).reshape(ex.shape) <= inlier_dist,
                        axis=1)
        i = int(np.argmax(ratio))
        if ratio[i] > best[0]:
            best = (float(ratio[i]), (pose[0] + float(dx[i]), pose[1] + float(dy[i]), yaw))
    return best


def alias_margin(field: DistanceField, beams: np.ndarray, current: Tuple[float, float, float],
                 alternatives: Sequence[Tuple[float, float, float]],
                 inlier_dist: float) -> Optional[float]:
    """
    현재 추정의 (국소 정밀화한) 인라이어 비율 − 별칭 가설들의 (국소 정밀화한) 최대 인라이어 비율.

    별칭이 없으면 None. 작거나 음수면 같은 스캔으로 현재 추정과 별칭을 가를 수 없다.
    """
    if not alternatives or len(beams) == 0:
        return None
    cur, _ = refined_ratio(field, beams, current, inlier_dist)
    alt = max(refined_ratio(field, beams, p, inlier_dist)[0] for p in alternatives)
    return cur - alt


def base_beams(ranges: Sequence[float], angle_min: float, angle_increment: float,
               range_max: float, max_beams: int,
               sensor_offset: Tuple[float, float, float]) -> np.ndarray:
    """스캔 → base_footprint 프레임 끝점 (B, 2) (max_beams 로 균등 솎음)."""
    return _beams(ranges, angle_min, angle_increment, range_max, max_beams, sensor_offset)


def search(field: DistanceField, free: np.ndarray, ranges: Sequence[float], angle_min: float,
           angle_increment: float, range_max: float,
           sensor_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
           params: Optional[SeedParams] = None) -> List[Hypothesis]:
    """스캔 1 개로 전역 가설 목록 (인라이어 비율 내림차순, 최대 max_candidates)."""
    params = params or SeedParams()
    coarse = _beams(ranges, angle_min, angle_increment, range_max, params.coarse_beams,
                    sensor_offset)
    fine = _beams(ranges, angle_min, angle_increment, range_max, params.fine_beams,
                  sensor_offset)
    if len(coarse) < 10:
        return []
    rows, cols = candidate_cells(field, free, params.step_xy, params.clearance)
    if len(rows) == 0:
        return []
    spec = field.spec
    # 테두리 = 가장 긴 빔 (coarse/fine 은 서로 다른 빔 부분집합) + fine 탐색의 위치 이동 폭
    longest = max(float(np.max(np.hypot(b[:, 0], b[:, 1]))) for b in (coarse, fine))
    scorer = GridScorer(field, longest + params.step_xy)
    base = scorer.base_index(rows, cols)

    # coarse: 헤딩마다 모든 위치를 한 번에 (절사 없음 — 판별 빔 유지, brief §4.C (ii))
    seeds: List[Tuple[int, int, float, float]] = []   # (행, 열, yaw, 점수)
    for yaw in np.arange(-math.pi, math.pi, params.step_yaw):
        sc = scorer.scores(base, scorer.beam_offsets(coarse, float(yaw)), params.d_max)
        k = min(params.top_k, len(sc))
        for i in np.argpartition(sc, k - 1)[:k]:
            seeds.append((int(rows[i]), int(cols[i]), float(yaw), float(sc[i])))
    centers = cell_centers(spec, [h[0] for h in seeds], [h[1] for h in seeds])
    coarse_hyps = [Hypothesis(float(centers[i, 0]), float(centers[i, 1]), h[2], h[3],
                              cell=(h[0], h[1])) for i, h in enumerate(seeds)]
    kept = nms(coarse_hyps, params.nms_xy, params.nms_yaw, params.top_k)

    # fine: 후보 주변 ±step_xy (fine_step_xy 셀 간격), ±step_yaw (fine_step_yaw), 절사 평균
    fs = max(int(round(params.fine_step_xy / spec.resolution)), 1)
    kxy = int(round(params.step_xy / (fs * spec.resolution)))
    kyaw = int(round(params.step_yaw / params.fine_step_yaw))
    dr, dc = [a.ravel() * fs for a in np.meshgrid(np.arange(-kxy, kxy + 1),
                                                  np.arange(-kxy, kxy + 1), indexing='ij')]
    refined: List[Hypothesis] = []
    for h in kept:
        r0, c0 = h.cell
        rr, cc = r0 + dr, c0 + dc
        local = scorer.base_index(rr, cc)
        best = (math.inf, 0, 0.0)
        for j in range(-kyaw, kyaw + 1):
            yaw = h.yaw + j * params.fine_step_yaw
            sc = scorer.scores(local, scorer.beam_offsets(fine, yaw), params.d_max, params.trim)
            i = int(np.argmin(sc))
            if sc[i] < best[0]:
                best = (float(sc[i]), i, yaw)
        xy = cell_centers(spec, [rr[best[1]]], [cc[best[1]]])[0]
        pose = (float(xy[0]), float(xy[1]), math.atan2(math.sin(best[2]), math.cos(best[2])))
        refined.append(Hypothesis(*pose, best[0],
                                  inlier_ratio(field, fine, pose, params.rank_inlier_dist),
                                  cell=(int(rr[best[1]]), int(cc[best[1]]))))
    # 최종 순위는 절사하지 않은 인라이어 비율: 절사 평균은 가림에 강하지만 별칭(대칭·주기 구조)을
    # 가르는 소수 빔까지 버려 별칭이 이길 수 있다 (Gazebo 납치 시험에서 실측, docs/algorithms/slam.md §5)
    return nms(refined, params.nms_xy, params.nms_yaw, params.max_candidates,
               key=lambda h: (-h.ratio, h.score))

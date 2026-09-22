"""
하네스 자체 지표 계산 (rclpy 비의존 순수 모듈).

명세 지표의 1차 측정은 amr_evaluation 로거(pose_error_logger, response_time_logger)가 하고
analyze 로 판정한다. 이 모듈은 프로브가 따로 모은 원시 기록으로 같은 지표를 독립 계산한다.
  - 교차 검증: 두 계산이 어긋나면 로거·하네스 어느 한쪽의 시간 정렬이 틀렸다는 뜻
  - 대체: amr_evaluation 이 설치되지 않은 워크스페이스에서도 시나리오가 판정을 낼 수 있게
구간 라벨·임계값은 amr_evaluation 의 기본값(config/amr_evaluation.yaml)과 같게 둔다.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

STOP = '정지'
STRAIGHT = '직선'
TURN = '회전'
MOTION_SEGMENTS = (STOP, STRAIGHT, TURN)

# 명세 4.3 기준 [m]
POSE_THRESHOLDS = {STOP: 0.03, STRAIGHT: 0.05, TURN: 0.08}

POSE_COLUMNS = ['timestamp', 'gt_x', 'gt_y', 'est_x', 'est_y', 'error', 'yaw_error', 'segment']
RESPONSE_COLUMNS = ['cmd_time', 'response_time', 'latency_ms', 'cmd_id']


@dataclass(frozen=True)
class MotionThresholds:
    """운동 상태 분류 임계값 (amr_evaluation segments.MotionThresholds 와 같은 기본값)."""

    stop_linear: float = 0.02       # [m/s]
    stop_angular: float = 0.02      # [rad/s]
    straight_angular: float = 0.05  # [rad/s]


def classify_motion(v: float, w: float, th: MotionThresholds = MotionThresholds()) -> str:
    """GT 속도 → 정지 / 직선 / 회전."""
    if abs(v) < th.stop_linear and abs(w) < th.stop_angular:
        return STOP
    if abs(w) < th.straight_angular:
        return STRAIGHT
    return TURN


def wrap(a: float) -> float:
    """(-pi, pi] 래핑."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Sample:
    """평면 자세 + 속도 표본 (t = header.stamp [s])."""

    t: float
    x: float
    y: float
    yaw: float
    v: float = 0.0
    w: float = 0.0


def yaw_of(q) -> float:
    """쿼터니언(x, y, z, w 속성) → yaw [rad]."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sample_from_odom(msg) -> Sample:
    """nav_msgs/Odometry (속성 접근만) → Sample. v 는 평면 속도 크기 (twist 는 child 프레임)."""
    st = msg.header.stamp
    p = msg.pose.pose.position
    lin = msg.twist.twist.linear
    return Sample(st.sec + st.nanosec * 1e-9, p.x, p.y, yaw_of(msg.pose.pose.orientation),
                  math.hypot(lin.x, lin.y), msg.twist.twist.angular.z)


def interpolate(track: Sequence[Sample], t: float, max_gap: float = 0.2) -> Optional[Sample]:
    """
    시간순 track 을 t 에서 선형 보간한다.

    t 가 범위 밖이거나(외삽 금지) 둘러싼 두 표본 간격이 max_gap 보다 크면 None.
    yaw 는 최단 각 차이로 보간한다.
    """
    n = len(track)
    if n == 0 or t < track[0].t or t > track[-1].t:
        return None
    times = [s.t for s in track]
    i = int(np.searchsorted(times, t))
    if i < n and track[i].t == t:
        return track[i]
    a, b = track[i - 1], track[i]
    if b.t - a.t > max_gap or b.t <= a.t:
        return None
    r = (t - a.t) / (b.t - a.t)
    return Sample(t, a.x + r * (b.x - a.x), a.y + r * (b.y - a.y),
                  wrap(a.yaw + r * wrap(b.yaw - a.yaw)),
                  a.v + r * (b.v - a.v), a.w + r * (b.w - a.w))


@dataclass(frozen=True)
class PoseErrorRow:
    """pose_error.csv 한 행."""

    t: float
    gt: Sample
    est: Sample
    error: float
    yaw_error: float
    segment: str

    def as_list(self) -> list:
        return [self.t, self.gt.x, self.gt.y, self.est.x, self.est.y, self.error,
                self.yaw_error, self.segment]


def pair_pose_errors(gt: Sequence[Sample], est: Sequence[Sample],
                     th: MotionThresholds = MotionThresholds(),
                     max_gap: float = 0.2) -> List[PoseErrorRow]:
    """추정 표본마다 GT 를 그 스탬프에 보간해 오차 행을 만든다 (보간 불가 표본은 버림)."""
    gt_sorted = sorted(gt, key=lambda s: s.t)
    rows = []
    for e in sorted(est, key=lambda s: s.t):
        g = interpolate(gt_sorted, e.t, max_gap)
        if g is None:
            continue
        rows.append(PoseErrorRow(e.t, g, e, math.hypot(e.x - g.x, e.y - g.y),
                                 abs(wrap(e.yaw - g.yaw)), classify_motion(g.v, g.w, th)))
    return rows


@dataclass(frozen=True)
class Summary:
    """오차 요약 통계."""

    count: int
    rmse: float = math.nan
    max: float = math.nan
    mean: float = math.nan
    p95: float = math.nan

    def as_dict(self, scale: float = 1.0, digits: int = 4) -> Dict[str, float]:
        def r(v: float) -> float:
            return round(v * scale, digits) if math.isfinite(v) else v
        return {'count': self.count, 'rmse': r(self.rmse), 'max': r(self.max),
                'mean': r(self.mean), 'p95': r(self.p95)}


def summarize(values: Sequence[float]) -> Summary:
    """유한한 값들의 RMSE / 최대 / 평균 / 95 백분위."""
    arr = np.asarray([v for v in values if math.isfinite(v)], dtype=float)
    if arr.size == 0:
        return Summary(0)
    return Summary(int(arr.size), float(np.sqrt(np.mean(arr ** 2))), float(np.max(arr)),
                   float(np.mean(arr)), float(np.percentile(arr, 95)))


def pose_summary(rows: Sequence[PoseErrorRow]) -> Dict[str, Summary]:
    """구간별 + 전체('전체') 위치 오차 요약."""
    out = {seg: summarize([r.error for r in rows if r.segment == seg])
           for seg in MOTION_SEGMENTS}
    out['전체'] = summarize([r.error for r in rows])
    return out


def pose_verdicts(summary: Dict[str, Summary], thresholds: Dict[str, float] = None,
                  min_samples: int = 100) -> List[Tuple[str, float, float, bool, int]]:
    """
    구간별 판정: (구간, RMSE, 기준, 통과, 표본 수).

    표본이 min_samples 미만인 구간은 불합격 (판정 근거 부족 = 시나리오 설계 오류).
    """
    thresholds = thresholds or POSE_THRESHOLDS
    out = []
    for seg, thr in thresholds.items():
        s = summary.get(seg, Summary(0))
        ok = s.count >= min_samples and math.isfinite(s.rmse) and s.rmse <= thr
        out.append((seg, s.rmse, thr, bool(ok), s.count))
    return out


def first_motion(track: Sequence[Sample], since: float, v_thr: float = 0.05,
                 w_thr: float = 0.1) -> Optional[Sample]:
    """시각 since 이후 |v| ≥ v_thr 또는 |w| ≥ w_thr 인 첫 표본 (없으면 None)."""
    for s in track:
        if s.t >= since and (abs(s.v) >= v_thr or abs(s.w) >= w_thr):
            return s
    return None


def latency_summary(latencies_ms: Sequence[float]) -> Dict[str, float]:
    """지연 [ms] 요약: count / mean / max / p95 / min."""
    arr = np.asarray([v for v in latencies_ms if math.isfinite(v)], dtype=float)
    if arr.size == 0:
        return {'count': 0, 'mean': math.nan, 'max': math.nan, 'p95': math.nan,
                'min': math.nan}
    return {'count': int(arr.size), 'mean': round(float(np.mean(arr)), 2),
            'max': round(float(np.max(arr)), 2),
            'p95': round(float(np.percentile(arr, 95)), 2),
            'min': round(float(np.min(arr)), 2)}


def ttc_circle(px: float, py: float, vx: float, vy: float, radius: float) -> float:
    """
    등속 가정 접촉 시간 [s]: 상대 위치 p(장애물 − 로봇), 상대 속도 v 에서 |p + v·t| = radius 인 가장 이른 t ≥ 0.

    이미 radius 안이면 0, 다가오지 않거나 스치지 않으면 inf (명세 4.7 TTC — 시나리오 08 의 조우 기록).
    """
    c = px * px + py * py - radius * radius
    if c <= 0.0:
        return 0.0
    a = vx * vx + vy * vy
    b = 2.0 * (px * vx + py * vy)
    if a <= 1e-12 or b >= 0.0:
        return math.inf
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return math.inf
    return (-b - math.sqrt(disc)) / (2.0 * a)


@dataclass(frozen=True)
class DeviationEpisode:
    """계획 경로 이탈 구간: 이탈 > out_thr 로 나갔다가 ≤ back_thr 로 돌아올 때까지."""

    t_start: float
    t_peak: float
    peak: float
    t_end: Optional[float]      # None = 시행 끝까지 돌아오지 않음

    @property
    def returned(self) -> bool:
        return self.t_end is not None

    def return_time(self, t_last: float) -> float:
        """최대 이탈 시각 → 복귀 시각 [s] (돌아오지 않았으면 t_last 까지 — 판정은 returned 로)."""
        return (self.t_end if self.t_end is not None else t_last) - self.t_peak


def deviation_episodes(times: Sequence[float], devs: Sequence[float], out_thr: float = 0.3,
                       back_thr: float = 0.15) -> List[DeviationEpisode]:
    """
    시간순 (t, 이탈) 에서 이탈 구간 목록 (명세 4.7 "원래 경로로 5 s 이내 복귀" 판정 입력).

    이탈 > out_thr 이면 구간 시작, 최대점을 따라가다 이탈 ≤ back_thr 이면 끝 (히스테리시스).
    """
    out: List[DeviationEpisode] = []
    cur = None
    for t, d in zip(times, devs):
        if not math.isfinite(d):
            continue
        if cur is None:
            if d > out_thr:
                cur = [t, t, d]
            continue
        if d > cur[2]:
            cur[1], cur[2] = t, d
        if d <= back_thr:
            out.append(DeviationEpisode(cur[0], cur[1], cur[2], t))
            cur = None
    if cur is not None:
        out.append(DeviationEpisode(cur[0], cur[1], cur[2], None))
    return out


CONTACT_MOVING_V = 0.05     # [m/s] 접촉 순간 로봇이 움직이고 있었다고 보는 GT 속도


def speeds_at(track: Sequence[Sample], times: Sequence[float],
              max_gap: float = 0.2) -> List[float]:
    """
    시각마다 GT 평면 속도 |v| (보간, 표본이 없거나 간격이 max_gap 초과면 NaN).

    08 접촉 분류용: 접촉 = 로봇이 움직이며 부딪침(|v| > CONTACT_MOVING_V) 인지, 정지한 로봇에 장애물이
    와서 닿음인지. 판정(접촉 0)은 바꾸지 않고 원인 귀속 근거로만 남긴다.
    """
    out = []
    for t in times:
        s = interpolate(track, t, max_gap)
        out.append(abs(s.v) if s is not None else math.nan)
    return out


LANE_MOVING_V = 0.05        # [m/s] 이보다 느린 장애물은 진행축이 없다 (차선 좌표 없음)


def lane_coords(ox: float, oy: float, vx: float, vy: float,
                rx: float, ry: float) -> Tuple[float, float]:
    """
    장애물 진행축 기준 로봇 좌표 (along: 진행 방향 +, lateral: 진행 방향 왼쪽 +).

    08 접촉 원인 귀속용: |lateral| 이 두 반지름 합보다 작으면 로봇이 장애물이 지나갈 자리에 서 있었다는
    뜻이고(정지선을 못 지킴), 크면 장애물이 옆에서 들어온 것이다. 멈춘 장애물은 축이 없어 (NaN, NaN).
    """
    speed = math.hypot(vx, vy)
    if speed < LANE_MOVING_V:
        return math.nan, math.nan
    ux, uy = vx / speed, vy / speed
    dx, dy = rx - ox, ry - oy
    return dx * ux + dy * uy, -dx * uy + dy * ux

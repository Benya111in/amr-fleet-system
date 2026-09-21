"""
성능 지표 계산 (rclpy 비의존 순수 모듈).

- 위치 추정 오차: GT 를 추정 시각에 선형 보간(외삽 금지)한 뒤 위치·헤딩 오차
- 경로 추종 CTE: GT 위치에서 계획 경로(폴리라인) 최근접 선분까지의 부호 있는 수직 거리
- 응답 시간: 작업 명령 시각 → 첫 움직임 시각
- CPU 사용률: /proc/stat 두 스냅샷의 차분
- 요약 통계: mean / RMSE / max / p95, SE(2) 정렬
"""

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Deque, Dict, List, Optional, Sequence, Tuple

from amr_evaluation import segments
import numpy as np


def wrap_angle(a: float) -> float:
    """각도를 (-pi, pi] 로 래핑."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → yaw [rad] (ZYX 오일러의 Z)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_to_sec(sec: int, nanosec: int) -> float:
    """builtin_interfaces/Time 필드 → float [s]."""
    return float(sec) + float(nanosec) * 1e-9


@dataclass(frozen=True)
class PoseSample:
    """시각이 찍힌 2D 자세 + 속도 (GT/추정 공용)."""

    t: float
    x: float
    y: float
    yaw: float = 0.0
    v: float = 0.0      # [m/s] 전진 속도 (body frame)
    w: float = 0.0      # [rad/s] yaw 속도


def interpolate_pose(a: PoseSample, b: PoseSample, t: float) -> PoseSample:
    """
    a.t ≤ t ≤ b.t 인 두 샘플 사이를 선형 보간한다 (yaw 는 최단 호).

    t 가 구간 밖이면 ValueError — 외삽은 하지 않는다.
    """
    if not (a.t <= t <= b.t):
        raise ValueError(f'interpolation time {t} outside [{a.t}, {b.t}]')
    if b.t == a.t:
        return a
    r = (t - a.t) / (b.t - a.t)
    dyaw = wrap_angle(b.yaw - a.yaw)
    return PoseSample(
        t=t,
        x=a.x + r * (b.x - a.x),
        y=a.y + r * (b.y - a.y),
        yaw=wrap_angle(a.yaw + r * dyaw),
        v=a.v + r * (b.v - a.v),
        w=a.w + r * (b.w - a.w),
    )


class PoseBuffer:
    """
    시간순 자세 버퍼. 임의 시각의 자세를 선형 보간으로 돌려준다.

    max_age [s] 보다 오래된 샘플은 버리고, 이웃 샘플 간격이 max_gap [s] 를 넘는 구간은
    보간을 거부한다 (끊긴 GT 사이를 이어 붙이면 가짜 오차가 생긴다).
    """

    def __init__(self, max_age: float = 5.0, max_gap: float = 0.2):
        self.max_age = max_age
        self.max_gap = max_gap
        self._buf: Deque[PoseSample] = deque()

    def __len__(self) -> int:
        return len(self._buf)

    @property
    def oldest_time(self) -> Optional[float]:
        return self._buf[0].t if self._buf else None

    @property
    def latest_time(self) -> Optional[float]:
        return self._buf[-1].t if self._buf else None

    def add(self, sample: PoseSample) -> None:
        """샘플 추가. 시각이 역행하면(시뮬 리셋 등) 버퍼를 비우고 다시 시작한다."""
        if self._buf and sample.t < self._buf[-1].t:
            self._buf.clear()
        self._buf.append(sample)
        while self._buf and sample.t - self._buf[0].t > self.max_age:
            self._buf.popleft()

    def interpolate(self, t: float) -> Optional[PoseSample]:
        """시각 t 를 감싸는 두 샘플이 있으면 보간값, 없으면(범위 밖·간격 초과) None."""
        if len(self._buf) < 1:
            return None
        if t < self._buf[0].t or t > self._buf[-1].t:
            return None
        times = [s.t for s in self._buf]
        k = int(np.searchsorted(times, t, side='left'))
        if times[k] == t:
            return self._buf[k]
        a, b = self._buf[k - 1], self._buf[k]
        if b.t - a.t > self.max_gap:
            return None
        return interpolate_pose(a, b, t)


def position_error(gt: PoseSample, est: PoseSample) -> float:
    """유클리드 위치 오차 [m]."""
    return math.hypot(gt.x - est.x, gt.y - est.y)


def yaw_error(gt_yaw: float, est_yaw: float) -> float:
    """헤딩 오차 [rad], 부호 있음 (추정 − GT), (-pi, pi]."""
    return wrap_angle(est_yaw - gt_yaw)


@dataclass
class PoseErrorRow:
    """pose_error.csv 한 행 (명세 열 + 추가 열)."""

    timestamp: float
    gt_x: float
    gt_y: float
    est_x: float
    est_y: float
    error: float
    yaw_error: float
    segment: str

    def as_list(self) -> list:
        return [self.timestamp, self.gt_x, self.gt_y, self.est_x, self.est_y,
                self.error, self.yaw_error, self.segment]


class PoseErrorPairer:
    """
    GT 스트림과 추정 스트림을 시간 정렬해 오차 행을 만든다.

    추정 샘플은 대기열에 두었다가 GT 가 그 시각을 지나친 뒤에만 보간한다 (최근접 GT 샘플로
    외삽하지 않는다). GT 범위 앞에 놓인(너무 늦게 도착한) 추정은 버리고 개수만 센다.
    """

    def __init__(self, thresholds: segments.MotionThresholds = segments.MotionThresholds(),
                 gt_max_age: float = 5.0, gt_max_gap: float = 0.2, max_wait: float = 1.0):
        self.gt = PoseBuffer(gt_max_age, gt_max_gap)
        self.thresholds = thresholds
        self.max_wait = max_wait
        self._pending: Deque[PoseSample] = deque()
        self.dropped = 0

    def add_ground_truth(self, sample: PoseSample) -> None:
        self.gt.add(sample)

    def add_estimate(self, sample: PoseSample) -> None:
        self._pending.append(sample)

    @property
    def pending(self) -> int:
        return len(self._pending)

    def flush(self) -> List[PoseErrorRow]:
        """보간 가능한 대기 추정을 모두 처리해 행 목록을 돌려준다."""
        rows: List[PoseErrorRow] = []
        latest = self.gt.latest_time
        if latest is None:
            return rows
        keep: Deque[PoseSample] = deque()
        while self._pending:
            est = self._pending.popleft()
            if est.t > latest:
                # GT 가 아직 이 시각에 도달하지 않았다 — 기다린다 (max_wait 이상 앞서면 폐기)
                if est.t - latest > self.max_wait:
                    self.dropped += 1
                else:
                    keep.append(est)
                continue
            gt = self.gt.interpolate(est.t)
            if gt is None:
                self.dropped += 1
                continue
            rows.append(PoseErrorRow(
                timestamp=est.t, gt_x=gt.x, gt_y=gt.y, est_x=est.x, est_y=est.y,
                error=position_error(gt, est), yaw_error=yaw_error(gt.yaw, est.yaw),
                segment=segments.classify_motion(gt.v, gt.w, self.thresholds)))
        self._pending = keep
        return rows


@dataclass(frozen=True)
class CrossTrackResult:
    """CTE 계산 결과."""

    cte: float               # [m] 부호 있음 (경로 진행 방향 기준 좌측 +)
    planned_x: float         # 최근접 경로점
    planned_y: float
    segment_index: int       # 최근접 선분 (정점 i → i+1); 정점 하나뿐이면 0


def cross_track_error(points: np.ndarray, px: float, py: float) -> Optional[CrossTrackResult]:
    """
    점 (px, py) 에서 폴리라인까지의 부호 있는 최단 거리.

    각 선분에 대해 사영 매개변수를 [0, 1] 로 잘라 최근접점을 구하고(양 끝 처리), 거리가 가장
    짧은 선분을 고른다. 부호는 그 선분의 진행 방향 벡터와의 외적 z 성분: 좌측이 +.
    빈 경로면 None, 정점 하나면 그 점까지의 거리(부호 +).
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    n = len(pts)
    if n == 0:
        return None
    p = np.array([px, py], dtype=float)
    if n == 1:
        d = float(np.linalg.norm(p - pts[0]))
        return CrossTrackResult(d, float(pts[0, 0]), float(pts[0, 1]), 0)
    a = pts[:-1]
    d = pts[1:] - a
    seg_len2 = np.einsum('ij,ij->i', d, d)
    rel = p - a
    dots = np.einsum('ij,ij->i', rel, d)
    t = np.where(seg_len2 > 1e-12, dots / np.maximum(seg_len2, 1e-12), 0.0)
    t = np.clip(t, 0.0, 1.0)
    closest = a + t[:, None] * d
    dist = np.linalg.norm(p - closest, axis=1)
    i = int(np.argmin(dist))
    # 진행 방향: 길이 0 선분이면 이웃 선분의 방향을 빌린다
    direction = d[i]
    if seg_len2[i] <= 1e-12:
        for j in list(range(i + 1, n - 1)) + list(range(i - 1, -1, -1)):
            if seg_len2[j] > 1e-12:
                direction = d[j]
                break
    cross = direction[0] * rel[i][1] - direction[1] * rel[i][0]
    sign = 1.0 if cross >= 0.0 else -1.0
    return CrossTrackResult(sign * float(dist[i]), float(closest[i, 0]), float(closest[i, 1]), i)


@dataclass
class ResponseRow:
    """response_time.csv 한 행 (명세 열 + 추가 열 cmd_id)."""

    cmd_time: float
    response_time: float
    latency_ms: float
    cmd_id: str = ''

    def as_list(self) -> list:
        return [self.cmd_time, self.response_time, self.latency_ms, self.cmd_id]


@dataclass
class _PendingCommand:
    t: float
    cmd_id: str


class ResponseTimeMatcher:
    """
    명령 시각과 "첫 움직임" 시각을 짝지어 지연을 계산한다.

    움직임 판정: |v| ≥ linear_threshold 또는 |w| ≥ angular_threshold.
    require_rest=True 면 명령 도착 시점에 이미 움직이고 있던 경우(직전 운동 샘플이 움직임)
    는 측정하지 않는다 (명세의 "명령 수신 ~ 로봇 반응" 은 정지 상태에서 출발하는 지연).
    timeout [s] 안에 움직임이 없으면 미응답으로 버리고 개수만 센다.
    """

    def __init__(self, linear_threshold: float = 0.05, angular_threshold: float = 0.1,
                 require_rest: bool = True, timeout: float = 10.0):
        self.linear_threshold = linear_threshold
        self.angular_threshold = angular_threshold
        self.require_rest = require_rest
        self.timeout = timeout
        self._pending: List[_PendingCommand] = []
        self._last_moving = False
        self.skipped_moving = 0
        self.timed_out = 0

    def is_moving(self, v: float, w: float) -> bool:
        return abs(v) >= self.linear_threshold or abs(w) >= self.angular_threshold

    def add_command(self, t: float, cmd_id: str = '') -> bool:
        """명령 등록. 측정 대상으로 받아들였으면 True."""
        if self.require_rest and self._last_moving:
            self.skipped_moving += 1
            return False
        self._pending.append(_PendingCommand(t, cmd_id))
        return True

    def add_motion(self, t: float, v: float, w: float) -> List[ResponseRow]:
        """운동 샘플 입력. 이 샘플로 응답이 확정된 명령들의 행을 돌려준다."""
        rows: List[ResponseRow] = []
        moving = self.is_moving(v, w)
        still_pending: List[_PendingCommand] = []
        for cmd in self._pending:
            if t - cmd.t > self.timeout:
                self.timed_out += 1
                continue
            if moving and t >= cmd.t:
                rows.append(ResponseRow(cmd.t, t, (t - cmd.t) * 1000.0, cmd.cmd_id))
                continue
            still_pending.append(cmd)
        self._pending = still_pending
        self._last_moving = moving
        return rows

    @property
    def pending(self) -> int:
        return len(self._pending)


def parse_proc_stat(text: str) -> Dict[str, List[int]]:
    """/proc/stat 본문 → {'cpu': [...], 'cpu0': [...], ...} (jiffies 정수 열)."""
    out: Dict[str, List[int]] = {}
    for line in text.splitlines():
        parts = line.split()
        if not parts or not parts[0].startswith('cpu'):
            continue
        out[parts[0]] = [int(p) for p in parts[1:]]
    return out


def cpu_percent(prev: Sequence[int], cur: Sequence[int]) -> float:
    """
    두 /proc/stat 스냅샷 사이의 CPU 사용률 [%].

    busy = total − idle − iowait (user nice system idle iowait irq softirq steal ...).
    총 jiffies 변화가 0 이면 0.
    """
    p = list(prev) + [0] * (8 - len(prev))
    c = list(cur) + [0] * (8 - len(cur))
    idle_prev = p[3] + p[4]
    idle_cur = c[3] + c[4]
    total_prev = sum(p[:8])
    total_cur = sum(c[:8])
    d_total = total_cur - total_prev
    if d_total <= 0:
        return 0.0
    d_idle = idle_cur - idle_prev
    return max(0.0, min(100.0, 100.0 * (d_total - d_idle) / d_total))


@dataclass
class Summary:
    """값 배열의 요약 통계."""

    count: int = 0
    mean: float = float('nan')
    rmse: float = float('nan')
    max: float = float('nan')
    p95: float = float('nan')
    extras: Dict[str, float] = field(default_factory=dict)


def summarize(values: Sequence[float]) -> Summary:
    """절댓값 기준 mean / RMSE / max / p95. 빈 입력이면 count=0 과 NaN."""
    arr = np.abs(np.asarray(values, dtype=float))
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return Summary()
    return Summary(
        count=int(arr.size),
        mean=float(np.mean(arr)),
        rmse=float(math.sqrt(np.mean(arr ** 2))),
        max=float(np.max(arr)),
        p95=float(np.percentile(arr, 95)),
    )


def se2_align(src: np.ndarray, dst: np.ndarray) -> Tuple[float, float, float]:
    """
    점 집합 src 를 dst 에 최소제곱으로 맞추는 강체 변환 (yaw, tx, ty) — Umeyama, 스케일 없음.

    dst ≈ R(yaw)·src + t. map↔world 프레임의 상수 오프셋을 보정할 때 쓴다.
    """
    s = np.asarray(src, dtype=float).reshape(-1, 2)
    d = np.asarray(dst, dtype=float).reshape(-1, 2)
    if len(s) < 2 or len(s) != len(d):
        raise ValueError('need at least two matching point pairs')
    ms, md = s.mean(axis=0), d.mean(axis=0)
    h = (s - ms).T @ (d - md)
    u, _, vt = np.linalg.svd(h)
    sign = 1.0 if np.linalg.det(vt.T @ u.T) >= 0 else -1.0
    r = vt.T @ np.diag([1.0, sign]) @ u.T
    yaw = math.atan2(r[1, 0], r[0, 0])
    t = md - r @ ms
    return yaw, float(t[0]), float(t[1])


def apply_se2(points: np.ndarray, yaw: float, tx: float, ty: float) -> np.ndarray:
    """(N, 2) 점들에 SE(2) 변환 적용."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]])
    return p @ r.T + np.array([tx, ty])

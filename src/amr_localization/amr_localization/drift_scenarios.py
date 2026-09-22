"""
드리프트 실험 주행 시나리오 (rclpy 비의존 — 운동학 시뮬레이터로 pytest 검증).

시나리오는 구간(Segment)의 목록이고, 구간은 지면 진실(GT) 자세를 되먹임해 목표 거리/각을 정확히
채운다 (오도메트리를 되먹임하면 측정 대상 오차가 주행 경로에 섞인다).
  straight L : 사다리꼴 속도 v = min(v_max, v_prev + aΔt, sqrt(2a·남은 거리)), 헤딩 유지 P 제어
  rotate Θ   : 같은 사다리꼴로 ω 를 내며 GT 헤딩 변화(언랩 누적)가 Θ 에 이르면 정지
  pause T    : 정지 유지 (시작/끝 정착 — 정지 오차 측정 구간)
log=False 인 구간(반복 사이 방향 되돌리기 등)은 CSV 에 기록하지 않는다.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Tuple

Pose = Tuple[float, float, float]


def wrap(a: float) -> float:
    """각을 (−π, π] 로."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class MotionLimits:
    """속도·가속 한계 (robot_params.yaml limits 이내로 보수적으로)."""

    linear_speed: float = 0.5        # [m/s]
    angular_speed: float = 0.5       # [rad/s]
    linear_accel: float = 0.5        # [m/s²]
    angular_accel: float = 1.0       # [rad/s²]
    min_linear: float = 0.03         # [m/s] 목표 직전 최소 속도 (정지 마찰로 멈추지 않게)
    min_angular: float = 0.05        # [rad/s]
    heading_gain: float = 2.0        # [1/s] 직진 헤딩 유지
    max_heading_rate: float = 0.3    # [rad/s]
    distance_tol: float = 0.003      # [m]
    angle_tol: float = 0.002         # [rad]


@dataclass
class Segment:
    """주행 구간."""

    kind: str                  # 'straight' | 'rotate' | 'pause'
    amount: float              # [m] | [rad] (부호 = 방향) | [s]
    log: bool = True


@dataclass
class _SegmentState:
    start_pose: Pose
    start_t: float
    progress: float = 0.0
    last_yaw: float = 0.0


@dataclass
class ScenarioRunner:
    """구간 목록을 GT 되먹임으로 실행한다. step() 을 제어 주기마다 부른다."""

    segments: List[Segment]
    limits: MotionLimits = field(default_factory=MotionLimits)

    def __post_init__(self) -> None:
        """상태 초기화."""
        self.index = 0
        self.state: Optional[_SegmentState] = None
        self.v = 0.0
        self.w = 0.0
        self.last_t: Optional[float] = None

    @property
    def done(self) -> bool:
        """모든 구간 완료."""
        return self.index >= len(self.segments)

    @property
    def logging(self) -> bool:
        """현재 구간을 기록하는지."""
        return not self.done and self.segments[self.index].log

    def step(self, t: float, pose: Pose) -> Tuple[float, float]:
        """GT 자세 → (v, ω) 명령. 완료 후에는 (0, 0)."""
        dt = 0.0 if self.last_t is None else max(t - self.last_t, 0.0)
        self.last_t = t
        while not self.done:
            seg = self.segments[self.index]
            if self.state is None:
                self.state = _SegmentState(pose, t, 0.0, pose[2])
            finished, v, w = self._run(seg, t, dt, pose)
            if not finished:
                self.v, self.w = v, w
                return v, w
            self.index += 1
            self.state = None
            self.v = self.w = 0.0
            if seg.kind != 'pause':
                return 0.0, 0.0  # 구간 경계에서 한 주기 정지 명령
        return 0.0, 0.0

    def _run(self, seg: Segment, t: float, dt: float, pose: Pose) -> Tuple[bool, float, float]:
        lim = self.limits
        st = self.state
        if seg.kind == 'pause':
            return t - st.start_t >= seg.amount, 0.0, 0.0
        if seg.kind == 'straight':
            h0 = st.start_pose[2]
            sign = 1.0 if seg.amount >= 0.0 else -1.0
            s = ((pose[0] - st.start_pose[0]) * math.cos(h0)
                 + (pose[1] - st.start_pose[1]) * math.sin(h0)) * sign
            remaining = abs(seg.amount) - s
            if remaining <= lim.distance_tol:
                return True, 0.0, 0.0
            v = min(lim.linear_speed, abs(self.v) + lim.linear_accel * dt,
                    math.sqrt(2.0 * lim.linear_accel * remaining))
            v = max(v, lim.min_linear)
            w = lim.heading_gain * wrap(h0 - pose[2])
            w = max(-lim.max_heading_rate, min(lim.max_heading_rate, w))
            return False, sign * v, w
        if seg.kind == 'rotate':
            st.progress += wrap(pose[2] - st.last_yaw)
            st.last_yaw = pose[2]
            sign = 1.0 if seg.amount >= 0.0 else -1.0
            remaining = abs(seg.amount) - sign * st.progress
            if remaining <= lim.angle_tol:
                return True, 0.0, 0.0
            w = min(lim.angular_speed, abs(self.w) + lim.angular_accel * dt,
                    math.sqrt(2.0 * lim.angular_accel * remaining))
            w = max(w, lim.min_angular)
            return False, 0.0, sign * w
        raise ValueError(f'unknown segment kind {seg.kind!r}')


def build_scenario(name: str, rep: int, straight_length: float = 10.0,
                   rotation_angle: float = 2.0 * math.pi,
                   rect: Tuple[float, float] = (5.0, 4.0), settle: float = 1.0
                   ) -> List[Segment]:
    """
    시나리오 이름 → 구간 목록.

    straight   : L 직진. rep ≥ 1 이면 먼저 π 회전[미기록]해 직전 직진의 반대 방향으로 간다 — 다른
                 시나리오는 모두 시작 자세로 돌아오므로 직진만 왕복시키면 실험 영역이 L × (사각형) 안에
                 머문다 (홀수 rep 만 뒤집으면 rep 2 가 rep 1 과 같은 방향으로 더 나가 벽에 닿는다)
    rotate     : 제자리 Θ 회전 (rep 홀수면 반대 방향)
    square_ccw : a × b 직사각형 반시계 (a 먼저, 좌회전)
    square_cw  : 같은 직사각형 시계 방향 (먼저 π/2 좌회전[미기록] 후 b 먼저, 우회전, 끝에 원위치[미기록])
    """
    a, b = rect
    pause = Segment('pause', settle)
    if name == 'straight':
        pre = [Segment('rotate', math.pi, log=False)] if rep >= 1 else []
        return pre + [pause, Segment('straight', straight_length), pause]
    if name == 'rotate':
        sign = -1.0 if rep % 2 == 1 else 1.0
        return [pause, Segment('rotate', sign * rotation_angle), pause]
    if name == 'square_ccw':
        legs = []
        for side in (a, b, a, b):
            legs += [Segment('straight', side), Segment('rotate', math.pi / 2.0)]
        return [pause] + legs + [pause]
    if name == 'square_cw':
        legs = []
        for side in (b, a, b, a):
            legs += [Segment('straight', side), Segment('rotate', -math.pi / 2.0)]
        return ([Segment('rotate', math.pi / 2.0, log=False), pause] + legs + [pause]
                + [Segment('rotate', -math.pi / 2.0, log=False)])
    raise ValueError(f'unknown scenario {name!r}')

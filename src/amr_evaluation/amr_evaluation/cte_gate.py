"""
CTE 표본 채택 규칙 (rclpy 비의존 순수 모듈).

CTE 는 "로봇이 지금 그 계획 경로를 따라가고 있을 때" 의 수직 거리다. 작업 주기(sequences.md §1)의
MoveTo 뒤에는 DockAt(경로가 아니라 마커를 보고 cmd_vel 직접 제어)·적재 대기(5~15 s)·다음 MoveTo 가
이어지므로, 마지막으로 받은 plan 에 대고 매 GT 표본을 재면 주차·도킹·적재 중 표본이 섞여 CTE 판정을
뒤집는다. 아래 순서로 검사해 처음 걸린 사유로 표본을 버리고 사유별로 센다:

  no_plan       경로를 아직 받지 못했거나 정점이 부족하다
  nav_inactive  항법 상태 원천(<ns>/navigate_to_pose/_action/status)을 받았고, 최신 목표(수락 시각이
                가장 늦은 것)가 ACCEPTED/EXECUTING 이 아니다 (성공·취소·중단·취소 중·목표 없음)
  phase         executor/phase 를 받았고 값이 active_phases(기본 moving) 밖이다
                (docking / loading / charging / idle / error)
  stale_plan    경로가 최신 목표보다 오래됐다: plan 스탬프 < 최신 목표 수락 스탬프, 또는 plan 수신 시각
                + phase_grace < executor/phase 가 active 로 바뀐 수신 시각, 또는 plan 을 받은 지
                plan_timeout(기본 3 s) 이 지났다 — bt_navigator 가 주행 중 1 Hz 로 재계획하므로
                (components.md §3.3 RateController) 목표가 끝나면 경로가 더 오지 않는다
  stopped       GT |v| < min_speed 이고 |ω| < min_angular (정지·주차)
  endpoint      최근접점이 경로 시작점 앞/끝점 너머로 잘렸다 (그 거리는 종방향 — cte_logger 가 판정)

항법 상태·phase 를 한 번도 받지 못했으면 그 규칙은 건너뛰고 stale_plan(plan_timeout)·stopped·endpoint
만 적용한다. GT 시각이 크게 거꾸로 가면(시뮬 리셋) 경로·목표·phase 기록을 모두 지운다.
"""

from collections import Counter
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

NO_PLAN = 'no_plan'
NAV_INACTIVE = 'nav_inactive'
PHASE = 'phase'
STALE_PLAN = 'stale_plan'
STOPPED = 'stopped'
ENDPOINT = 'endpoint'
SKIP_REASONS = (NO_PLAN, NAV_INACTIVE, PHASE, STALE_PLAN, STOPPED, ENDPOINT)

# action_msgs/GoalStatus.status
GOAL_ACCEPTED = 1
GOAL_EXECUTING = 2
NAV_ACTIVE_STATES = (GOAL_ACCEPTED, GOAL_EXECUTING)


@dataclass(frozen=True)
class GateConfig:
    """채택 규칙 파라미터 (cte_logger 파라미터와 같은 이름·기본값)."""

    min_speed: float = 0.02            # [m/s]
    min_angular: float = 0.02          # [rad/s]
    # [s] 경로 수신 뒤 이 시간이 지나면 stale (0 이면 끔). 주행 중에는 1 Hz 재계획으로 1 s 마다 새로 오므로
    # 3 s 면 재계획 두 번 누락·계획 지연까지 견디고, 상태 원천이 없을 때 도착 뒤 옛 경로 채점을 막는다.
    plan_timeout: float = 3.0
    active_phases: Tuple[str, ...] = ('moving',)
    # [s] phase 가 moving 으로 바뀌기 직전에 받은 경로도 새 주행의 것으로 본다 (목표 전송과 phase 발행
    # 순서가 구현마다 달라서). 이전 주행의 경로는 도킹·적재(≥ 5 s) 를 사이에 두므로 구분된다.
    phase_grace: float = 1.0


class PlanGate:
    """경로·항법 상태·phase 수신 기록으로 CTE 표본을 채택/거부한다."""

    def __init__(self, config: GateConfig = GateConfig()):
        self.config = config
        self.skipped: Counter = Counter()
        self.accepted = 0
        self.reset()

    def reset(self) -> None:
        """시뮬 리셋 등: 경로·목표·phase 기록을 지운다 (카운터는 유지)."""
        self.plan_stamp: Optional[float] = None
        self.plan_rx: Optional[float] = None
        self.nav_seen = False
        self.nav_active = False
        self.goal_stamp: Optional[float] = None
        self.phase: Optional[str] = None
        self.phase_start_rx: Optional[float] = None

    # --- 입력 ---
    def on_plan(self, stamp: float, rx_time: float) -> None:
        """경로 수신. stamp 가 0 이면 수신 시각을 쓴다."""
        self.plan_stamp = stamp if stamp > 0.0 else rx_time
        self.plan_rx = rx_time

    def clear_plan(self) -> None:
        self.plan_stamp = None
        self.plan_rx = None

    def on_nav_status(self, goals: Iterable[Tuple[float, int]]) -> None:
        """navigate_to_pose 상태 목록 [(수락 스탬프, status), ...] → 최신 목표의 활성 여부."""
        goals = list(goals)
        self.nav_seen = True
        if not goals:
            self.nav_active = False
            return
        stamp, status = max(goals, key=lambda g: g[0])
        self.goal_stamp = stamp
        self.nav_active = int(status) in NAV_ACTIVE_STATES

    def on_phase(self, phase: str, rx_time: float) -> None:
        """executor/phase 수신. 비활성 → 활성 으로 바뀐 시각을 새 주행의 시작으로 기록한다."""
        phase = phase.strip().lower()
        was_active = self.phase is not None and self.phase in self.config.active_phases
        self.phase = phase
        if phase in self.config.active_phases and not was_active:
            self.phase_start_rx = rx_time

    # --- 판정 ---
    def check(self, rx_time: float, v: float, w: float) -> Optional[str]:
        """GT 표본 하나의 사전 판정: 버릴 사유 또는 None (endpoint 는 check_projection 에서)."""
        reason = self._reason(rx_time, v, w)
        if reason is not None:
            self.skipped[reason] += 1
        return reason

    def _reason(self, rx_time: float, v: float, w: float) -> Optional[str]:
        cfg = self.config
        if self.plan_rx is None:
            return NO_PLAN
        if self.nav_seen and not self.nav_active:
            return NAV_INACTIVE
        if self.phase is not None and self.phase not in cfg.active_phases:
            return PHASE
        if self.goal_stamp is not None and self.plan_stamp < self.goal_stamp:
            return STALE_PLAN
        if (self.phase_start_rx is not None
                and self.plan_rx + cfg.phase_grace < self.phase_start_rx):
            return STALE_PLAN
        if cfg.plan_timeout > 0.0 and rx_time - self.plan_rx > cfg.plan_timeout:
            return STALE_PLAN
        if abs(v) < cfg.min_speed and abs(w) < cfg.min_angular:
            return STOPPED
        return None

    def check_projection(self, clamped: bool) -> Optional[str]:
        """CTE 사영이 끝점에서 잘렸으면 endpoint, 아니면 채택."""
        if clamped:
            self.skipped[ENDPOINT] += 1
            return ENDPOINT
        self.accepted += 1
        return None

    def summary(self) -> str:
        """'채택 N, 제외 no_plan a / stopped b ...' 형식 요약."""
        parts = [f'{k} {self.skipped[k]}' for k in SKIP_REASONS if self.skipped[k]]
        return f'채택 {self.accepted}, 제외 ' + (' / '.join(parts) if parts else '0')


def goal_status_tuples(status_list: Sequence) -> list:
    """action_msgs/GoalStatusArray.status_list (덕 타이핑) → [(수락 스탬프 [s], status)]."""
    out = []
    for st in status_list:
        stamp = st.goal_info.stamp
        out.append((float(stamp.sec) + float(stamp.nanosec) * 1e-9, int(st.status)))
    return out

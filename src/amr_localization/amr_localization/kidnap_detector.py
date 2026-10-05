"""
납치(Kidnapped Robot) 감지·복구 상태기 (명세 4.3, rclpy 비의존).

kidnap_monitor_node 가 메시지를 넣고, 돌려받은 동작(Action)을 ROS 호출로 옮긴다.
감지 신호 3 가지 (docs/algorithms/slam.md §5):
  1. 공분산 급증  : AMCL 자세 공분산 trace(xx+yy) > cov_thresh 또는 var(yaw) > yaw_cov_thresh
  2. 점프         : 연속 두 AMCL 갱신 사이 AMCL 상대 이동과 odometry/filtered 상대 이동의 차
                    |Δp_amcl − Δp_odom| > jump_thresh 또는 |Δθ| 차 > jump_yaw_thresh
  3. 스캔-맵 불일치: 빔 끝점이 점유 셀에서 inlier_dist 이내인 비율 ρ 가 match_thresh 미만
                    match_window 회 연속 (정지 중 납치는 AMCL 이 갱신조차 하지 않으므로 이것이 주 신호)
상태: TRACKING → SUSPECT(suspect_time 지속) → LOST(재초기화 + 회전) → 수렴 확인 → TRACKING.
수렴: AMCL trace < converge_cov 이고 ρ ≥ converge_match 이며, 전역 탐색이 낸 다른 가설(별칭)들을 같은 odom
이동만큼 옮긴 자세보다 인라이어 비율이 converge_margin 이상 높은 것(alias_margin, 양쪽 모두 국소 정밀화 후 비교)이
converge_count 회 연속.
점대칭·주기 배치의 별칭은 ρ 0.70~0.93 이 나와 ρ 하한만으로는 틀린 가설을 받아들인다 (리뷰) — 별칭과 가를 수
없으면(여백 부족) 수렴을 선언하지 않고 lost 를 유지한다 (틀린 자세로 주행하는 것보다 안전 측).
회전이 끝나도 수렴하지 않으면 spin_retry 회까지 다시 돌고, 그래도 안 되면 전역 재초기화를
reinit_retry 회 반복한 뒤 FAILED (lost 유지, 수렴하면 언제든 복귀).
"""

from dataclasses import dataclass, field
import enum
import math
from typing import List, Optional, Sequence, Tuple

Pose = Tuple[float, float, float]  # (x [m], y [m], yaw [rad])


class State(enum.Enum):
    """감시 상태."""

    TRACKING = 'TRACKING'
    SUSPECT = 'SUSPECT'
    RECOVERING = 'RECOVERING'
    FAILED = 'FAILED'


class ActionType(enum.Enum):
    """노드가 수행할 동작."""

    PUBLISH_LOST = 'publish_lost'          # value: bool
    REINITIALIZE = 'reinitialize'          # value: 시도 번호 (1부터). 시드 initialpose 또는 AMCL 전역
    SPIN = 'spin'                          # Nav2 spin (value: 목표 회전각 [rad])
    CANCEL_SPIN = 'cancel_spin'
    SET_EKF_POSE = 'set_ekf_pose'          # value: (pose, cov_xy_trace, var_yaw)
    SEED_POSE = 'seed_pose'                # value: Pose — 외부 증거(마커)로 역산한 자세를 그대로 시드


@dataclass
class Action:
    """상태기 출력 동작."""

    kind: ActionType
    value: object = None


@dataclass
class KidnapParams:
    """감지·복구 파라미터 (config/kidnap_monitor.yaml 에 의미·근거)."""

    cov_thresh: float = 0.5            # [m²] AMCL var(x)+var(y) 상한 (σ ≈ 0.5 m)
    yaw_cov_thresh: float = 0.5        # [rad²] AMCL var(yaw) 상한 (σ ≈ 40°)
    jump_thresh: float = 1.0           # [m] AMCL/odom 상대 이동 불일치
    jump_yaw_thresh: float = 0.5       # [rad]
    match_thresh: float = 0.5          # 스캔-맵 인라이어 비율 하한
    match_window: int = 5              # 연속 저하 스캔 수
    min_match_beams: int = 30          # 유효 빔이 이보다 적으면 판정 보류
    # 스캔 정합기(scan_matcher_node) 거부율 경보 — match_ratio 와는 다른 신호다. 정합기는 360 빔
    # 점-대-면 정합의 **수렴 여부**를 보고, match_ratio 는 "지도 벽을 뚫은 빔" 비율만 본다.
    # 후자는 통로를 막은 미지 장애물 앞의 거짓 LOST 를 없애려고 "일찍 끝난 빔"을 분모에서 빼도록
    # 비대칭으로 만들었는데(scan_map_match.match_ratio 주석), 그 때문에 옆 통로로 6 m 옮겨졌을 때
    # 생기는 불일치를 거의 못 본다. 통합 11 실측: LOST 235 건 중 cov 0 · jump 0 · inlier 1 건뿐이고
    # 나머지 234 건이 전부 마커 보정이었다 — LiDAR 경로가 사실상 눈이 멀어 있었다.
    # 정합기 신호는 분리도가 크다: 정상 주행(6 m 벽 앞 정지 포함) 283~300/300 수락,
    # 납치 뒤 0/300 이 600 s 연속(reg1), 복구 직후 288/300 복귀.
    reg_reject_ratio: float = 0.8      # 창 안 거부 비율이 이보다 크면 경보
    reg_window: int = 20               # 판정 창 (스캔 수)
    reg_min_samples: int = 10          # 창에 이만큼 차야 판정
    # 마커 보정으로 LOST 를 선언하려면 정합기도 동의해야 한다: 창 안 거부 비율이 이보다 낮으면
    # (= 정합기가 "여기 맞다") 마커 쪽 오검출로 보고 무시한다.
    reg_marker_veto_ratio: float = 0.5
    suspect_time: float = 1.0          # [s] SUSPECT 지속 → LOST
    converge_cov: float = 0.1          # [m²] 수렴 판정 trace 상한
    converge_match: float = 0.7        # 수렴 판정 인라이어 비율 하한
    converge_margin: float = 0.05      # 수렴 판정: ρ(현재) − max ρ(별칭 가설) 하한 (국소 정밀화 후)
    converge_count: int = 5            # 연속 만족 횟수
    # [s] 바깥(운영자·상위 시스템)이 initialpose 로 자세를 잡아 준 뒤 이 시간 동안은 별칭 여백을
    # 요구하지 않는다. 여백 게이트는 **스스로** 고른 가설이 틀릴 수 있어서 두는 것인데, 외부 자세는
    # 스캔 정합 바깥에서 온 독립 증거라 그 애매함의 대상이 아니다 (마커 보정을 믿는 것과 같은 이유).
    # 이게 없으면 주기 별칭 자리에서는 참 자세를 알려 줘도 영원히 lost 다 — 지도에 ρ 차가
    # converge_margin 미만인 자리가 실제로 있다 (docs/research/state-estimation/checks/alias_margin.py:
    # test_12 피해 로봇 자리 0.034 < 0.05). 창은 LOST 재선언과 수렴 성공에서 닫힌다.
    external_trust_window: float = 30.0
    gate_log_period: float = 5.0          # 여백 게이트 판정을 로그에 남기는 최소 간격 [s]
    spin_angle: float = 2.0 * math.pi  # [rad] 복구 회전량
    spin_retry: int = 1                # 재초기화(시도) 한 번당 회전 횟수
    reinit_retry: int = 4              # 재초기화 시도 횟수 (노드: 앞 시도는 가설 시드, 마지막은 AMCL 전역)
    recovery_timeout: float = 90.0     # [s] LOST 이후 이 시간 안에 못 찾으면 FAILED
    cooldown: float = 3.0              # [s] 복구 직후 재감지 유예 (EKF 가 새 자세로 옮겨 가는 시간)
    # [m] 마커로 역산한 자세가 이보다 멀면 위치 추정이 틀렸다고 본다. 역산 자세 자체의 오차보다
    # 넉넉해야 한다: 재위치추정은 yaw 표준편차 12.8° 까지 받아들이는데 5 m 거리에서 그만큼이면
    # 위치로 1.1 m 라, 1.0 m 로 두면 정상 주행 중의 보정 오차가 잘못된 씨앗을 심는다 (통합 10
    # 실측: 도크 접근이 틀어져 마커를 못 보고 search_timeout, 위치 오차 1.84 m). 실제 납치는
    # 통로 간격(6 m) 규모라 2.5 m 로도 충분히 걸린다.
    marker_fix_min_error: float = 2.5
    # [s] 불일치가 이만큼 이어져야 납치로 본다. 한 프레임만 보고 움직이면 정상 상황의 전이를
    # 납치로 오판한다 — 로봇을 옮기고 곧바로 초기 자세를 알려 주는 구간에서 보정이 먼저 도착하면
    # 32~57 m 불일치가 잠깐 보인다 (통합 10 실측: 그때마다 360° 회전 복구가 시작돼 도킹이 실패).
    # 진짜 납치는 알려 주는 쪽이 없으므로 불일치가 사라지지 않는다.
    marker_fix_persist: float = 0.6


@dataclass
class Event:
    """감지·복구 이력 한 건 (로그/리포트용)."""

    t: float
    state: str
    reason: str


@dataclass
class StampedPose:
    t: float
    pose: Pose


def wrap_angle(a: float) -> float:
    """각을 (−π, π] 로."""
    return math.atan2(math.sin(a), math.cos(a))


def relative_pose(a: Pose, b: Pose) -> Pose:
    """상대 자세 T_a⁻¹ T_b (a 좌표계에서 본 b)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap_angle(b[2] - a[2]))


def compose(a: Pose, b: Pose) -> Pose:
    """T_a T_b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2]))


def interpolate_pose(samples: Sequence[StampedPose], t: float) -> Optional[Pose]:
    """시각 t 의 자세를 선형 보간 (범위 밖이면 가장 가까운 끝, 비었으면 None)."""
    if not samples:
        return None
    if t <= samples[0].t:
        return samples[0].pose
    if t >= samples[-1].t:
        return samples[-1].pose
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid].t <= t:
            lo = mid
        else:
            hi = mid
    a, b = samples[lo], samples[hi]
    u = (t - a.t) / (b.t - a.t) if b.t > a.t else 0.0
    yaw = a.pose[2] + u * wrap_angle(b.pose[2] - a.pose[2])
    return (a.pose[0] + u * (b.pose[0] - a.pose[0]),
            a.pose[1] + u * (b.pose[1] - a.pose[1]), wrap_angle(yaw))


def jump_discrepancy(amcl_prev: Pose, amcl_now: Pose,
                     odom_prev: Pose, odom_now: Pose) -> Tuple[float, float]:
    """
    AMCL 상대 이동과 odom 상대 이동의 차 (위치 [m], 헤딩 [rad]).

    map→odom 오프셋과 무관하게 두 추정의 '증분'만 비교하므로 정상 추적 중에는 AMCL 보정량
    (수 cm) 수준이고, 납치 복구·오수렴 시에는 이동 거리 수준으로 커진다.
    """
    da = relative_pose(amcl_prev, amcl_now)
    do = relative_pose(odom_prev, odom_now)
    return math.hypot(da[0] - do[0], da[1] - do[1]), abs(wrap_angle(da[2] - do[2]))


@dataclass
class StopUpdateScheduler:
    """
    정지 직후 AMCL 무이동 갱신(request_nomotion_update) 일정.

    AMCL 은 update_min_d/a 를 넘게 움직일 때만 갱신하므로 정지하면 마지막 갱신의 오차가 그대로 굳는다.
    움직이다 settle_time 동안 |v| < v_thresh, |ω| < w_thresh 이면 period 간격으로 count 번 갱신을 요청해
    같은 자세의 스캔으로 파티클을 한 번 더 모은다 (연속 호출은 파티클 고갈을 부르므로 횟수 제한).
    """

    count: int = 3                 # 정지당 요청 횟수 (0 = 끔)
    period: float = 1.0            # [s]
    settle_time: float = 0.5       # [s]
    v_thresh: float = 0.01         # [m/s]
    w_thresh: float = 0.01         # [rad/s]

    def __post_init__(self) -> None:
        """내부 상태."""
        self._moved = False
        self._still_since: Optional[float] = None
        self._remaining = 0
        self._next = 0.0

    def update(self, t: float, v: float, w: float) -> bool:
        """속도 샘플(odometry/filtered)마다 호출. 지금 요청해야 하면 True."""
        if abs(v) >= self.v_thresh or abs(w) >= self.w_thresh:
            self._moved = True
            self._still_since = None
            self._remaining = 0
            return False
        if self._still_since is None:
            self._still_since = t
        if self._moved and t - self._still_since >= self.settle_time:
            self._moved = False
            self._remaining = self.count
            self._next = t
        if self._remaining > 0 and t >= self._next:
            self._remaining -= 1
            self._next = t + self.period
            return True
        return False


@dataclass
class KidnapDetector:
    """감지·복구 상태기. 시각은 초 단위 단조 증가(ROS 시각)."""

    params: KidnapParams = field(default_factory=KidnapParams)
    odom_buffer_sec: float = 5.0

    def __post_init__(self) -> None:
        """내부 상태 초기화."""
        self.state = State.TRACKING
        self.lost = False
        self.events: List[Event] = []
        self._odom: List[StampedPose] = []
        self._last_amcl: Optional[StampedPose] = None
        self._amcl_cov: Tuple[float, float] = (0.0, 0.0)
        self._amcl_pose: Optional[Pose] = None
        self._anchor: Optional[Tuple[Pose, Pose]] = None   # (AMCL 자세, 그 시각 odom 자세)
        self._low_match = 0
        self._last_match: Optional[float] = None
        self._last_margin: Optional[float] = None    # ρ(현재) − max ρ(별칭)
        self.alias_rejections = 0                     # 여백 부족으로 수렴을 보류한 판정 수
        self.external_trust_uses = 0                  # 외부 자세를 믿고 여백 요구를 건너뛴 판정 수
        self._external_trust_until = -math.inf        # 바깥이 자세를 잡아 준 뒤 여백 유예 끝 시각
        self.backend_ready = True                     # 측위 백엔드(AMCL)가 active 인가
        self._backend_down_since: Optional[float] = None
        self.backend_down_time = 0.0                  # 백엔드가 내려가 있던 누적 시간 [s]
        self._gate_logged_at = -math.inf              # 여백 게이트 판정을 마지막으로 적은 시각
        self._gate_logged_kind: Optional[bool] = None  # 그때 적은 것이 우회였나(True)/보류였나(False)
        self.spin_ready = True                        # 회전 수단(spin 액션 또는 cmd_vel)이 있는가
        self._spin_down_since: Optional[float] = None
        self.spin_unavailable_time = 0.0              # 회전 수단이 없던 누적 시간 [s]
        self._amcl_alarm = False      # 지속 경보: 공분산
        self._amcl_reason = ''
        self._match_alarm = False     # 지속 경보: 스캔-맵 불일치
        self._match_reason = ''
        self._reg_hist: List[bool] = []   # 스캔 정합기 수락/거부 최근 이력
        self._reg_alarm = False       # 지속 경보: 정합기 거부율
        self._reg_reason = ''
        self._alarm_reason = ''
        self._alarm_active = False
        self._suspect_since = 0.0
        self._lost_since = 0.0
        self._converged_count = 0
        self._spins_left = 0
        self._reinits_done = 0
        self._spin_active = False
        self._cooldown_until = -math.inf
        self._marker_mismatch_since: Optional[float] = None
        self.detect_time: Optional[float] = None       # 마지막 LOST 선언 시각
        self.recovery_times: List[float] = []          # LOST → TRACKING 소요 [s]

    # ------------------------------------------------------------------ 입력
    def on_odom(self, t: float, pose: Pose) -> None:
        """odometry/filtered (odom 프레임, 연속) 자세."""
        self._odom.append(StampedPose(t, pose))
        while self._odom and self._odom[0].t < t - self.odom_buffer_sec:
            self._odom.pop(0)

    def on_amcl(self, t: float, pose: Pose, cov_xy_trace: float,
                var_yaw: float) -> List[Action]:
        """amcl_pose 갱신. 공분산·점프 경보를 평가한다."""
        self._amcl_cov = (cov_xy_trace, var_yaw)
        self._amcl_pose = pose
        odom_at = interpolate_pose(self._odom, t)
        self._anchor = None if odom_at is None else (pose, odom_at)
        reasons = []
        if cov_xy_trace > self.params.cov_thresh:
            reasons.append(f'cov_xy {cov_xy_trace:.3f} > {self.params.cov_thresh:.3f} m^2')
        if var_yaw > self.params.yaw_cov_thresh:
            reasons.append(f'var_yaw {var_yaw:.3f} > {self.params.yaw_cov_thresh:.3f} rad^2')
        self._amcl_alarm = bool(reasons)
        self._amcl_reason = '; '.join(reasons)
        # 점프는 순간 사건: SUSPECT 로 들어가는 계기만 되고, LOST 확정은 지속 경보(공분산/불일치)가 한다
        jump = ''
        prev = self._last_amcl
        self._last_amcl = StampedPose(t, pose)
        if prev is not None and self._odom:
            o_prev = interpolate_pose(self._odom, prev.t)
            o_now = interpolate_pose(self._odom, t)
            dp, dth = jump_discrepancy(prev.pose, pose, o_prev, o_now)
            if dp > self.params.jump_thresh or dth > self.params.jump_yaw_thresh:
                jump = f'jump {dp:.2f} m / {math.degrees(dth):.0f} deg vs odom'
        return self._evaluate(t, jump)

    def odom_pose_at(self, t: float) -> Optional[Pose]:
        """시각 t 의 odom 자세 (버퍼 보간, 없으면 None)."""
        return interpolate_pose(self._odom, t)

    def map_pose_at(self, t: float) -> Optional[Pose]:
        """
        시각 t 의 map 자세 = 마지막 AMCL 자세 ∘ (그 이후 odom 상대 이동).

        AMCL 은 움직일 때만 갱신하므로 정지 중 납치되면 이 예측이 옛 자세에 머물고, 스캔과 어긋난다.
        """
        if self._anchor is None:
            return None
        odom_now = interpolate_pose(self._odom, t)
        if odom_now is None:
            return None
        return compose(self._anchor[0], relative_pose(self._anchor[1], odom_now))

    def on_external_pose_reset(self, t: float) -> None:
        """
        바깥에서 자세를 재설정했다 (initialpose). 그 수렴 구간은 납치가 아니다.

        운영자나 상위 시스템이 자세를 잡아 주면 추정이 새 자리로 옮겨 가는 동안 마커 보정과
        크게 어긋난다. 이를 납치로 읽으면 잡아 준 자세를 도로 덮어쓴다 — 통합 10 실측:
        시행마다 순간 이동 + initialpose 뒤 수렴 구간에 LOST 가 선언돼 하네스의 "안정되었나"
        확인이 실패했다. 우리가 스스로 시드를 준 뒤와 같은 유예를 준다.
        """
        self._cooldown_until = max(self._cooldown_until, t + self.params.cooldown)
        self._marker_mismatch_since = None
        # 스캔에서 유래한 이력은 **이전 자세**에 대한 것이다 — 자세가 바뀌었으니 비운다.
        # 이것이 없으면 순간 이동 직후 등록 창(reg_window 20 스캔 @ 10 Hz = 2 s)에 이동 전 스캔이
        # 남아 거부가 쌓이고, 쿨다운(3 s)이 끝나기도 전에 "scan registration rejected 17/20" 으로
        # LOST 가 선언된다 (logs/R12b 실측: amr_05 이 배치 2.48 s 만에, 전역 탐색이 참 자세를
        # inlier 0.914 로 1 순위에 올린 상태에서 lost 가 됐다). 그리고 _declare_lost 가 아래
        # 여백 유예까지 닫아 버려, 그 뒤 자가 복구가 별칭 자리에서 영원히 실패한다.
        self._reset_reg()
        self._low_match = 0
        self._match_alarm = False
        self._match_reason = ''
        self._alarm_active = self._amcl_alarm
        # 진행 중인 의심도 **이전 자세**에 대한 것이다 — 거둬들인다. 쿨다운만으로는 못 막는다:
        # tick 의 SUSPECT → LOST 승격은 쿨다운을 보지 않으므로, 순간 이동과 시드 사이에 이미
        # SUSPECT 로 들어갔으면 시드가 와도 suspect_time 뒤 LOST 가 선언된다 (logs/R12b 의 2.48 s).
        if self.state == State.SUSPECT:
            self._set_state(t, State.TRACKING,
                            'external pose reset: suspicion was about the old pose')
        # 별칭 여백 유예를 연다. 쿨다운만으로는 **재선언**만 막을 뿐, 이미 lost 인 상태에서
        # 수렴을 가로막는 여백 게이트는 그대로라 주기 별칭 자리에서는 영원히 복구하지 못한다.
        self._external_trust_until = t + self.params.external_trust_window
        # **항상** 적는다. 예전에는 위 SUSPECT 분기에서만 기록이 남아, 창이 열렸는지를
        # 로그로 확인할 수 없었다 (R12a·R12b 진단이 여기서 막혔다).
        self.events.append(Event(
            t, self.state.value,
            f'external pose reset: alias margin trusted for '
            f'{self.params.external_trust_window:.0f} s'))

    def set_backend_ready(self, t: float, ready: bool) -> List[Action]:
        """
        측위 백엔드(AMCL)가 active 인지 알린다. inactive 동안 복구는 **쉰다**.

        왜: lifecycle_manager 가 change_state 응답을 유실하면 amcl 이 inactive 에 머문다
        (amr_bringup/lifecycle_watchdog 머리말). 그 동안 이 상태기는 복구를 정상적으로 돌리는데,
        심는 initialpose 는 amcl 이 "not yet in the active state" 로 **전부 버린다** — 즉 성공할 수
        없는 시도로 reinit_retry 를 소진하고 FAILED 로 떨어진다 (logs/X12: amr_02·03·05 가 셋 다
        global re-init #4 까지 태우고 recovery exhausted, 그 뒤 작업 배정에서 영구 제외됐다).

        백엔드가 돌아오면 (a) 쉰 시간을 복구 시한에서 빼고, (b) 시도 횟수를 되돌리고,
        (c) **별칭 여백 유예를 연다** — 백엔드가 active 로 올라올 때 적용하는 자세는 설정
        (initial_pose.*) 이나 보관된 initialpose 에서 온 것이라 우리 스캔 정합 바깥의 증거다
        (watchdog 이 올린 amcl 이 "Setting pose: 20.000 -16.000 1.571" 로 정답을 넣은 것이 실측이다).
        그것이 없으면 점대칭 창고의 대기 구역처럼 여백이 임계 미만인 자리에서는 AMCL 이 맞게
        있는데도 게이트가 증명하지 못해 lost 가 영원히 풀리지 않는다.
        """
        if ready == self.backend_ready:
            return []
        self.backend_ready = ready
        if not ready:
            self._backend_down_since = t
            self.events.append(Event(t, self.state.value, 'localization backend inactive'))
            return []
        if self._backend_down_since is not None:
            self.backend_down_time += t - self._backend_down_since
            self._backend_down_since = None
        self.events.append(Event(t, self.state.value, 'localization backend active'))
        self._external_trust_until = t + self.params.external_trust_window
        if self.state in (State.RECOVERING, State.FAILED):
            # 백엔드가 죽어 있는 동안 태운 시도는 성공할 수 없었다 — 되돌리고 시한도 다시 센다
            self._reinits_done = 0
            self._lost_since = t
            if self.state == State.FAILED:
                self._set_state(t, State.RECOVERING, 'backend active: retry recovery')
            # **반드시 복구를 다시 시작한다.** 백엔드가 죽은 동안 on_spin_done 이 아무 동작도
            # 내지 않았으므로 이 시점에 돌고 있는 회전이 없다 — 여기서 깨우지 않으면 상태기가
            # 영원히 멈춰 선다 (logs/Y12 실측: amr_04 가 재시도 1 회에서 멈춘 채 조용했다).
            return self._reinitialize(t, 'localization backend came back')
        return []

    def set_spin_ready(self, t: float, ready: bool) -> List[Action]:
        """
        회전 수단(Nav2 spin 액션 또는 fallback cmd_vel)이 있는지 알린다. 없는 동안 복구는 **쉰다**.

        왜: 수단이 없을 때 kidnap_monitor_node 는 "회전 없이 대기" 라고 적고도 곧바로
        on_spin_done(False) 를 불렀다. 그래서 회전·재관측이 한 번도 일어나지 않은 채 재시도
        예산(spin + re-init)이 **밀리초 단위로** 소진되고 FAILED 로 떨어졌다
        (logs/S12a: amr_04 가 4.7 ms 에 re-init #1~#4 를 태우고 recovery exhausted,
        51.6 s 뒤 behavior_server 가 활성이 되어서야 backend 폴링이 되살렸다).

        성질은 set_backend_ready 와 같다 — **재시도가 성공할 전제가 없을 때 유한한 예산을
        깎아서는 안 된다**. 다만 여기서는 FAILED 를 되돌리지 않는다: 이 가드가 있으면 이 경로로
        FAILED 에 도달하지 않고, "회전이 생겼다" 를 일반적인 복구 방아쇠로 만들면 다른 원인으로
        실패한 로봇까지 되살려 진짜 고장을 가린다.
        """
        if ready == self.spin_ready:
            return []
        if not ready:
            self.spin_ready = False
            self._spin_down_since = t
            # 돌고 있는 회전이 없다고 기록해 둔다 — 없는 목표에 CANCEL_SPIN 을 내지 않도록
            self._spin_active = False
            self.events.append(Event(t, self.state.value, 'rotation mechanism unavailable'))
            return []
        self.note_spin_available(t)
        if self.state == State.RECOVERING and self.backend_ready:
            # 예산을 쓰지 않고 멈춰 있던 회전을 **다시 시작한다** (여기서 깨우지 않으면
            # on_spin_done 이 올 일이 없어 상태기가 멈춘다 — 77cbcf1 에서 낸 회귀와 같은 모양).
            return self._spin(t, 'rotation mechanism came back')
        return []

    def note_spin_available(self, t: float) -> None:
        """
        회전을 지금 시작하므로 가용 표시만 한다 — **동작은 내지 않는다**.

        start_spin 안에서 set_spin_ready 를 부르면 SPIN 동작이 또 나와 재귀한다. 지금 시작하는
        회전이 곧 그 동작이므로 깃발과 누적 시간만 맞춘다.
        """
        if self.spin_ready:
            return
        self.spin_ready = True
        if self._spin_down_since is not None:
            waited = t - self._spin_down_since
            self.spin_unavailable_time += waited
            # 복구 시한은 **복구가 가능했던 시간**만 세야 한다. 기다린 만큼 기준점을 밀지 않으면
            # 수단이 돌아온 그 tick 에 recovery_timeout 이 즉시 터져 가드를 넣은 뜻이 없어진다.
            # (set_backend_ready 는 _lost_since = t 로 통째로 되돌리는데, 그러면 오르내림이
            #  반복될 때 시한이 무한정 미뤄진다 — 여기서는 기다린 만큼만 민다)
            self._lost_since += waited
            self._spin_down_since = None
        self.events.append(Event(t, self.state.value, 'rotation mechanism available'))

    def on_marker_fix(self, t: float, pose: Pose) -> List[Action]:
        """
        지도에 등록된 마커로 역산한 로봇 자세 (외부 증거, amr_behavior docking_server_node).

        스캔-맵 정합은 같은 형태의 평행 통로를 구분하지 못한다 (통합 시나리오 11: 6 m 옆으로 옮겨도
        inlier·공분산·점프 어디에도 안 걸렸다). ArUco 마커는 지도에서 유일하므로 그 애매함을 끊는다.
        현재 추정과 marker_fix_min_error 이상 어긋나면 LOST 를 선언하고, 역산 자세를 그대로 시드로 준다
        (전역 가설 탐색보다 훨씬 빠르고, 별칭 가설로 다시 수렴할 위험도 없다).
        """
        # 이미 복구 중이면 씨앗은 이미 줬다 — 매 관측마다 다시 심으면 AMCL 이 수렴할 틈이 없다
        # (통합 10 실측: 한 실행에서 LOST·씨앗이 77 회 반복돼 도킹이 3/10 실패했다).
        if self._amcl_pose is None or t < self._cooldown_until or self.lost:
            return []
        error = math.hypot(pose[0] - self._amcl_pose[0], pose[1] - self._amcl_pose[1])
        if error < self.params.marker_fix_min_error:
            self._marker_mismatch_since = None
            return []
        if self._marker_mismatch_since is None:
            self._marker_mismatch_since = t
            return []
        if t - self._marker_mismatch_since < self.params.marker_fix_persist:
            return []
        # 스캔 정합기가 "지금 자리가 맞다" 고 하고 있으면 마커 쪽이 틀린 것이다. ArUco 오검출은
        # 실재한다 — 통합 12 실측(reg1): 정지한 amr_05 가 36 m 밖의 마커 17 을 2.15 m 앞으로
        # 읽어 30.32 m 어긋난 자세를 냈고, 같은 시각 정합기는 86 ok / 1 rejected (inlier 0.93)
        # 로 멀쩡했다. 그 LOST 가 풀리지 않아 548 s 동안 작업을 205 회 거절했다 (s12 는 1873 회).
        # 진짜 납치는 반대다 — 통합 11 reg1 에서 정합기가 0 ok / 300 rejected 를 600 s 연속 냈다.
        # 그래서 "정합기가 동의하지 않을 때만" 마커 보정을 LOST 근거로 쓴다. 표본이 모자라면
        # (기동 직후, 지도 없음, lost 중 정합기 정지) 예전처럼 마커를 믿는다.
        if (len(self._reg_hist) >= self.params.reg_min_samples
                and 1.0 - sum(self._reg_hist) / len(self._reg_hist)
                < self.params.reg_marker_veto_ratio):
            self.events.append(Event(
                t, self.state.value,
                f'marker fix {error:.2f} m 무시: 스캔 정합 수락률 '
                f'{sum(self._reg_hist)}/{len(self._reg_hist)}'))
            self._marker_mismatch_since = None
            return []
        self._marker_mismatch_since = None
        reason = (f'marker fix {error:.2f} m from estimate '
                  f'({self.params.marker_fix_persist:.1f} s 이상 지속)')
        self.lost = True
        self.detect_time = t
        self._lost_since = t
        self._reinits_done = 0
        self._converged_count = 0
        self._low_match = 0
        self._reset_reg()
        self._last_match = None
        self._last_margin = None
        self._set_state(t, State.RECOVERING, f'LOST: {reason}')
        # 시드가 있으므로 전역 재초기화(REINITIALIZE)는 하지 않는다. 회전은 그대로 — AMCL 이 갱신을
        # 하려면 움직여야 하고, 회전은 제자리에서 안전하다.
        self._cooldown_until = t + self.params.cooldown   # 씨앗이 EKF·AMCL 로 퍼질 시간을 준다
        # 회전은 남긴다: AMCL 은 움직여야 갱신하므로, 정지 상태로 씨앗만 주면 수렴 판정이 오지
        # 않아 localization/lost 가 풀리지 않는다 (실측으로 확인 — 회전을 뺐더니 순간 이동 뒤
        # 안정화가 아예 끝나지 않았다). 제자리 회전이라 위치는 그대로다.
        return ([Action(ActionType.PUBLISH_LOST, True), Action(ActionType.SEED_POSE, pose)]
                + self._spin(t, reason))

    def on_match(self, t: float, ratio: float, valid_beams: int,
                 alias_margin: Optional[float] = None) -> List[Action]:
        """
        스캔-맵 인라이어 비율 (kidnap_monitor_node 가 map 자세로 계산).

        alias_margin: 복구 중 ρ(현재 추정) − max ρ(다른 가설, 별칭) (global_seed.alias_margin; 없으면 None).
        """
        if valid_beams < self.params.min_match_beams:
            return self._evaluate(t, '')
        self._last_match = ratio
        self._last_margin = alias_margin
        if ratio < self.params.match_thresh:
            self._low_match += 1
        else:
            self._low_match = 0
        self._match_alarm = self._low_match >= self.params.match_window
        self._match_reason = (
            f'scan-map inlier {ratio:.2f} < {self.params.match_thresh:.2f} '
            f'for {self._low_match} scans' if self._match_alarm else '')
        return self._evaluate(t, '')

    def on_scan_match(self, t: float, accepted: bool) -> List[Action]:
        """
        스캔 정합기 한 스캔의 수락/거부. 창 안 거부 비율이 reg_reject_ratio 를 넘으면 경보.

        정합기는 lost 중에는 측정을 멈추므로(scan_matcher.yaml lost_topic) "거부"와 "미발행"은
        다르다 — 미발행은 여기 들어오지 않아 이력이 그대로 멈춘다.
        """
        self._reg_hist.append(bool(accepted))
        if len(self._reg_hist) > self.params.reg_window:
            del self._reg_hist[:-self.params.reg_window]
        n = len(self._reg_hist)
        if n < self.params.reg_min_samples:
            self._reg_alarm, self._reg_reason = False, ''
            return self._evaluate(t, '')
        rejected = n - sum(self._reg_hist)
        ratio = rejected / n
        self._reg_alarm = ratio > self.params.reg_reject_ratio
        self._reg_reason = (
            f'scan registration rejected {rejected}/{n} '
            f'({ratio:.2f} > {self.params.reg_reject_ratio:.2f})' if self._reg_alarm else '')
        return self._evaluate(t, '')

    def _reset_reg(self) -> None:
        """정합기 경보 이력 초기화 (상태 전이 때 이전 구간 이력을 끌고 가지 않는다)."""
        self._reg_hist.clear()
        self._reg_alarm = False
        self._reg_reason = ''

    def on_spin_done(self, t: float, succeeded: bool) -> List[Action]:
        """회전 동작 종료 (성공/실패/취소)."""
        self._spin_active = False
        if self.state != State.RECOVERING:
            return []  # FAILED 이후에는 더 시도하지 않는다 (수렴하면 on_match/on_amcl 로 복귀)
        actions: List[Action] = []
        if not self.backend_ready:
            # 백엔드가 inactive 면 어떤 시드도 적용되지 않는다 — 시도를 쓰지 않고 기다린다
            # (set_backend_ready 가 돌아올 때 다시 시작한다)
            return []
        if not self.spin_ready:
            # 회전 수단이 없으면 재관측이 없다 — 같은 이유로 시도를 쓰지 않는다.
            # (monitor 는 이 경우 on_spin_done 을 부르지 않지만, 두 겹으로 막는다)
            return []
        if self._spins_left > 0:
            self._spins_left -= 1
            actions += self._spin(t, 'spin finished without convergence' if succeeded
                                  else 'spin aborted; retrying')
        elif self._reinits_done < self.params.reinit_retry:
            actions += self._reinitialize(t, 'not converged after spins; global re-init')
        else:
            actions += self._fail(t, 'recovery exhausted (spins and re-inits)')
        return actions

    def tick(self, t: float) -> List[Action]:
        """주기 호출: 시간 기반 전이 (SUSPECT 만료, 복구 시간 초과)."""
        actions: List[Action] = []
        if self.state == State.SUSPECT and t - self._suspect_since >= self.params.suspect_time:
            if self._alarm_active:
                actions += self._declare_lost(t, self._alarm_reason)
            else:
                self._set_state(t, State.TRACKING, 'alarm cleared')
        elif (self.state == State.RECOVERING and self.backend_ready and self.spin_ready
              and t - self._lost_since > self.params.recovery_timeout):
            # spin_ready 도 본다: 회전 수단이 없는 동안 흐른 시간으로 실패를 선언하면 예산 가드를
            # 넣은 뜻이 없어진다. 대가는 수단이 영구히 없으면 RECOVERING 에 머문다는 것인데,
            # 그래도 lost 가 서 있으니 fleet 는 그 로봇을 배정에서 뺀다(robot_status.py 의 lost→ERROR)
            # 그리고 AMCL 단독 수렴 경로(_evaluate 의 _converged)는 회전 없이도 계속 열려 있다.
            actions += self._fail(t, f'recovery timeout {self.params.recovery_timeout:.0f} s')
        return actions

    # ------------------------------------------------------------------ 내부
    def _evaluate(self, t: float, transient: str) -> List[Action]:
        """경보 갱신 + 상태 전이. transient = 순간 사건(점프) 설명, 없으면 ''."""
        self._alarm_active = self._amcl_alarm or self._match_alarm or self._reg_alarm
        self._alarm_reason = '; '.join(
            r for r in (self._amcl_reason, self._match_reason, self._reg_reason) if r)

        if self.state == State.TRACKING:
            if (self._alarm_active or transient) and t >= self._cooldown_until:
                self._suspect_since = t
                self._set_state(t, State.SUSPECT, '; '.join(
                    r for r in (self._alarm_reason, transient) if r))
            return []
        if self.state == State.SUSPECT:
            return self.tick(t)
        # RECOVERING / FAILED: 수렴 확인
        if self._converged(t):
            self._converged_count += 1
        else:
            self._converged_count = 0
        if self._converged_count >= self.params.converge_count:
            return self._recover(t)
        return []

    def _converged(self, t: float) -> bool:
        cov_ok = (self._last_amcl is not None and self._last_amcl.t >= self._lost_since
                  and self._amcl_cov[0] < self.params.converge_cov
                  and self._amcl_cov[1] < self.params.yaw_cov_thresh)
        match_ok = self._last_match is not None and self._last_match >= self.params.converge_match
        if cov_ok and match_ok and self._last_margin is not None and (
                self._last_margin < self.params.converge_margin):
            if t < self._external_trust_until:
                # 바깥이 자세를 잡아 줬다 — 여백은 "내가 고른 가설이 맞는가" 를 묻는 것이라
                # 이 경우에는 묻지 않는다. ρ·공분산 조건은 그대로 요구한다.
                self.external_trust_uses += 1
                self._note_gate(t, 'alias margin bypassed by external trust', True)
                return True
            self.alias_rejections += 1        # 별칭과 가를 수 없다 → 수렴 보류
            self._note_gate(t, 'alias margin blocks convergence', False)
            return False
        return cov_ok and match_ok

    def _note_gate(self, t: float, what: str, bypassed: bool) -> None:
        """
        여백 게이트의 판정을 이벤트로 남긴다 (간격 제한).

        왜 필요한가: 이 판정은 match 주기(10 Hz)로 일어나고 결과가 `alias_rejections` ·
        `external_trust_uses` **카운터**로만 쌓인다. 그 카운터는 상태 토픽의 JSON 으로만 나가고
        `launch.log` 에도 CSV 에도 남지 않아서, 실측 실패(`R12a`·`R12b`·`E12` 의 "배치 뒤 위치
        추정이 안정되지 않음")를 로그만으로는 **진단할 수 없었다**. 신뢰 창이 열렸는지조차
        알 수 없었다 — 그 기록은 SUSPECT 였을 때만 남았기 때문이다.

        매 판정을 적으면 10 Hz 로 로그가 넘치므로, 구간의 **첫 판정**과 그 뒤
        `gate_log_period` 간격만 적고 여백 값을 함께 남긴다.
        """
        stale = t - self._gate_logged_at >= self.params.gate_log_period
        if self._gate_logged_kind != bypassed or stale:
            self._gate_logged_kind = bypassed
            self._gate_logged_at = t
            margin = '—' if self._last_margin is None else f'{self._last_margin:.3f}'
            self.events.append(Event(
                t, self.state.value,
                f'{what}: margin {margin} < {self.params.converge_margin:.3f} '
                f'(rejections {self.alias_rejections}, trust uses {self.external_trust_uses})'))

    def _set_state(self, t: float, state: State, reason: str) -> None:
        self.state = state
        self.events.append(Event(t, state.value, reason))

    def _declare_lost(self, t: float, reason: str) -> List[Action]:
        # 새로 LOST 를 선언했다는 것은 바깥이 잡아 준 자세가 더는 유효하지 않다는 뜻이다 — 유예를 닫는다.
        self._external_trust_until = -math.inf
        self.lost = True
        self.detect_time = t
        self._lost_since = t
        self._reinits_done = 0
        self._converged_count = 0
        self._low_match = 0
        self._reset_reg()
        self._last_match = None
        self._last_margin = None
        self._set_state(t, State.RECOVERING, f'LOST: {reason}')
        return [Action(ActionType.PUBLISH_LOST, True)] + self._reinitialize(t, reason)

    def _reinitialize(self, t: float, reason: str) -> List[Action]:
        self._reinits_done += 1
        self._spins_left = self.params.spin_retry - 1
        self.events.append(Event(t, self.state.value, f'global re-init #{self._reinits_done}'))
        actions = []
        if self._spin_active:
            actions.append(Action(ActionType.CANCEL_SPIN))
            self._spin_active = False
        actions.append(Action(ActionType.REINITIALIZE, self._reinits_done))
        return actions + self._spin(t, reason)

    def _spin(self, t: float, reason: str) -> List[Action]:
        self._spin_active = True
        self.events.append(Event(t, self.state.value, f'spin: {reason}'))
        return [Action(ActionType.SPIN, self.params.spin_angle)]

    def _fail(self, t: float, reason: str) -> List[Action]:
        self._set_state(t, State.FAILED, reason)
        actions = []
        if self._spin_active:
            actions.append(Action(ActionType.CANCEL_SPIN))
            self._spin_active = False
        return actions

    def _recover(self, t: float) -> List[Action]:
        actions: List[Action] = []
        if self._spin_active:
            actions.append(Action(ActionType.CANCEL_SPIN))
            self._spin_active = False
        self.recovery_times.append(t - self._lost_since)
        self.lost = False
        self._external_trust_until = -math.inf      # 수렴했으니 유예는 끝난다
        self._cooldown_until = t + self.params.cooldown
        self._low_match = 0
        self._reset_reg()
        self._amcl_alarm = False
        self._match_alarm = False
        self._alarm_active = False
        self._set_state(t, State.TRACKING,
                        f'recovered in {t - self._lost_since:.1f} s')
        if self._amcl_pose is not None:
            actions.append(Action(ActionType.SET_EKF_POSE,
                                  (self._amcl_pose, self._amcl_cov[0], self._amcl_cov[1])))
        actions.append(Action(ActionType.PUBLISH_LOST, False))
        return actions

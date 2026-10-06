"""
정본 safety φ — `amr_perception::SafetyGate` 의 배치 torch 이식 (계약 §7.4 의 정본).

원본: src/amr_perception/src/safety_gate.cpp (이하 :행), include/amr_perception/safety_gate.hpp,
src/amr_perception/src/geometry2d.cpp. safety_node.cpp 가 50 Hz 로 부르는 SafetyGate 의 상태기계를
env 배치 축 N 으로 그대로 옮겼다 — Isaac Lab 에 의존하지 않는다 (torch 만, CPU·CUDA, float64).

옮긴 규칙 (모두 원본과 같은 연산 순서):
  - 스윕 풋프린트 접근 거리 (직진·원호·끝 자세 접선 직선, 옆 여유 규칙, 전체 길이 상한)   SweptFootprint :67-195
  - 접근 영역 (정지 포락선 = 정지 거리 + 제동 거리, 전체 = max(경고+히스테리시스, 포락선)+여유) sweptRegion :334-347
  - 운동 가설 (명령 · 신선한 측정 · 정지 중 마지막 명령 의도)                         motionHypotheses :507-530
  - 측정 속도 1차 저역 통과, 지연 보정 (촬영 뒤 측정 운동의 역변환)                   :381-422
  - 자기 차체 필터, 빔 창 순위 통계 (물리 폭 창, 원형/비원형), 점군 순위 값           :494-499, :241-277, :199-207
  - 채널 확정 (2 프레임 또는 즉시 여유), STOP 래치와 원인별 해제, 존 히스테리시스      :537-552, :737-822
  - 상한 (존 0.5/0.2, 여유 거리, 도킹 예외, 낡은 필수 점군 전진, TTC), 곡률 보존 스케일 :824-873
  - 근접 정지 중 탈출 (접촉 가드 창 중앙값이 줄지 않는 명령)                          escapeAllowed :554-580
  - TTC 증명된 후퇴, 몸 원통 안 양보 탈출 (배포 설정은 꺼짐)                          :582-642, :921-976
  - 도킹 예외 다각형 (ray casting), 명령 낡음                                         :501-505, :669-675
옮기지 않은 것: E-stop 래치·reset, 센서 생존 감시(지연·고장) — 시뮬레이션에서 생기지 않는 입력이다
(env 는 E-stop 이 없고 센서가 매 주기 온다). estop_active 는 늘 False.

빔 창 순위 통계는 원본의 순차 가지치기(:255, values[i] >= best 면 그 창을 건너뜀 — 결과가 빔 순서에 따라
"창 통계의 최소" 보다 클 수 있다)까지 그대로 따른다. 창마다 k 번째 값은 웨이블릿 행렬 분위 질의로, 순차 갱신
사슬은 희소 표 + 포인터 도약으로 병렬로 정확히 푼다 (window_statistic). 비교·색인만 써서 반올림 차이가 없다.

교차 시험: rl/isaac/tools/phi_crosscheck/ (C++ 원본을 직접 빌드해 같은 입력 열을 대조).
"""

from dataclasses import dataclass
import math

import torch

F64 = torch.float64
INF = math.inf
TWO_PI = 2.0 * math.pi                 # kTwoPi :20
STRAIGHT_RADIUS = 1.0e5                # kStraightRadius :22
ESCAPE_TOLERANCE = 0.01                # kEscapeTolerance :24
ESCAPE_STEPS = 5                       # kEscapeSteps :25
YIELD_CLEARANCE_TOLERANCE = 0.02       # kYieldClearanceTolerance :28

ZONE_CLEAR, ZONE_WARNING, ZONE_CRITICAL, ZONE_STOP = 0, 1, 2, 3
CAUSE_LIDAR, CAUSE_DEPTH, CAUSE_CONTACT, CAUSE_DOCK = 1, 2, 4, 8
# 채널 순서 (lidar_main_, lidar_excl_, guard_, cloud_main_, cloud_excl_ — safety_gate.hpp:381-385)
CH_LIDAR, CH_LIDAR_EXCL, CH_GUARD, CH_CLOUD, CH_CLOUD_EXCL = range(5)


@dataclass(frozen=True)
class GateParams:
    """
    SafetyParams (safety_gate.hpp:91-168) — 기본값 = safety_node 배포 설정.

    safety_node.cpp:99-171 이 config/robot_params.yaml (robot·limits·safety) 와
    src/amr_perception/config/perception.yaml (safety_node 블록) 에서 읽는 값이다. hpp 기본과 다른 것은 둘:
    yield_escape_enabled (hpp true → 설정 false),
    cloud_required (hpp false → depth_cloud.enabled true).
    tests/test_safety_gate_torch.py 가 두 설정 파일·hpp·노드 원문과 대조한다.
    """

    footprint_length: float = 0.60
    footprint_width: float = 0.40
    emergency_stop_distance: float = 0.30
    critical_zone_distance: float = 0.50
    warning_zone_distance: float = 1.00
    warning_zone_max_speed: float = 0.5
    critical_zone_max_speed: float = 0.2
    stop_release_distance: float = 0.50
    zone_hysteresis: float = 0.05
    reaction_latency: float = 0.15
    max_deceleration: float = 1.0
    clearance_speed_limit_enabled: bool = True
    max_linear_velocity: float = 2.0
    min_linear_velocity: float = -0.5
    max_angular_velocity: float = 1.5
    ttc_limit_enabled: bool = True
    ttc_critical: float = 2.15
    ttc_max_age: float = 0.5
    degraded_mode_max_speed: float = 0.2
    command_timeout: float = 0.5
    swept_margin: float = 0.0
    region_margin: float = 0.10
    motion_epsilon_v: float = 0.01
    motion_epsilon_w: float = 0.05
    hold_motion_intent: bool = True
    measured_velocity_timeout: float = 0.2
    measured_velocity_time_constant: float = 0.1
    self_filter_margin: float = 0.02
    beam_window_width: float = 0.04
    beam_window: int = 5
    beam_support: float = 0.6
    guard_window_width: float = 0.06
    guard_window: int = 9
    contact_guard_distance: float = 0.02
    stop_confirm_frames: int = 2
    immediate_stop_margin: float = 0.05
    cloud_min_points: int = 3
    cloud_timeout: float = 0.3
    cloud_required: bool = True
    cloud_max_age: float = 0.4
    latency_compensation: bool = True
    max_compensation_age: float = 0.5
    allow_escape: bool = True
    escape_horizon: float = 0.3
    escape_max_speed: float = 0.2
    retreat_max_speed: float = 0.5
    ttc_only_dynamic: bool = True
    yield_escape_enabled: bool = False
    yield_escape_speed: float = 0.5
    dynamic_obstacle_radius: float = 0.30
    yield_escape_overlap_margin: float = 0.10
    yield_escape_min_gain: float = 0.02
    yield_escape_min_obstacle_speed: float = 0.15
    exclusion_stop_distance: float = 0.10
    exclusion_timeout: float = 0.3


DEPLOYED = GateParams()


# ======================================================================== 기하 (geometry2d.cpp)
def normalize_angle(a):
    """(-π, π] 로 정규화 — geometry2d.cpp:47-54 와 같은 식."""
    r = torch.fmod(a + math.pi, 2.0 * math.pi)
    r = torch.where(r <= 0.0, r + 2.0 * math.pi, r)
    return r - math.pi


def pose_apply(tx, ty, yaw, qx, qy):
    """Pose2D(tx, ty, yaw).apply(q) — R(yaw)·q + t (:29-32). 모든 인자는 브로드캐스트 가능한 텐서."""
    c = torch.cos(yaw)
    s = torch.sin(yaw)
    return c * qx - s * qy + tx, s * qx + c * qy + ty


def pose_inverse(tx, ty, yaw):
    """Pose2D::inverse (:40-45): R^T, t' = −(R^T t), yaw' = normalizeAngle(−yaw)."""
    c = torch.cos(yaw)
    s = torch.sin(yaw)
    ix = -(c * tx + s * ty)
    iy = -(-s * tx + c * ty)
    return ix, iy, normalize_angle(-yaw)


def unicycle_delta(v, w, t):
    """등속 원호 운동 뒤 상대 자세 (:31-38). |w| < 1e-9 이면 직선."""
    small = w.abs() < 1e-9
    w_safe = torch.where(small, torch.ones_like(w), w)
    th = w * t
    r = v / w_safe
    x = torch.where(small, v * t, r * torch.sin(th))
    y = torch.where(small, torch.zeros_like(v), r * (1.0 - torch.cos(th)))
    yaw = torch.where(small, torch.zeros_like(v), th)
    return x, y, yaw


def distance_to_rectangle(px, py, length, width):
    """원점 중심 사각형 변까지 거리, 안쪽 0 (:56-62)."""
    dx = torch.clamp(px.abs() - 0.5 * length, min=0.0)
    dy = torch.clamp(py.abs() - 0.5 * width, min=0.0)
    return torch.hypot(dx, dy)


def point_in_polygon(px, py, poly, n_poly):
    """
    다각형 안 판정 — ray casting (:75-94).

    px·py [N, K], poly [N, P, 2] (앞 n_poly 개가 꼭짓점), n_poly [N] → [N, K] bool.

    변 (i, j = i−1 순환) 마다 교차를 센다. n_poly < 3 이면 False.
    """
    P = poly.shape[1]
    out = torch.zeros(px.shape, dtype=torch.bool, device=px.device)
    if P == 0:
        return out
    ar = torch.arange(P, device=px.device).unsqueeze(0)
    npl = n_poly.clamp(min=1).unsqueeze(1)
    jdx = torch.remainder(ar - 1, npl)                                    # [N, P]
    ax, ay = poly[..., 0], poly[..., 1]
    bx = poly[..., 0].gather(1, jdx)
    by = poly[..., 1].gather(1, jdx)
    edge_ok = ar < n_poly.unsqueeze(1)
    for i in range(P):
        a_x, a_y = ax[:, i:i + 1], ay[:, i:i + 1]
        b_x, b_y = bx[:, i:i + 1], by[:, i:i + 1]
        crosses = (a_y > py) != (b_y > py)
        dy = torch.where(crosses, b_y - a_y, torch.ones_like(b_y - a_y))
        x_int = a_x + (py - a_y) * (b_x - a_x) / dy
        flip = crosses & (px < x_int) & edge_ok[:, i:i + 1]
        out = out ^ flip
    return out & (n_poly >= 3).unsqueeze(1)


# ======================================================================== 스윕 풋프린트
def max_point_speed(a, b, v, w):
    """풋프린트 꼭짓점 최고 속력 (:98-109) — 반길이 a, 반폭 b (float), v·w 텐서."""
    best = torch.zeros_like(v)
    for x in (-a, a):
        for y in (-b, b):
            best = torch.maximum(best, torch.hypot(v - w * y, w * x))
    return best


def braking_distance(speed, p):
    """d(u) = u·t + u²/(2a) (:302-306)."""
    u = speed.abs()
    return u * p.reaction_latency + u * u / (2.0 * p.max_deceleration)


def clearance_speed_limit(distance, stop_distance, p):
    """여유 거리 연속 상한 (:308-318) — 무한 거리 = +inf."""
    margin = torch.clamp(distance - stop_distance, min=0.0)
    margin = torch.where(torch.isfinite(margin), margin, torch.zeros_like(margin))
    at = p.max_deceleration * p.reaction_latency
    u = -at + torch.sqrt(at * at + 2.0 * p.max_deceleration * margin)
    return torch.where(torch.isfinite(distance), u, torch.full_like(u, INF))


def classify_zone(distance, p):
    """히스테리시스 없는 존 (:320-332). 무한 = CLEAR."""
    z = torch.full(distance.shape, ZONE_CLEAR, dtype=torch.long, device=distance.device)
    z = torch.where(distance <= p.warning_zone_distance, torch.full_like(z, ZONE_WARNING), z)
    z = torch.where(distance <= p.critical_zone_distance, torch.full_like(z, ZONE_CRITICAL), z)
    return torch.where(distance <= p.emergency_stop_distance, torch.full_like(z, ZONE_STOP), z)


class Region:
    """
    SweptFootprint 한 개의 배치판 — 운동 (v, w) [N] 의 접근 영역 (sweptRegion :334-347 + 생성자 :67-96).

    active [N] 이 False 인 env 는 영역이 없다 (approach = +inf).
    """

    def __init__(self, v, w, active, p):
        """sweptRegion(Motion{v, w}) 와 같은 매개변수."""
        a = 0.5 * p.footprint_length
        b_half = 0.5 * p.footprint_width
        u0 = max_point_speed(a, b_half + p.swept_margin, v, w)
        envelope = p.emergency_stop_distance + braking_distance(u0, p)
        total = torch.clamp(envelope, min=p.warning_zone_distance + p.zone_hysteresis) + \
            p.region_margin
        self.a = a
        self.b = b_half + max(p.swept_margin, 0.0)
        self.b_real = b_half
        linear = v.abs() > p.motion_epsilon_v
        rotating = w.abs() > p.motion_epsilon_w
        u = max_point_speed(self.a, self.b, v, w)
        self.empty = (~linear & ~rotating) | (u <= 0.0) | (total <= 0.0) | ~active
        w_safe = torch.where(w.abs() > 0.0, w, torch.ones_like(w))
        self.straight = (w.abs() < 1e-12) | ((v / w_safe).abs() > STRAIGHT_RADIUS)
        self.radius = torch.where(self.straight, torch.zeros_like(v), v / w_safe)
        u_safe = torch.where(u > 0.0, u, torch.ones_like(u))
        arc_time = torch.where(linear, torch.minimum(envelope, total), total) / u_safe
        self.arc_angle = torch.clamp(w.abs() * arc_time, max=TWO_PI)
        self.has_tail = linear & (envelope < total) & ~self.straight
        tx, ty, tyaw = unicycle_delta(v, w, arc_time)
        self.tail_inv = pose_inverse(tx, ty, tyaw)
        self.tail_offset = v.abs() * arc_time
        self.total = total
        self.v = v
        self.w = w

    def approach(self, px, py):
        """점 [N, K] 의 접근 거리 (approach :169-195) — 안 = 0, 영역 밖 = +inf."""
        def col(t):
            return t.unsqueeze(1)
        a, b, b_real = self.a, self.b, self.b_real
        inf = torch.full_like(px, INF)
        beside = px.abs() <= a
        inside = beside & (py.abs() <= b_real)
        bsel = torch.where(beside & (py.abs() <= b), torch.full_like(py, b_real),
                           torch.full_like(py, b))
        vpos = col(self.v > 0.0)

        def straight_approach(qx, qy):
            ahead = torch.where(vpos, qx, -qx) - a
            ok = (qy.abs() <= bsel) & (ahead >= 0.0)
            return torch.where(ok, ahead, inf)
        s_str = straight_approach(px, py)
        # 원호 (arcApproach :120-167)
        R = col(self.radius)
        dx = px
        dy = py - R
        rho = torch.hypot(dx, dy)
        th0 = torch.atan2(dy, dx)
        dirc = torch.where(col(self.w) > 0.0, -1.0, 1.0).to(px.dtype)
        rho2 = rho * rho
        best = inf

        def consider(best, x, y, ok):
            delta = torch.fmod(dirc * (torch.atan2(y - R, x) - th0), TWO_PI)
            delta = torch.where(delta < 0.0, delta + TWO_PI, delta)
            return torch.where(ok, torch.minimum(best, delta), best)
        for sx in (-a, a):
            h2 = rho2 - sx * sx
            h = torch.sqrt(torch.clamp(h2, min=0.0))
            for y in (R + h, R - h):
                best = consider(best, torch.full_like(px, sx), y, (h2 >= 0.0) & (y.abs() <= bsel))
        for sgn in (-1.0, 1.0):
            sy = sgn * bsel
            h2 = rho2 - (sy - R) * (sy - R)
            h = torch.sqrt(torch.clamp(h2, min=0.0))
            for x in (h, -h):
                best = consider(best, x, sy, (h2 >= 0.0) & (x.abs() <= a))
        s_arc = torch.where((best > col(self.arc_angle)) | (rho < 1e-9), inf, rho * best)
        # 원호 끝 자세의 접선 직선 (:185-192)
        ix, iy, iyaw = self.tail_inv
        qx, qy = pose_apply(col(ix), col(iy), col(iyaw), px, py)
        ahead = straight_approach(qx, qy)
        s_tail = torch.where(col(self.has_tail) & torch.isfinite(ahead),
                             col(self.tail_offset) + ahead, inf)
        s_arc = torch.where(torch.isfinite(s_arc), s_arc, s_tail)
        s = torch.where(col(self.straight), s_str, s_arc)
        s = torch.where(s <= col(self.total), s, inf)
        s = torch.where(inside, torch.zeros_like(s), s)
        return torch.where(col(self.empty), inf, s)


# ======================================================================== 강건 통계
def kth_smallest(values, support):
    """순위 값 — kthSmallest (:199-207). values [N, K] (+inf = 없음), 유한 값이 k 개 미만이면 +inf."""
    k = max(int(support), 1)
    if values.shape[1] < k:
        return torch.full(values.shape[:1], INF, dtype=values.dtype, device=values.device)
    return torch.sort(values, dim=1).values[:, k - 1]


class WindowSpec:
    """physicalWindowStatistic 의 창 반길이·순위 (빔 거리에서 정해지고 이분 탐색 반복 동안 고정)."""

    def __init__(self, ranges, angle_increment, width, min_window, support_ratio, circular,
                 half_cap=None):
        """창 매개변수 — 빔 거리 ranges [N, B] (무효 = NaN) → half·k [N, B], 패딩 색인."""
        n = ranges.shape[1]
        self.n = n
        self.circular = circular
        min_half = max(min_window, 1) // 2
        max_half = max(min_half, (n - 1) // 2)
        dphi = max(abs(angle_increment), 1e-6)
        r = torch.where(torch.isfinite(ranges) & (ranges > 0.0), ranges, torch.ones_like(ranges))
        half = torch.ceil(0.5 * width / (r * dphi))
        half = torch.clamp(half, min=float(min_half), max=float(max_half)).long()
        length = 2 * half + 1
        k = torch.ceil(support_ratio * length.to(ranges.dtype) - 1e-9).long().clamp(min=1)
        self.k = k
        if half_cap is None:
            H = int(half.max()) if half.numel() else min_half     # GPU 동기화 한 번
        else:
            # env 용: 동기화 없이 고정 패딩. half_cap 은 유한 값 중심(유효 빔, r ≥ range_min)의 반길이 상한이어야
            # 한다 — 그러면 아래 clamp 는 중심 빔에서 아무것도 바꾸지 않는다 (max_half_for_range 로 정한다).
            H = int(half_cap)
        self.half = half.clamp(max=H)
        self.H = H
        dev = ranges.device
        pos = torch.arange(-H, n + H, device=dev)
        if circular:
            self.pad_idx = torch.remainder(pos, n)
            self.pad_ok = None
        else:
            self.pad_idx = pos.clamp(0, n - 1)
            self.pad_ok = (pos >= 0) & (pos < n)
        i = torch.arange(n, device=dev).unsqueeze(0)
        self.hi_idx = i + H + self.half + 1          # 누적합 P[j] = Σ_{m<j} 지시
        self.lo_idx = i + H - self.half


def max_half_for_range(r_min, angle_increment, p=DEPLOYED):
    """반지름 r_min 이상인 빔 중심 창의 반길이 상한 (빔·가드 창 중 큰 쪽) — WindowSpec half_cap 용."""
    dphi = max(abs(angle_increment), 1e-6)
    width = max(p.beam_window_width, p.guard_window_width)
    return int(math.ceil(0.5 * width / (r_min * dphi))) + 1


def window_kth(values, spec):
    """
    창 i 마다 k_i 번째 작은 값 s_i [N, B] — 패딩 열 위의 웨이블릿 행렬 분위 질의 (정확, 비교만).

    값을 패딩한 열(길이 L = B + 2H)의 순위로 바꾸고(같은 값은 위치 순 — 순서 통계의 값은 같다), 순위 비트 평면마다
    안정 분할한 웨이블릿 행렬에서 구간 [i+H−h_i, i+H+h_i] 의 k_i 번째 순위를 찾는다. 창 밖(비원형 스캔 밖) = +inf.
    """
    N, n = values.shape
    dev = values.device
    vpad = values[:, spec.pad_idx]
    if spec.pad_ok is not None:
        vpad = torch.where(spec.pad_ok.unsqueeze(0), vpad, torch.full_like(vpad, INF))
    L = vpad.shape[1]
    order = torch.argsort(vpad, dim=1, stable=True)
    svp = vpad.gather(1, order)
    ar = torch.arange(L, device=dev).unsqueeze(0).expand(N, -1)
    cur = torch.empty_like(order).scatter_(1, order, ar)              # 순위 열
    nbits = max(1, int(math.ceil(math.log2(L))))
    zero = torch.zeros(N, 1, dtype=torch.long, device=dev)
    R0, Z = [], []
    for b in range(nbits - 1, -1, -1):
        zbit = ((cur >> b) & 1) == 0
        c0 = torch.cat([zero, torch.cumsum(zbit.long(), dim=1)], dim=1)   # rank0(x) = c0[:, x]
        nz = c0[:, -1:]
        R0.append(c0)
        Z.append(nz)
        newpos = torch.where(zbit, c0[:, :-1], nz + (ar - c0[:, :-1]))
        cur = torch.empty_like(cur).scatter_(1, newpos, cur)
    lo = spec.lo_idx.expand(N, -1).clone()
    hi = spec.hi_idx.expand(N, -1).clone()
    k0 = (spec.k - 1).expand(N, -1).clone()
    res = torch.zeros(N, n, dtype=torch.long, device=dev)
    for li, b in enumerate(range(nbits - 1, -1, -1)):
        c0 = R0[li]
        r0l = c0.gather(1, lo)
        r0h = c0.gather(1, hi)
        z = r0h - r0l
        go0 = k0 < z
        res = res | ((~go0).long() << b)
        k0 = torch.where(go0, k0, k0 - z)
        lo = torch.where(go0, r0l, Z[li] + (lo - r0l))
        hi = torch.where(go0, r0h, Z[li] + (hi - r0h))
    return svp.gather(1, res.clamp(max=L - 1))


def window_statistic(values, spec):
    """
    빔 창 순위 통계 — physicalWindowStatistic (:241-277) 를 원본의 가지치기까지 그대로, 반환 [N].

    원본은 빔 i = 0..n−1 을 차례로 보며 `values[i]` 가 유한하고 지금까지의 best 보다 작을 때만 창 i 의 k 번째 값으로
    best 를 줄인다 (:255 "이미 더 작은 결과가 있으면 볼 필요 없다"). 그런데 창의 k 번째 값은 중심 값보다 작을 수 있어서
    이 가지치기는 결과를 바꾼다 — 원본 값은 "창 통계의 최소" 이상이고 빔 순서에 따라 다르다 (교차 시험에서 확인,
    A3a 노트). 정본 φ 가 실제로 내는 값을 따르려고 같은 순차 규칙을 병렬로 정확히 푼다:
      m_i = max(v_i, s_i) (유한 중심만), best 는 m_i < best 인 첫 i 에서 s_i 로 바뀐다 (그때 s_i ≤ m_i < best).
      다음 갱신 위치 next(j) = j 뒤에서 m < s_j 인 첫 위치 — m 의 희소 표 내림 탐색으로 구하고, 갱신 사슬의 끝은
      포인터 도약(log n 번)으로 찾는다. 비교·색인만 쓴다.
    """
    N, n = values.shape
    dev = values.device
    if n == 0:
        return torch.full((N,), INF, dtype=values.dtype, device=dev)
    s = window_kth(values, spec)
    fin = torch.isfinite(values)
    inf_t = torch.full_like(values, INF)
    m = torch.where(fin, torch.maximum(values, s), inf_t)
    sc = torch.where(fin, s, inf_t)
    # 희소 표: ST[k][i] = min m[i : i + 2^k] (범위 밖 = +inf)
    K = max(1, int(math.ceil(math.log2(n))))
    pad = torch.full((N, 1 << K), INF, dtype=values.dtype, device=dev)
    st = [torch.cat([m, pad], dim=1)]
    for k in range(1, K + 1):
        prev = st[-1]
        sh = 1 << (k - 1)
        st.append(torch.minimum(prev, torch.cat([prev[:, sh:], prev[:, :sh].new_full(
            (N, sh), INF)], dim=1)))
    # 뒤쪽 최소 (존재 판정): suf[j] = min m[j:]
    suf = torch.flip(torch.cummin(torch.flip(m, [1]), dim=1).values, [1])
    suf = torch.cat([suf, suf.new_full((N, 1), INF)], dim=1)
    ar = torch.arange(n, device=dev).unsqueeze(0).expand(N, -1)
    exists = suf[:, 1:] < sc                                   # j 뒤에 m < s_j 가 있나
    pos = ar + 1
    for k in range(K, -1, -1):
        blk = st[k].gather(1, pos.clamp(max=n + (1 << K) - 1))
        pos = torch.where(blk >= sc, pos + (1 << k), pos)
    nxt = torch.where(exists, pos.clamp(max=n - 1), ar)
    for _ in range(max(1, int(math.ceil(math.log2(n))) + 1)):
        nxt = nxt.gather(1, nxt)
    # 첫 갱신 = m 이 유한한 첫 위치
    finm = torch.isfinite(m)
    has = finm.any(dim=1)
    j1 = torch.argmax(finm.long(), dim=1)
    last = nxt.gather(1, j1.unsqueeze(1)).squeeze(1)
    out = sc.gather(1, last.unsqueeze(1)).squeeze(1)
    return torch.where(has, out, torch.full_like(out, INF))


# ======================================================================== 게이트
class SafetyGateBatch:
    """
    SafetyGate (safety_gate.hpp:269-386) 의 배치판 — env N 개가 각자 게이트 하나를 가진다.

    스캔 기하(빔 수, 각 증분, 원형 여부, LiDAR 원점)는 env 공통이고 생성 때 고정한다. 점군·트랙·도킹 예외 다각형의
    최대 개수(max_cloud·max_tracks·max_poly)도 고정 (0 이면 그 입력이 없다). 모든 시각은 초 단위 float.
    """

    def __init__(self, n, device, params=DEPLOYED, n_beams=720, angle_increment=None,
                 circular=True, origin=(0.15, 0.0), max_cloud=0, max_tracks=0, max_poly=0,
                 half_cap=None):
        """상태 버퍼를 만들고 reset 한다."""
        self.p = params
        self.n = n
        self.device = device
        self.B = n_beams
        self.inc = float(angle_increment if angle_increment is not None else
                         (3.13286601 + 3.14159265) / 719.0)
        self.circular = bool(circular)
        self.origin = (float(origin[0]), float(origin[1]))
        self.Cmax, self.Tmax, self.Pmax = int(max_cloud), int(max_tracks), int(max_poly)
        self.half_cap = half_cap
        z = lambda: torch.zeros(n, dtype=F64, device=device)  # noqa: E731
        self.input_v, self.input_w, self.input_stamp = z(), z(), z()
        self.meas_v, self.meas_w, self.meas_stamp = z(), z(), z()
        self.intent_v, self.intent_w = z(), z()
        self.have_intent = torch.zeros(n, dtype=torch.bool, device=device)
        self.beams = torch.full((n, n_beams, 2), math.nan, dtype=F64, device=device)
        self.ranges = torch.full((n, n_beams), math.nan, dtype=F64, device=device)
        self.has_beams = torch.zeros(n, dtype=torch.bool, device=device)
        self.scan_stamp, self.counted_scan_stamp = z(), z()
        self.cloud = torch.full((n, self.Cmax, 2), math.nan, dtype=F64, device=device)
        self.cloud_stamp, self.cloud_capture, self.counted_cloud_stamp = z(), z(), z()
        self.min_ttc, self.ttc_stamp, self.tracks_stamp = z(), z(), z()
        # 트랙 [N, T, 6] = (ttc, is_dynamic, px, py, vx, vy), 유효 [N, T]
        self.tracks = torch.full((n, self.Tmax, 6), math.nan, dtype=F64, device=device)
        self.tracks_ok = torch.zeros(n, self.Tmax, dtype=torch.bool, device=device)
        self.poly = torch.zeros(n, self.Pmax, 2, dtype=F64, device=device)
        self.n_poly = torch.zeros(n, dtype=torch.long, device=device)
        self.excl_stamp = z()
        self.prox = torch.zeros(n, dtype=torch.bool, device=device)
        self.causes = torch.zeros(n, dtype=torch.long, device=device)
        self.zone = torch.zeros(n, dtype=torch.long, device=device)
        self.ch_val = torch.full((n, 5), INF, dtype=F64, device=device)
        self.ch_cnt = torch.zeros(n, 5, dtype=torch.long, device=device)
        self.any_cloud = False      # 점군·트랙·예외를 한 번이라도 넣었나 (계산을 건너뛰는 용도 — 결과는 같다)
        self.any_tracks = False
        self.any_poly = False
        self.reset(None)

    # ------------------------------------------------------------------ 입력
    def _m(self, mask):
        if mask is None:
            return torch.ones(self.n, dtype=torch.bool, device=self.device)
        return mask

    @staticmethod
    def _col(t, like):
        if not torch.is_tensor(t):
            return torch.full(like.shape, float(t), dtype=F64, device=like.device)
        return t.to(F64)

    def reset(self, ids):
        """새 게이트 상태 (생성자 직후와 같다). ids None = 전부."""
        sl = slice(None) if ids is None else ids
        for t in (self.input_v, self.input_w, self.meas_v, self.meas_w, self.intent_v,
                  self.intent_w):
            t[sl] = 0.0
        for t in (self.input_stamp, self.meas_stamp, self.scan_stamp, self.counted_scan_stamp,
                  self.cloud_stamp, self.cloud_capture, self.counted_cloud_stamp, self.ttc_stamp,
                  self.tracks_stamp, self.excl_stamp):
            t[sl] = -1.0
        self.have_intent[sl] = False
        self.beams[sl] = math.nan
        self.ranges[sl] = math.nan
        self.has_beams[sl] = False
        self.cloud[sl] = math.nan
        self.min_ttc[sl] = INF
        self.tracks[sl] = math.nan
        self.tracks_ok[sl] = False
        self.poly[sl] = 0.0
        self.n_poly[sl] = 0
        self.prox[sl] = False
        self.causes[sl] = 0
        self.zone[sl] = ZONE_CLEAR
        self.ch_val[sl] = INF
        self.ch_cnt[sl] = 0

    def set_command(self, v, w, stamp, mask=None):
        """명령 입력 — setCommand (:400-405): 유한하지 않으면 0."""
        m = self._m(mask)
        v = self._col(v, self.input_v)
        w = self._col(w, self.input_w)
        self.input_v = torch.where(m, torch.where(torch.isfinite(v), v, torch.zeros_like(v)),
                                   self.input_v)
        self.input_w = torch.where(m, torch.where(torch.isfinite(w), w, torch.zeros_like(w)),
                                   self.input_w)
        self.input_stamp = torch.where(m, self._col(stamp, self.input_stamp), self.input_stamp)

    def set_measured(self, v, w, stamp, mask=None):
        """측정 속도 — setMeasuredVelocity (:407-422): 1 차 저역 통과 (dt/(τ+dt)), 끊기면 재설정."""
        p = self.p
        v = self._col(v, self.meas_v)
        w = self._col(w, self.meas_w)
        stamp = self._col(stamp, self.meas_stamp)
        m = self._m(mask) & torch.isfinite(v) & torch.isfinite(w)
        dt = torch.where(self.meas_stamp >= 0.0, stamp - self.meas_stamp,
                         torch.full_like(stamp, -1.0))
        tau = p.measured_velocity_time_constant
        direct = (dt <= 0.0) | (dt > p.measured_velocity_timeout) | (tau <= 0.0)
        dt_safe = torch.where(direct, torch.ones_like(dt), dt)
        alpha = dt_safe / (tau + dt_safe)
        nv = torch.where(direct, v, self.meas_v + alpha * (v - self.meas_v))
        nw = torch.where(direct, w, self.meas_w + alpha * (w - self.meas_w))
        self.meas_v = torch.where(m, nv, self.meas_v)
        self.meas_w = torch.where(m, nw, self.meas_w)
        self.meas_stamp = torch.where(m, stamp, self.meas_stamp)

    def set_scan(self, beams, stamp, mask=None):
        """스캔 — setScan (:349-361): base_link 빔 끝점 [N, B, 2] (무효 = NaN), 거리 = |빔 − 원점|."""
        m = self._m(mask)
        dx = beams[..., 0] - self.origin[0]
        dy = beams[..., 1] - self.origin[1]
        rng = torch.sqrt(dx * dx + dy * dy)
        self.beams = torch.where(m.view(-1, 1, 1), beams.to(F64), self.beams)
        self.ranges = torch.where(m.view(-1, 1), rng, self.ranges)
        self.has_beams = self.has_beams | m
        self.scan_stamp = torch.where(m, self._col(stamp, self.scan_stamp), self.scan_stamp)

    def set_cloud(self, points, capture, received, mask=None):
        """깊이 점군 — setCloud (:374-379): [N, C, 2] (없는 점 = NaN)."""
        m = self._m(mask)
        if self.Cmax:
            self.cloud = torch.where(m.view(-1, 1, 1), points.to(F64), self.cloud)
            self.any_cloud = True
        self.cloud_capture = torch.where(m, self._col(capture, self.cloud_capture),
                                         self.cloud_capture)
        self.cloud_stamp = torch.where(m, self._col(received, self.cloud_stamp), self.cloud_stamp)

    def set_tracks(self, tracks, ok, capture, received, mask=None):
        """추적 결과 — setTracks (:444-450): [N, T, 6] = (ttc, 동적, px, py, vx, vy), ok [N, T]."""
        m = self._m(mask)
        p = self.p
        self.tracks = torch.where(m.view(-1, 1, 1), tracks.to(F64), self.tracks)
        self.tracks_ok = torch.where(m.view(-1, 1), ok, self.tracks_ok)
        ttc = self.tracks[..., 0]
        use = self.tracks_ok & ~torch.isnan(ttc)
        if p.ttc_only_dynamic:
            use = use & (self.tracks[..., 1] > 0.5)
        mt = torch.where(use, ttc, torch.full_like(ttc, INF))
        mt = mt.min(dim=1).values if self.Tmax else torch.full_like(self.min_ttc, INF)
        self.min_ttc = torch.where(m, mt, self.min_ttc)
        self.tracks_stamp = torch.where(m, self._col(capture, self.tracks_stamp),
                                        self.tracks_stamp)
        self.ttc_stamp = torch.where(m, self._col(received, self.ttc_stamp), self.ttc_stamp)
        self.any_tracks = self.any_tracks or bool(self.Tmax)

    def set_min_ttc(self, ttc, stamp, mask=None):
        """기하 없는 TTC — setMinTtc (:436-442): 트랙 목록을 비운다."""
        m = self._m(mask)
        ttc = self._col(ttc, self.min_ttc)
        self.min_ttc = torch.where(m, torch.where(torch.isnan(ttc), torch.full_like(ttc, INF),
                                                  ttc), self.min_ttc)
        self.ttc_stamp = torch.where(m, self._col(stamp, self.ttc_stamp), self.ttc_stamp)
        self.tracks_ok = torch.where(m.view(-1, 1), torch.zeros_like(self.tracks_ok),
                                     self.tracks_ok)
        self.tracks_stamp = torch.where(m, torch.full_like(self.tracks_stamp, -1.0),
                                        self.tracks_stamp)

    def set_exclusion(self, poly, n_poly, stamp, mask=None):
        """도킹 예외 — setExclusionPolygon (:488-492): base_link 다각형 [N, P, 2], 꼭짓점 수 [N]."""
        m = self._m(mask)
        if self.Pmax:
            self.poly = torch.where(m.view(-1, 1, 1), poly.to(F64), self.poly)
            self.any_poly = True
        self.n_poly = torch.where(m, n_poly.long(), self.n_poly)
        self.excl_stamp = torch.where(m, self._col(stamp, self.excl_stamp), self.excl_stamp)

    # ------------------------------------------------------------------ 내부
    def _moving(self, v, w):
        p = self.p
        return (v.abs() > p.motion_epsilon_v) | (w.abs() > p.motion_epsilon_w)

    def _meas_fresh(self, now):
        return (self.meas_stamp >= 0.0) & \
            (now - self.meas_stamp <= self.p.measured_velocity_timeout)

    def _latency(self, stamp, now):
        """지연 보정 — latencyCorrection (:381-390) → 자세 (x, y, yaw) [N] (보정 없음 = 0, 0, 0)."""
        p = self.p
        use = self._meas_fresh(now) & (stamp >= 0.0)
        if not p.latency_compensation:
            use = torch.zeros_like(use)
        age = torch.clamp(now - stamp, min=0.0, max=p.max_compensation_age)
        x, y, yaw = unicycle_delta(self.meas_v, self.meas_w, age)
        ix, iy, iyaw = pose_inverse(x, y, yaw)
        zero = torch.zeros_like(ix)
        return (torch.where(use, ix, zero), torch.where(use, iy, zero),
                torch.where(use, iyaw, zero))

    def _is_self(self, x, y):
        p = self.p
        hl = 0.5 * p.footprint_length - p.self_filter_margin
        hw = 0.5 * p.footprint_width - p.self_filter_margin
        return (x.abs() < hl) & (y.abs() < hw)

    def _excluded(self, qx, qy, now):
        p = self.p
        if not self.any_poly:
            return torch.zeros(qx.shape, dtype=torch.bool, device=qx.device)
        live = (self.n_poly >= 3) & (self.excl_stamp >= 0.0) & \
            (now - self.excl_stamp <= p.exclusion_timeout)
        return live.unsqueeze(1) & point_in_polygon(qx, qy, self.poly, self.n_poly)

    def _robust(self, values, spec):
        return window_statistic(values, spec)

    def _escape_allowed(self, v, w, px, py, now_d, g_now, guard_spec):
        """탈출 판정 — escapeAllowed (:554-580): 가드 거리 창 중앙값이 0.3 s 예측 동안 0.01 m 넘게 줄지 않는가."""
        p = self.p
        ok = torch.ones(self.n, dtype=torch.bool, device=self.device)
        fin = torch.isfinite(now_d)
        for step in range(1, ESCAPE_STEPS + 1):
            t = torch.full_like(v, p.escape_horizon * step / ESCAPE_STEPS)
            x, y, yaw = unicycle_delta(v, w, t)
            ix, iy, iyaw = pose_inverse(x, y, yaw)
            qx, qy = pose_apply(ix.unsqueeze(1), iy.unsqueeze(1), iyaw.unsqueeze(1),
                                px, py)
            after = torch.where(fin, distance_to_rectangle(qx, qy, p.footprint_length,
                                                           p.footprint_width),
                                torch.full_like(now_d, INF))
            g = self._robust(after, guard_spec)
            ok = ok & ~(g < g_now - ESCAPE_TOLERANCE)
        return ok

    def _yield_allowed(self, v, w, now):
        """양보 탈출 판정 — yieldEscapeAllowed (:582-642) 를 트랙마다 → [N, T] bool."""
        p = self.p
        tr = self.tracks
        dyn = tr[..., 1] > 0.5
        px, py, vx, vy = tr[..., 2], tr[..., 3], tr[..., 4], tr[..., 5]
        fin = torch.isfinite(px) & torch.isfinite(py) & torch.isfinite(vx) & torch.isfinite(vy)
        fresh = (self.tracks_stamp >= 0.0) & (now - self.tracks_stamp <= p.ttc_max_age)
        age = torch.clamp(now - self.tracks_stamp, min=0.0, max=p.max_compensation_age)
        cx, cy, cyaw = self._latency(self.tracks_stamp, now)
        cx, cy, cyaw = cx.unsqueeze(1), cy.unsqueeze(1), cyaw.unsqueeze(1)
        agec = age.unsqueeze(1)
        o0x, o0y = pose_apply(cx, cy, cyaw, px + agec * vx, py + agec * vy)
        c, s = torch.cos(cyaw), torch.sin(cyaw)
        vox, voy = c * vx - s * vy, s * vx + c * vy
        r_sum = 0.5 * math.hypot(p.footprint_length, p.footprint_width) + \
            p.dynamic_obstacle_radius
        inside = ~(torch.sqrt(o0x * o0x + o0y * o0y) > r_sum + p.yield_escape_overlap_margin)
        speed = torch.sqrt(vox * vox + voy * voy)
        has_line = speed >= p.yield_escape_min_obstacle_speed
        sp_safe = torch.where(has_line, speed, torch.ones_like(speed))
        ux = torch.where(has_line, vox / sp_safe, torch.zeros_like(vox))
        uy = torch.where(has_line, voy / sp_safe, torch.zeros_like(voy))

        def lateral(cxp, cyp):
            dxp = cxp - o0x
            dyp = cyp - o0y
            return (ux * dyp - uy * dxp).abs()

        def clearance(rx, ry, ryaw, ox, oy):
            ix, iy, iyaw = pose_inverse(rx, ry, ryaw)
            qx, qy = pose_apply(ix, iy, iyaw, ox, oy)
            return distance_to_rectangle(qx, qy, p.footprint_length, p.footprint_width) - \
                p.dynamic_obstacle_radius
        zero = torch.zeros_like(o0x)
        start = torch.where(has_line, lateral(zero, zero), torch.sqrt(o0x * o0x + o0y * o0y))
        prev = start
        ok = torch.ones_like(inside)
        vv = v.unsqueeze(1).expand_as(o0x)
        ww = w.unsqueeze(1).expand_as(o0x)
        for step in range(1, ESCAPE_STEPS + 1):
            tau = p.escape_horizon * step / ESCAPE_STEPS
            rx, ry, ryaw = unicycle_delta(vv, ww, torch.full_like(vv, tau))
            ox = o0x + tau * vox
            oy = o0y + tau * voy
            dxr, dyr = rx - ox, ry - oy
            cur = torch.where(has_line, lateral(rx, ry), torch.sqrt(dxr * dxr + dyr * dyr))
            ok = ok & ~(cur < prev - ESCAPE_TOLERANCE)
            prev = torch.maximum(prev, cur)
            ok = ok & ~(clearance(rx, ry, ryaw, ox, oy) <
                        clearance(zero, zero, zero, ox, oy) - YIELD_CLEARANCE_TOLERANCE)
        res = ok & (prev >= start + p.yield_escape_min_gain)
        return res & inside & dyn & fin & fresh.unsqueeze(1) & self.tracks_ok

    # ------------------------------------------------------------------ 판정
    def evaluate(self, now):
        """
        판정 — 원본 SafetyGate 의 판정 함수 (:644-992). 반환 dict (모두 [N]).

        v, w: 제한된 명령 / zone / prox (STOP 래치) / causes (원인 비트: 1 lidar, 2 depth,
        4 contact, 8 dock) / escaping / retreat / yield_escaping / ttc_limited / cloud_stale /
        command_stale / speed_limit / angular_limit / min_distance / lidar_distance /
        cloud_distance / contact_distance / exclusion_distance / min_ttc / v_in / w_in.
        """
        p = self.p
        h = p.zone_hysteresis
        now = self._col(now, self.input_v)
        dev = self.device
        inf_n = torch.full((self.n,), INF, dtype=F64, device=dev)
        # 입력 명령 (:669-675)
        cmd_fresh = (self.input_stamp >= 0.0) & (now - self.input_stamp <= p.command_timeout)
        zero = torch.zeros_like(now)
        v_in = torch.where(cmd_fresh, torch.clamp(self.input_v, p.min_linear_velocity,
                                                  p.max_linear_velocity), zero)
        w_in = torch.where(cmd_fresh, torch.clamp(self.input_w, -p.max_angular_velocity,
                                                  p.max_angular_velocity), zero)
        # 운동 가설 (:507-530) — 의도 갱신은 상태를 바꾼다
        mv_c = self._moving(v_in, w_in)
        self.intent_v = torch.where(mv_c, v_in, self.intent_v)
        self.intent_w = torch.where(mv_c, w_in, self.intent_w)
        self.have_intent = self.have_intent | mv_c
        use_meas = self._meas_fresh(now) & self._moving(self.meas_v, self.meas_w)
        use_intent = ~mv_c & ~use_meas & self.have_intent
        if not p.hold_motion_intent:
            use_intent = torch.zeros_like(use_intent)
        reg_a = Region(torch.where(mv_c, v_in, self.intent_v),
                       torch.where(mv_c, w_in, self.intent_w), mv_c | use_intent, p)
        reg_b = Region(self.meas_v, self.meas_w, use_meas, p)

        def approach(qx, qy):
            return torch.minimum(reg_a.approach(qx, qy), reg_b.approach(qx, qy))
        # LiDAR (:693-711)
        sx, sy, syaw = self._latency(self.scan_stamp, now)
        rx, ry = self.beams[..., 0], self.beams[..., 1]
        valid = torch.isfinite(rx) & torch.isfinite(ry) & ~self._is_self(rx, ry)
        qx, qy = pose_apply(sx.unsqueeze(1), sy.unsqueeze(1), syaw.unsqueeze(1),
                            rx, ry)
        inf_b = torch.full_like(qx, INF)
        guard_d = torch.where(valid, distance_to_rectangle(qx, qy, p.footprint_length,
                                                           p.footprint_width), inf_b)
        ex = valid & self._excluded(qx, qy, now)
        s_all = approach(qx, qy)
        main_s = torch.where(valid & ~ex, s_all, inf_b)
        excl_s = torch.where(ex, s_all, inf_b)
        beam_spec = WindowSpec(self.ranges, self.inc, p.beam_window_width, p.beam_window,
                               p.beam_support, self.circular, self.half_cap)
        guard_spec = WindowSpec(self.ranges, self.inc, p.guard_window_width, p.guard_window,
                                0.5, self.circular, self.half_cap)
        lidar_main = self._robust(main_s, beam_spec)
        lidar_excl = self._robust(excl_s, beam_spec) if self.any_poly else inf_n
        guard = self._robust(guard_d, guard_spec)
        # 깊이 점군 (:713-735)
        cloud_fresh = (self.cloud_stamp >= 0.0) & (now - self.cloud_stamp <= p.cloud_timeout) & \
            (now - self.cloud_capture <= p.cloud_max_age)
        cloud_stale = cloud_fresh.logical_not() & p.cloud_required
        cloud_main, cloud_excl = inf_n, inf_n
        if self.Cmax and self.any_cloud:
            cx, cy, cyaw = self._latency(self.cloud_capture, now)
            px_, py_ = self.cloud[..., 0], self.cloud[..., 1]
            cval = torch.isfinite(px_) & torch.isfinite(py_) & ~self._is_self(px_, py_)
            cqx, cqy = pose_apply(cx.unsqueeze(1), cy.unsqueeze(1),
                                  cyaw.unsqueeze(1), px_, py_)
            cs = approach(cqx, cqy)
            keep = cval & torch.isfinite(cs)
            cex = keep & self._excluded(cqx, cqy, now)
            inf_c = torch.full_like(cs, INF)
            cm = kth_smallest(torch.where(keep & ~cex, cs, inf_c), p.cloud_min_points)
            ce = kth_smallest(torch.where(cex, cs, inf_c), p.cloud_min_points)
            cloud_main = torch.where(cloud_fresh, cm, inf_n)
            cloud_excl = torch.where(cloud_fresh, ce, inf_n)
        # 시간 일관성 (:737-750)
        new_scan = (self.scan_stamp >= 0.0) & (self.scan_stamp != self.counted_scan_stamp)
        self.counted_scan_stamp = torch.where(new_scan, self.scan_stamp, self.counted_scan_stamp)
        new_cloud = cloud_fresh & (self.cloud_stamp != self.counted_cloud_stamp)
        self.counted_cloud_stamp = torch.where(new_cloud, self.cloud_stamp,
                                               self.counted_cloud_stamp)
        vals = torch.stack([lidar_main, lidar_excl, guard, cloud_main, cloud_excl], dim=1)
        thr = torch.tensor([p.emergency_stop_distance, p.exclusion_stop_distance,
                            p.contact_guard_distance, p.emergency_stop_distance,
                            p.exclusion_stop_distance], dtype=F64, device=dev).unsqueeze(0)
        newf = torch.stack([new_scan, new_scan, new_scan, new_cloud, new_cloud], dim=1)
        self.ch_val = vals
        self.ch_cnt = torch.where(newf, torch.where(vals <= thr, self.ch_cnt + 1,
                                                    torch.zeros_like(self.ch_cnt)), self.ch_cnt)
        d_main = torch.minimum(lidar_main, cloud_main)
        d_excl = torch.minimum(lidar_excl, cloud_excl)
        # STOP 래치 (:760-801)
        conf = (self.ch_val <= thr) & ((self.ch_cnt >= p.stop_confirm_frames) |
                                       (self.ch_val <= thr - p.immediate_stop_margin))
        causes_now = conf[:, CH_LIDAR].long() * CAUSE_LIDAR + \
            conf[:, CH_CLOUD].long() * CAUSE_DEPTH + conf[:, CH_GUARD].long() * CAUSE_CONTACT + \
            (conf[:, CH_LIDAR_EXCL] | conf[:, CH_CLOUD_EXCL]).long() * CAUSE_DOCK
        anyc = causes_now != 0
        base = torch.where(self.prox, self.causes, torch.zeros_like(self.causes))
        has = lambda c: (self.causes & c) != 0  # noqa: E731
        lidar_clear = ~has(CAUSE_LIDAR) | (lidar_main > p.stop_release_distance)
        depth_clear = ~has(CAUSE_DEPTH) | (cloud_fresh & (cloud_main > p.stop_release_distance))
        excl_clear = ~has(CAUSE_DOCK) | (d_excl > p.exclusion_stop_distance + h)
        guard_clear = ~has(CAUSE_CONTACT) | (guard > p.contact_guard_distance + h)
        release = ~anyc & self.prox & lidar_clear & depth_clear & excl_clear & guard_clear
        self.causes = torch.where(anyc, base | causes_now,
                                  torch.where(release, torch.zeros_like(self.causes),
                                              self.causes))
        self.prox = torch.where(anyc, torch.ones_like(self.prox),
                                torch.where(release, torch.zeros_like(self.prox), self.prox))
        # 존 (:803-822)
        raw = classify_zone(d_main, p)
        raw = torch.where(raw == ZONE_STOP, torch.full_like(raw, ZONE_CRITICAL), raw)
        boundary = torch.where(self.zone == ZONE_CRITICAL,
                               torch.full_like(d_main, p.critical_zone_distance),
                               torch.full_like(d_main, p.warning_zone_distance))
        take = (raw >= self.zone) | (self.zone == ZONE_STOP) | (d_main > boundary + h)
        zone = torch.where(take, raw, self.zone)
        self.zone = torch.where(self.prox, torch.full_like(zone, ZONE_STOP), zone)
        # 상한 (:824-857)
        u_cap = inf_n
        u_cap = torch.where(self.zone == ZONE_WARNING,
                            torch.minimum(u_cap, torch.full_like(u_cap, p.warning_zone_max_speed)),
                            u_cap)
        u_cap = torch.where(self.zone == ZONE_CRITICAL,
                            torch.minimum(u_cap, torch.full_like(u_cap,
                                                                 p.critical_zone_max_speed)),
                            u_cap)
        if p.clearance_speed_limit_enabled:
            u_cap = torch.minimum(u_cap, clearance_speed_limit(d_main, p.emergency_stop_distance,
                                                               p))
        fin_ex = torch.isfinite(d_excl)
        ucx = torch.minimum(u_cap, torch.full_like(u_cap, p.critical_zone_max_speed))
        if p.clearance_speed_limit_enabled:
            ucx = torch.minimum(ucx, clearance_speed_limit(d_excl, p.exclusion_stop_distance, p))
        u_cap = torch.where(fin_ex, ucx, u_cap)
        v_fwd = torch.where(cloud_stale, torch.full_like(u_cap, p.degraded_mode_max_speed), inf_n)
        ttc_fresh = (self.ttc_stamp >= 0.0) & (now - self.ttc_stamp <= p.ttc_max_age)
        min_ttc = torch.where(ttc_fresh, self.min_ttc, inf_n)
        ttc_on = ttc_fresh & (self.min_ttc <= p.ttc_critical)
        if not p.ttc_limit_enabled:
            ttc_on = torch.zeros_like(ttc_on)
        v_ttc = torch.where(ttc_on, torch.clamp(p.max_deceleration *
                                                (self.min_ttc - p.reaction_latency), min=0.0),
                            inf_n)
        speed_limit = torch.minimum(torch.minimum(u_cap, v_ttc),
                                    torch.minimum(v_fwd, torch.full_like(u_cap,
                                                                         p.max_linear_velocity)))
        r_circ = 0.5 * math.hypot(p.footprint_length, p.footprint_width)
        angular_limit = torch.minimum(torch.full_like(u_cap, p.max_angular_velocity),
                                      u_cap / r_circ)
        half_l = 0.5 * p.footprint_length
        half_w = 0.5 * p.footprint_width

        def scale(v, w, u_lim, v_lim):
            s = torch.ones_like(v)
            u = max_point_speed(half_l, half_w, v, w)
            u_safe = torch.where(u > 0.0, u, torch.ones_like(u))
            s = torch.where(u > u_lim, torch.minimum(s, u_lim / u_safe), s)
            av = v.abs()
            av_safe = torch.where(av > 0.0, av, torch.ones_like(av))
            s = torch.where(av > v_lim, torch.minimum(s, v_lim / av_safe), s)
            return v * s, w * s
        # 탈출 판정이 필요한 env (계산은 순수 함수라 필요한 곳만 골라 써도 결과가 같다)
        depth_blind = has(CAUSE_DEPTH) & ~cloud_fresh
        approaching = depth_blind | \
            ((has(CAUSE_LIDAR) | has(CAUSE_DEPTH)) & (d_main <= p.stop_release_distance)) | \
            (has(CAUSE_DOCK) & (d_excl <= p.exclusion_stop_distance + h)) | \
            (d_main <= p.emergency_stop_distance) | (d_excl <= p.exclusion_stop_distance)
        moving_in = self._moving(v_in, w_in)
        stale = ~cmd_fresh
        esc_need = ~stale & self.prox & ~approaching & moving_in & p.allow_escape
        ttc_binds = torch.isfinite(v_ttc) & (v_ttc < v_in.abs()) & (v_ttc < u_cap)
        ret_need = ~stale & ~self.prox & ttc_binds & p.allow_escape & cloud_fresh & \
            self.has_beams
        need = esc_need | ret_need
        if bool(need.any()):
            esc_ok = self._escape_allowed(v_in, w_in, qx, qy, guard_d, guard, guard_spec)
        else:
            esc_ok = torch.zeros_like(need)
        escaping = esc_need & esc_ok
        retreat = ret_need & esc_ok
        # 몸 원통 안 양보 탈출 (:940-961)
        v_yield = v_ttc
        yield_esc = torch.zeros_like(need)
        if p.allow_escape and p.yield_escape_enabled and self.Tmax and self.any_tracks:
            yneed = ~stale & ~self.prox & ~retreat & ttc_binds & self.tracks_ok.any(dim=1)
            if bool(yneed.any()):
                ttc = self.tracks[..., 0]
                use = self.tracks_ok & ~torch.isnan(ttc)
                if p.ttc_only_dynamic:
                    use = use & (self.tracks[..., 1] > 0.5)
                yok = self._yield_allowed(v_in, w_in, now)
                ex_t = use & (ttc <= p.ttc_critical) & yok
                rest = use & ~ex_t
                ttc_rest = torch.where(rest, ttc, torch.full_like(ttc, INF)).min(dim=1).values
                exempt = ex_t.any(dim=1)
                v_rest = torch.where(ttc_rest <= p.ttc_critical,
                                     torch.clamp(p.max_deceleration *
                                                 (ttc_rest - p.reaction_latency), min=0.0),
                                     inf_n)
                vy_ = torch.minimum(v_rest, torch.full_like(v_rest, p.yield_escape_speed))
                use_y = yneed & exempt
                v_yield = torch.where(use_y, vy_, v_ttc)
                yield_esc = use_y & (vy_ > v_ttc)
        # 명령 결정 (:874-976)
        # (1) 근접 정지 중 탈출
        v_e, w_e = scale(v_in, w_in, torch.minimum(torch.full_like(u_cap, p.escape_max_speed),
                                                   u_cap),
                         torch.where(v_in > 0.0, torch.minimum(v_ttc, v_fwd), v_ttc))
        # (2) 평상
        u_ret = torch.minimum(u_cap, torch.full_like(u_cap, p.retreat_max_speed))
        u_lim = torch.where(retreat, u_ret,
                            torch.where(yield_esc,
                                        torch.minimum(u_cap,
                                                      torch.full_like(u_cap,
                                                                      p.yield_escape_speed)),
                                        u_cap))
        v_lim = torch.where(retreat, inf_n, torch.where(yield_esc, v_yield, v_ttc))
        v_n, w_n = scale(v_in, w_in, u_lim, torch.where(v_in > 0.0, torch.minimum(v_lim, v_fwd),
                                                        v_lim))
        ttc_limited = ~stale & ~self.prox & torch.isfinite(v_lim) & (v_in.abs() > v_lim) & \
            (v_lim <= u_cap)
        v = torch.where(stale, zero, torch.where(self.prox, torch.where(escaping, v_e, zero), v_n))
        w = torch.where(stale, zero, torch.where(self.prox, torch.where(escaping, w_e, zero), w_n))
        return {
            'v': v, 'w': w, 'zone': self.zone.clone(), 'prox': self.prox.clone(),
            'causes': self.causes.clone(), 'escaping': escaping, 'retreat': retreat,
            'yield_escaping': yield_esc, 'ttc_limited': ttc_limited, 'cloud_stale': cloud_stale,
            'command_stale': stale, 'speed_limit': speed_limit, 'angular_limit': angular_limit,
            'min_distance': d_main, 'lidar_distance': lidar_main, 'cloud_distance': cloud_main,
            'contact_distance': guard, 'exclusion_distance': d_excl, 'min_ttc': min_ttc,
            'v_in': v_in, 'w_in': w_in,
        }

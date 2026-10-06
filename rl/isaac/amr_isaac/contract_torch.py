"""
계약 v1.2.0 의 배치 torch 구현 — Isaac Lab env 가 쓰는 관측·보상·비용·적분·프로파일러·safety_lite.

기준 구현 rl/contract/amr_rl_contract/ (numpy, 비벡터화) 를 배치 축 B 로 옮겼다. 규범 순서는
golden > 기준 구현 > contract.md 이고, 이 모듈은 rl/isaac/tests/test_contract_golden.py 가 golden 32 개와
허용오차 안에서 대조한다. Isaac Sim 에 의존하지 않는다 (torch 만) — CPU 에서도 시험할 수 있다.

보상 (v1.2.0, contract.md §6): 설계 매개변수는 기준 구현의 reward.RewardSpec 을 그대로 받는다
(기본 reward.SPEC = v1.2.0, reward.SPEC_V110 = v1.1.0). v1.2.0 의 세 변경 —
  ① Φ·진행의 거리 d = max(d_arc, 목표 직선거리)  (전이 전·후 모두)
  ② 진행 효용 κ_b·(d − d')  (종단 스텝도 실제 d', PBRS 처럼 0 으로 두지 않는다)
  ③ 이탈 힌지 −q_d·1[e > 1 m]  (이탈 구간 여부와 무관)
— 은 reward_step 에 있고, env 가 쓰는 전이 전 거리 캐시는 observe_with_cache 가 만든다.

수치 정밀도: 기하·경로·보상은 float64 로 계산하고 관측만 float32 로 내보낸다
(contract.md §12 "d_arc 와 shaping 은 float64 로").
상수는 기준 구현의 constants 를 그대로 읽는다 (값을 복사하지 않는다 — 계약 판이 바뀌면 같이 바뀐다).
"""

from dataclasses import dataclass, field, fields
import math
import os
import sys
from typing import Optional

import torch

_CONTRACT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                          '..', '..', 'contract'))
if _CONTRACT not in sys.path:
    sys.path.insert(0, _CONTRACT)

from amr_rl_contract import constants as C  # noqa: E402
from amr_rl_contract import reward as RW  # noqa: E402
from amr_rl_contract.observation import OBS_DIM, OBS_LAYOUT  # noqa: E402

F64 = torch.float64
INF = math.inf
N_PREV = C.N_PREV_ACTIONS


# ======================================================================== 상태
@dataclass
class BatchState:
    """
    계약 State 의 배치판 (모양 [B, …]). 무반사·상한 없음·TTC 없음 = +inf.

    path_pts 는 [B, P, 2] 로 채우고 path_n 이 실제 점 수다 (나머지는 마지막 점 반복).
    tracks 는 [B, T, 6] = (x, y, vx, vy, is_dynamic, valid), track_id 는 [B, T].
    """

    pose: torch.Tensor                 # [B, 3] (x, y, θ) 지도 좌표
    path_pts: torch.Tensor             # [B, P, 2]
    path_n: torch.Tensor               # [B] long
    scan: torch.Tensor                 # [B, 720] 무효 = nan/inf/범위 밖
    v_meas: torch.Tensor
    w_meas: torch.Tensor
    a_v_meas: torch.Tensor
    a_w_meas: torch.Tensor
    v_cmd: torch.Tensor
    w_cmd: torch.Tensor
    prev_actions: torch.Tensor         # [B, 3, 2] 최근 순
    v_cap: torch.Tensor
    scan_age: torch.Tensor
    track_age: torch.Tensor
    tracks: torch.Tensor               # [B, T, 6]
    track_id: torch.Tensor             # [B, T] long
    zone: torch.Tensor                 # [B] long
    speed_limit: torch.Tensor          # [B] (+inf = 없음)
    proximity_stop: torch.Tensor       # [B] bool
    min_ttc: torch.Tensor              # [B] (+inf = 없음)
    in_dev: torch.Tensor               # [B] bool
    dev_peak: torch.Tensor
    dev_t: torch.Tensor
    path_cum: Optional[torch.Tensor] = field(default=None)   # [B, P] 누적 호길이 (캐시)

    @property
    def batch(self):
        """배치 크기."""
        return self.pose.shape[0]

    def ensure_cum(self):
        """누적 호길이 캐시를 만든다."""
        if self.path_cum is None:
            self.path_cum = path_cumulative(self.path_pts, self.path_n)
        return self.path_cum

    def clone(self):
        """깊은 복사."""
        kw = {}
        for f in fields(self):
            v = getattr(self, f.name)
            kw[f.name] = v.clone() if isinstance(v, torch.Tensor) else v
        return BatchState(**kw)


def _none_inf(v):
    return INF if v is None else float(v)


def states_from_dicts(dicts, device='cpu', max_tracks=None):
    """시험용: golden JSON 의 state dict 목록 → BatchState."""
    B = len(dicts)
    P = max(len(d['path']) for d in dicts)
    T = max([len(d.get('tracks', [])) for d in dicts] + [1])
    if max_tracks is not None:
        T = max(T, max_tracks)
    kw = dict(dtype=F64, device=device)
    path = torch.zeros(B, P, 2, **kw)
    path_n = torch.zeros(B, dtype=torch.long, device=device)
    scan = torch.empty(B, C.LIDAR_SAMPLES, **kw)
    tracks = torch.zeros(B, T, 6, **kw)
    tid = torch.zeros(B, T, dtype=torch.long, device=device)
    for b, d in enumerate(dicts):
        pts = torch.tensor(d['path'], **kw).reshape(-1, 2)
        n = pts.shape[0]
        path[b, :n] = pts
        path[b, n:] = pts[-1]
        path_n[b] = n
        scan[b] = torch.tensor([math.nan if r is None else float(r)
                                for r in d['scan_ranges']], **kw)
        for i, t in enumerate(d.get('tracks', [])):
            tracks[b, i] = torch.tensor([t['x'], t['y'], t['vx'], t['vy'],
                                         1.0 if t['is_dynamic'] else 0.0, 1.0], **kw)
            tid[b, i] = int(t['id'])

    def col(key, default=0.0):
        return torch.tensor([float(d.get(key, default)) for d in dicts], **kw)

    prev = torch.zeros(B, N_PREV, 2, **kw)
    for b, d in enumerate(dicts):
        pa = d.get('prev_actions', [[0.0, 0.0]] * N_PREV)[:N_PREV]
        for i, a in enumerate(pa):
            prev[b, i] = torch.tensor([float(a[0]), float(a[1])], **kw)
    sf = [d.get('safety', {}) for d in dicts]
    dv = [d.get('dev', {}) for d in dicts]
    return BatchState(
        pose=torch.tensor([d['pose'] for d in dicts], **kw),
        path_pts=path, path_n=path_n, scan=scan,
        v_meas=col('v_meas'), w_meas=col('w_meas'),
        a_v_meas=col('a_v_meas'), a_w_meas=col('a_w_meas'),
        v_cmd=col('v_cmd'), w_cmd=col('w_cmd'), prev_actions=prev,
        v_cap=col('v_cap', C.V_MAX_OP), scan_age=col('scan_age'), track_age=col('track_age'),
        tracks=tracks, track_id=tid,
        zone=torch.tensor([int(s.get('zone', 0)) for s in sf], dtype=torch.long, device=device),
        speed_limit=torch.tensor([_none_inf(s.get('speed_limit')) for s in sf], **kw),
        proximity_stop=torch.tensor([bool(s.get('proximity_stop', False)) for s in sf],
                                    device=device),
        min_ttc=torch.tensor([_none_inf(s.get('min_ttc')) for s in sf], **kw),
        in_dev=torch.tensor([bool(x.get('in_dev', False)) for x in dv], device=device),
        dev_peak=torch.tensor([float(x.get('peak', 0.0)) for x in dv], **kw),
        dev_t=torch.tensor([float(x.get('t_since_peak', 0.0)) for x in dv], **kw),
    )


# ======================================================================== 기하
def wrap_angle(a):
    """[−π, π) 로 감는다 (geometry.wrap_angle 과 같은 식)."""
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def to_robot_frame(pose, pts):
    """지도 좌표 점 [B, N, 2] → 로봇 좌표 (pose [B, 3])."""
    c = torch.cos(pose[:, 2]).unsqueeze(1)
    s = torch.sin(pose[:, 2]).unsqueeze(1)
    dx = pts[..., 0] - pose[:, 0:1]
    dy = pts[..., 1] - pose[:, 1:2]
    return torch.stack([c * dx + s * dy, -s * dx + c * dy], dim=-1)


def rotate_to_robot(theta, v):
    """지도 좌표 벡터 [B, N, 2] 를 로봇 축으로 회전 (theta [B])."""
    c = torch.cos(theta).unsqueeze(1)
    s = torch.sin(theta).unsqueeze(1)
    return torch.stack([c * v[..., 0] + s * v[..., 1], -s * v[..., 0] + c * v[..., 1]], dim=-1)


def footprint_clearance(pts):
    """로봇 좌표 점 [..., 2] 에서 풋프린트 사각형까지 거리 (안쪽 0)."""
    dx = torch.clamp(pts[..., 0].abs() - C.HALF_L, min=0.0)
    dy = torch.clamp(pts[..., 1].abs() - C.HALF_W, min=0.0)
    return torch.hypot(dx, dy)


def beam_angles(device='cpu', dtype=F64):
    """빔 i 의 각 = angle_min + i·inc (constants 와 같은 식)."""
    return C.LIDAR_ANGLE_MIN + torch.arange(C.LIDAR_SAMPLES, device=device, dtype=dtype) * \
        C.LIDAR_ANGLE_INC


def scan_valid(scan):
    """유효 빔 = 유한 ∧ range_min ≤ r ≤ range_max."""
    return torch.isfinite(scan) & (scan >= C.LIDAR_RANGE_MIN) & (scan <= C.LIDAR_RANGE_MAX)


def scan_points(scan):
    """유효 빔 끝점 (로봇 좌표 [B, 720, 2], 무효는 LiDAR 원점) 과 유효 여부."""
    valid = scan_valid(scan)
    ang = beam_angles(scan.device, scan.dtype)
    rr = torch.where(valid, scan, torch.zeros_like(scan))
    pts = torch.stack([C.LIDAR_X + rr * torch.cos(ang), C.LIDAR_Y + rr * torch.sin(ang)], dim=-1)
    return pts, valid


def vo_time(px, py, wx, wy, radius, tau):
    """VO 진입 시각 — velocityObstacleTime 배치판 (geometry.vo_time 과 같은 분기), 없음 = +inf."""
    pn2 = px * px + py * py
    closing = px * wx + py * wy
    r2 = radius * radius
    inside = pn2 < r2
    wn2 = wx * wx + wy * wy
    disc = closing * closing - wn2 * (pn2 - r2)
    ok = (~inside) & (wn2 >= 1e-12) & (closing > 0.0) & (disc > 0.0)
    safe_wn2 = torch.where(wn2 >= 1e-12, wn2, torch.ones_like(wn2))
    t = (closing - torch.sqrt(torch.clamp(disc, min=0.0))) / safe_wn2
    t_out = torch.where(t <= tau, torch.clamp(t, min=0.0), torch.full_like(t, INF))
    res = torch.where(ok, t_out, torch.full_like(t, INF))
    res = torch.where(inside, torch.where(closing > 0.0, torch.zeros_like(t),
                                          torch.full_like(t, INF)), res)
    return res


# ======================================================================== 경로
def path_cumulative(path_pts, path_n):
    """누적 호길이 [B, P] (패딩 구간 길이 0)."""
    seg = torch.hypot(path_pts[:, 1:, 0] - path_pts[:, :-1, 0],
                      path_pts[:, 1:, 1] - path_pts[:, :-1, 1])
    P = path_pts.shape[1]
    idx = torch.arange(P - 1, device=path_pts.device).unsqueeze(0)
    seg = torch.where(idx < (path_n.unsqueeze(1) - 1), seg, torch.zeros_like(seg))
    return torch.cat([torch.zeros_like(seg[:, :1]), torch.cumsum(seg, dim=1)], dim=1)


def path_length(state):
    """경로 전체 호길이 [B]."""
    cum = state.ensure_cum()
    return cum.gather(1, (state.path_n - 1).clamp(min=0).unsqueeze(1)).squeeze(1)


def project_onto_path(state, p):
    """
    점 p [B, 2] 를 경로에 투영 — 전역 최근접점, 동률(1e-12)이면 작은 호길이 (앞 구간).

    반환 (s, dist, signed_e, seg_yaw). 기준 구현 geometry.project_onto_path 와 같은 규칙.
    """
    pts = state.path_pts
    cum = state.ensure_cum()
    B, P, _ = pts.shape
    a = pts[:, :-1]
    b = pts[:, 1:]
    sx = b[..., 0] - a[..., 0]
    sy = b[..., 1] - a[..., 1]
    ln2 = sx * sx + sy * sy
    idx = torch.arange(P - 1, device=pts.device).unsqueeze(0)
    seg_ok = (idx < (state.path_n.unsqueeze(1) - 1)) & (ln2 >= 1e-18)
    px = p[:, 0:1]
    py = p[:, 1:2]
    safe = torch.where(ln2 >= 1e-18, ln2, torch.ones_like(ln2))
    u = ((px - a[..., 0]) * sx + (py - a[..., 1]) * sy) / safe
    u = u.clamp(0.0, 1.0)
    qx = a[..., 0] + u * sx
    qy = a[..., 1] + u * sy
    d = torch.hypot(px - qx, py - qy)
    d = torch.where(seg_ok, d, torch.full_like(d, INF))
    dmin = d.min(dim=1, keepdim=True).values
    # 순차 규칙(d < best − 1e-12 일 때만 교체)과 같은 결과: dmin + 1e-12 안의 첫 구간
    cand = d <= dmin + 1e-12
    first = torch.argmax(cand.to(torch.int8), dim=1)
    g = first.unsqueeze(1)
    s = cum[:, :-1].gather(1, g).squeeze(1) + u.gather(1, g).squeeze(1) * \
        torch.sqrt(ln2.gather(1, g).squeeze(1))
    dist = d.gather(1, g).squeeze(1)
    gsx = sx.gather(1, g).squeeze(1)
    gsy = sy.gather(1, g).squeeze(1)
    gax = a[..., 0].gather(1, g).squeeze(1)
    gay = a[..., 1].gather(1, g).squeeze(1)
    cross = gsx * (p[:, 1] - gay) - gsy * (p[:, 0] - gax)
    sign = torch.where(cross >= 0.0, torch.ones_like(cross), -torch.ones_like(cross))
    yaw = torch.atan2(gsy, gsx)
    # 퇴화: 점 1 개 또는 모든 구간 길이 0 → 첫 점까지 거리, yaw 0
    degen = ~seg_ok.any(dim=1)
    d0 = torch.hypot(p[:, 0] - pts[:, 0, 0], p[:, 1] - pts[:, 0, 1])
    s = torch.where(degen, torch.zeros_like(s), s)
    dist = torch.where(degen, d0, dist)
    signed = torch.where(degen, d0, sign * dist)
    yaw = torch.where(degen, torch.zeros_like(yaw), yaw)
    return s, dist, signed, yaw


def interpolate_at(state, s):
    """호길이 s [B] (또는 [B, K]) 의 경로 점 [B, (K,) 2] (끝에서 자른다)."""
    pts = state.path_pts
    cum = state.ensure_cum()
    squeeze = s.dim() == 1
    if squeeze:
        s = s.unsqueeze(1)
    B, P, _ = pts.shape
    total = path_length(state).unsqueeze(1)
    n = state.path_n.unsqueeze(1)
    # searchsorted(side='right') − 1 을 유효 점 범위에서
    big = torch.where(torch.arange(P, device=pts.device).unsqueeze(0) < n, cum,
                      torch.full_like(cum, INF))
    i = torch.searchsorted(big.contiguous(), s.contiguous(), right=True) - 1
    i = torch.minimum(torch.clamp(i, min=0), torch.clamp(n - 2, min=0))
    c0 = cum.gather(1, i)
    c1 = cum.gather(1, torch.clamp(i + 1, max=P - 1))
    ln = c1 - c0
    u = torch.where(ln < 1e-18, torch.zeros_like(ln), (s - c0) / torch.where(
        ln < 1e-18, torch.ones_like(ln), ln))
    x0 = pts[..., 0].gather(1, i)
    y0 = pts[..., 1].gather(1, i)
    x1 = pts[..., 0].gather(1, torch.clamp(i + 1, max=P - 1))
    y1 = pts[..., 1].gather(1, torch.clamp(i + 1, max=P - 1))
    x = x0 + u * (x1 - x0)
    y = y0 + u * (y1 - y0)
    first_x = pts[:, 0:1, 0].expand_as(x)
    first_y = pts[:, 0:1, 1].expand_as(y)
    last = pts.gather(1, (state.path_n - 1).clamp(min=0).view(B, 1, 1).expand(B, 1, 2))
    last_x = last[:, :, 0].expand_as(x)
    last_y = last[:, :, 1].expand_as(y)
    lo = s <= 0.0
    hi = s >= total
    x = torch.where(lo, first_x, torch.where(hi, last_x, x))
    y = torch.where(lo, first_y, torch.where(hi, last_y, y))
    out = torch.stack([x, y], dim=-1)
    return out.squeeze(1) if squeeze else out


def goal_point(state):
    """경로 끝점 [B, 2]."""
    B = state.batch
    return state.path_pts.gather(1, (state.path_n - 1).clamp(min=0).view(B, 1, 1)
                                 .expand(B, 1, 2)).squeeze(1)


def remaining_arc(state, p=None):
    """d_arc = 경로 길이 − 투영 호길이 [B]."""
    if p is None:
        p = state.pose[:, :2]
    s = project_onto_path(state, p)[0]
    return torch.clamp(path_length(state) - s, min=0.0)


def goal_distance(p, goal):
    """경로 끝점(목표)까지 직선거리 [B], 지도 좌표 — 기준 구현 reward.goal_distance 의 배치판."""
    return torch.hypot(p[:, 0] - goal[:, 0], p[:, 1] - goal[:, 1])


def progress_distance(d_arc, d_goal, spec=RW.SPEC):
    """
    Φ·진행 효용의 거리 [B] — reward.progress_distance 의 배치판.

    'arc' = d_arc (v1.1.0), 'eff' = max(d_arc, 목표 직선거리) (v1.2.0, contract.md §6).
    """
    if spec.phi_dist == 'eff':
        return torch.maximum(d_arc, d_goal.to(d_arc.dtype))
    if spec.phi_dist != 'arc':
        raise ValueError(spec.phi_dist)
    return d_arc


def state_progress_distance(state, spec=RW.SPEC):
    """상태의 progress_distance (자세 = state.pose)."""
    return progress_distance(remaining_arc(state), goal_distance(state.pose, goal_point(state)),
                             spec)


# ======================================================================== 관측
def lidar_sectors(scan):
    """720 빔 → 180 섹터 풋프린트 여유거리 최솟값 / 10 m (observation.lidar_sectors)."""
    pts, valid = scan_points(scan)
    clr = torch.where(valid, footprint_clearance(pts), torch.full_like(scan, C.SECTOR_CLIP))
    clr = torch.clamp(clr, max=C.SECTOR_CLIP)
    sec = clr.reshape(scan.shape[0], C.N_SECTORS, C.BEAMS_PER_SECTOR).min(dim=2).values
    return sec / C.SECTOR_CLIP


def select_tracks(state, max_range=C.TRACK_RANGE):
    """
    트랙 선별·외삽·로봇 좌표·(거리, id) 정렬 (observation.select_tracks).

    반환: order [B, T] (정렬된 인덱스), keep [B, T] (정렬 순서의 유효), p [B, T, 2], v [B, T, 2],
    dist [B, T], is_dyn [B, T], speed [B, T] — p 이하는 정렬 순서.
    """
    tr = state.tracks
    vx, vy = tr[..., 2], tr[..., 3]
    speed = torch.hypot(vx, vy)
    dyn = tr[..., 4] > 0.5
    valid = tr[..., 5] > 0.5
    age = torch.clamp(state.track_age, min=0.0).unsqueeze(1)
    pw = torch.stack([tr[..., 0] + vx * age, tr[..., 1] + vy * age], dim=-1)
    p = to_robot_frame(state.pose, pw)
    v = rotate_to_robot(state.pose[:, 2], tr[..., 2:4])
    d = torch.hypot(p[..., 0], p[..., 1])
    keep = valid & (dyn | (speed >= C.DYN_FAST_SPEED)) & (d <= max_range)
    # (거리, id) 오름차순: 안정 정렬 두 번 (id 먼저, 그 다음 거리)
    key_id = torch.where(keep, state.track_id.to(F64), torch.full_like(d, INF))
    o1 = torch.argsort(key_id, dim=1, stable=True)
    d1 = torch.where(keep, d, torch.full_like(d, INF)).gather(1, o1)
    o2 = torch.argsort(d1, dim=1, stable=True)
    order = o1.gather(1, o2)

    def g(x):
        if x.dim() == 3:
            return x.gather(1, order.unsqueeze(-1).expand_as(x))
        return x.gather(1, order)
    return order, g(keep), g(p), g(v), g(d), g(dyn), g(speed)


def track_ttc(p, v, v_meas):
    """관측용 TTC (로봇 전방 직진 v_meas, 원 0.361 + 0.30, 지평 5 s)."""
    wx = v_meas.unsqueeze(1) - v[..., 0]
    wy = -v[..., 1]
    return vo_time(p[..., 0], p[..., 1], wx, wy, C.ROBOT_RADIUS + C.OBSTACLE_RADIUS,
                   C.TTC_HORIZON)


def build_observation(state, return_parts=False):
    """관측 [B, 291] float32 — observation.build_observation 과 같은 블록 순서."""
    B = state.batch
    dev = state.pose.device
    parts = {}
    parts['lidar_sectors'] = lidar_sectors(state.scan.to(F64))

    prev = state.prev_actions
    ego = torch.stack([state.v_meas / C.V_MAX_HW, state.w_meas / C.W_MAX,
                       state.a_v_meas / C.A_MAX, state.a_w_meas / C.ALPHA_MAX,
                       state.v_cmd / C.V_MAX_HW, state.w_cmd / C.W_MAX], dim=1)
    pa = torch.stack([prev[..., 0] / C.A_MAX, prev[..., 1] / C.ALPHA_MAX], dim=-1).reshape(B, -1)
    parts['ego'] = torch.clamp(torch.cat([ego, pa], dim=1), -3.0, 3.0)

    parts['timing'] = torch.stack([
        torch.clamp(state.scan_age / C.SCAN_AGE_NORM, 0.0, 1.0),
        torch.clamp(state.track_age / C.TRACK_AGE_NORM, 0.0, 1.0),
        torch.clamp(state.v_cap / C.V_MAX_HW, 0.0, 1.0)], dim=1)

    xy = state.pose[:, :2]
    th = state.pose[:, 2]
    s0, _dist, e_signed, seg_yaw = project_onto_path(state, xy)
    k = torch.arange(1, C.N_SUBGOALS + 1, device=dev, dtype=F64).unsqueeze(0)
    sg = interpolate_at(state, s0.unsqueeze(1) + C.SUBGOAL_SPACING * k)      # [B, 5, 2]
    q = to_robot_frame(state.pose, sg)
    d = torch.hypot(q[..., 0], q[..., 1])
    bearing = torch.where(d > 1e-9, torch.atan2(q[..., 1], q[..., 0]), torch.zeros_like(d))
    blk = torch.stack([torch.clamp(d / C.SUBGOAL_DIST_NORM, 0.0, 2.0), torch.sin(bearing),
                       torch.cos(bearing)], dim=-1).reshape(B, -1)
    e_psi = wrap_angle(th - seg_yaw)
    parts['path'] = torch.cat([blk, torch.stack([
        torch.clamp(e_signed / C.DEV_PEAK_NORM, -2.0, 2.0), torch.sin(e_psi), torch.cos(e_psi)],
        dim=1)], dim=1)

    d_arc = torch.clamp(path_length(state) - s0, min=0.0)
    g = to_robot_frame(state.pose, goal_point(state).unsqueeze(1)).squeeze(1)
    dg = torch.hypot(g[:, 0], g[:, 1])
    bg = torch.where(dg > 1e-9, torch.atan2(g[:, 1], g[:, 0]), torch.zeros_like(dg))
    parts['goal'] = torch.stack([torch.clamp(d_arc, max=C.GOAL_CLIP) / C.GOAL_CLIP,
                                 torch.clamp(dg, max=C.GOAL_CLIP) / C.GOAL_CLIP,
                                 torch.sin(bg), torch.cos(bg)], dim=1)

    zone = torch.nn.functional.one_hot(state.zone.clamp(0, 3), 4).to(F64)
    parts['safety'] = torch.cat([zone, torch.stack([
        torch.clamp(state.speed_limit, max=C.V_MAX_HW) / C.V_MAX_HW,
        state.proximity_stop.to(F64),
        torch.clamp(state.min_ttc, max=C.TTC_HORIZON) / C.TTC_HORIZON], dim=1)], dim=1)

    ind = state.in_dev.to(F64)
    parts['dev'] = torch.stack([
        ind, ind * torch.clamp(state.dev_peak / C.DEV_PEAK_NORM, 0.0, 2.0),
        ind * torch.clamp(state.dev_t / C.DEV_TIMER_NORM, 0.0, 2.0)], dim=1)

    K = C.K_TRACKS
    _o, keep, p, v, dist, dyn, _sp = select_tracks(state)
    T = keep.shape[1]
    if T < K:
        pad = K - T
        keep = torch.cat([keep, torch.zeros(B, pad, dtype=torch.bool, device=dev)], 1)
        p = torch.cat([p, torch.zeros(B, pad, 2, dtype=F64, device=dev)], 1)
        v = torch.cat([v, torch.zeros(B, pad, 2, dtype=F64, device=dev)], 1)
        dist = torch.cat([dist, torch.zeros(B, pad, dtype=F64, device=dev)], 1)
        dyn = torch.cat([dyn, torch.zeros(B, pad, dtype=torch.bool, device=dev)], 1)
    keep, p, v, dist, dyn = keep[:, :K], p[:, :K], v[:, :K], dist[:, :K], dyn[:, :K]
    ttc = track_ttc(p, v, state.v_meas)
    feat = torch.stack([p[..., 0] / C.TRACK_POS_NORM, p[..., 1] / C.TRACK_POS_NORM,
                        v[..., 0] / C.TRACK_VEL_NORM, v[..., 1] / C.TRACK_VEL_NORM,
                        dist / C.TRACK_POS_NORM,
                        torch.clamp(ttc, max=C.TTC_HORIZON) / C.TTC_HORIZON,
                        dyn.to(F64), torch.ones_like(dist)], dim=-1)
    feat = torch.where(keep.unsqueeze(-1), feat, torch.zeros_like(feat))
    parts['tracks'] = feat.reshape(B, -1)

    obs = torch.cat([parts[name] for name, _ in OBS_LAYOUT], dim=1).to(torch.float32)
    assert obs.shape == (B, OBS_DIM), obs.shape
    if return_parts:
        parts['s0'] = s0          # 투영 호길이
        parts['d_arc'] = d_arc    # 남은 호길이 (자르지 않은 값)
        parts['d_goal'] = goal_distance(xy, goal_point(state))   # 목표 직선거리 (지도 좌표)
        return obs, parts
    return obs


def observe_with_cache(state, spec=RW.SPEC):
    """
    관측 [B, 291] 과 전이 전 거리 캐시 d(s) [B] — env 의 build_obs 가 부른다.

    d(s) = progress_distance(d_arc, 목표 직선거리) 이고 다음 전이의 Φ(s)·진행 효용의 d 로 쓴다
    (reward_step 의 d_pre). 리셋 직후·결정마다 관측과 같은 자세로 계산된다.
    """
    obs, parts = build_observation(state, return_parts=True)
    return obs, progress_distance(parts['d_arc'], parts['d_goal'], spec)


# ======================================================================== 보상·비용
@dataclass
class StepOut:
    """전이 하나의 보상·비용·종료 (배치)."""

    reward: torch.Tensor
    terms: dict
    cost: torch.Tensor          # [B, 5]
    in_dev: torch.Tensor
    dev_peak: torch.Tensor
    dev_t: torch.Tensor
    terminated: torch.Tensor
    success: torch.Tensor
    e: torch.Tensor
    phi_pre: torch.Tensor
    phi_post: torch.Tensor
    d_pre: torch.Tensor         # 전이 전 progress_distance
    d_post: torch.Tensor        # 전이 뒤 progress_distance (종단이어도 실제 값)
    d_goal_post: torch.Tensor   # 전이 뒤 목표 직선거리


def dev_update(in_dev, peak, t, e, dt=C.DT):
    """이탈 상태기계 한 스텝 (reward.dev_update 와 같은 순서)."""
    zero = torch.zeros_like(e)
    start = (~in_dev) & (e > C.DEV_OUT)
    new_peak = in_dev & (e > peak)
    n_in = in_dev | start
    n_peak = torch.where(start | new_peak, e, torch.where(in_dev, peak, zero))
    n_t = torch.where(start | new_peak, zero, torch.where(in_dev, t + dt, zero))
    end = in_dev & (e <= C.DEV_BACK)
    n_in = n_in & ~end
    n_peak = torch.where(n_in, n_peak, zero)
    n_t = torch.where(n_in, n_t, zero)
    return n_in, n_peak, n_t


def is_success(pose, goal, v_meas, collision):
    """목표 xy ≤ 0.10 ∧ |v_meas| ≤ 0.05 ∧ 충돌 아님."""
    d = torch.hypot(pose[:, 0] - goal[:, 0], pose[:, 1] - goal[:, 1])
    return (d <= C.GOAL_XY_TOL) & (v_meas.abs() <= C.GOAL_STOP_SPEED) & ~collision


def intervention_cost(pairs, n_valid=None):
    """
    안전 개입 크기 — pairs [B, K, 4] = (v_in, w_in, v_out, w_out) 의 평균.

    n_valid [B] 가 있으면 앞 n_valid 개만 평균 (0 이면 0).
    """
    if pairs is None or pairs.shape[1] == 0:
        return torch.zeros(pairs.shape[0] if pairs is not None else 0, dtype=F64)
    c = ((pairs[..., 0] - pairs[..., 2]) / C.INTERVENTION_V_NORM) ** 2 + \
        ((pairs[..., 1] - pairs[..., 3]) / C.INTERVENTION_W_NORM) ** 2
    if n_valid is None:
        return c.mean(dim=1)
    idx = torch.arange(pairs.shape[1], device=pairs.device).unsqueeze(0)
    m = (idx < n_valid.unsqueeze(1)).to(c.dtype)
    return (c * m).sum(1) / torch.clamp(n_valid.to(c.dtype), min=1.0)


def reward_step(pre, post_pose, post_v_meas, action_exec, collision, pairs=None,
                pairs_n=None, d_pre=None, spec=RW.SPEC):
    """
    전이 (pre, a, post) 의 보상·비용 (reward.step 과 같은 식). post 의 경로 = pre 의 경로.

    pre: BatchState (결정 시점), post_pose [B, 3], post_v_meas [B], action_exec [B, 2],
    collision [B] bool, pairs [B, K, 4] (선택).
    d_pre: 전이 전 progress_distance [B] (env 캐시, observe_with_cache). 주면 pre.pose 를 쓰지 않는다 —
      env 는 전이 뒤 자세가 이미 버퍼에 있어 pre.pose 가 전이 뒤 자세다. None 이면 pre.pose 로 계산.
    spec: reward.RewardSpec (기본 계약 v1.2.0). 정체 종단 후보(stall_terminal)는 지원하지 않는다.
    """
    if spec.stall_terminal:
        raise ValueError('stall_terminal 후보는 Isaac env 에서 지원하지 않는다 (계약 v1.2.0 = truncation)')
    goal = goal_point(pre)
    success = is_success(post_pose, goal, post_v_meas, collision)
    terminated = collision | success
    _s, e, _se, _y = project_onto_path(pre, post_pose[:, :2])
    in_dev, peak, t = dev_update(pre.in_dev, pre.dev_peak, pre.dev_t, e)
    if d_pre is None:
        d_pre = state_progress_distance(pre, spec)
    d_goal_post = goal_distance(post_pose, goal)
    d_post = progress_distance(torch.clamp(path_length(pre) - _s, min=0.0), d_goal_post, spec)
    phi_pre = -spec.kappa * d_pre
    phi_post = torch.where(terminated, torch.zeros_like(d_post), -spec.kappa * d_post)
    a_prev = pre.prev_actions[:, 0]
    dav = (action_exec[:, 0] - a_prev[:, 0]) / C.A_MAX
    daw = (action_exec[:, 1] - a_prev[:, 1]) / C.ALPHA_MAX
    zero = torch.zeros_like(e)
    terms = {
        'time': torch.full_like(e, -spec.time_cost),
        'shape': C.GAMMA * phi_post - phi_pre,
        'smooth': -C.SMOOTH_WEIGHT * (dav * dav + daw * daw),
        'track': torch.where(in_dev, zero,
                             -C.TRACK_WEIGHT * torch.clamp(e / C.CTE_REF, max=C.TRACK_CLIP) ** 2),
        'goal': torch.where(success, torch.full_like(e, spec.goal_reward), zero),
        'collision': torch.where(collision, torch.full_like(e, -spec.collision_penalty), zero),
    }
    if spec.progress_kappa:
        # 진행 효용: 종단 스텝도 실제 d' (Φ 처럼 0 으로 두면 멀리서의 충돌이 κ_b·d 를 돌려받는다)
        terms['progress'] = spec.progress_kappa * (d_pre - d_post)
    if spec.dev_hinge:
        terms['dev'] = torch.where(e > C.DEV_LIMIT, torch.full_like(e, -spec.dev_hinge), zero)
    if pairs is None:
        inter = zero
    else:
        inter = intervention_cost(pairs.to(F64), pairs_n)
    cost = torch.stack([
        collision.to(F64), inter,
        (e > C.DEV_LIMIT).to(F64),
        (in_dev & (t > C.RETURN_LIMIT)).to(F64),
        ((~in_dev) & (e > C.CTE_TRACK_LIMIT)).to(F64)], dim=1)
    reward = sum(terms.values())
    return StepOut(reward, terms, cost, in_dev, peak, t, terminated, success, e,
                   phi_pre, phi_post, d_pre, d_post, d_goal_post)


def time_limit(path_len):
    """에피소드 시간 상한 [s] = min(120, 20 + 3·L)."""
    return torch.clamp(C.TIME_LIMIT_BASE + C.TIME_LIMIT_PER_M * path_len, max=C.TIME_LIMIT_MAX)


# ======================================================================== 행동 적분
def integrate_b(u, v_cmd, w_cmd, v_cap):
    """비교군 B 적분 (kinematics.integrate_b) — 반환 (v', w', a_v_exec, a_w_exec)."""
    u0 = torch.clamp(u[:, 0], -1.0, 1.0)
    u1 = torch.clamp(u[:, 1], -1.0, 1.0)
    a_v = u0 * C.A_MAX
    a_w = u1 * C.ALPHA_MAX
    v_hi = torch.clamp(torch.clamp(v_cap, max=C.V_MAX_HW), min=C.V_MIN)
    v_new = torch.minimum(torch.clamp(v_cmd + a_v * C.DT, min=C.V_MIN), v_hi)
    w_new = torch.clamp(w_cmd + a_w * C.DT, -C.W_MAX, C.W_MAX)
    return v_new, w_new, (v_new - v_cmd) / C.DT, (w_new - w_cmd) / C.DT


# ======================================================================== 프로파일러
class JerkFilterBatch:
    """jerk_limiter.cpp JerkLimitedFilter::step 의 배치판 (profiler.JerkLimitedFilter)."""

    def __init__(self, B, v_min, v_max, a_max, j_max, device='cpu', dtype=F64):
        """한계와 상태 (v, a)."""
        self.v_min, self.v_max, self.a_max, self.j = v_min, v_max, a_max, j_max
        self.v = torch.zeros(B, device=device, dtype=dtype)
        self.a = torch.zeros(B, device=device, dtype=dtype)

    def step(self, v_ref, dt):
        """한 스텝 (j > 0 유한 가정) — 반환: 새 속도."""
        v_ref = torch.clamp(v_ref, self.v_min, self.v_max)
        a_max = max(1e-9, self.a_max)
        j = self.j
        snap = ((v_ref - self.v).abs() <= j * dt * dt) & (self.a.abs() <= j * dt)
        c = v_ref - self.v - 0.5 * self.a * dt
        sgn = torch.where(c >= 0.0, torch.ones_like(c), -torch.ones_like(c))
        a_star = sgn * j * (-0.5 * dt + torch.sqrt(0.25 * dt * dt + 2.0 * c.abs() / j))
        jerk = torch.clamp((a_star - self.a) / dt, -j, j)
        a1 = torch.clamp(self.a + jerk * dt, -a_max, a_max)
        a1 = torch.minimum(torch.maximum(a1, self.a - j * dt), self.a + j * dt)
        v_new = self.v + 0.5 * (self.a + a1) * dt
        self.v = torch.where(snap, v_ref, v_new)
        self.a = torch.where(snap, torch.zeros_like(a1), a1)
        return self.v


class ProfilerBatch:
    """velocity_profiler 기준 경로 배치판 (profiler.ProfilerReference)."""

    def __init__(self, B, device='cpu', dtype=F64):
        """필터 두 개와 곡률 모드 상태."""
        self.lin = JerkFilterBatch(B, -0.5, C.V_MAX_HW, C.A_MAX, C.JERK_MAX, device, dtype)
        self.ang = JerkFilterBatch(B, -C.W_MAX, C.W_MAX, C.ALPHA_MAX, C.ANG_JERK_MAX,
                                   device, dtype)
        self.w_ref = torch.zeros(B, device=device, dtype=dtype)
        self.curv = torch.zeros(B, dtype=torch.bool, device=device)

    def reset(self, ids, v=0.0, a=0.0, w=0.0):
        """지정 env 의 상태 재설정."""
        self.lin.v[ids] = v
        self.lin.a[ids] = a
        self.ang.v[ids] = w
        self.ang.a[ids] = 0.0
        self.w_ref[ids] = w
        self.curv[ids] = False

    def step(self, v_target, w_target, dt=C.PROFILER_DT):
        """한 스텝 — 반환 (v_ref, w_ref)."""
        v_target = torch.clamp(v_target, self.lin.v_min, self.lin.v_max)
        w_target = torch.clamp(w_target, self.ang.v_min, self.ang.v_max)
        v_ref = self.lin.step(v_target, dt).clone()
        curv = v_target.abs() > 0.05
        safe_v = torch.where(curv, v_target, torch.ones_like(v_target))
        kappa = w_target / safe_v
        w_des = torch.clamp(kappa * v_ref, self.ang.v_min, self.ang.v_max)
        dw = self.ang.a_max * dt
        w_c = torch.minimum(torch.maximum(w_des, self.w_ref - dw), self.w_ref + dw)
        # 곡률 모드에서 빠져나오면 각 필터를 현재 w_ref 로 재설정한 뒤 스텝
        leave = (~curv) & self.curv
        self.ang.v = torch.where(leave, self.w_ref, self.ang.v)
        self.ang.a = torch.where(leave, torch.zeros_like(self.ang.a), self.ang.a)
        w_n = self.ang.step(w_target, dt).clone()
        # 곡률 모드 env 는 각 필터 스텝을 버리고 (w_ref, 0) 으로 재설정
        self.ang.v = torch.where(curv, w_c, self.ang.v)
        self.ang.a = torch.where(curv, torch.zeros_like(self.ang.a), self.ang.a)
        self.w_ref = torch.where(curv, w_c, w_n)
        self.curv = curv
        return v_ref, self.w_ref.clone()


# ======================================================================== safety_lite
def clearance_speed_limit(distance, stop_distance=C.E_STOP_DIST):
    """safety_gate.cpp:308-317 — 무한 거리 = +inf."""
    margin = torch.clamp(distance - stop_distance, min=0.0)
    at = C.SAFETY_DECEL * C.REACTION_LATENCY
    u = -at + torch.sqrt(at * at + 2.0 * C.SAFETY_DECEL * torch.where(
        torch.isfinite(margin), margin, torch.zeros_like(margin)))
    return torch.where(torch.isfinite(distance), u, torch.full_like(u, INF))


def max_point_speed(v, w):
    """풋프린트 꼭짓점 최고 속력 (safety_gate.cpp:98-108)."""
    best = torch.zeros_like(v)
    for x in (-C.HALF_L, C.HALF_L):
        for y in (-C.HALF_W, C.HALF_W):
            best = torch.maximum(best, torch.hypot(v - w * y, w * x))
    return best


def classify_zone(distance):
    """존 (히스테리시스 없음). 무한 = CLEAR."""
    z = torch.full(distance.shape, C.ZONE_CLEAR, dtype=torch.long, device=distance.device)
    z = torch.where(distance <= C.WARNING_DIST, torch.full_like(z, C.ZONE_WARNING), z)
    z = torch.where(distance <= C.CRITICAL_DIST, torch.full_like(z, C.ZONE_CRITICAL), z)
    z = torch.where(distance <= C.E_STOP_DIST, torch.full_like(z, C.ZONE_STOP), z)
    return z


def safety_filter(v, w, d_main, min_ttc, proximity_stop):
    """φ(v, w) 배치판 (safety.safety_filter). 반환 dict: v, w, zone, speed_limit, u_cap, v_ttc."""
    raw = classify_zone(d_main)
    zone = torch.where(proximity_stop, torch.full_like(raw, C.ZONE_STOP), raw)
    zone = torch.where((zone == C.ZONE_STOP) & ~proximity_stop,
                       torch.full_like(zone, C.ZONE_CRITICAL), zone)
    u_cap = torch.full_like(v, INF)
    u_cap = torch.where(zone == C.ZONE_WARNING, torch.full_like(v, C.WARNING_MAX_SPEED), u_cap)
    u_cap = torch.where(zone == C.ZONE_CRITICAL, torch.full_like(v, C.CRITICAL_MAX_SPEED), u_cap)
    u_cap = torch.minimum(u_cap, clearance_speed_limit(d_main))
    ttc_on = torch.isfinite(min_ttc) & (min_ttc <= C.TTC_CRITICAL)
    v_ttc = torch.where(ttc_on, torch.clamp(C.SAFETY_DECEL * (min_ttc - C.REACTION_LATENCY),
                                            min=0.0), torch.full_like(v, INF))
    speed_limit = torch.clamp(torch.minimum(u_cap, v_ttc), max=C.V_MAX_HW)
    s = torch.ones_like(v)
    u = max_point_speed(v, w)
    u_safe = torch.where(u > 0, u, torch.ones_like(u))
    s = torch.where(u > u_cap, torch.minimum(s, u_cap / u_safe), s)
    av = v.abs()
    av_safe = torch.where(av > 0, av, torch.ones_like(av))
    s = torch.where(av > v_ttc, torch.minimum(s, v_ttc / av_safe), s)
    vo = torch.where(proximity_stop, torch.zeros_like(v), v * s)
    wo = torch.where(proximity_stop, torch.zeros_like(w), w * s)
    return {'v': vo, 'w': wo, 'zone': zone, 'speed_limit': speed_limit, 'u_cap': u_cap,
            'v_ttc': v_ttc}

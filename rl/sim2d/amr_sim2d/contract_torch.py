"""
계약(rl/contract v1.1.0)의 torch 배치 구현 — 관측·마스크·보상·비용·safety_lite·프로파일러·행동 적분.

각 함수는 numpy 기준 구현(amr_rl_contract/*.py)의 같은 이름 함수를 env 축 N 으로 벡터화한 것이다.
분기는 torch.where 로 옮겼고, 기준 구현의 순차 루프 의미(동률 처리 등)를 그대로 지킨다.
시험: rl/sim2d/tests/test_golden_torch.py (golden 31 사례), test_env.py (env 상태 무작위 대조).

정밀도 규칙 (contract.md §12): 경로 투영·d_arc·shaping 은 float64, 나머지는 입력 dtype.
"""

from dataclasses import dataclass, fields
import math

import torch

from ._contract import C
from amr_rl_contract.reward import RewardSpec, SPEC_V110  # noqa: E402,F401  (_contract 가 경로를 넣는다)

INF = math.inf
_BEAM_ANG = None


# ------------------------------------------------------------------ 상태 컨테이너
@dataclass
class BState:
    """결정 시각의 배치 계약 상태 (state.py State 의 배치판). 모든 텐서의 0 번 축 = env."""

    pose: torch.Tensor          # [N,3] float64 지도 좌표 (x, y, θ)
    path_pts: torch.Tensor      # [N,P,2] float64 (n 이후는 마지막 점 반복)
    path_n: torch.Tensor        # [N] long  유효 점 수
    path_cum: torch.Tensor      # [N,P] float64 누적 호길이 (n 이후 +inf)
    path_len: torch.Tensor      # [N] float64
    ranges: torch.Tensor        # [N,720] (inf = 무반사·무효)
    v_meas: torch.Tensor
    w_meas: torch.Tensor
    a_v_meas: torch.Tensor
    a_w_meas: torch.Tensor
    v_cmd: torch.Tensor
    w_cmd: torch.Tensor
    prev_actions: torch.Tensor  # [N,3,2] 최근 순
    v_cap: torch.Tensor
    scan_age: torch.Tensor
    track_age: torch.Tensor
    trk: torch.Tensor           # [N,T,4] 지도 좌표 x, y, vx, vy
    trk_dyn: torch.Tensor       # [N,T] bool
    trk_valid: torch.Tensor     # [N,T] bool
    trk_id: torch.Tensor        # [N,T] long
    zone: torch.Tensor          # [N] long
    speed_limit: torch.Tensor   # [N] (inf = 없음)
    proximity_stop: torch.Tensor  # [N] bool
    min_ttc: torch.Tensor       # [N] (inf = 없음)
    dev_in: torch.Tensor        # [N] bool
    dev_peak: torch.Tensor
    dev_t: torch.Tensor

    def index(self, idx):
        """부분집합 (env 축 인덱스)."""
        return BState(**{f.name: getattr(self, f.name)[idx] for f in fields(self)})


def path_tensors(paths, device, p_max=None, dtype=torch.float64):
    """경로 목록(각 [n,2]) → (pts [N,P,2], n [N], cum [N,P], length [N])."""
    n = torch.tensor([len(p) for p in paths], dtype=torch.long)
    P = int(p_max or max(2, int(n.max())))
    pts = torch.zeros(len(paths), P, 2, dtype=dtype)
    for i, p in enumerate(paths):
        t = torch.as_tensor(p, dtype=dtype).reshape(-1, 2)
        pts[i, :len(t)] = t
        pts[i, len(t):] = t[-1]
    pts = pts.to(device)
    n = n.to(device)
    cum, length = path_cum(pts, n)
    return pts, n, cum, length


def path_cum(pts, n):
    """누적 호길이 (n 이후 +inf) 와 전체 길이."""
    seg = torch.linalg.norm(pts[:, 1:] - pts[:, :-1], dim=-1)
    cum = torch.cat([torch.zeros_like(seg[:, :1]), torch.cumsum(seg, dim=1)], dim=1)
    idx = torch.arange(pts.shape[1], device=pts.device)
    length = cum.gather(1, (n - 1).clamp_min(0)[:, None])[:, 0]
    cum = torch.where(idx[None, :] < n[:, None], cum, torch.full_like(cum, INF))
    return cum, length


# ------------------------------------------------------------------ 기하
def first_true(m):
    """
    행마다 첫 참 인덱스 [N] (참이 없으면 0).

    argmax 를 쓰지 않는다: torch.compile(inductor) 의 argmax 가 동률 행에서 첫 인덱스가 아닌(심지어 값이 0 인)
    인덱스를 돌려준 사례를 2026-10-06 이 컨테이너에서 확인했다 (경로 투영 s 가 6.4 m → 0). min 축약은 안전하다.
    """
    k = torch.arange(m.shape[1], device=m.device)
    big = torch.full_like(k, m.shape[1])
    i = torch.where(m, k[None, :], big[None, :]).min(dim=1).values
    return torch.where(i >= m.shape[1], torch.zeros_like(i), i)


def wrap_angle(a):
    """[−π, π) 로 감는다 (geometry.wrap_angle)."""
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def to_robot(pose, xy):
    """지도 좌표 점 xy[N,...,2] → 로봇 좌표 (pose [N,3])."""
    shape = [pose.shape[0]] + [1] * (xy.dim() - 2)
    x = pose[:, 0].reshape(shape)
    y = pose[:, 1].reshape(shape)
    c = torch.cos(pose[:, 2]).reshape(shape)
    s = torch.sin(pose[:, 2]).reshape(shape)
    dx = xy[..., 0] - x
    dy = xy[..., 1] - y
    return torch.stack([c * dx + s * dy, -s * dx + c * dy], dim=-1)


def rotate_to_robot(theta, v):
    """지도 좌표 벡터 v[N,...,2] 를 로봇 축으로 회전 (theta [N])."""
    shape = [theta.shape[0]] + [1] * (v.dim() - 2)
    c = torch.cos(theta).reshape(shape)
    s = torch.sin(theta).reshape(shape)
    return torch.stack([c * v[..., 0] + s * v[..., 1], -s * v[..., 0] + c * v[..., 1]], dim=-1)


def footprint_clearance(px, py):
    """로봇 좌표 점 → 풋프린트 사각형까지 거리 (안쪽 0)."""
    dx = (px.abs() - C.HALF_L).clamp_min(0.0)
    dy = (py.abs() - C.HALF_W).clamp_min(0.0)
    return torch.hypot(dx, dy)


def integrate_arc(v, w, t):
    """원점·θ=0 에서 (v, w) 원호를 t 초 적분 (geometry.integrate_arc). 브로드캐스트."""
    th = w * t
    straight = w.abs() < 1e-9
    w_safe = torch.where(straight, torch.ones_like(w), w)
    r = v / w_safe
    x = torch.where(straight, v * t, r * torch.sin(th))
    y = torch.where(straight, torch.zeros_like(th), r * (1.0 - torch.cos(th)))
    return x, y, th


def chord_velocity(v, w, tau):
    """velocity_obstacle.cpp chordVelocity (θ = 0)."""
    half = 0.5 * w * tau
    small = half.abs() <= 1e-9
    h_safe = torch.where(small, torch.ones_like(half), half)
    mag = torch.where(small, v, v * torch.sin(h_safe) / h_safe)
    return mag * torch.cos(half), mag * torch.sin(half)


def vo_time(px, py, wx, wy, radius, tau):
    """velocity_obstacle.cpp velocityObstacleTime — 진입 시각, 없으면 inf. 브로드캐스트."""
    pn2 = px * px + py * py
    closing = px * wx + py * wy
    r2 = radius * radius
    inside = pn2 < r2
    wn2 = wx * wx + wy * wy
    disc = closing * closing - wn2 * (pn2 - r2)
    ok = (wn2 >= 1e-12) & (closing > 0.0) & (disc > 0.0)
    wn2_safe = torch.where(wn2 >= 1e-12, wn2, torch.ones_like(wn2))
    t = (closing - torch.sqrt(disc.clamp_min(0.0))) / wn2_safe
    t_out = torch.where(ok & (t <= tau), t.clamp_min(0.0), torch.full_like(t, INF))
    t_in = torch.where(closing > 0.0, torch.zeros_like(t), torch.full_like(t, INF))
    return torch.where(inside, t_in, t_out)


# ------------------------------------------------------------------ 경로
def project_onto_path(pts, n, cum, p):
    """
    점 p[N,2] 의 경로 투영 — 전역 최근접, 동률(1e-12 안)이면 작은 인덱스.

    반환 (s, dist, signed_e, seg_yaw), 모두 [N]. 기준 구현 geometry.project_onto_path.
    """
    a = pts[:, :-1]
    sxy = pts[:, 1:] - a
    ln2 = (sxy * sxy).sum(-1)
    k = torch.arange(pts.shape[1] - 1, device=pts.device)
    valid = (k[None, :] < (n - 1)[:, None]) & (ln2 >= 1e-18)
    ln2_safe = torch.where(valid, ln2, torch.ones_like(ln2))
    rel = p[:, None, :] - a
    u = ((rel * sxy).sum(-1) / ln2_safe).clamp(0.0, 1.0)
    q = a + u[..., None] * sxy
    d = torch.linalg.norm(p[:, None, :] - q, dim=-1)
    d = torch.where(valid, d, torch.full_like(d, INF))
    dmin = d.min(dim=1).values
    first = d <= (dmin + 1e-12)[:, None]
    i = first_true(first)
    gi = i[:, None]
    s = cum.gather(1, gi)[:, 0].nan_to_num(posinf=0.0) + u.gather(1, gi)[:, 0] * torch.sqrt(
        ln2_safe.gather(1, gi)[:, 0])
    sx = sxy[..., 0].gather(1, gi)[:, 0]
    sy = sxy[..., 1].gather(1, gi)[:, 0]
    ax = a[..., 0].gather(1, gi)[:, 0]
    ay = a[..., 1].gather(1, gi)[:, 0]
    cross = sx * (p[:, 1] - ay) - sy * (p[:, 0] - ax)
    sign = torch.where(cross >= 0.0, torch.ones_like(cross), -torch.ones_like(cross))
    seg_yaw = torch.atan2(sy, sx)
    # 유효 구간이 없는 경로 (점 1 개 또는 길이 0): 첫 점까지 거리, yaw 0
    none = ~valid.any(dim=1)
    d0 = torch.linalg.norm(p - pts[:, 0], dim=-1)
    s = torch.where(none, torch.zeros_like(s), s)
    dist = torch.where(none, d0, dmin)
    e = torch.where(none, d0, sign * dmin)
    seg_yaw = torch.where(none, torch.zeros_like(seg_yaw), seg_yaw)
    return s, dist, e, seg_yaw


def interpolate_at(pts, n, cum, s):
    """호길이 s[N,K] 의 경로 점 [N,K,2] (끝에서 자른다)."""
    N, P, _ = pts.shape
    i = torch.searchsorted(cum.contiguous(), s.contiguous(), right=True) - 1
    i = i.clamp(min=0)
    i = torch.minimum(i, (n - 2).clamp_min(0)[:, None])
    c0 = cum.gather(1, i)
    i1 = (i + 1).clamp(max=P - 1)
    c1 = cum.gather(1, i1)
    ln = c1 - c0
    ln_ok = torch.isfinite(ln) & (ln >= 1e-18)
    u = torch.where(ln_ok, (s - c0) / torch.where(ln_ok, ln, torch.ones_like(ln)),
                    torch.zeros_like(s))
    p0 = pts.gather(1, i[..., None].expand(-1, -1, 2))
    p1 = pts.gather(1, i1[..., None].expand(-1, -1, 2))
    out = p0 + u[..., None] * (p1 - p0)
    first = pts[:, :1, :].expand_as(out)
    last = pts.gather(1, (n - 1).clamp_min(0)[:, None, None].expand(-1, 1, 2)).expand_as(out)
    length = cum.gather(1, (n - 1).clamp_min(0)[:, None])
    out = torch.where((s <= 0.0)[..., None], first, out)
    out = torch.where((s >= length)[..., None], last, out)
    return out


def goal_xy(pts, n):
    """경로 마지막 점 [N,2]."""
    return pts.gather(1, (n - 1).clamp_min(0)[:, None, None].expand(-1, 1, 2))[:, 0]


def remaining_arc(pts, n, cum, length, xy):
    """d_arc = 경로 길이 − 투영 호길이 (≥ 0)."""
    s = project_onto_path(pts, n, cum, xy)[0]
    return (length - s).clamp_min(0.0)


# ------------------------------------------------------------------ LiDAR
_TRIG = {}


def beam_trig(device, dtype):
    """빔 각의 (cos, sin) [720] — 장치·dtype 별로 한 번 계산해 둔다 (컴파일 그래프에서는 상수)."""
    key = (str(device), dtype)
    if key not in _TRIG:
        idx = torch.arange(C.LIDAR_SAMPLES, dtype=torch.float64)
        ang = C.LIDAR_ANGLE_MIN + idx * C.LIDAR_ANGLE_INC
        _TRIG[key] = (torch.cos(ang).to(device, dtype), torch.sin(ang).to(device, dtype))
    return _TRIG[key]


def scan_points(ranges):
    """[N,720] 거리 → 끝점 (x, y) 로봇 좌표 [N,720] 둘과 유효 마스크."""
    valid = torch.isfinite(ranges) & (ranges >= C.LIDAR_RANGE_MIN) & (ranges <= C.LIDAR_RANGE_MAX)
    c, s = beam_trig(ranges.device, ranges.dtype)
    rr = torch.where(valid, ranges, torch.zeros_like(ranges))
    return C.LIDAR_X + rr * c, C.LIDAR_Y + rr * s, valid


def lidar_sectors(ranges):
    """720 빔 → 180 섹터 풋프린트 여유거리 최솟값 / 10 (observation.lidar_sectors)."""
    px, py, valid = scan_points(ranges)
    clr = torch.where(valid, footprint_clearance(px, py), torch.full_like(px, C.SECTOR_CLIP))
    clr = clr.clamp_max(C.SECTOR_CLIP)
    return clr.reshape(-1, C.N_SECTORS, C.BEAMS_PER_SECTOR).min(dim=-1).values / C.SECTOR_CLIP


# ------------------------------------------------------------------ 트랙
def select_tracks(st, max_range):
    """
    observation.select_tracks 의 배치판.

    반환: (order [N,T] 정렬 인덱스, ok [N,T] 정렬 뒤 유효, d, p[...,2], v[...,2], dyn, speed) — 모두 정렬 순.
    """
    trk = st.trk
    speed = torch.hypot(trk[..., 2], trk[..., 3])
    age = st.track_age.clamp_min(0.0)[:, None]
    pxy = trk[..., 0:2] + trk[..., 2:4] * age[..., None]
    p = to_robot(st.pose, pxy)
    v = rotate_to_robot(st.pose[:, 2], trk[..., 2:4])
    d = torch.hypot(p[..., 0], p[..., 1])
    ok = st.trk_valid & (st.trk_dyn | (speed >= C.DYN_FAST_SPEED)) & (d <= max_range)
    key = torch.where(ok, d, torch.full_like(d, INF))
    o1 = torch.argsort(st.trk_id, dim=1, stable=True)
    o2 = torch.argsort(key.gather(1, o1), dim=1, stable=True)
    order = o1.gather(1, o2)

    def g(x):
        if x.dim() == 3:
            return x.gather(1, order[..., None].expand(-1, -1, x.shape[-1]))
        return x.gather(1, order)
    return order, g(ok), g(d), g(p), g(v), g(st.trk_dyn), g(speed)


def track_ttc(px, py, vx, vy, v_meas):
    """관측용 트랙 TTC (observation.track_ttc)."""
    return vo_time(px, py, v_meas - vx, -vy, C.ROBOT_RADIUS + C.OBSTACLE_RADIUS, C.TTC_HORIZON)


# ------------------------------------------------------------------ 관측
OBS_LAYOUT = (
    ('lidar_sectors', C.N_SECTORS), ('ego', 6 + 2 * C.N_PREV_ACTIONS), ('timing', 3),
    ('path', 3 * C.N_SUBGOALS + 3), ('goal', 4), ('safety', 7), ('dev', 3),
    ('tracks', C.K_TRACKS * C.TRACK_FEATS),
)
OBS_DIM = sum(n for _, n in OBS_LAYOUT)


def build_observation(st, proj=None, out_dtype=torch.float32):
    """배치 상태 → 관측 [N,291] (observation.build_observation). proj = 미리 계산한 투영."""
    N = st.pose.shape[0]
    fd = st.pose.dtype
    parts = [lidar_sectors(st.ranges).to(fd)]

    pa = st.prev_actions
    ego = torch.stack([st.v_meas / C.V_MAX_HW, st.w_meas / C.W_MAX, st.a_v_meas / C.A_MAX,
                       st.a_w_meas / C.ALPHA_MAX, st.v_cmd / C.V_MAX_HW, st.w_cmd / C.W_MAX],
                      dim=1)
    prev = torch.stack([pa[..., 0] / C.A_MAX, pa[..., 1] / C.ALPHA_MAX], dim=-1).reshape(N, -1)
    parts.append(torch.cat([ego.to(fd), prev.to(fd)], dim=1).clamp(-3.0, 3.0))

    parts.append(torch.stack([(st.scan_age / C.SCAN_AGE_NORM).clamp(0.0, 1.0),
                              (st.track_age / C.TRACK_AGE_NORM).clamp(0.0, 1.0),
                              (st.v_cap / C.V_MAX_HW).clamp(0.0, 1.0)], dim=1).to(fd))

    if proj is None:
        proj = project_onto_path(st.path_pts, st.path_n, st.path_cum, st.pose[:, :2])
    s0, _dist, e_signed, seg_yaw = proj
    ks = torch.arange(1, C.N_SUBGOALS + 1, device=st.pose.device, dtype=fd) * C.SUBGOAL_SPACING
    sg = interpolate_at(st.path_pts, st.path_n, st.path_cum, s0[:, None] + ks[None, :])
    q = to_robot(st.pose, sg)
    d = torch.hypot(q[..., 0], q[..., 1])
    b = torch.where(d > 1e-9, torch.atan2(q[..., 1], q[..., 0]), torch.zeros_like(d))
    blk = torch.stack([(d / C.SUBGOAL_DIST_NORM).clamp(0.0, 2.0), torch.sin(b), torch.cos(b)],
                      dim=-1).reshape(N, -1)
    e_psi = wrap_angle(st.pose[:, 2] - seg_yaw)
    parts.append(torch.cat([blk, torch.stack([(e_signed / C.DEV_PEAK_NORM).clamp(-2.0, 2.0),
                                              torch.sin(e_psi), torch.cos(e_psi)], dim=1)], dim=1))

    d_arc = (st.path_len - s0).clamp_min(0.0)
    g = to_robot(st.pose, goal_xy(st.path_pts, st.path_n)[:, None, :])[:, 0]
    dg = torch.hypot(g[:, 0], g[:, 1])
    bg = torch.where(dg > 1e-9, torch.atan2(g[:, 1], g[:, 0]), torch.zeros_like(dg))
    parts.append(torch.stack([d_arc.clamp_max(C.GOAL_CLIP) / C.GOAL_CLIP,
                              dg.clamp_max(C.GOAL_CLIP) / C.GOAL_CLIP,
                              torch.sin(bg), torch.cos(bg)], dim=1))

    zone = torch.nn.functional.one_hot(st.zone.long(), 4).to(fd)
    parts.append(torch.cat([zone, torch.stack([
        st.speed_limit.to(fd).clamp_max(C.V_MAX_HW) / C.V_MAX_HW,
        st.proximity_stop.to(fd),
        st.min_ttc.to(fd).clamp_max(C.TTC_HORIZON) / C.TTC_HORIZON], dim=1)], dim=1))

    din = st.dev_in.to(fd)
    parts.append(torch.stack([din, din * (st.dev_peak / C.DEV_PEAK_NORM).clamp(0.0, 2.0),
                              din * (st.dev_t / C.DEV_TIMER_NORM).clamp(0.0, 2.0)], dim=1))

    _o, ok, dd, p, v, dyn, _sp = select_tracks(st, C.TRACK_RANGE)
    K = C.K_TRACKS
    T = ok.shape[1]
    if T < K:
        pad = K - T
        ok = torch.cat([ok, torch.zeros(N, pad, dtype=torch.bool, device=ok.device)], 1)
        dd = torch.cat([dd, torch.zeros(N, pad, dtype=dd.dtype, device=ok.device)], 1)
        p = torch.cat([p, torch.zeros(N, pad, 2, dtype=p.dtype, device=ok.device)], 1)
        v = torch.cat([v, torch.zeros(N, pad, 2, dtype=v.dtype, device=ok.device)], 1)
        dyn = torch.cat([dyn, torch.zeros(N, pad, dtype=torch.bool, device=ok.device)], 1)
    ok, dd, p, v, dyn = ok[:, :K], dd[:, :K], p[:, :K], v[:, :K], dyn[:, :K]
    ttc = track_ttc(p[..., 0], p[..., 1], v[..., 0], v[..., 1], st.v_meas[:, None].to(fd))
    feat = torch.stack([p[..., 0] / C.TRACK_POS_NORM, p[..., 1] / C.TRACK_POS_NORM,
                        v[..., 0] / C.TRACK_VEL_NORM, v[..., 1] / C.TRACK_VEL_NORM,
                        dd / C.TRACK_POS_NORM, ttc.clamp_max(C.TTC_HORIZON) / C.TTC_HORIZON,
                        dyn.to(fd), torch.ones_like(dd)], dim=-1)
    feat = torch.where(ok[..., None], feat, torch.zeros_like(feat))
    parts.append(feat.reshape(N, -1))
    obs = torch.cat(parts, dim=1)
    assert obs.shape[1] == OBS_DIM
    return obs.to(out_dtype)


# ------------------------------------------------------------------ 행동
_LAT_AV = torch.tensor(C.LATTICE_AV, dtype=torch.float64)
_LAT_AW = torch.tensor(C.LATTICE_AW, dtype=torch.float64)


def lattice(device, dtype):
    """셀 0..98 의 (a_v, a_w) [99] 둘."""
    av = _LAT_AV.repeat_interleave(len(C.LATTICE_AW))
    aw = _LAT_AW.repeat(len(C.LATTICE_AV))
    return av.to(device, dtype), aw.to(device, dtype)


def integrate_b(u, v_cmd, w_cmd, v_cap):
    """kinematics.integrate_b — 반환 (v', w', a_v_exec, a_w_exec)."""
    u0 = u[:, 0].to(v_cmd.dtype).clamp(-1.0, 1.0)
    u1 = u[:, 1].to(v_cmd.dtype).clamp(-1.0, 1.0)
    v_hi = torch.clamp(torch.minimum(v_cap, torch.full_like(v_cap, C.V_MAX_HW)), min=C.V_MIN)
    v_new = torch.minimum(torch.clamp(v_cmd + u0 * C.A_MAX * C.DT, min=C.V_MIN), v_hi)
    w_new = (w_cmd + u1 * C.ALPHA_MAX * C.DT).clamp(-C.W_MAX, C.W_MAX)
    return v_new, w_new, (v_new - v_cmd) / C.DT, (w_new - w_cmd) / C.DT


def fallback_command(v_cmd, w_cmd):
    """kinematics.fallback_command."""
    dv = C.A_MAX * C.DT
    v_new = torch.where(v_cmd > 0.0, (v_cmd - dv).clamp_min(0.0), (v_cmd + dv).clamp_max(0.0))
    dw = torch.minimum(w_cmd.abs(), torch.full_like(w_cmd, C.ALPHA_MAX * C.DT))
    w_new = torch.where(w_cmd != 0.0, w_cmd - torch.sign(w_cmd) * dw, torch.zeros_like(w_cmd))
    return v_new, w_new, (v_new - v_cmd) / C.DT, (w_new - w_cmd) / C.DT


def integrate_c(index, v_cmd, w_cmd):
    """kinematics.integrate_c — index [N] long (99 = 폴백)."""
    av, aw = lattice(v_cmd.device, v_cmd.dtype)
    ic = index.clamp(0, C.N_CELLS - 1)
    a_v = av[ic]
    a_w = aw[ic]
    fb = fallback_command(v_cmd, w_cmd)
    is_fb = index == C.FALLBACK_INDEX
    v_new = torch.where(is_fb, fb[0], v_cmd + a_v * C.DT)
    w_new = torch.where(is_fb, fb[1], w_cmd + a_w * C.DT)
    return v_new, w_new, torch.where(is_fb, fb[2], a_v), torch.where(is_fb, fb[3], a_w)


def nearest_cell(a_v, a_w, mask=None, fallback_accel=None):
    """kinematics.nearest_cell 배치판 — 정규화 거리 최소, 동률(1e-12)이면 작은 인덱스."""
    av, aw = lattice(a_v.device, a_v.dtype)
    d = (((av[None, :] - a_v[:, None]) / C.A_MAX) ** 2 +
         ((aw[None, :] - a_w[:, None]) / C.ALPHA_MAX) ** 2)
    if mask is not None:
        fa_v, fa_w = fallback_accel
        dfb = ((fa_v - a_v) / C.A_MAX) ** 2 + ((fa_w - a_w) / C.ALPHA_MAX) ** 2
        d = torch.cat([d, dfb[:, None]], dim=1)
        d = torch.where(mask.bool(), d, torch.full_like(d, INF))
    # 순차 규칙 "d < best − 1e-12 일 때만 교체" 와 같게: 최소 + 1e-12 이하인 첫 인덱스
    dmin = d.min(dim=1).values
    return first_true(d <= (dmin + 1e-12)[:, None])


def bc_label(v_dwa, w_dwa, v_cmd, w_cmd, mask):
    """kinematics.bc_label 배치판 (v1.1.0 규칙)."""
    a_v = (v_dwa - v_cmd) / C.DT
    a_w = (w_dwa - w_cmd) / C.DT
    fb = fallback_command(v_cmd, w_cmd)
    return nearest_cell(a_v, a_w, mask, (fb[2], fb[3]))


# ------------------------------------------------------------------ 마스크 (비교군 C)
def sim_time(v):
    """dwa.cpp simTime."""
    return (v.abs() / C.A_MAX + 0.5).clamp(C.SIM_TIME_MIN, C.SIM_TIME_MAX)


def compute_mask(st, proj=None, max_cand=9, roll_dtype=torch.float32, static=False):
    """
    mask.compute_mask 배치판.

    저크 조건이 셀을 env 당 최대 9 개로 줄이므로(a_v 창 0.4 / 간격 0.2, a_ω 창 1.2 / 간격 0.5), 가행성·VO 는
    그 후보만 계산한다. 후보가 max_cand 보다 많으면(저크 표 바뀜) 예외.
    static=True 면 후보 수를 max_cand 로 고정하고 호스트 동기화(검사·조기 건너뛰기)를 하지 않는다 —
    torch.compile 용. 결과는 같다 (시험 test_golden_torch 가 두 경로를 다 돌린다).
    반환 dict 는 기준 구현과 같은 키 (텐서, env 축 포함).
    """
    N = st.pose.shape[0]
    dev = st.pose.device
    fd = st.v_cmd.dtype
    av, aw = lattice(dev, fd)
    v1 = st.v_cmd[:, None] + av[None, :] * C.DT
    w1 = st.w_cmd[:, None] + aw[None, :] * C.DT
    tol = C.LATTICE_TOL
    v_hi = torch.minimum(st.v_cap, torch.full_like(st.v_cap, C.V_MAX_HW))[:, None]
    speed_ok = (v1 >= C.V_MIN - tol) & (v1 <= v_hi + tol) & (w1.abs() <= C.W_MAX + tol)
    apv = st.prev_actions[:, 0, 0][:, None]
    apw = st.prev_actions[:, 0, 1][:, None]
    jerk_ok = ((av[None, :] - apv).abs() <= C.JERK_STEP_V + tol) & \
        ((aw[None, :] - apw).abs() <= C.JERK_STEP_W + tol)
    cand = speed_ok & jerk_ok
    if static:
        K = max_cand
    else:
        n_cand = int(cand.sum(dim=1).max()) if N > 0 else 0
        if n_cand > max_cand:
            raise RuntimeError(f'후보 셀 {n_cand} > {max_cand}')
        K = max(1, n_cand)
    order = torch.argsort((~cand).to(torch.int8), dim=1, stable=True)[:, :K]
    c_ok = cand.gather(1, order)                         # [N,K]
    cv = v1.gather(1, order)
    cw = w1.gather(1, order)

    # d_goal = max(남은 호길이, 목표 직선거리) (dwa.cpp:449-459)
    if proj is None:
        proj = project_onto_path(st.path_pts, st.path_n, st.path_cum, st.pose[:, :2])
    rem = (st.path_len - proj[0]).clamp_min(0.0)
    gxy = goal_xy(st.path_pts, st.path_n)
    d_goal = torch.maximum(rem, torch.linalg.norm(gxy - st.pose[:, :2], dim=-1))

    px, py, valid = scan_points(st.ranges.to(roll_dtype))
    T = sim_time(cv)
    n_k = torch.ceil(T / C.SIM_DT - 1e-9).clamp_min(1)
    k_max = int(math.ceil(C.SIM_TIME_MAX / C.SIM_DT - 1e-9))
    hit = torch.zeros_like(c_ok)
    cvr = cv.to(roll_dtype)
    cwr = cw.to(roll_dtype)
    big = torch.tensor(1e6, dtype=roll_dtype, device=dev)
    pxv = torch.where(valid, px, big)[:, None, :]      # 무효 점은 멀리 보낸다
    pyv = torch.where(valid, py, big)[:, None, :]
    for k in range(1, k_max + 1):
        t = k * C.SIM_DT
        s_k = cv.abs() * t
        act = c_ok & (k <= n_k) & ~(s_k > (d_goal[:, None] + C.GOAL_OVERSHOOT_MARGIN))
        if not static and not bool(act.any()):
            continue
        x, y, th = integrate_arc(cvr, cwr, t)
        c = torch.cos(th)[..., None]
        s = torch.sin(th)[..., None]
        dx = pxv - x[..., None]
        dy = pyv - y[..., None]
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        inside = ((lx.abs() <= C.HALF_L + C.FP_PAD) & (ly.abs() <= C.HALF_W + C.FP_PAD)).any(-1)
        hit = hit | (act & inside)
    fp_c = c_ok & ~hit

    # VO
    _o, ok, dd, p, v, _dyn, sp = select_tracks(st, C.VO_MAX_RANGE)
    vo_ok_trk = ok & (sp >= C.DYN_SPEED_THRESHOLD)                # [N,T]
    cx, cy = chord_velocity(cv, cw, C.VO_TIME_HORIZON)             # [N,K]
    R = C.ROBOT_RADIUS + C.OBSTACLE_RADIUS + C.VO_MARGIN
    if vo_ok_trk.shape[1] > 0:
        tt = vo_time(p[:, None, :, 0], p[:, None, :, 1], cx[..., None] - v[:, None, :, 0],
                     cy[..., None] - v[:, None, :, 1], R, C.VO_TIME_HORIZON)   # [N,K,T]
        tt = torch.where(vo_ok_trk[:, None, :], tt, torch.full_like(tt, INF))
        vo_c = tt.min(dim=-1).values
    else:
        vo_c = torch.full_like(cv, INF)
    vo_c = torch.where(fp_c, vo_c, torch.full_like(vo_c, INF))

    fp_ok = torch.zeros(N, C.N_CELLS, dtype=torch.bool, device=dev)
    fp_ok.scatter_(1, order, fp_c)
    fp_ok &= cand
    vo_t = torch.full((N, C.N_CELLS), INF, dtype=fd, device=dev)
    vo_t.scatter_(1, order, torch.where(c_ok, vo_c, torch.full_like(vo_c, INF)))
    sel = speed_ok & jerk_ok & fp_ok
    vo_free = ~torch.isfinite(vo_t)
    cells = sel & vo_free
    saturated = ~cells.any(dim=1) & sel.any(dim=1)
    t_max = torch.where(sel, vo_t, torch.full_like(vo_t, -INF)).max(dim=1).values
    sat_cells = sel & (vo_t >= (t_max - C.VO_TIME_TIE)[:, None])
    cells = torch.where(saturated[:, None], sat_cells, cells)
    mask = torch.cat([cells, torch.ones(N, 1, dtype=torch.bool, device=dev)], dim=1)
    return {'mask': mask, 'speed_ok': speed_ok, 'jerk_ok': jerk_ok, 'fp_ok': fp_ok,
            'vo_time': vo_t, 'saturated': saturated,
            'fallback': torch.stack(fallback_command(st.v_cmd, st.w_cmd), dim=1),
            'v_next': v1, 'w_next': w1}


# ------------------------------------------------------------------ 보상·비용·종료
def potential_arc(d_arc, terminal, kappa=C.KAPPA):
    """Φ = −κ·d, 종단 0 (d = progress_distance)."""
    return torch.where(terminal, torch.zeros_like(d_arc), -kappa * d_arc)


def progress_distance(d_arc, goal_dist, spec=SPEC_V110):
    """reward.progress_distance 배치판: 'arc' = d_arc, 'eff' = max(d_arc, 목표 직선거리)."""
    if spec.phi_dist == 'eff':
        return torch.maximum(d_arc, goal_dist.to(d_arc.dtype))
    if spec.phi_dist != 'arc':
        raise ValueError(spec.phi_dist)
    return d_arc


def dev_update(dev_in, peak, t, e, dt=C.DT):
    """reward.dev_update 배치판."""
    start = ~dev_in & (e > C.DEV_OUT)
    new_peak = dev_in & (e > peak)
    in1 = dev_in | start
    peak1 = torch.where(start | new_peak, e, torch.where(dev_in, peak, torch.zeros_like(peak)))
    t_keep = torch.where(dev_in, t + dt, torch.zeros_like(t))
    t1 = torch.where(start | new_peak, torch.zeros_like(t), t_keep)
    end = dev_in & (e <= C.DEV_BACK)
    in1 = in1 & ~end
    peak1 = torch.where(in1, peak1, torch.zeros_like(peak1))
    t1 = torch.where(in1, t1, torch.zeros_like(t1))
    return in1, peak1, t1


def reward_step(d_arc_pre, d_arc_post, e_post, goal_dist_post, v_meas_post, a_prev, a_exec,
                collision, dev_pre, interv, spec=SPEC_V110, goal_dist_pre=None, stalled=None):
    """
    reward.step 배치판.

    d_arc_pre/post: 경로 투영 남은 호길이 (float64), e_post: 경로 최근접 거리, goal_dist_post: 목표 직선거리,
    a_prev = pre.prev_actions[:,0], a_exec: 이번 실행 가속도, dev_pre = (in, peak, t), interv: 개입 비용 [N].
    spec: 보상 설계 (기본 v1.1.0, 파이썬 상수 — 컴파일 그래프에 박힌다). goal_dist_pre: spec.phi_dist == 'eff'
    일 때 필요. stalled: 이 전이 뒤 정체 판정 [N] bool — spec.stall_terminal 일 때만 쓴다.
    반환 dict: reward, terms(dict), cost [N,5], dev(tuple), terminated, success, stall_term.
    """
    success = (~collision & (goal_dist_post <= C.GOAL_XY_TOL) &
               (v_meas_post.abs() <= C.GOAL_STOP_SPEED))
    if spec.stall_terminal:
        stall_term = stalled & ~collision & ~success
    else:
        stall_term = torch.zeros_like(success)
    terminated = collision | success | stall_term
    dev = dev_update(*dev_pre, e_post)
    if spec.phi_dist == 'eff':
        d_pre = progress_distance(d_arc_pre, goal_dist_pre, spec)
    else:
        d_pre = d_arc_pre
    d_post = progress_distance(d_arc_post, goal_dist_post, spec)
    phi_pre = -spec.kappa * d_pre
    phi_post = potential_arc(d_post, terminated, spec.kappa)
    dav = (a_exec[:, 0] - a_prev[:, 0]) / C.A_MAX
    daw = (a_exec[:, 1] - a_prev[:, 1]) / C.ALPHA_MAX
    fd = d_arc_pre.dtype
    zero = torch.zeros_like(d_arc_pre)
    terms = {
        'time': torch.full_like(d_arc_pre, -spec.time_cost),
        'shape': C.GAMMA * phi_post - phi_pre,
        'smooth': (-C.SMOOTH_WEIGHT * (dav * dav + daw * daw)).to(fd),
        'track': torch.where(dev[0], torch.zeros_like(e_post),
                             -C.TRACK_WEIGHT * (e_post / C.CTE_REF).clamp_max(C.TRACK_CLIP) ** 2),
        'goal': torch.where(success, torch.full_like(d_arc_pre, spec.goal_reward), zero),
        'collision': torch.where(collision, torch.full_like(d_arc_pre, -spec.collision_penalty),
                                 zero),
    }
    if spec.progress_kappa:
        terms['progress'] = spec.progress_kappa * (d_pre - d_post)
    if spec.dev_hinge:
        terms['dev'] = torch.where(e_post > C.DEV_LIMIT, torch.full_like(zero, -spec.dev_hinge),
                                   zero)
    if spec.stall_terminal:
        terms['stall'] = torch.where(stall_term, torch.full_like(zero, -spec.stall_penalty), zero)
    cost = torch.stack([
        collision.to(fd), interv.to(fd), (e_post > C.DEV_LIMIT).to(fd),
        (dev[0] & (dev[2] > C.RETURN_LIMIT)).to(fd),
        (~dev[0] & (e_post > C.CTE_TRACK_LIMIT)).to(fd)], dim=1)
    reward = sum(terms.values())
    return {'reward': reward, 'terms': terms, 'cost': cost, 'dev': dev,
            'terminated': terminated, 'success': success, 'stall_term': stall_term,
            'phi_pre': phi_pre, 'phi_post': phi_post}


def intervention_cost(v_in, w_in, v_out, w_out, count=None):
    """
    reward.intervention_cost 배치판 — [N,S] 틱들의 평균 ((Δv/V_OP)² + (Δw/W_MAX)²).

    count: env 별 유효 틱 수 [N] (None = S). 유효하지 않은 틱은 0 을 넣어 둔다.
    """
    d = (((v_in - v_out) / C.INTERVENTION_V_NORM) ** 2 +
         ((w_in - w_out) / C.INTERVENTION_W_NORM) ** 2)
    tot = d.sum(dim=1)
    if count is None:
        return tot / d.shape[1]
    return torch.where(count > 0, tot / count.clamp_min(1).to(tot.dtype), torch.zeros_like(tot))


def time_limit(path_len):
    """reward.time_limit."""
    return torch.clamp(C.TIME_LIMIT_BASE + C.TIME_LIMIT_PER_M * path_len, max=C.TIME_LIMIT_MAX)


# ------------------------------------------------------------------ safety_lite
def clearance_speed_limit(distance):
    """여유거리 속도 상한 — safety 모듈의 같은 이름 함수 (거리 inf 이면 상한 inf)."""
    margin = (distance - C.E_STOP_DIST).clamp_min(0.0)
    at = C.SAFETY_DECEL * C.REACTION_LATENCY
    lim = -at + torch.sqrt(at * at + 2.0 * C.SAFETY_DECEL * margin)
    return torch.where(torch.isfinite(distance), lim, torch.full_like(distance, INF))


def max_point_speed(v, w):
    """safety.max_point_speed — 꼭짓점 최고 속력."""
    best = torch.zeros_like(v)
    for x in (-C.HALF_L, C.HALF_L):
        for y in (-C.HALF_W, C.HALF_W):
            best = torch.maximum(best, torch.hypot(v - w * y, w * x))
    return best


def classify_zone(distance):
    """safety.classify_zone."""
    z = torch.full(distance.shape, C.ZONE_CLEAR, dtype=torch.long, device=distance.device)
    z = torch.where(distance <= C.WARNING_DIST, torch.full_like(z, C.ZONE_WARNING), z)
    z = torch.where(distance <= C.CRITICAL_DIST, torch.full_like(z, C.ZONE_CRITICAL), z)
    z = torch.where(distance <= C.E_STOP_DIST, torch.full_like(z, C.ZONE_STOP), z)
    return torch.where(torch.isfinite(distance), z, torch.full_like(z, C.ZONE_CLEAR))


def safety_filter(v, w, d_main, min_ttc, proximity_stop):
    """safety.safety_filter 배치판. d_main·min_ttc 의 inf = 없음."""
    zone = torch.where(proximity_stop, torch.full_like(d_main, C.ZONE_STOP, dtype=torch.long),
                       classify_zone(d_main))
    zone = torch.where((zone == C.ZONE_STOP) & ~proximity_stop,
                       torch.full_like(zone, C.ZONE_CRITICAL), zone)
    u_cap = torch.full_like(v, INF)
    u_cap = torch.where(zone == C.ZONE_WARNING, torch.full_like(v, C.WARNING_MAX_SPEED), u_cap)
    u_cap = torch.where(zone == C.ZONE_CRITICAL, torch.full_like(v, C.CRITICAL_MAX_SPEED), u_cap)
    u_cap = torch.minimum(u_cap, clearance_speed_limit(d_main.to(v.dtype)))
    ttc_on = torch.isfinite(min_ttc) & (min_ttc <= C.TTC_CRITICAL)
    v_ttc = torch.where(ttc_on, (C.SAFETY_DECEL * (min_ttc - C.REACTION_LATENCY)).clamp_min(0.0),
                        torch.full_like(min_ttc, INF)).to(v.dtype)
    speed_limit = torch.minimum(torch.minimum(u_cap, v_ttc), torch.full_like(v, C.V_MAX_HW))
    s = torch.ones_like(v)
    u = max_point_speed(v, w)
    u_safe = torch.where(u > 0.0, u, torch.ones_like(u))
    s = torch.where(u > u_cap, torch.minimum(s, u_cap / u_safe), s)
    av = v.abs()
    av_safe = torch.where(av > 0.0, av, torch.ones_like(av))
    s = torch.where(av > v_ttc, torch.minimum(s, v_ttc / av_safe), s)
    v_out = torch.where(proximity_stop, torch.zeros_like(v), v * s)
    w_out = torch.where(proximity_stop, torch.zeros_like(w), w * s)
    return {'v': v_out, 'w': w_out, 'zone': zone, 'speed_limit': speed_limit,
            'u_cap': u_cap, 'v_ttc': v_ttc}


# ------------------------------------------------------------------ 프로파일러 기준 모델
def jlf_step(v, a, v_ref, dt, v_min, v_max, a_max, j):
    """jerk_limiter.cpp JerkLimitedFilter::step 배치판 (j > 0 유한)."""
    v_ref = v_ref.clamp(v_min, v_max)
    snap = ((v_ref - v).abs() <= j * dt * dt) & (a.abs() <= j * dt)
    c = v_ref - v - 0.5 * a * dt
    sgn = torch.where(c >= 0.0, torch.ones_like(c), -torch.ones_like(c))
    a_star = sgn * j * (-0.5 * dt + torch.sqrt(0.25 * dt * dt + 2.0 * c.abs() / j))
    jerk = ((a_star - a) / dt).clamp(-j, j)
    a1 = (a + jerk * dt).clamp(-max(1e-9, a_max), max(1e-9, a_max))
    a1 = torch.maximum(torch.minimum(a1, a + j * dt), a - j * dt)
    v1 = v + 0.5 * (a + a1) * dt
    return torch.where(snap, v_ref, v1), torch.where(snap, torch.zeros_like(a1), a1)


class ProfilerBatch:
    """profiler.ProfilerReference 배치판 (velocity_profiler.cpp:150-178)."""

    def __init__(self, n, device, dtype=torch.float64):
        """상태 0."""
        z = torch.zeros(n, device=device, dtype=dtype)
        self.lin_v, self.lin_a = z.clone(), z.clone()
        self.ang_v, self.ang_a = z.clone(), z.clone()
        self.w_ref = z.clone()
        self.curv_mode = torch.zeros(n, dtype=torch.bool, device=device)

    def reset(self, idx):
        """지정한 env 의 상태를 0 으로."""
        for name in ('lin_v', 'lin_a', 'ang_v', 'ang_a', 'w_ref'):
            getattr(self, name)[idx] = 0.0
        self.curv_mode[idx] = False

    def step(self, v_target, w_target, dt=C.PROFILER_DT):
        """한 스텝 → (v_ref, w_ref)."""
        v_t = v_target.clamp(-0.5, C.V_MAX_HW)
        w_t = w_target.clamp(-C.W_MAX, C.W_MAX)
        self.lin_v, self.lin_a = jlf_step(self.lin_v, self.lin_a, v_t, dt, -0.5, C.V_MAX_HW,
                                          C.A_MAX, C.JERK_MAX)
        v_ref = self.lin_v
        curv = v_t.abs() > 0.05
        v_t_safe = torch.where(curv, v_t, torch.ones_like(v_t))
        w_des = ((w_t / v_t_safe) * v_ref).clamp(-C.W_MAX, C.W_MAX)
        dw = C.ALPHA_MAX * dt
        w_curv = torch.minimum(torch.maximum(w_des, self.w_ref - dw), self.w_ref + dw)
        ang_v0 = torch.where(self.curv_mode, self.w_ref, self.ang_v)
        ang_a0 = torch.where(self.curv_mode, torch.zeros_like(self.ang_a), self.ang_a)
        w_nc, a_nc = jlf_step(ang_v0, ang_a0, w_t, dt, -C.W_MAX, C.W_MAX, C.ALPHA_MAX,
                              C.ANG_JERK_MAX)
        self.w_ref = torch.where(curv, w_curv, w_nc)
        self.ang_v = self.w_ref.clone()
        self.ang_a = torch.where(curv, torch.zeros_like(a_nc), a_nc)
        self.curv_mode = curv
        return v_ref, self.w_ref

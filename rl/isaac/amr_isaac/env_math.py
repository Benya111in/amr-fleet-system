"""
Isaac env 의 순수 기하·운동 함수 — Isaac Lab 비의존 (torch + 계약만). env.py 가 import 한다.

CPU pytest(amr-ai/rl:0.3)가 이 모듈을 바로 불러 시험한다 (tests/test_env_math.py):
  - footprint_hits: 창고 정적 사각형 129 개 경계 근처 자세에서 numpy 기준판
    warehouse_geom.footprint_hits_obb 와 정확히 같은지
  - integrate_pose: 계약 geometry.integrate_arc(원점 기준 정확 적분)를 자세로 옮긴 값·미세 Euler 와 대조
  - approach_distance: 앞·옆·안·뒤·후진·제자리 회전·정지·무효 빔 사례
  - move_and_collide: 접촉한 env 는 그 자리에 멈추고(자세 고정·속도 0) 접촉 래치가 유지되는지
  - select_paths_by_length: 경로 풀에서 길이 범위만 남기기 (경로 점·오프셋 재구성)
  - reward_spec_for: env 설정의 보상 판 이름 → 기준 구현 RewardSpec
  - EpisodeLog: 에피소드 진단 누적·끝난 에피소드 사본 (편향 없는 첫 에피소드 평가가 읽는다)
"""

import math

import numpy as np
import torch

from . import contract_torch as CT
from . import safety_gate_torch as SG
from .contract_torch import C, F64, RW

# 진단: |ω| > 1.0 rad/s 이고 |v| < 0.1 m/s 인 결정 스텝 = 제자리 회전 (rl/sim2d env.py SPIN_W·SPIN_V 와 같다)
SPIN_W, SPIN_V = 1.0, 0.1
OUTCOME_CODES = ('success', 'collision', 'time_limit', 'stall')


def footprint_hits(pose, obb, half_l=C.HALF_L, half_w=C.HALF_W):
    """
    풋프린트 사각형 [N] 이 정적 사각형 [M] 중 하나와 겹치는가 (분리축 정리, 경계 포함).

    pose [N, 3] (x, y, θ), obb [M, 5] (cx, cy, hx, hy, yaw).
    warehouse_geom.footprint_hits_obb 의 torch 판.
    """
    th = pose[:, 2:3]
    a0 = torch.stack([torch.cos(th), torch.sin(th)], -1)              # [N, 1, 2]
    a1 = torch.stack([-torch.sin(th), torch.cos(th)], -1)
    yaw = obb[:, 4].unsqueeze(0)
    b0 = torch.stack([torch.cos(yaw), torch.sin(yaw)], -1)            # [1, M, 2]
    b1 = torch.stack([-torch.sin(yaw), torch.cos(yaw)], -1)
    d = torch.stack([obb[:, 0].unsqueeze(0) - pose[:, 0:1],
                     obb[:, 1].unsqueeze(0) - pose[:, 1:2]], -1)      # [N, M, 2]
    hx = obb[:, 2].unsqueeze(0)
    hy = obb[:, 3].unsqueeze(0)
    sep = torch.zeros(d.shape[:2], dtype=torch.bool, device=d.device)
    for ax in (a0.expand_as(d), a1.expand_as(d), b0.expand_as(d), b1.expand_as(d)):
        ra = half_l * (ax * a0).sum(-1).abs() + half_w * (ax * a1).sum(-1).abs()
        rb = hx * (ax * b0).sum(-1).abs() + hy * (ax * b1).sum(-1).abs()
        sep |= (ax * d).sum(-1).abs() > ra + rb
    return (~sep).any(dim=1)


def integrate_pose(pose, v, w, dt):
    """정확 원호 적분 (dwa integrateArc 와 같은 식, θ 는 감는다)."""
    x, y, th = pose[:, 0], pose[:, 1], pose[:, 2]
    small = w.abs() < 1e-9
    w_safe = torch.where(small, torch.ones_like(w), w)
    th1 = th + w * dt
    xs = x + v * dt * torch.cos(th)
    ys = y + v * dt * torch.sin(th)
    xa = x + (v / w_safe) * (torch.sin(th1) - torch.sin(th))
    ya = y - (v / w_safe) * (torch.cos(th1) - torch.cos(th))
    return torch.stack([torch.where(small, xs, xa), torch.where(small, ys, ya),
                        CT.wrap_angle(th1)], dim=1)


def approach_distance(pts_r, valid, v, w):
    """
    safety_lite 에 넣는 접근 거리 D [N] — safety_gate 스윕 풋프린트(SweptFootprint)의 근사.

    로봇 좌표 스캔 끝점 pts_r [N, K, 2]. 풋프린트 안 점 = 0. 직진(|v| ≥ 1e-3): 진행 방향 앞, 차폭
    띠(|y| ≤ 0.20) 안 점까지의 세로 거리. 제자리 회전(|v| < 1e-3, |ω| ≥ 1e-3): 외접원 밖 반지름 거리.
    정지: +inf. 원호(v·ω ≠ 0)도 직진 띠로 근사한다 — 정본(원호 접촉각·꼬리 직선·가설 여러 개·강건 통계)과
    다르다 (README 「계약과의 차이」).
    """
    px, py = pts_r[..., 0], pts_r[..., 1]
    inf = torch.full_like(px, math.inf)
    inside = valid & (px.abs() <= C.HALF_L) & (py.abs() <= C.HALF_W)
    band = valid & (py.abs() <= C.HALF_W)
    fwd = (v >= 0).unsqueeze(1)
    ahead = torch.where(fwd, px - C.HALF_L, -px - C.HALF_L)
    d_lin = torch.where(band & (ahead > 0), ahead, inf).min(dim=1).values
    rad = torch.hypot(px, py) - C.ROBOT_RADIUS
    d_rot = torch.where(valid, rad.clamp(min=0.0), inf).min(dim=1).values
    moving = v.abs() >= 1e-3
    rotating = (~moving) & (w.abs() >= 1e-3)
    d = torch.where(moving, d_lin, torch.where(rotating, d_rot, torch.full_like(d_lin, math.inf)))
    return torch.where(inside.any(dim=1), torch.zeros_like(d), d)


def move_and_collide(pose, v, w, collided, obb, dt):
    """
    물리 서브스텝 하나의 이동과 접촉 판정 (접촉 = 그 자리 정지, 결정 주기 안 래치).

    이미 접촉한 env(collided) 는 움직이지 않는다. 나머지는 (v, w) 로 원호 적분한 뒤 풋프린트 대 정적
    사각형 판정을 하고, 이번에 닿으면 그 적분 뒤 자세에 멈춘다. 접촉한 env 의 속도는 0.
    반환 (pose, v, w, collided) — 입력 텐서는 바꾸지 않는다.
    """
    moving = ~collided
    new_pose = integrate_pose(pose, v, w, dt)
    pose = torch.where(moving.unsqueeze(1), new_pose, pose)
    collided = collided | (footprint_hits(pose, obb) & moving)
    zero = torch.zeros_like(v)
    return pose, torch.where(collided, zero, v), torch.where(collided, zero, w), collided


def select_paths_by_length(pts, offs, length, lo=None, hi=None):
    """
    경로 풀에서 길이 lo ≤ L ≤ hi 인 경로만 남긴다 → (pts, offs, length, kept).

    pts [Σn, 2], offs [K+1] (경로 k = pts[offs[k]:offs[k+1]]), length [K] (numpy). None = 그 쪽 제한 없음.
    kept = 남긴 경로의 원래 번호 (오름차순). 경로 안 점 순서와 경로 순서는 그대로다.
    """
    pts = np.asarray(pts)
    offs = np.asarray(offs)
    length = np.asarray(length)
    keep = np.ones(length.shape[0], dtype=bool)
    if lo is not None:
        keep &= length >= lo
    if hi is not None:
        keep &= length <= hi
    kept = np.flatnonzero(keep)
    if kept.size == 0:
        raise ValueError(f'길이 [{lo}, {hi}] 인 경로가 풀에 없다')
    n = offs[1:] - offs[:-1]
    owner = np.repeat(np.arange(length.shape[0]), n)
    new_offs = np.concatenate([[0], np.cumsum(n[kept])]).astype(offs.dtype)
    return pts[keep[owner]], new_offs, length[kept], kept


def reward_spec_for(version):
    """보상 판 이름 → reward.RewardSpec ('1.2.0' = 계약 기본 SPEC, '1.1.0' = 대조 SPEC_V110)."""
    specs = {'1.2.0': RW.SPEC, '1.1.0': RW.SPEC_V110}
    if version not in specs:
        raise ValueError(f'보상 판 {version!r} 없음 (가능: {sorted(specs)})')
    spec = specs[version]
    if version == C.CONTRACT_VERSION and spec != RW.SPEC:
        raise ValueError('계약 판과 기본 RewardSpec 이 어긋난다')
    return spec


class EpisodeLog:
    """
    에피소드 진단 누적과 끝난 에피소드 사본 (env 의 _get_dones 가 매 결정 스텝 부른다).

    누적: 이동 거리 Σ|v|·Δt, 제자리 회전 스텝, 최대 이탈 e, 목표 최소 직선거리.
    끝난 env 는 리셋 전에 fin_* 로 사본을 남기고 fin_count 를 올린다 — 평가가 env 마다 첫 에피소드만 읽는다
    (먼저 끝난 에피소드부터 세면 짧은 결말 쪽으로 치우친다, rl/sim2d ppo.evaluate 와 같은 추정량).
    """

    def __init__(self, n, n_costs, device):
        """버퍼 [n]."""
        self.n = n
        z = lambda dt=F64: torch.zeros(n, dtype=dt, device=device)  # noqa: E731
        self.travel, self.maxe, self.spin = z(), z(), z(torch.long)
        self.gmin = torch.full((n,), math.inf, dtype=F64, device=device)
        self.fin_count = z(torch.long)
        self.fin = {k: z() for k in ('return', 'path_len', 'd_start', 'd_final', 'travel',
                                     'maxe', 'gmin', 'time_s')}
        self.fin['steps'] = z(torch.long)
        self.fin['spin'] = z(torch.long)
        self.fin['outcome'] = z(torch.long)
        self.fin['path_id'] = z(torch.long)
        self.fin['cost'] = torch.zeros(n, n_costs, dtype=F64, device=device)

    def accumulate(self, v, w, e, d_goal):
        """결정 스텝 하나 (전이 뒤 v·ω·e·목표 직선거리)."""
        self.travel += v.abs() * C.DT
        self.spin += ((w.abs() > SPIN_W) & (v.abs() < SPIN_V)).long()
        self.maxe = torch.maximum(self.maxe, e)
        self.gmin = torch.minimum(self.gmin, d_goal)

    def finish(self, done, outcome, ep):
        """
        끝난 env(done [n] bool)의 사본을 남긴다.

        outcome [n, 4] bool (success, collision, time_limit, stall — 끝난 env 에서 정확히 하나),
        ep = dict(return, path_len, d_start, d_final, steps, cost, path_id) [n] 텐서.
        """
        code = torch.argmax(outcome.to(torch.int8), dim=1)
        vals = dict(ep)
        vals.update({'outcome': code, 'travel': self.travel, 'maxe': self.maxe,
                     'gmin': self.gmin, 'spin': self.spin,
                     'time_s': ep['steps'].to(F64) * C.DT})
        for k, buf in self.fin.items():
            v = vals[k].to(buf.dtype)
            m = done.view(-1, *([1] * (buf.dim() - 1)))
            buf.copy_(torch.where(m, v, buf))
        self.fin_count += done.long()

    def reset(self, ids):
        """리셋한 env 의 누적을 비운다."""
        self.travel[ids] = 0.0
        self.maxe[ids] = 0.0
        self.spin[ids] = 0
        self.gmin[ids] = math.inf


SAFETY_FILTERS = ('lite', 'canonical')
CLOUD_MODES = ('empty', 'none')


class CanonicalPhi:
    """
    정본 safety φ 의 env 결선 — `safety_gate_torch.SafetyGateBatch` 를 safety_node 처럼 부른다.

    시간: 결정 스텝(0.1 s) 마다 새 스캔(촬영 시각 = 그 결정의 시작), 50 Hz 서브스텝마다
    측정 속도(플랜트 속도 = wheel_odom) → 명령(프로파일러 출력 = cmd_vel_smoothed) → evaluate.
    LiDAR: 720 빔 원형, 원점 base_link (LIDAR_X, LIDAR_Y), 무효(비유한·범위 밖) 빔 = NaN
    (safety_node.cpp:412-428).
    깊이 점군 (Isaac 에 깊이 카메라가 없다):
      'empty' — 스캔마다 "신선하고 비어 있는" 점군 (카메라 정상, LiDAR 가 못 보는 낮은 물체 없음 — 창고 상자 129 개가
                모두 LiDAR 평면을 지난다). 배포 설정 cloud_required 라 점군이 아예 없으면 전진이 0.2 m/s 로 묶인다.
      'none'  — 점군을 넣지 않는다 (= 깊이 카메라 고장과 같다: 전진 0.2 m/s 상한, 후퇴 특례 없음). 진단용.
    트랙(TTC)·도킹 예외: 정적 지도라 넣지 않는다 (+inf, 없음).
    """

    def __init__(self, n, device, params=SG.DEPLOYED, cloud_mode='empty'):
        """게이트 배치와 빔 각."""
        if cloud_mode not in CLOUD_MODES:
            raise ValueError(f'cloud_mode {cloud_mode!r} (가능: {CLOUD_MODES})')
        self.cloud_mode = cloud_mode
        cap = SG.max_half_for_range(0.99 * C.LIDAR_RANGE_MIN, C.LIDAR_ANGLE_INC, params)
        self.gate = SG.SafetyGateBatch(
            n, device, params, n_beams=C.LIDAR_SAMPLES, angle_increment=C.LIDAR_ANGLE_INC,
            circular=True, origin=(C.LIDAR_X, C.LIDAR_Y), half_cap=cap)
        self.beam_ang = CT.beam_angles(device, F64)
        self.cos_a = torch.cos(self.beam_ang).unsqueeze(0)
        self.sin_a = torch.sin(self.beam_ang).unsqueeze(0)

    def reset(self, ids):
        """리셋한 env 의 게이트를 새로 (생성 직후 상태)."""
        self.gate.reset(ids)

    def beams(self, ranges):
        """스캔 거리 [N, 720] → base_link 빔 끝점 [N, 720, 2] (무효 = NaN)."""
        valid = CT.scan_valid(ranges)
        rr = torch.where(valid, ranges, torch.zeros_like(ranges))
        bx = torch.where(valid, C.LIDAR_X + rr * self.cos_a, torch.full_like(rr, math.nan))
        by = torch.where(valid, C.LIDAR_Y + rr * self.sin_a, torch.full_like(rr, math.nan))
        return torch.stack([bx, by], dim=-1)

    def set_scan(self, ranges, stamp):
        """새 스캔 (모든 env 같은 시각) — 'empty' 면 같은 시각의 빈 점군도."""
        self.gate.set_scan(self.beams(ranges.to(F64)), stamp)
        if self.cloud_mode == 'empty':
            self.gate.set_cloud(None, stamp, stamp)

    def step(self, v_ref, w_ref, v_meas, w_meas, now):
        """서브스텝 하나: 측정 → 명령 → evaluate (반환 = SafetyGateBatch.evaluate 의 dict)."""
        self.gate.set_measured(v_meas, w_meas, now)
        self.gate.set_command(v_ref, w_ref, now)
        return self.gate.evaluate(now)

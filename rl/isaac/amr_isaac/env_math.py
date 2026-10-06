"""
Isaac env 의 순수 기하·운동 함수 — Isaac Lab 비의존 (torch + 계약만). env.py 가 import 한다.

CPU pytest(amr-ai/rl:0.3)가 이 모듈을 바로 불러 시험한다 (tests/test_env_math.py):
  - footprint_hits: 창고 정적 사각형 129 개 경계 근처 자세에서 numpy 기준판
    warehouse_geom.footprint_hits_obb 와 정확히 같은지
  - integrate_pose: 계약 geometry.integrate_arc(원점 기준 정확 적분)를 자세로 옮긴 값·미세 Euler 와 대조
  - approach_distance: 앞·옆·안·뒤·후진·제자리 회전·정지·무효 빔 사례
  - move_and_collide: 접촉한 env 는 그 자리에 멈추고(자세 고정·속도 0) 접촉 래치가 유지되는지
"""

import math

import torch

from . import contract_torch as CT
from .contract_torch import C


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

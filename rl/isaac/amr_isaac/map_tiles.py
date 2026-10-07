"""
타일 지도의 env 별 충돌 판정 (b1-mapgen) — Isaac Lab 비의존 (torch + 계약).

지도 변형 평가는 변형 지도 여러 장을 세계 좌표의 서로 다른 자리(타일)에 함께 놓는다 (RayCaster 는 메시 하나만
받는다 — env_mapgen.py). 그러면 env 마다 자기 타일의 사각형만 봐야 하므로 env_math.footprint_hits 의 "모든 env 가
같은 사각형 [M, 5]" 대신 env 별 사각형 [N, M, 5] 를 받는 판을 둔다. 식·연산 순서는 env_math.footprint_hits 와
같다 — tests/test_map_tiles.py 가 같은 사각형이면 판정이 비트 단위로 같음을 확인한다. 빈 칸은 PAD_RECT (아주 먼 작은
사각형)로 채운다.
"""

import math

import numpy as np
import torch

from .contract_torch import C
from .env_math import integrate_pose

PAD_RECT = (1.0e6, 1.0e6, 0.01, 0.01, 0.0)


def footprint_hits_per_env(pose, obb_env, half_l=C.HALF_L, half_w=C.HALF_W):
    """
    풋프린트 [N] 이 자기 env 의 사각형 [N, M] 중 하나와 겹치는가 (분리축, 경계 포함).

    pose [N, 3], obb_env [N, M, 5]. env_math.footprint_hits 와 같은 식 (그쪽은 obb 가 env 공통 [M, 5]).
    """
    th = pose[:, 2:3]
    a0 = torch.stack([torch.cos(th), torch.sin(th)], -1)              # [N, 1, 2]
    a1 = torch.stack([-torch.sin(th), torch.cos(th)], -1)
    yaw = obb_env[..., 4]                                              # [N, M]
    b0 = torch.stack([torch.cos(yaw), torch.sin(yaw)], -1)            # [N, M, 2]
    b1 = torch.stack([-torch.sin(yaw), torch.cos(yaw)], -1)
    d = torch.stack([obb_env[..., 0] - pose[:, 0:1],
                     obb_env[..., 1] - pose[:, 1:2]], -1)             # [N, M, 2]
    hx = obb_env[..., 2]
    hy = obb_env[..., 3]
    sep = torch.zeros(d.shape[:2], dtype=torch.bool, device=d.device)
    for ax in (a0.expand_as(d), a1.expand_as(d), b0, b1):
        ra = half_l * (ax * a0).sum(-1).abs() + half_w * (ax * a1).sum(-1).abs()
        rb = hx * (ax * b0).sum(-1).abs() + hy * (ax * b1).sum(-1).abs()
        sep |= (ax * d).sum(-1).abs() > ra + rb
    return (~sep).any(dim=1)


def move_and_collide_per_env(pose, v, w, collided, obb_env, dt):
    """env_math.move_and_collide 와 같은 일 — 사각형만 env 별 [N, M, 5]."""
    moving = ~collided
    new_pose = integrate_pose(pose, v, w, dt)
    pose = torch.where(moving.unsqueeze(1), new_pose, pose)
    collided = collided | (footprint_hits_per_env(pose, obb_env) & moving)
    zero = torch.zeros_like(v)
    return pose, torch.where(collided, zero, v), torch.where(collided, zero, w), collided


def padded_tile_rects(rect_list, device, dtype=torch.float64):
    """타일별 세계 좌표 사각형 목록 [[M_t, 5], …] → [T, M_max, 5] (빈 칸 = PAD_RECT)."""
    m = max(int(np.asarray(r).shape[0]) for r in rect_list)
    out = torch.tensor(PAD_RECT, dtype=dtype, device=device).repeat(len(rect_list), m, 1)
    for t, r in enumerate(rect_list):
        r = torch.as_tensor(np.asarray(r, dtype=np.float64), dtype=dtype, device=device)
        out[t, :r.shape[0]] = r
    return out


def pad_is_far(rects, margin=1.0e5):
    """PAD_RECT 가 창고 어떤 좌표에서도 충분히 먼가 (안전 점검)."""
    r = np.asarray(rects, dtype=np.float64).reshape(-1, 5)
    reach = np.abs(r[:, :2]).max() + np.hypot(r[:, 2], r[:, 3]).max()
    return math.hypot(PAD_RECT[0], PAD_RECT[1]) - reach > margin

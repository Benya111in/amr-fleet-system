"""
관측만 쓰는 경로 추종 휴리스틱 (비교군 B 행동 공간) — 처리량 측정의 행동과 BC 시연자.

DWA(비교군 A)가 아니다: 장애물을 보지 않고, A* subgoal k=2(경로 1 m 앞) 방향으로 0.8 m/s 를 내며 목표 근처에서
선형 감속해 목표 0.08 m 안에서 선다. 관측 인덱스는 계약 §4 OBS_LAYOUT (path 195–212, goal 213–216, ego 180–191).
"""

import torch

V_DES = 0.8
I_SG2_SIN, I_SG2_COS = 199, 200      # path 블록 subgoal k=2 의 sin·cos 방위
I_GOAL_D = 214                       # goal 블록 목표 직선거리/10
I_VCMD, I_WCMD = 184, 185            # ego v_cmd/2, ω_cmd/1.5


def path_follower(obs, v_des=V_DES):
    """관측 [N,291] → 행동 u [N,2] ∈ [−1,1]² (tanh 뒤 공간)."""
    b = torch.atan2(obs[:, I_SG2_SIN], obs[:, I_SG2_COS])
    v_cmd, w_cmd = obs[:, I_VCMD] * 2.0, obs[:, I_WCMD] * 1.5
    gd = obs[:, I_GOAL_D] * 10.0
    v = v_des * torch.clamp(torch.cos(b), min=0.0) * torch.clamp(gd, max=1.0)
    v = torch.where(gd < 0.08, torch.zeros_like(v), v)
    w = torch.clamp(2.0 * b, -1.2, 1.2)
    return torch.stack([torch.clamp((v - v_cmd) / 0.1, -1, 1),
                        torch.clamp((w - w_cmd) / 0.2, -1, 1)], 1)

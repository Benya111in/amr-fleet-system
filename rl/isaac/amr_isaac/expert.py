"""
특권 정보 전문가 (경로 순수 추종) — BC 예열과 env 점검용.

env 의 자세·경로(특권)로 1.0 m 앞 경로점을 따라가는 순수 추종 + 목표 거리 비례 감속을 가속도 행동으로 낸다.
장애물 회피는 하지 않는다 (A* 경로가 정적 장애물과 떨어져 있고 이 env 에는 동적 장애물이 없다).
반환 u ∈ [−1, 1]² (env 의 tanh 뒤 행동과 같은 공간; raw = atanh(u)).
"""

import torch

from . import contract_torch as CT


def expert_u(env, lookahead=1.0, v_gain=0.7, stop_gain=0.6, w_gain=1.5, w_max=1.0):
    """전문가 행동 u [N, 2] (tanh 공간)."""
    st = env.contract_state()
    s0 = CT.project_onto_path(st, env.pose[:, :2])[0]
    tgt = CT.interpolate_at(st, s0 + lookahead)
    q = CT.to_robot_frame(env.pose, tgt.unsqueeze(1)).squeeze(1)
    b = torch.atan2(q[:, 1], q[:, 0])
    g = CT.goal_point(st) - env.pose[:, :2]
    gd = torch.hypot(g[:, 0], g[:, 1])
    v_t = torch.clamp(v_gain * torch.cos(b), min=0.0)
    v_t = torch.minimum(torch.minimum(v_t, stop_gain * gd), env.v_cap)
    w_t = torch.clamp(w_gain * b, -w_max, w_max)
    a_v = ((v_t - env.v_cmd) / CT.C.DT).clamp(-CT.C.A_MAX, CT.C.A_MAX)
    a_w = ((w_t - env.w_cmd) / CT.C.DT).clamp(-CT.C.ALPHA_MAX, CT.C.ALPHA_MAX)
    return torch.stack([a_v / CT.C.A_MAX, a_w / CT.C.ALPHA_MAX], 1)

#!/usr/bin/env python3
"""
체크포인트 재평가 (2b r6-dyn) — 동적 장애물 평가 분포에서 시작점 거르기 있음/없음 두 가지로, 새 진단 지표 포함.

    python3 rl/sim2d/tools/dyn_eval.py --models /runs/g2/<run>/model_00228.pt --arm B \
        --windows 0 3 --out /runs/g2/<run>/reeval.json

추정량은 ppo.evaluate (env 마다 첫 에피소드, 평가 seed 고정). 정책은 결정론적 (B: tanh(μ), C: argmax).
--policy 로 스크립트 정책(dyn_diag.ESCAPE 이름)이나 'expert' 를 대신 잴 수 있다.
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import torch  # noqa: E402

from amr_sim2d import dyn_diag as D  # noqa: E402
from amr_sim2d.env import EnvConfig, Sim2DEnv  # noqa: E402
from amr_sim2d.ppo import ActorCriticB, ActorCriticC, evaluate  # noqa: E402


class _Scripted:
    """스크립트 정책을 evaluate 가 부르는 model.act 꼴로."""

    def __init__(self, fn, env):
        self.fn, self.env = fn, env

    def act(self, obs, mask=None, deterministic=True):
        """→ (u, u, None)."""
        u = self.fn(obs, self.env.t)
        return u, u, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', nargs='*', default=[])
    ap.add_argument('--policy', nargs='*', default=[], help='dyn_diag.ESCAPE 이름 또는 expert')
    ap.add_argument('--arm', default='B', choices=['B', 'C'])
    ap.add_argument('--windows', type=float, nargs='*', default=[0.0, 3.0])
    ap.add_argument('--radius', type=float, default=1.0)
    ap.add_argument('--seed', type=int, default=4242)
    ap.add_argument('--episodes', type=int, default=2048)
    ap.add_argument('--path-len', type=float, nargs=2, default=[3.0, 15.0])
    ap.add_argument('--n-obstacles', type=int, nargs=2, default=[0, 4])
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    res = {'args': vars(a), 'results': {}}
    for w in a.windows:
        env = Sim2DEnv(EnvConfig(num_envs=a.episodes, seed=a.seed, arm=a.arm,
                                 obstacle_mode='mixed', n_obstacles=tuple(a.n_obstacles),
                                 path_len=tuple(a.path_len), start_clear_window=w,
                                 start_clear_radius=a.radius), log=lambda *x: None)
        for mp in a.models:
            sd = torch.load(mp, map_location='cuda')
            model = (ActorCriticB() if sd.get('arm', a.arm) == 'B' else ActorCriticC()).cuda()
            model.load_state_dict(sd['model'])
            model.eval()
            s = evaluate(model, env, a.episodes, seed=a.seed)
            s['steps'] = sd.get('steps')
            res['results'][f'{mp}@w{w}'] = s
            print(f"[{os.path.basename(os.path.dirname(mp))}/{os.path.basename(mp)} w{w}] 성공 "
                  f"{s['success']:.3f} 충돌 {s['collision']:.3f} (주행 {s['collision_moving']:.3f} "
                  f"정지 {s['collision_stationary']:.3f} 회전 {s['collision_turning']:.3f}) 정체 "
                  f"{s['stalled']:.3f} 회전 {s['spin_frac']:.3f} 개입 {s['cost_intervention']:.2f} "
                  f"최소 이격 {s.get('min_sep_median', float('nan')):.2f}", flush=True)
        for pn in a.policy:
            if pn == 'expert':
                from amr_sim2d.expert import PrivilegedExpert
                fn = PrivilegedExpert(env)
            else:
                fn = D.ESCAPE[pn]
            s = evaluate(_Scripted(fn, env), env, a.episodes, seed=a.seed)
            res['results'][f'{pn}@w{w}'] = s
            print(f"[{pn} w{w}] 성공 {s['success']:.3f} 충돌 {s['collision']:.3f} (주행 "
                  f"{s['collision_moving']:.3f} 정지 {s['collision_stationary']:.3f}) 정체 "
                  f"{s['stalled']:.3f} 개입 {s['cost_intervention']:.2f} 최소 이격 "
                  f"{s.get('min_sep_median', float('nan')):.2f}", flush=True)
        del env
        torch.cuda.empty_cache()
    with open(a.out, 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()

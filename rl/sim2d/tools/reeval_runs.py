#!/usr/bin/env python3
"""
저장된 체크포인트를 편향 없는 평가(ppo.evaluate: env 마다 첫 에피소드)로 다시 잰다 → <run>/reeval.jsonl.

    python3 rl/sim2d/tools/reeval_runs.py --runs /runs/B_bc3 /runs/B_noobs1 --episodes 2048 \
        --code-commit $(git rev-parse HEAD) [--seeds 1000 2000] [--heuristic] \
        [--legacy-norm B_main1 B_lag1 ...]

평가 env 는 각 run 의 config.json(args) 로 학습 때 평가와 같게 만든다 (비교군·장애물·개수·경로 길이·DR·safety).
env 수만 --episodes 로 바꿔 한 라운드에 끝낸다. 시드는 --seeds 를 주지 않으면 학습 시드 + 1000 (학습 때 평가
env 와 같은 경로 풀). 같은 시드의 체크포인트들은 같은 첫 에피소드 집합(시작·목표·DR·장애물)으로 잰다.
--legacy-norm 의 run 은 관측 정규화 분산 하한(6be890c) 전에 학습됐으므로 하한 없이(var_floor 0) 정규화한다.
--heuristic 은 같은 설정에서 경로 추종 휴리스틱(BC 시연자, 장애물을 보지 않는다)을 함께 잰다 (ckpt = heuristic).
"""

import argparse
import glob
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import torch  # noqa: E402

from amr_sim2d.env import EnvConfig, Sim2DEnv  # noqa: E402
from amr_sim2d.heuristic import path_follower  # noqa: E402
from amr_sim2d.ppo import ActorCriticB, ActorCriticC, evaluate  # noqa: E402

KEEP = ('episodes', 'success', 'success_se', 'collision', 'collision_moving', 'stalled',
        'timeout', 'return', 'length', 'path_len', 'time_s', 'unfinished', 'rounds', 'eval_seed',
        'estimator', 'fallback_rate')


class _Heuristic:
    """경로 추종 휴리스틱을 정책 인터페이스로 (B 행동 공간)."""

    def act(self, obs, mask=None, deterministic=True):
        """관측 → (u, u, 0)."""
        u = path_follower(obs)
        return u, u, torch.zeros(obs.shape[0], device=obs.device)


def ckpt_update(path):
    """model_00114.pt → 114, model_-0001.pt → −1."""
    return int(re.search(r'model_(-?\d+)\.pt$', path).group(1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', nargs='+', required=True)
    ap.add_argument('--episodes', type=int, default=2048)
    ap.add_argument('--seeds', type=int, nargs='*', default=None)
    ap.add_argument('--legacy-norm', nargs='*', default=[])
    ap.add_argument('--heuristic', action='store_true')
    ap.add_argument('--no-compile', action='store_true')
    ap.add_argument('--out-name', default='reeval.jsonl')
    ap.add_argument('--code-commit', default='unknown')
    a = ap.parse_args()
    for run in a.runs:
        name = os.path.basename(run.rstrip('/'))
        cfg = json.load(open(os.path.join(run, 'config.json')))
        args = cfg['args']
        hidden = tuple(cfg.get('ppo', {}).get('hidden', (256, 256)))
        legacy = name in a.legacy_norm
        seeds = a.seeds if a.seeds else [int(args.get('seed', 0)) + 1000]
        cks = sorted(glob.glob(os.path.join(run, 'model_*.pt')), key=ckpt_update)
        out = open(os.path.join(run, a.out_name), 'w')
        for seed in seeds:
            env = Sim2DEnv(EnvConfig(
                num_envs=a.episodes, seed=seed, arm=args['arm'],
                obstacle_mode=args.get('obstacles', 'mixed'),
                n_obstacles=tuple(args.get('n_obstacles', (0, 4))),
                path_len=tuple(args.get('path_len', (3.0, 15.0))),
                dr=not args.get('no_dr', False), safety=not args.get('no_safety', False),
                compile=not a.no_compile), log=lambda *x: None)
            jobs = [(os.path.basename(c)[:-3], c) for c in cks]
            if a.heuristic and args['arm'] == 'B':
                jobs.append(('heuristic', None))
            for tag, path in jobs:
                t0 = time.time()
                if path is None:
                    model, steps, upd = _Heuristic(), None, None
                else:
                    ck = torch.load(path, map_location=env.dev)
                    model = (ActorCriticB(hidden) if args['arm'] == 'B'
                             else ActorCriticC(hidden)).to(env.dev)
                    model.load_state_dict(ck['model'])
                    if legacy:
                        model.norm.var_floor = 0.0
                    model.eval()
                    steps, upd = ck.get('steps'), ckpt_update(path)
                with torch.no_grad():
                    s = evaluate(model, env, a.episodes, seed=seed)
                row = {'run': name, 'ckpt': tag, 'update': upd, 'env_steps': steps,
                       'var_floor': (None if path is None else model.norm.var_floor),
                       'code_commit': a.code_commit, 'eval_s': round(time.time() - t0, 1),
                       'loadavg1': os.getloadavg()[0],
                       **{k: s[k] for k in KEEP if k in s}}
                out.write(json.dumps(row) + '\n')
                out.flush()
                print(f"{name} {tag} seed {seed}: success {s['success']:.4f} ± "
                      f"{s.get('success_se', 0):.4f} collision {s['collision']:.4f} "
                      f"stall {s['stalled']:.4f} timeout {s['timeout']:.4f} "
                      f"path {s['path_len']:.2f} n {s['episodes']} unfinished {s['unfinished']} "
                      f"({row['eval_s']} s)", flush=True)
        out.close()


if __name__ == '__main__':
    main()

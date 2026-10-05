"""
학습 정책 평가 (결정론적 action = tanh(mu)) — held-out 경로 풀(seed 1)·명목 동역학, 에피소드 결과율.

    rl/isaac/run_isaac.sh eval rl/isaac/scripts/eval_policy.py --policy <run>/policy_final.pt \
        --num_envs 512 --episodes 2000

같은 정책을 계약형 모듈(policy_export.PolicyB)로 다시 짜서 돌린다 — ONNX 로 내보내는 것과 같은 계산이다.
env 는 raw 행동에 tanh 를 걸므로 raw = mu 를 넣으면 실행 행동 = tanh(mu) = ONNX `action`.
ONNX 왕복 시험용으로 실제 관측 묶음과 그 PolicyB 출력을 <out>/obs_batch.npz 에 남긴다.
"""

import argparse
import json
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--policy', required=True)
ap.add_argument('--num_envs', type=int, default=512)
ap.add_argument('--episodes', type=int, default=2000)
ap.add_argument('--max_steps', type=int, default=6000)
ap.add_argument('--train_pool', action='store_true', help='학습 풀(seed 0)·DR 로 평가')
ap.add_argument('--out', default=None)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac.env import AmrNavEnv, OUTCOMES  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg, AmrNavEvalEnvCfg  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402


def main():
    """평가."""
    out = args.out or os.path.join(os.path.dirname(args.policy), 'eval_train_pool'
                                   if args.train_pool else 'eval_heldout')
    os.makedirs(out, exist_ok=True)
    ck = torch.load(args.policy, map_location='cpu')
    pol = PE.PolicyB.from_rsl_state_dict(ck['state_dict'], ck.get('activation', 'elu'))
    cfg = AmrNavEnvCfg() if args.train_pool else AmrNavEvalEnvCfg()
    cfg.scene.num_envs = args.num_envs
    env = AmrNavEnv(cfg)
    pol = pol.to(env.device).eval()
    obs, _ = env.reset()
    env.pop_stats()
    keep_obs, keep_out = [], []
    t0 = time.time()
    steps = 0
    with torch.inference_mode():
        while steps < args.max_steps:
            o = obs['policy']
            action, mu, log_std = pol(o)
            if len(keep_obs) < 8 and steps % 50 == 0:
                keep_obs.append(o[:64].cpu().numpy())
                keep_out.append(np.concatenate([action[:64].cpu().numpy(), mu[:64].cpu().numpy(),
                                                log_std[:64].cpu().numpy()], 1))
            obs, _r, _t, _tr, _ = env.step(mu)
            steps += 1
            if steps % 100 == 0 and int(env.stat_episodes) >= args.episodes:
                break
    torch.cuda.synchronize()
    st = env.pop_stats()
    ep = max(st['episodes'], 1)
    res = {'policy': args.policy, 'pool': 'train(seed0)+DR' if args.train_pool else
           'heldout(seed1)+nominal', 'num_envs': args.num_envs, 'steps': steps,
           'episodes': st['episodes'], 'wall_s': round(time.time() - t0, 1),
           **{f'{o}_rate': st[o] / ep for o in OUTCOMES},
           **{f'{o}_count': st[o] for o in OUTCOMES},
           'mean_return': st['return_sum'] / ep, 'mean_len_s': st['len_steps_sum'] / ep * C.DT,
           'mean_path_len_m': st['path_len_sum'] / ep,
           **{f'cost_{k}_per_ep': st[f'cost_{k}'] / ep for k in C.COST_NAMES},
           'note': '진행 중이던 에피소드는 세지 않는다 (끝난 에피소드만)'}
    np.savez_compressed(os.path.join(out, 'obs_batch.npz'), obs=np.concatenate(keep_obs),
                        out=np.concatenate(keep_out))
    with open(os.path.join(out, 'eval.json'), 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('EVAL ' + json.dumps(res, ensure_ascii=False), flush=True)


try:
    main()
    rc = 0
except Exception:
    import traceback
    traceback.print_exc()
    rc = 1
os._exit(rc)

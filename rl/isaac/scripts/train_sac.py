"""
SAC 학습 (skrl 2.1.0, 비교군 B) — PPO 와 같은 env·관측·행동(연속, tanh)·보상·DR·φ. 학습 곡선·체크포인트·요약을 남긴다.

    rl/isaac/run_isaac.sh sac_s0 rl/isaac/scripts/train_sac.py --num_envs 1024 --env_steps 39.3e6 \
        --run sac_s0 --seed 0 --phi canonical --path_len 3:15

모델·정규화·하이퍼파라미터: amr_isaac/sac_skrl.py (근거 표는 README 「SAC」). 결정론 평가는 tanh(μ) —
eval_unbiased.py 가 policy_final.pt 의 형식(amr_sac_v1)을 보고 PolicySAC 로 읽는다.
산출: $AMR_ISAAC_RUNS/train/<run>/ — curve.jsonl (chunk 마다), policy_final.pt
(+ --save_at_env_steps 사본),
agent_final.pt (critic·target·최적화기·α 포함 전체 상태), summary.json, dr_snapshot.json.
"""

import argparse
import json
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--num_envs', type=int, default=1024)
ap.add_argument('--env_steps', type=float, default=39.3e6,
                help='총 env-step (전이 수, PPO 의 2048×300×64 와 같은 저울)')
ap.add_argument('--chunk_steps', type=int, default=64,
                help='기록 단위 (env 스텝 — PPO 의 rollout 64 와 같게)')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--run', default='sac_s0')
ap.add_argument('--dr', default='on', choices=('on', 'off'))
ap.add_argument('--reward', default='1.2.0')
ap.add_argument('--path_len', default='3:15')
ap.add_argument('--phi', default='canonical', choices=('lite', 'canonical'))
ap.add_argument('--phi_cloud', default='empty', choices=('empty', 'none'))
ap.add_argument('--hidden', default='512,256,128')
ap.add_argument('--activation', default='elu')
ap.add_argument('--lr', type=float, default=None)
ap.add_argument('--batch_size', type=int, default=None)
ap.add_argument('--gradient_steps', type=int, default=None)
ap.add_argument('--alpha0', type=float, default=None, help='초기 엔트로피 계수')
ap.add_argument('--learning_starts', type=int, default=None, help='env 스텝')
ap.add_argument('--memory', type=float, default=None, help='재생 버퍼 전이 수')
ap.add_argument('--save_at_env_steps', type=float, default=0.0)
ap.add_argument('--root', default=os.path.join(os.environ.get(
    'AMR_ISAAC_RUNS', '/home/CAPYI_AI/runs/r7-isaac12'), 'train'))
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import gymnasium  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import sac_skrl as SK  # noqa: E402
from amr_isaac.env import AmrNavEnv, DIAG_NAMES, OUTCOMES  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402


def main():
    """학습."""
    out = os.path.join(args.root, args.run)
    os.makedirs(out, exist_ok=True)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    cfg = AmrNavEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.seed = args.seed
    cfg.reward_version = args.reward
    cfg.safety_filter = args.phi
    cfg.phi_cloud_mode = args.phi_cloud
    if args.path_len:
        cfg.path_len_min, cfg.path_len_max = (float(x) for x in args.path_len.split(':'))
    cfg.dr.enable = args.dr == 'on'
    t0 = time.time()
    env = AmrNavEnv(cfg)
    dev = env.device
    N = env.num_envs
    obs_space = gymnasium.spaces.Box(-np.inf, np.inf, (C.OBS_DIM,))
    act_space = gymnasium.spaces.Box(-np.inf, np.inf, (2,))
    hp = {k: v for k, v in {'learning_rate': args.lr, 'batch_size': args.batch_size,
                            'gradient_steps': args.gradient_steps,
                            'initial_entropy_value': args.alpha0,
                            'learning_starts': args.learning_starts,
                            'memory_transitions': args.memory}.items() if v is not None}
    hidden = tuple(int(x) for x in args.hidden.split(','))
    agent, memory, scaler, hcfg = SK.make_agent(obs_space, act_space, N, dev, hp=hp,
                                                hidden=hidden, activation=args.activation)
    agent.init()
    agent.enable_training_mode(True)
    obs = env.reset(seed=args.seed)[0]['policy']
    with open(os.path.join(out, 'dr_snapshot.json'), 'w') as f:
        json.dump({'dr': args.dr, 'after_first_reset': env.dr_snapshot()}, f, indent=1)
    setup_s = time.time() - t0
    T = int(round(args.env_steps / N))
    print(f'SETUP {setup_s:.1f}s SAC steps {T} x {N} hp {json.dumps(hcfg)}', flush=True)
    env.pop_stats()
    curve = open(os.path.join(out, 'curve.jsonl'), 'w')
    ckpts = []
    t_train = time.time()
    tc = time.time()
    upd_s = 0.0
    for t in range(T):
        scaler.update(obs)
        with torch.no_grad():
            raw, _ = agent.act(obs, None, timestep=t, timesteps=T)
        nobs_d, rew, term, trunc, extras = env.step(raw)
        nobs = nobs_d['policy']
        mem_next = SK.next_obs_for_memory(obs, nobs, term, trunc)
        agent.record_transition(observations=obs, states=None, actions=raw,
                                rewards=rew.view(-1, 1), next_observations=mem_next,
                                next_states=None, terminated=term.view(-1, 1),
                                truncated=trunc.view(-1, 1), infos=extras, timestep=t,
                                timesteps=T)
        tu = time.time()
        agent.post_interaction(timestep=t, timesteps=T)
        upd_s += time.time() - tu
        obs = nobs
        if (t + 1) % args.chunk_steps == 0 or t == T - 1:
            torch.cuda.synchronize()
            dt = time.time() - tc
            st = env.pop_stats()
            ep = max(st['episodes'], 1)
            with torch.no_grad():
                mu, o = agent.policy.compute({'observations': scaler(obs)})
                q1, _ = agent.critic_1.act({'observations': scaler(obs), 'taken_actions': mu})
            n_steps = (t + 1) * N
            rec = {
                'iteration': (t + 1) // args.chunk_steps, 'env_steps': n_steps,
                'wall_s': round(time.time() - t_train, 1),
                'chunk_env_steps_per_s': round(args.chunk_steps * N / dt, 1),
                'update_frac': round(upd_s / dt, 3),
                'episodes': st['episodes'],
                **{f'{o_}_rate': st[o_] / ep for o_ in OUTCOMES},
                'mean_return': st['return_sum'] / ep,
                'mean_len_s': st['len_steps_sum'] / ep * C.DT,
                'mean_path_len_m': st['path_len_sum'] / ep,
                **{f'cost_{k}_per_ep': st[f'cost_{k}'] / ep for k in C.COST_NAMES},
                **{f'diag_{k}': st[f'{k}_sum'] / ep for k in DIAG_NAMES},
                'alpha': float(agent._entropy_coefficient),
                'log_std_mean': float(o['log_std'].mean()),
                'q_mean_at_mu': float(q1.mean()),
                'memory_filled': int(len(memory)),
                'torch_max_alloc_MiB': round(torch.cuda.max_memory_allocated() / 2**20, 1),
            }
            curve.write(json.dumps(rec) + '\n')
            curve.flush()
            print('CURVE ' + json.dumps(rec), flush=True)
            if args.save_at_env_steps > 0 and not ckpts and n_steps >= args.save_at_env_steps:
                name = f'policy_it{rec["iteration"]}.pt'
                torch.save(SK.export_state(agent), os.path.join(out, name))
                ckpts.append({'file': name, 'iteration': rec['iteration'], 'env_steps': n_steps,
                              'wall_s': rec['wall_s']})
            tc = time.time()
            upd_s = 0.0
    torch.save(SK.export_state(agent), os.path.join(out, 'policy_final.pt'))
    torch.save(SK.full_state(agent), os.path.join(out, 'agent_final.pt'))
    # 학습 정책과 내보낸 PolicySAC 가 같은 μ 를 내는지
    from amr_isaac import policy_export as PE
    pb = PE.load_policy(os.path.join(out, 'policy_final.pt'))
    with torch.no_grad():
        mu_a, _ = agent.policy.compute({'observations': scaler(obs)})
        mu_b = pb(obs.cpu())[1]
    diff = float((mu_a.cpu() - mu_b).abs().max())
    summary = {'run': args.run,
               'algo': 'SAC (skrl 2.1.0, squashed Gaussian, twin Q, auto entropy)',
               'num_envs': N, 'env_steps': T * N, 'decision_steps_per_env': T,
               'contract_version': C.CONTRACT_VERSION, 'reward_version': args.reward,
               'reward_spec': str(env.reward_spec), 'path_len': args.path_len,
               'pool_paths': env.pool_size, 'constraint': 'none (unconstrained SAC)',
               'checkpoints': ckpts, 'setup_s': round(setup_s, 1),
               'train_wall_s': round(time.time() - t_train, 1), 'seed': args.seed,
               'dr': args.dr == 'on', 'safety_filter': args.phi,
               'phi_cloud_mode': args.phi_cloud if args.phi == 'canonical' else None,
               'hyperparameters': hcfg, 'final_alpha': float(agent._entropy_coefficient),
               'policy_export_mu_max_abs_diff': diff}
    with open(os.path.join(out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    print('TRAIN_DONE ' + json.dumps(summary), flush=True)


try:
    main()
    rc = 0
except Exception:
    import traceback
    traceback.print_exc()
    rc = 1
os._exit(rc)

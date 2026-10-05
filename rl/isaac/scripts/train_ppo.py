"""
rsl_rl PPO 학습 (비교군 B) — 학습 곡선 JSONL·체크포인트·요약을 남기고 스스로 종료한다.

    rl/isaac/run_isaac.sh train rl/isaac/scripts/train_ppo.py --num_envs 1024 --iterations 300 \
        --run ppo_b_s0 --seed 0

산출: /home/CAPYI_AI/runs/r1-isaac/train/<run>/ — curve.jsonl (chunk 마다: 반복, env-step, 처리량,
에피소드 결과율, 평균 반환, 비용 합, GPU 메모리), model_*.pt (rsl_rl), summary.json, HEAD.txt 는
실행 전에 run_isaac.sh 를 부른 쪽이 쓴다.
"""

import argparse
import json
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--num_envs', type=int, default=1024)
ap.add_argument('--iterations', type=int, default=300)
ap.add_argument('--chunk', type=int, default=10)
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--run', default='ppo_b_s0')
ap.add_argument('--no_dr', action='store_true')
ap.add_argument('--vcap_curriculum', default='',
                help="v_cap 커리큘럼 'LO:HI0:N0:N1:P1' — 반복 N0 까지 v_cap ~ U[LO, HI0], "
                     "N1 까지 상한을 명목값(1.0)으로·명목 비율을 P1 로 선형, 빈 값 = 끔")
ap.add_argument('--entropy', type=float, default=None)
ap.add_argument('--lagrange', default='',
                help="PPO-Lagrangian 'λ0,…,λ4/η0,…,η4/d1' — 초기 승수/쌍대 상승률/개입 예산, 빈 값 = 끔")
ap.add_argument('--bc_steps', type=int, default=0,
                help='0 보다 크면 전문가(경로 순수 추종) 시연으로 actor 를 BC 예열한다 (env step 수)')
ap.add_argument('--bc_noise', type=float, default=0.15)
ap.add_argument('--bc_updates', type=int, default=3000)
ap.add_argument('--critic_warmup', type=int, default=20, help='BC 뒤 actor 를 얼려 둘 PPO 반복 수')
ap.add_argument('--init_std', type=float, default=None)
ap.add_argument('--lr', type=float, default=None)
ap.add_argument('--schedule', default=None, help="'fixed' 또는 'adaptive'")
ap.add_argument('--clip', type=float, default=None)
ap.add_argument('--curriculum', default='',
                help="경로 길이 상한 커리큘럼 'L0:L1:N' — 반복 N 까지 L0→L1 m 선형, 빈 값 = 끔")
ap.add_argument('--root', default='/home/CAPYI_AI/runs/r1-isaac/train')
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import torch  # noqa: E402
from rsl_rl.networks import EmpiricalNormalization  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

from amr_isaac import policy_export as PE  # noqa: E402
from amr_isaac.expert import expert_u  # noqa: E402
from amr_isaac.agents.rsl_rl_ppo_cfg import AmrNavPPORunnerCfg  # noqa: E402
from amr_isaac.env import AmrNavEnv, OUTCOMES  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402


class ContractNormalization(EmpiricalNormalization):
    """rsl_rl 정규화의 통계 갱신은 그대로, 적용 식만 계약식으로 (onnx_io.md, 분산 하한 포함)."""

    def forward(self, x):
        """clip((x − μ)/√(max(σ², floor) + 1e-8), −10, 10)."""
        return PE.contract_normalize(x, self._mean, self._var)


def swap_normalizers(policy):
    """actor·critic 정규화 모듈을 계약식으로 바꾼다 (버퍼 이름이 같아 저장·불러오기 호환)."""
    for name in ('actor_obs_normalizer', 'critic_obs_normalizer'):
        old = getattr(policy, name)
        if not isinstance(old, EmpiricalNormalization):
            raise RuntimeError(f'{name} 가 EmpiricalNormalization 이 아니다: {type(old)}')
        new = ContractNormalization(old._mean.shape[-1], eps=old.eps, until=old.until)
        new.load_state_dict(old.state_dict())
        new.to(old._mean.device)
        setattr(policy, name, new)


def bc_warm_start(env, venv, pol, out):
    """전문가 시연 수집 (잡음 섞어 실행, 레이블은 깨끗한 행동) → 정규화 통계 → actor MSE(tanh 공간)."""
    t0 = time.time()
    obs = venv.get_observations()['policy']
    xs, us = [], []
    env.pop_stats()
    for _ in range(args.bc_steps):
        u = expert_u(env).to(torch.float32)
        xs.append(obs.clone())
        us.append(u.clone())
        ue = (u + args.bc_noise * torch.randn_like(u)).clamp(-0.995, 0.995)
        td, _r, _d, _e = venv.step(torch.atanh(ue))
        obs = td['policy']
    st = env.pop_stats()
    X = torch.cat(xs)
    U = torch.cat(us)
    for nm in ('actor_obs_normalizer', 'critic_obs_normalizer'):
        nz = getattr(pol, nm)
        nz._mean[:] = X.mean(0, keepdim=True)
        nz._var[:] = X.var(0, unbiased=False, keepdim=True)
        nz._std[:] = torch.sqrt(nz._var)
        nz.count.fill_(X.shape[0])
        # BC 로 맞춘 actor 는 이 통계에 묶여 있다 — 롤아웃 중 통계가 움직이면 입력 분포가 바뀌어 정책이
        # 무너진다 (첫 시도에서 actor 를 얼린 채로도 성공 98.9 % → 5.3 %). 통계를 여기서 고정한다.
        nz.until = int(X.shape[0])
    opt = torch.optim.Adam(pol.actor.parameters(), lr=1e-3)
    n = X.shape[0]
    losses = []
    for k in range(args.bc_updates):
        idx = torch.randint(0, n, (4096,), device=X.device)
        x = PE.contract_normalize(X[idx], pol.actor_obs_normalizer._mean,
                                  pol.actor_obs_normalizer._var)
        loss = ((torch.tanh(pol.actor(x)) - U[idx]) ** 2).mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
        if k % 500 == 0 or k == args.bc_updates - 1:
            losses.append([k, float(loss)])
    rec = {'bc_samples': n, 'bc_s': round(time.time() - t0, 1), 'bc_loss': losses,
           'expert_noisy_episodes': st['episodes'],
           'expert_noisy_success_rate': st['success'] / max(st['episodes'], 1),
           'expert_noisy_collision_rate': st['collision'] / max(st['episodes'], 1)}
    with open(os.path.join(out, 'bc.json'), 'w') as f:
        json.dump(rec, f, indent=1)
    print('BC ' + json.dumps(rec), flush=True)
    del X, U, xs, us
    torch.cuda.empty_cache()


def gpu_mem_mib():
    """이 프로세스의 torch 할당·예약 [MiB]."""
    return (round(torch.cuda.max_memory_allocated() / 2**20, 1),
            round(torch.cuda.memory_reserved() / 2**20, 1))


def main():
    """학습."""
    out = os.path.join(args.root, args.run)
    os.makedirs(out, exist_ok=True)
    env_cfg = AmrNavEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed
    if args.no_dr:
        env_cfg.dr.enable = False
    agent = AmrNavPPORunnerCfg()
    agent.seed = args.seed
    agent.max_iterations = args.iterations
    agent.device = 'cuda:0'
    if args.entropy is not None:
        agent.algorithm.entropy_coef = args.entropy
    if args.lr is not None:
        agent.algorithm.learning_rate = args.lr
    if args.schedule is not None:
        agent.algorithm.schedule = args.schedule
    if args.clip is not None:
        agent.algorithm.clip_param = args.clip
    t0 = time.time()
    env = AmrNavEnv(env_cfg)
    venv = RslRlVecEnvWrapper(env, clip_actions=agent.clip_actions)
    runner = OnPolicyRunner(venv, agent.to_dict(), log_dir=out, device=agent.device)
    swap_normalizers(runner.alg.policy)
    pol = runner.alg.policy
    if args.init_std is not None:
        with torch.no_grad():
            pol.log_std.fill_(float(torch.log(torch.tensor(args.init_std))))
    if args.bc_steps > 0:
        bc_warm_start(env, venv, pol, out)
        torch.save({'state_dict': {k: v.detach().cpu() for k, v in pol.state_dict().items()},
                    'activation': agent.policy.activation,
                    'hidden': agent.policy.actor_hidden_dims}, os.path.join(out, 'policy_bc.pt'))
        venv.reset()
        frozen = [p for p in list(pol.actor.parameters()) + [pol.log_std]]
        for p in frozen:
            p.requires_grad_(False)
        sched = runner.alg.schedule
        runner.alg.schedule = 'fixed'
    setup_s = time.time() - t0
    print(f'SETUP {setup_s:.1f}s policy {runner.alg.policy}', flush=True)
    env.pop_stats()
    curve = open(os.path.join(out, 'curve.jsonl'), 'w')
    steps_per_iter = agent.num_steps_per_env * args.num_envs
    it = 0
    t_train = time.time()
    cur = [float(x) for x in args.curriculum.split(':')] if args.curriculum else None
    vc = [float(x) for x in args.vcap_curriculum.split(':')] if args.vcap_curriculum else None
    lag = None
    if args.lagrange:
        l0, eta, d1 = args.lagrange.split('/')
        lag = {'eta': [float(x) for x in eta.split(',')],
               'budget': [C.COST_BUDGETS[0], float(d1)] + list(C.COST_BUDGETS[2:])}
        env.lagrange[:] = torch.tensor([float(x) for x in l0.split(',')], dtype=torch.float64)
    while it < args.iterations:
        n = min(args.chunk, args.iterations - it)
        if args.bc_steps > 0 and frozen and it >= args.critic_warmup:
            for p in frozen:
                p.requires_grad_(True)
            frozen = []
            runner.alg.schedule = sched
            print(f'UNFREEZE actor at iteration {it}', flush=True)
        if cur:
            env.curriculum_lmax = cur[0] + (cur[1] - cur[0]) * min(1.0, it / cur[2])
        if vc:
            f = min(1.0, max(0.0, (it - vc[2]) / max(vc[3] - vc[2], 1.0)))
            env.vcap_lo = vc[0]
            env.vcap_hi = vc[1] + (env_cfg.v_cap - vc[1]) * f
            env.vcap_p_nominal = vc[4] * f
        tc = time.time()
        runner.learn(num_learning_iterations=n, init_at_random_ep_len=False)
        torch.cuda.synchronize()
        dt = time.time() - tc
        it += n
        st = env.pop_stats()
        ep = max(st['episodes'], 1)
        if lag and st['episodes'] > 0:
            # 쌍대 상승: λ_i ← max(0, λ_i + η_i·(J_i − d_i)/d_i), J = 끝난 에피소드의 비할인 비용 합 평균
            for i, k in enumerate(C.COST_NAMES):
                d = lag['budget'][i]
                err = min(max((st[f'cost_{k}'] / ep - d) / d, -1.0), 5.0)
                env.lagrange[i] = max(0.0, float(env.lagrange[i]) + lag['eta'][i] * err)
        pol = runner.alg.policy
        rec = {
            'iteration': it, 'env_steps': it * steps_per_iter,
            'wall_s': round(time.time() - t_train, 1),
            'chunk_env_steps_per_s': round(n * steps_per_iter / dt, 1),
            'episodes': st['episodes'],
            **{f'{o}_rate': st[o] / ep for o in OUTCOMES},
            'mean_return': st['return_sum'] / ep,
            'mean_len_s': st['len_steps_sum'] / ep * C.DT,
            'mean_path_len_m': st['path_len_sum'] / ep,
            **{f'cost_{k}_per_ep': st[f'cost_{k}'] / ep for k in C.COST_NAMES},
            'action_std': [round(float(v), 4) for v in torch.exp(pol.log_std).tolist()],
            'lr': runner.alg.learning_rate,
            'curriculum_lmax': env.curriculum_lmax,
            'lagrange': [round(float(x), 5) for x in env.lagrange.tolist()],
            'vcap': [env.vcap_lo, env.vcap_hi, env.vcap_p_nominal],
            'torch_max_alloc_MiB': gpu_mem_mib()[0], 'torch_reserved_MiB': gpu_mem_mib()[1],
        }
        curve.write(json.dumps(rec) + '\n')
        curve.flush()
        print('CURVE ' + json.dumps(rec), flush=True)
    runner.save(os.path.join(out, f'model_{it}.pt'))
    sd = runner.alg.policy.state_dict()
    torch.save({'state_dict': {k: v.cpu() for k, v in sd.items()},
                'activation': agent.policy.activation,
                'hidden': agent.policy.actor_hidden_dims}, os.path.join(out, 'policy_final.pt'))
    # 학습 정책(act_inference) 과 계약 모듈(PolicyB) 이 같은 mu 를 내는지 (정규화 교체 확인)
    pb = PE.PolicyB.from_rsl_state_dict({k: v.cpu() for k, v in sd.items()},
                                        agent.policy.activation)
    obs = venv.get_observations()
    with torch.inference_mode():
        mu_rsl = runner.alg.policy.act_inference(obs).cpu()
        mu_pb = pb(obs['policy'].cpu())[1]
    diff = float((mu_rsl - mu_pb).abs().max())
    summary = {'run': args.run, 'iterations': it, 'num_envs': args.num_envs,
               'env_steps': it * steps_per_iter, 'setup_s': round(setup_s, 1),
               'train_wall_s': round(time.time() - t_train, 1),
               'policy_vs_rsl_mu_max_abs_diff': diff, 'seed': args.seed,
               'dr': not args.no_dr, 'curriculum': args.curriculum,
               'vcap_curriculum': args.vcap_curriculum, 'lagrange': args.lagrange,
               'lagrange_final': [float(x) for x in env.lagrange.tolist()],
               'entropy_coef': agent.algorithm.entropy_coef, 'bc_steps': args.bc_steps,
               'critic_warmup': args.critic_warmup, 'init_std': args.init_std,
               'lr': agent.algorithm.learning_rate, 'schedule': agent.algorithm.schedule,
               'clip': agent.algorithm.clip_param}
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

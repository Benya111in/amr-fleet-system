#!/usr/bin/env python3
"""
PPO 학습 (비교군 B 또는 C) — 학습 곡선 로그, 주기 평가, 체크포인트, 끝에 ONNX 내보내기 + onnxruntime 왕복.

    python3 rl/sim2d/tools/train_ppo.py --arm B --envs 2048 --total-steps 20e6 \
        --run-dir /runs/<태그> --git-commit $(git rev-parse HEAD)

산출물 (run-dir): config.json, train.jsonl (갱신마다), eval.jsonl, model_*.pt, policy.onnx,
onnx_check.json.
평가는 학습 env 와 다른 시드의 별도 env 인스턴스에서 결정론적 정책(B: tanh(μ), C: argmax)으로 한다.
추정량은 env 마다 첫 에피소드 하나 (ppo.evaluate), 평가마다 같은 시드로 다시 심어 체크포인트끼리 같은
시작·목표·DR·장애물 배치를 쓴다. --eval-episodes ≤ --eval-envs 면 한 라운드로 끝난다.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import torch  # noqa: E402

from amr_sim2d.env import EnvConfig, Sim2DEnv  # noqa: E402
from amr_sim2d.ppo import (PPOConfig, Trainer, config_dict, episode_summary,  # noqa: E402
                           evaluate, export_onnx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', default='B', choices=['B', 'C'])
    ap.add_argument('--envs', type=int, default=2048)
    ap.add_argument('--total-steps', type=float, default=2e7)
    ap.add_argument('--rollout', type=int, default=64)
    ap.add_argument('--epochs', type=int, default=5)
    ap.add_argument('--minibatches', type=int, default=8)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--ent-coef', type=float, default=0.0)
    ap.add_argument('--target-kl', type=float, default=0.03)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--obstacles', default='mixed')
    ap.add_argument('--n-obstacles', type=int, nargs=2, default=[0, 4])
    ap.add_argument('--path-len', type=float, nargs=2, default=[3.0, 15.0])
    ap.add_argument('--lagrangian', action='store_true', help='계약 §7 비용 제약 (PI 라그랑주)')
    ap.add_argument('--lag-ki', type=float, default=2e-4)
    ap.add_argument('--lag-kp', type=float, default=0.0)
    ap.add_argument('--lag-kd', type=float, default=0.0)
    ap.add_argument('--lag-max', type=float, default=2.0)
    ap.add_argument('--lag-ema', type=float, default=0.0)
    ap.add_argument('--lag-channels', type=int, nargs='*', default=None,
                    help='제약할 비용 채널 (기본 = 예산이 있는 모든 채널; 0 = 충돌)')
    ap.add_argument('--lag-budget-collision', type=float, default=None,
                    help='충돌 예산 d (에피소드당, 기본 = 계약 0.02)')
    ap.add_argument('--bc-steps', type=int, default=0,
                    help='> 0 이면 경로 추종 휴리스틱 시연으로 정책 평균을 BC 초기화 (B 전용)')
    ap.add_argument('--bc-log-std', type=float, default=-1.0)
    ap.add_argument('--bc-expert', default='follower', choices=['follower', 'privileged'],
                    help='BC 시연자: 장애물을 보지 않는 경로 추종기 / 장애물 미래를 보는 특권 MPC (expert.py)')
    ap.add_argument('--dagger', type=int, default=0, help='BC 뒤 DAgger 반복 수 (학생으로 굴리고 시연자 레이블)')
    ap.add_argument('--kl-bc', type=float, default=0.0, help='KL(π ‖ π_BC) 계수 (BC 뒤 PPO)')
    ap.add_argument('--critic-warmup', type=int, default=0)
    ap.add_argument('--curriculum', type=float, default=0.0,
                    help='학습 앞 이 비율 동안 경로 상한 6 m→목표, 장애물 상한 0→목표로 선형 증가')
    ap.add_argument('--learnability', action='store_true',
                    help='장애물 배치 수준을 learnability p(1−p) 로 뽑는다 (--curriculum 단계 안에서)')
    ap.add_argument('--lp-uniform', type=float, default=0.2)
    ap.add_argument('--lp-ema', type=float, default=0.2)
    ap.add_argument('--start-clear-window', type=float, default=0.0,
                    help='시작점 거르기 창 [s] (학습·평가 env 모두, 0 = 끔)')
    ap.add_argument('--start-clear-radius', type=float, default=1.0)
    ap.add_argument('--final-eval-orig', action='store_true',
                    help='끝에 시작점 거르기 없는 평가 분포로 한 번 더 평가 (eval_orig.json)')
    ap.add_argument('--no-dr', action='store_true')
    ap.add_argument('--no-safety', action='store_true')
    ap.add_argument('--no-compile', action='store_true')
    ap.add_argument('--eval-every', type=int, default=20, help='갱신 수')
    ap.add_argument('--eval-envs', type=int, default=1024)
    ap.add_argument('--eval-episodes', type=int, default=1024)
    ap.add_argument('--reward', default='v1.2.0',
                    help='보상 설계 (amr_rl_contract.reward_variants.CANDIDATES 이름)')
    ap.add_argument('--eval-seed', type=int, default=None,
                    help='평가 env 시드 (경로 풀·첫 에피소드 고정). None = seed + 1000')
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--git-commit', default='unknown')
    a = ap.parse_args()
    os.makedirs(a.run_dir, exist_ok=True)

    eval_seed = a.seed + 1000 if a.eval_seed is None else a.eval_seed

    def env_cfg(n, seed, window=a.start_clear_window):
        return EnvConfig(num_envs=n, seed=seed, arm=a.arm, obstacle_mode=a.obstacles,
                         n_obstacles=tuple(a.n_obstacles), path_len=tuple(a.path_len),
                         dr=not a.no_dr, safety=not a.no_safety, compile=not a.no_compile,
                         reward=a.reward, start_clear_window=window,
                         start_clear_radius=a.start_clear_radius)
    log = print
    t_wall0 = time.time()
    env = Sim2DEnv(env_cfg(a.envs, a.seed), log=log)
    eval_env = Sim2DEnv(env_cfg(a.eval_envs, eval_seed), log=log)
    budgets = list(PPOConfig.budgets)
    if a.lag_budget_collision is not None:
        budgets[0] = a.lag_budget_collision
    pcfg = PPOConfig(rollout=a.rollout, epochs=a.epochs, minibatches=a.minibatches, lr=a.lr,
                     ent_coef=a.ent_coef, target_kl=a.target_kl, lagrangian=a.lagrangian,
                     lag_ki=a.lag_ki, lag_kp=a.lag_kp, lag_kd=a.lag_kd, lag_max=a.lag_max,
                     lag_ema=a.lag_ema, budgets=tuple(budgets), kl_bc=a.kl_bc,
                     lag_channels=None if a.lag_channels is None else tuple(a.lag_channels))
    sampler = None
    if a.learnability:
        from amr_sim2d.curriculum import LevelSampler
        sampler = LevelSampler(env.n_levels, uniform=a.lp_uniform, ema=a.lp_ema)
    tr = Trainer(env, pcfg, seed=a.seed)
    cfg = {'args': vars(a), 'ppo': config_dict(pcfg), 'env': {
        k: (list(v) if isinstance(v, tuple) else v) for k, v in env.cfg.__dict__.items()
        if k not in ('dr_ranges', 'nominal')}, 'dr_ranges': env.cfg.dr_ranges,
        'gpu': torch.cuda.get_device_name(0), 'torch': torch.__version__,
        'loadavg_start': os.getloadavg(), 'reward_spec': env.spec.__dict__,
        'eval_seed': eval_seed}
    with open(os.path.join(a.run_dir, 'config.json'), 'w') as f:
        json.dump(cfg, f, indent=1, default=str)
    ftr = open(os.path.join(a.run_dir, 'train.jsonl'), 'a')
    fev = open(os.path.join(a.run_dir, 'eval.jsonl'), 'a')

    def do_eval(upd):
        t0 = time.time()
        tr.model.eval()
        s = evaluate(tr.model, eval_env, a.eval_episodes, seed=eval_seed)
        tr.model.train()
        s.update({'update': upd, 'env_steps': tr.steps, 'eval_s': time.time() - t0})
        fev.write(json.dumps(s) + '\n')
        fev.flush()
        log(f"[eval] upd {upd} steps {tr.steps:.3g}: success {s.get('success', 0):.3f} "
            f"collision {s.get('collision', 0):.3f} stall {s.get('stalled', 0):.3f} "
            f"timeout {s.get('timeout', 0):.3f} progress {s.get('progress', 0):.3f} "
            f"spin {s.get('spin_frac', 0):.3f} off1m {s.get('offpath_1m_frac', 0):.3f} "
            f"coll_stat {s.get('collision_stationary', 0):.3f} "
            f"minsep {s.get('min_sep_median', float('nan')):.2f} "
            f"(n={s['episodes']}, unfinished {s.get('unfinished', 0)})", flush=True)
        torch.save({'model': tr.model.state_dict(), 'steps': tr.steps, 'arm': a.arm},
                   os.path.join(a.run_dir, f'model_{upd:05d}.pt'))
        return s

    n_upd = int(a.total_steps // (a.envs * a.rollout))
    do_eval(0)
    if a.bc_steps > 0 and a.bc_expert == 'follower':
        from amr_sim2d.heuristic import path_follower
        mse = tr.behavior_clone(path_follower, a.bc_steps, log=log)
        with torch.no_grad():
            tr.model.log_std.fill_(a.bc_log_std)
        tr.steps += a.bc_steps * a.envs
        s = do_eval(-1)
        log(f'[bc] 시연 {a.bc_steps * a.envs} 스텝, mse {mse:.4f}, BC 정책 평가 성공 '
            f"{s.get('success', 0):.3f}")
    elif a.bc_steps > 0:
        from amr_sim2d.expert import PrivilegedExpert
        ex = PrivilegedExpert(env)

        def demo(o):
            return ex(o, env.t)

        def student(o):
            return tr.model.act(o, deterministic=True)[0]
        t_bc = time.time()
        O, Z = tr.collect_demo(demo, a.bc_steps)
        mse = tr.fit_bc(O, Z, log=log)
        tr.steps += a.bc_steps * a.envs
        for it in range(a.dagger):
            O2, Z2 = tr.collect_demo(demo, a.bc_steps, executor=student)
            O, Z = torch.cat([O, O2]), torch.cat([Z, Z2])
            mse = tr.fit_bc(O, Z, log=log)
            tr.steps += a.bc_steps * a.envs
            log(f'[dagger] {it + 1}/{a.dagger} n={O.shape[0]} mse {mse:.4f}')
        del O, Z
        torch.cuda.empty_cache()
        with torch.no_grad():
            tr.model.log_std.fill_(a.bc_log_std)
        tr.set_bc_reference()
        # KL(π ‖ π_BC) 는 같은 원관측에서 두 정책을 비교한다 — 학생의 정규화 통계가 계속 바뀌면 같은 가중치에서도
        # KL > 0 이 되므로 BC 데이터의 통계로 고정한다
        tr.cfg.freeze_norm = a.kl_bc > 0
        tr.obs = env.reset()
        s = do_eval(-1)
        log(f'[bc] 특권 전문가 시연 {tr.steps} 스텝 (DAgger {a.dagger}), mse {mse:.4f}, '
            f"BC 정책 평가 성공 {s.get('success', 0):.3f}, 전문가 통계 {ex.stats}, "
            f'{time.time() - t_bc:.0f} s')
    if a.critic_warmup > 0:
        tr.warmup_critic(a.critic_warmup)
        log(f'[critic] 가치망 예열 {a.critic_warmup} 롤아웃')
    t_start = time.time()
    for upd in range(1, n_upd + 1):
        omax = a.n_obstacles[1]
        if a.curriculum > 0:
            f = min(1.0, (upd - 1) / max(1.0, a.curriculum * n_upd))
            pmax = 6.0 + f * (a.path_len[1] - 6.0)
            omax = int(round(f * a.n_obstacles[1]))
            env.set_curriculum((a.path_len[0], max(pmax, a.path_len[0] + 1.0)),
                               (min(a.n_obstacles[0], omax), omax))
        lp = None
        if sampler is not None:
            full = omax >= a.n_obstacles[1] and a.obstacles == 'mixed'
            lp = sampler.probs(sampler.allowed(omax, full))
            env.set_level_probs(lp)
        t0 = time.time()
        eps = tr.collect()
        t1 = time.time()
        st = tr.update()
        st['lambda'] = tr.update_lagrange(eps)
        if sampler is not None:
            sampler.update(eps)
            st['level_probs'] = [round(float(x), 4) for x in lp]
            st['level_success_ema'] = [round(float(x), 4) for x in sampler.p]
        t2 = time.time()
        row = {'update': upd, 'env_steps': tr.steps, 'wall_s': time.time() - t_start,
               'collect_s': t1 - t0, 'update_s': t2 - t1,
               'fps': a.envs * a.rollout / (t2 - t0), **st, **episode_summary(eps),
               'loadavg1': os.getloadavg()[0], 'cur_path_len': list(env.cfg.path_len),
               'cur_n_obstacles': list(env.cfg.n_obstacles)}
        ftr.write(json.dumps(row) + '\n')
        ftr.flush()
        if upd % 5 == 0 or upd == 1:
            log(f"upd {upd}/{n_upd} steps {tr.steps:.3g} fps {row['fps']:.0f} "
                f"succ {row.get('success', float('nan')):.3f} coll "
                f"{row.get('collision', float('nan')):.3f} ret {row.get('return', 0):.2f} "
                f"kl {st['kl']:.4f} ep {st['epochs_run']} ent {st['entropy']:.3f}", flush=True)
        if upd % a.eval_every == 0 or upd == n_upd:
            do_eval(upd)
    if a.final_eval_orig and a.start_clear_window > 0:
        orig_env = Sim2DEnv(env_cfg(a.eval_envs, eval_seed, window=0.0), log=log)
        tr.model.eval()
        so = evaluate(tr.model, orig_env, a.eval_episodes, seed=eval_seed)
        so.update({'update': n_upd, 'env_steps': tr.steps, 'start_clear_window': 0.0})
        with open(os.path.join(a.run_dir, 'eval_orig.json'), 'w') as f:
            json.dump(so, f)
        log(f"[eval orig] success {so.get('success', 0):.3f} collision "
            f"{so.get('collision', 0):.3f} stall {so.get('stalled', 0):.3f}")
        del orig_env
    with open(os.path.join(a.run_dir, 'wall.json'), 'w') as f:
        json.dump({'wall_s': time.time() - t_wall0, 'train_loop_s': time.time() - t_start,
                   'loadavg_end': os.getloadavg(),
                   'gpu_max_alloc_gb': torch.cuda.max_memory_allocated() / 1e9,
                   'gpu_max_reserved_gb': torch.cuda.max_memory_reserved() / 1e9}, f)
    meta = {'algo': 'ppo', 'seed': a.seed, 'git_commit': a.git_commit, 'train_steps': tr.steps,
            'reward_spec': env.spec.name}
    path = os.path.join(a.run_dir, 'policy.onnx')
    props = export_onnx(tr.model, path, meta)
    from onnx_check import roundtrip
    chk = roundtrip(tr.model, path, eval_env)
    chk['metadata'] = props
    with open(os.path.join(a.run_dir, 'onnx_check.json'), 'w') as f:
        json.dump(chk, f, indent=1)
    log(f"[onnx] {path}: {json.dumps({k: v for k, v in chk.items() if k != 'metadata'})}")


if __name__ == '__main__':
    main()

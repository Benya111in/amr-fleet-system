#!/usr/bin/env python3
"""
PPO 학습 (비교군 B 또는 C) — 학습 곡선 로그, 주기 평가, 체크포인트, 끝에 ONNX 내보내기 + onnxruntime 왕복.

    python3 rl/sim2d/tools/train_ppo.py --arm B --envs 2048 --total-steps 20e6 \
        --run-dir /runs/<태그> --git-commit $(git rev-parse HEAD)

산출물 (run-dir): config.json, train.jsonl (갱신마다), eval.jsonl, model_*.pt, policy.onnx,
onnx_check.json.
평가는 학습 env 와 다른 시드의 별도 env 인스턴스에서 결정론적 정책(B: tanh(μ), C: argmax)으로 한다.
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
    ap.add_argument('--bc-steps', type=int, default=0,
                    help='> 0 이면 경로 추종 휴리스틱 시연으로 정책 평균을 BC 초기화 (B 전용)')
    ap.add_argument('--bc-log-std', type=float, default=-1.0)
    ap.add_argument('--critic-warmup', type=int, default=0)
    ap.add_argument('--curriculum', type=float, default=0.0,
                    help='학습 앞 이 비율 동안 경로 상한 6 m→목표, 장애물 상한 0→목표로 선형 증가')
    ap.add_argument('--no-dr', action='store_true')
    ap.add_argument('--no-safety', action='store_true')
    ap.add_argument('--no-compile', action='store_true')
    ap.add_argument('--eval-every', type=int, default=20, help='갱신 수')
    ap.add_argument('--eval-envs', type=int, default=512)
    ap.add_argument('--eval-episodes', type=int, default=1000)
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--git-commit', default='unknown')
    a = ap.parse_args()
    os.makedirs(a.run_dir, exist_ok=True)

    def env_cfg(n, seed):
        return EnvConfig(num_envs=n, seed=seed, arm=a.arm, obstacle_mode=a.obstacles,
                         n_obstacles=tuple(a.n_obstacles), path_len=tuple(a.path_len),
                         dr=not a.no_dr, safety=not a.no_safety, compile=not a.no_compile)
    log = print
    env = Sim2DEnv(env_cfg(a.envs, a.seed), log=log)
    eval_env = Sim2DEnv(env_cfg(a.eval_envs, a.seed + 1000), log=log)
    pcfg = PPOConfig(rollout=a.rollout, epochs=a.epochs, minibatches=a.minibatches, lr=a.lr,
                     ent_coef=a.ent_coef, target_kl=a.target_kl, lagrangian=a.lagrangian,
                     lag_ki=a.lag_ki,
                     lag_kp=a.lag_kp)
    tr = Trainer(env, pcfg, seed=a.seed)
    cfg = {'args': vars(a), 'ppo': config_dict(pcfg), 'env': {
        k: (list(v) if isinstance(v, tuple) else v) for k, v in env.cfg.__dict__.items()
        if k not in ('dr_ranges', 'nominal')}, 'dr_ranges': env.cfg.dr_ranges,
        'gpu': torch.cuda.get_device_name(0), 'torch': torch.__version__,
        'loadavg_start': os.getloadavg()}
    with open(os.path.join(a.run_dir, 'config.json'), 'w') as f:
        json.dump(cfg, f, indent=1, default=str)
    ftr = open(os.path.join(a.run_dir, 'train.jsonl'), 'a')
    fev = open(os.path.join(a.run_dir, 'eval.jsonl'), 'a')

    def do_eval(upd):
        t0 = time.time()
        tr.model.eval()
        s = evaluate(tr.model, eval_env, a.eval_episodes)
        tr.model.train()
        s.update({'update': upd, 'env_steps': tr.steps, 'eval_s': time.time() - t0})
        fev.write(json.dumps(s) + '\n')
        fev.flush()
        log(f"[eval] upd {upd} steps {tr.steps:.3g}: success {s.get('success', 0):.3f} "
            f"collision {s.get('collision', 0):.3f} stall {s.get('stalled', 0):.3f} "
            f"timeout {s.get('timeout', 0):.3f} (n={s['episodes']})", flush=True)
        torch.save({'model': tr.model.state_dict(), 'steps': tr.steps, 'arm': a.arm},
                   os.path.join(a.run_dir, f'model_{upd:05d}.pt'))
        return s

    n_upd = int(a.total_steps // (a.envs * a.rollout))
    do_eval(0)
    if a.bc_steps > 0:
        from amr_sim2d.heuristic import path_follower
        mse = tr.behavior_clone(path_follower, a.bc_steps, log=log)
        with torch.no_grad():
            tr.model.log_std.fill_(a.bc_log_std)
        tr.steps += a.bc_steps * a.envs
        s = do_eval(-1)
        log(f'[bc] 시연 {a.bc_steps * a.envs} 스텝, mse {mse:.4f}, BC 정책 평가 성공 '
            f"{s.get('success', 0):.3f}")
    if a.critic_warmup > 0:
        tr.warmup_critic(a.critic_warmup)
        log(f'[critic] 가치망 예열 {a.critic_warmup} 롤아웃')
    t_start = time.time()
    for upd in range(1, n_upd + 1):
        if a.curriculum > 0:
            f = min(1.0, (upd - 1) / max(1.0, a.curriculum * n_upd))
            pmax = 6.0 + f * (a.path_len[1] - 6.0)
            omax = int(round(f * a.n_obstacles[1]))
            env.set_curriculum((a.path_len[0], max(pmax, a.path_len[0] + 1.0)),
                               (min(a.n_obstacles[0], omax), omax))
        t0 = time.time()
        eps = tr.collect()
        t1 = time.time()
        st = tr.update()
        st['lambda'] = tr.update_lagrange(eps)
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
    meta = {'algo': 'ppo', 'seed': a.seed, 'git_commit': a.git_commit, 'train_steps': tr.steps}
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

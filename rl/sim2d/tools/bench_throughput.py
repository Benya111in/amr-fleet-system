#!/usr/bin/env python3
"""
처리량 측정 — env 수별 env-steps/s 와 GPU 메모리 [측정].

    python3 rl/sim2d/tools/bench_throughput.py --envs 256 1024 4096 --steps 200 --out <json>

각 설정을 새 프로세스로 돌려 최대 메모리가 섞이지 않게 한다. 행동은 두 가지:
  heuristic — 경로 추종 휴리스틱(장애물 무시, 실제에 가까운 리셋 빈도), random — 균등 무작위(리셋 잦음).
워밍업(컴파일 포함) 뒤 steps 스텝의 벽시계 시간을 잰다. 공유 서버이므로 loadavg 를 함께 남긴다.
"""

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


def heuristic(obs):
    """경로 다음 subgoal(k=2) 방향으로 0.8 m/s, 목표 근처 감속 (장애물 무시)."""
    import torch
    b = torch.atan2(obs[:, 199], obs[:, 200])
    v_cmd, w_cmd = obs[:, 184] * 2.0, obs[:, 185] * 1.5
    gd = obs[:, 214] * 10.0
    v_des = 0.8 * torch.clamp(torch.cos(b), min=0.0) * torch.clamp(gd, max=1.0)
    v_des = torch.where(gd < 0.08, torch.zeros_like(v_des), v_des)
    w_des = torch.clamp(2.0 * b, -1.2, 1.2)
    return torch.stack([torch.clamp((v_des - v_cmd) / 0.1, -1, 1),
                        torch.clamp((w_des - w_cmd) / 0.2, -1, 1)], 1)


def run_one(n, steps, warmup, mode, arm, obstacles, compile_):
    import torch
    from amr_sim2d.env import EnvConfig, Sim2DEnv
    torch.cuda.reset_peak_memory_stats()
    env = Sim2DEnv(EnvConfig(num_envs=n, arm=arm, obstacle_mode=obstacles, compile=compile_),
                   log=lambda *a: None)
    obs = env.reset()
    g = torch.Generator(device='cuda')
    g.manual_seed(0)

    def act(o):
        if arm == 'C':
            lg = torch.rand(n, 100, device='cuda', generator=g)
            return torch.where(env.mask, lg, torch.full_like(lg, -1.0)).argmax(1)
        if mode == 'random':
            return torch.rand(n, 2, device='cuda', generator=g) * 2 - 1
        return heuristic(o)
    t0 = time.time()
    for _ in range(warmup):
        obs, *_ = env.step(act(obs))
    torch.cuda.synchronize()
    t_warm = time.time() - t0
    resets = 0
    t0 = time.time()
    for _ in range(steps):
        obs, r, te, tr, info = env.step(act(obs))
        if 'done_idx' in info:
            resets += int(info['done_idx'].numel())
    torch.cuda.synchronize()
    dt = time.time() - t0
    return {'envs': n, 'arm': arm, 'action': mode, 'obstacles': obstacles, 'compile': compile_,
            'steps': steps, 'seconds': dt, 'env_steps_per_s': n * steps / dt,
            'resets_per_step': resets / steps, 'warmup_s_incl_compile': t_warm,
            'max_mem_allocated_GB': torch.cuda.max_memory_allocated() / 1e9,
            'max_mem_reserved_GB': torch.cuda.max_memory_reserved() / 1e9,
            'loadavg': os.getloadavg(), 'gpu': torch.cuda.get_device_name(0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--envs', type=int, nargs='+', default=[256, 1024, 4096])
    ap.add_argument('--steps', type=int, default=200)
    ap.add_argument('--warmup', type=int, default=20)
    ap.add_argument('--modes', nargs='+', default=['heuristic', 'random'])
    ap.add_argument('--arm', default='B')
    ap.add_argument('--obstacles', default='random')
    ap.add_argument('--no-compile', action='store_true')
    ap.add_argument('--out', default=None)
    ap.add_argument('--child', action='store_true')
    a = ap.parse_args()
    if a.child:
        r = run_one(a.envs[0], a.steps, a.warmup, a.modes[0], a.arm, a.obstacles,
                    not a.no_compile)
        print('RESULT ' + json.dumps(r))
        return
    rows = []
    for mode in a.modes:
        for n in a.envs:
            cmd = [sys.executable, __file__, '--child', '--envs', str(n), '--steps', str(a.steps),
                   '--warmup', str(a.warmup), '--modes', mode, '--arm', a.arm,
                   '--obstacles', a.obstacles] + (['--no-compile'] if a.no_compile else [])
            out = subprocess.run(cmd, capture_output=True, text=True)
            line = [x for x in out.stdout.splitlines() if x.startswith('RESULT ')]
            if not line:
                print(out.stdout[-2000:], out.stderr[-4000:])
                raise SystemExit(f'실패: {cmd}')
            r = json.loads(line[0][7:])
            rows.append(r)
            print(f"{mode:9s} arm {a.arm} N={n:5d}: {r['env_steps_per_s']:10.0f} env-steps/s, "
                  f"리셋 {r['resets_per_step']:.1f}/step, 메모리 {r['max_mem_reserved_GB']:.2f} GB "
                  f"(할당 최대 {r['max_mem_allocated_GB']:.2f}), loadavg {r['loadavg'][0]:.1f}",
                  flush=True)
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(rows, f, indent=1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
2 라운드 실행 조율 (prereg_r2.md). 1 라운드 orchestrate_x.py 의 레인 실행기(run_lanes·lane_cmd·job_done·log_run)를
그대로 쓰고, 경로 집합·단계·팔만 2 라운드로 바꾼다. 원자료는 /home/CAPYI_AI/paper_research/runs/r2/<단계>/
(컨테이너 /runs/r2/<단계>/), 실행 기록은 같은 runs.jsonl (id 앞에 r2_).

    python3 orchestrate_r2.py paths --commit <sha> --dirty 0 --ws /home/CAPYI_AI/wt/dwa-abl
    python3 orchestrate_r2.py ros <equiv|tuning|c1|c2|c3|o> --commit <sha> --dirty 0 [--lanes 14]
    python3 orchestrate_r2.py chain --commit <sha> --dirty 0 [--procs 24]

경로 집합 (prereg_r2 §1):
- 튜닝-20  /runs/r2/paths_tune  seed 20261020, 매끈 10 · 통로 4 (시드 의존) · A* 6, 튜닝 O 6 장면
- held-out C /runs/r2/paths_c   seed 20261019, 매끈 32 · 통로 8 (시드 의존) · A* 20, O 20 장면
지도는 집합마다 그 집합의 통로 랙을 담은 composite.yaml (레인 컨테이너에 PR_MAP 으로 넘긴다).
"""
import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
import time

import orchestrate_x as ox
import params_x as params

RUNS_HOST = ox.RUNS_HOST
SCRIPTS_C = ox.SCRIPTS_C
SETS = {
    'tune': {'dir': '/runs/r2/paths_tune', 'seed': 20261020, 'n': (10, 4, 6), 'o': 6},
    'c': {'dir': '/runs/r2/paths_c', 'seed': 20261019, 'n': (32, 8, 20), 'o': 20},
}
NARROW_SUBSET = ['N1', 'N4', 'N6']   # prereg_r2 §3 C3 (1 라운드와 같은 규칙)


def set_json(key):
    d = SETS[key]['dir']
    return [f'{d}/heldout_smooth.json', f'{d}/heldout_narrow.json', f'{d}/heldout_astar.json']


def set_scenes(key):
    d = SETS[key]['dir']
    return [f'{d}/scenes_heldout_smooth.txt', f'{d}/scenes_heldout_narrow.txt',
            f'{d}/scenes_heldout_astar.txt']


def set_map(key):
    return f"{SETS[key]['dir']}/composite.yaml"


# ---------------------------------------------------------------------------------------------
# 경로 생성

def paths_stage(args):
    os.makedirs(os.path.join(RUNS_HOST, 'r2', 'paths_logs'), exist_ok=True)
    base = ['docker', 'run', '--rm', '--name', 'amr-ai-pr-paths', '--cpus', '4', '--memory', '8g',
            '--shm-size', '2g', '-e', f'ROS_DOMAIN_ID={ox.DOMAIN0}', '-e', 'ROS_LOCALHOST_ONLY=1',
            '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro',
            '-v', f'{ox.WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs']
    for key in args.sets.split(','):
        s = SETS[key]
        d = s['dir']
        if os.path.exists(ox.c2h(d) + '/SEALED'):
            print(f'{d} 봉인됨, 건너뜀')
            continue
        ns, nn, na = s['n']
        gen = ['--seed', str(s['seed']), '--narrow-random', '--n-smooth', str(ns),
               '--n-narrow', str(nn), '--n-astar', str(na)]
        steps = [
            ('make_paths', [], ['python3', f'{SCRIPTS_C}/make_paths_r2.py', '--out', d] + gen),
            ('plan_astar', ['-e', f'PR_MAP={d}/composite.yaml'],
             ['bash', f'{SCRIPTS_C}/ros_worker_x.sh', str(ox.DOMAIN0), f'amr_{ox.DOMAIN0}',
              '/ros2_ws/install/amr_navigation/share/amr_navigation/config/nav2_params.yaml',
              f'/runs/r2/paths_logs/launch_plan_{key}.log', '--',
              'python3', f'{SCRIPTS_C}/plan_astar_x.py', '--pairs', f'{d}/astar_pairs.json',
              '--out', f'{d}/heldout_astar.json', '--n', str(na),
              '--ros-args', '-r', '__ns:=/@NS@']),
            ('astar_scenes', [], ['python3', f'{SCRIPTS_C}/make_paths_r2.py', '--out', d,
                                  '--astar-scenes'] + gen),
            ('obstacles', [], ['python3', f'{SCRIPTS_C}/scenes_obstacles.py', '--paths', d,
                               '--seed', str(s['seed']), '--n', str(s['o'])]),
        ]
        for name, extra, cmd in steps:
            ox.docker_rm_if_mine('amr-ai-pr-paths')
            t0 = time.time()
            l0 = os.getloadavg()
            log = os.path.join(RUNS_HOST, 'r2', 'paths_logs', f'{key}_{name}.log')
            with open(log, 'w', encoding='utf-8') as lf:
                cmd_full = base[:4] + base[4:] + extra + [ox.IMAGE] + cmd
                rc = subprocess.run(cmd_full, stdout=lf, stderr=subprocess.STDOUT).returncode
            ox.log_run({'id': f'r2_paths_{key}_{name}', 'testbed': 'paths', 'commit': args.commit,
                        'dirty': args.dirty, 'config': {'cmd': cmd, 'ws': args.ws, 'rc': rc},
                        'seed': s['seed'], 'start': t0, 'end': time.time(), 'raw': d,
                        'loadavg': [l0, os.getloadavg()]})
            print(key, name, rc, flush=True)
            if rc != 0:
                return rc
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['paths'])
    ap.add_argument('stage', nargs='?', default='')
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    ap.add_argument('--ws', default=ox.WS_HOST)
    ap.add_argument('--sets', default='tune,c')
    args = ap.parse_args()
    if args.cmd == 'paths':
        return paths_stage(args)
    return 2


if __name__ == '__main__':
    sys.exit(main())

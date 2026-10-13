#!/usr/bin/env python3
"""
실행 조율 (prereg.md §4): chain_sim 조건 묶음과 ROS 시험대 단계를 돌리고 runs.jsonl 에 한 줄씩 남긴다.

    python3 orchestrate.py chain --commit <sha> --dirty <n>
    python3 orchestrate.py ros-tuning|ros-heldout|ros-noise --commit <sha> --dirty <n>

컨테이너 amr-ai-dwa-abl-dev 안에서 돈다 (/ros2_ws = worktree, /runs = runs/dwa-abl).
ROS 단계는 도메인 61·62·63 에 시험대 세 벌을 동시에 띄우고 경로를 나눠 맡긴다.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

import params

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_JSONL = os.path.abspath(os.path.join(HERE, '..', 'runs.jsonl'))
CHAIN = '/ros2_ws/install/amr_navigation/lib/amr_navigation/chain_sim'
P = '/runs/paths'
RAW = '/runs/raw'
HELDOUT_SCENES = [f'{P}/scenes_heldout_smooth.txt', f'{P}/scenes_heldout_narrow.txt',
                  f'{P}/scenes_heldout_astar.txt']
HELDOUT_JSON = [f'{P}/heldout_smooth.json', f'{P}/heldout_narrow.json', f'{P}/heldout_astar.json']
CHAIN_ARMS = ['dwa_full', 'pp', 'dwa_no_m1', 'dwa_no_m1c', 'dwa_no_m1e', 'dwa_no_m2',
              'dwa_m2_trap', 'dwa_no_m3', 'dwa_no_m4', 'dwa_no_m4a', 'dwa_no_m5', 'dwa_textbook']
NOISE = ['odom_noise_v=0.02', 'odom_noise_w=0.02', 'pose_noise_xy=0.01', 'pose_noise_yaw=0.005',
         'pose_noise_tau=1.0', 'lidar=1']
NOISE_IDS = ['S01_pwc', 'S02_pwl', 'S03_pwc', 'S04_pwl', 'S05_pwc', 'S06_pwl', 'N1', 'N4', 'N6',
             'A05', 'A06', 'A12']
WORKERS = [(61, 'amr_61'), (62, 'amr_62'), (63, 'amr_63')]


def sha(fn):
    with open(fn, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def log_run(entry):
    with open(RUNS_JSONL, 'a', encoding='utf-8') as f:
        f.write(json.dumps(entry, ensure_ascii=False) + '\n')


def summarize(fn, arm=None, since=0):
    rs = []
    with open(fn, encoding='utf-8') as f:
        for i, line in enumerate(f):
            if i >= since:
                rs.append(json.loads(line))
    if arm:
        rs = [r for r in rs if r['arm'] == arm]
    ok = sum(r['success'] for r in rs)
    ts = sorted(r['t_reach'] for r in rs if r['success'])
    return {'n': len(rs), 'success': ok,
            'median_t_reach': ts[len(ts) // 2] if ts else None}


def nlines(fn):
    if not os.path.exists(fn):
        return 0
    with open(fn, encoding='utf-8') as f:
        return sum(1 for _ in f)


def chain_job(args, rid, out, scenes, arm, extra, seed=None, note=''):
    cfg = f'/runs/cfg/{"pp" if arm == "pp" else arm}.cfg'
    cmd = [CHAIN, scenes, out, f'cfg={cfg}', f'arm={arm}'] + (['ctrl=pp'] if arm == 'pp' else [])
    cmd += extra + ([f'seed={seed}'] if seed is not None else [])
    n0 = nlines(out)
    t0 = time.time()
    load0 = os.getloadavg()
    subprocess.run(cmd, check=True)
    log_run({'id': rid, 'testbed': 'chain_sim', 'commit': args.commit, 'dirty': args.dirty,
             'config': {'arm': arm, 'cfg_sha': sha(cfg), 'args': extra, 'note': note},
             'paths': os.path.basename(scenes), 'paths_sha': sha(scenes), 'seed': seed,
             'start': t0, 'end': time.time(), 'raw': out, 'result': summarize(out, arm, n0),
             'loadavg': [load0, os.getloadavg()]})


def run_chain(args):
    subprocess.run([sys.executable, os.path.join(HERE, 'params.py'), '/runs/cfg'], check=True,
                   stdout=subprocess.DEVNULL)
    for arm in CHAIN_ARMS:   # 튜닝 집합 (시험대 교정·참고)
        chain_job(args, f'chain_tuning_{arm}', f'{RAW}/chain_tuning.jsonl',
                  f'{P}/scenes_tuning.txt', arm, [])
    for sc in HELDOUT_SCENES:
        fam = sc.split('_')[-1][:-4]
        for arm in CHAIN_ARMS:
            chain_job(args, f'chain_clean_{fam}_{arm}', f'{RAW}/chain_clean.jsonl', sc, arm, [])
        for arm in ('dwa_full', 'dwa_no_m3', 'dwa_no_m2', 'dwa_textbook', 'pp'):
            chain_job(args, f'chain_ideal_{fam}_{arm}', f'{RAW}/chain_ideal.jsonl', sc, arm,
                      ['plant=ideal'])
        for hz in (10, 40):
            for arm in ('dwa_full', 'dwa_no_m3'):
                chain_job(args, f'chain_rate{hz}_{fam}_{arm}', f'{RAW}/chain_rate.jsonl', sc, arm,
                          [f'rate={hz}'])
        for seed in range(1, 6):
            for arm in CHAIN_ARMS:
                chain_job(args, f'chain_noise_{fam}_{arm}_s{seed}', f'{RAW}/chain_noise.jsonl',
                          sc, arm, NOISE, seed)
    for plant in ('chain', 'ideal'):
        for hz in (10, 20, 40):
            for arm in ('dwa_full', 'dwa_no_m3'):
                tr = f'/runs/traces/probe_{plant}_{hz}'
                os.makedirs(tr, exist_ok=True)
                chain_job(args, f'chain_probe_{plant}_{hz}_{arm}', f'{RAW}/chain_probe.jsonl',
                          f'{P}/scenes_probe.txt', arm,
                          [f'plant={plant}', f'rate={hz}', f'trace_dir={tr}'])


def ros_stage(args, stage, plan):
    """plan: [(domain, ns, arms, ids, out, noise(seed, v, w) | None)] — 동시에 돈다."""
    procs = []
    for dom, ns, arms, ids, out, noise in plan:
        pfile = f'/runs/ros/params_{stage}_{ns}.yaml'
        params.ros_params_file(pfile, arms)
        bench = ['python3', os.path.join(HERE, 'ros_bench.py'), '--paths',
                 ','.join([f'{P}/tuning.json'] if stage == 'tuning' else HELDOUT_JSON),
                 '--ids', ','.join(ids), '--arms', ','.join(arms), '--out', out,
                 '--tag', f'{stage}', '--worker', ns, '--ns', '/@NS@', '--skip-done']
        nargs = []
        if noise:
            bench += ['--noise-seed', str(noise[0]), '--odom-noise-v', str(noise[1]),
                      '--odom-noise-w', str(noise[2])]
            nargs = [str(noise[0]), str(noise[1]), str(noise[2])]
        bench += ['--ros-args', '-r', '__ns:=/@NS@', '--log-level', 'warn']
        cmd = ['bash', os.path.join(HERE, 'ros_worker.sh'), str(dom), ns, pfile,
               f'/runs/logs/launch_{stage}_{ns}{"_s" + str(noise[0]) if noise else ""}.log'] + \
            nargs + ['--'] + bench
        n0 = nlines(out)
        log = open(f'/runs/logs/bench_{stage}_{ns}{"_s" + str(noise[0]) if noise else ""}.log',
                   'a', encoding='utf-8')
        procs.append((subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT), dom, ns, arms,
                      ids, out, noise, n0, time.time(), os.getloadavg(), pfile))
        time.sleep(20.0)    # 기동 부하가 겹치지 않게
    for p, dom, ns, arms, ids, out, noise, n0, t0, l0, pfile in procs:
        rc = p.wait()
        log_run({'id': f'ros_{stage}_{ns}' + (f'_s{noise[0]}' if noise else ''),
                 'testbed': 'ros_kinematic', 'commit': args.commit, 'dirty': args.dirty,
                 'config': {'arms': arms, 'params_sha': sha(pfile), 'domain': dom,
                            'noise': noise, 'rc': rc},
                 'paths': ids, 'seed': noise[0] if noise else 0, 'start': t0, 'end': time.time(),
                 'raw': out, 'result': summarize(out, None, n0) if os.path.exists(out) else None,
                 'loadavg': [l0, os.getloadavg()]})


def heldout_ids():
    ids = []
    for f in HELDOUT_JSON:
        with open(f, encoding='utf-8') as fh:
            ids += [e['id'] for e in json.load(fh)]
    return ids


def chosen_arms():
    with open(f'{RAW}/tuning_selection.json', encoding='utf-8') as f:
        sel = json.load(f)
    return [sel[fam]['chosen'] for fam in ('DWB', 'MPPI', 'RPP')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['chain', 'ros-tuning', 'ros-heldout', 'ros-noise'])
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    args = ap.parse_args()
    os.makedirs(RAW, exist_ok=True)
    os.makedirs('/runs/ros', exist_ok=True)
    os.makedirs('/runs/logs', exist_ok=True)
    if args.cmd == 'chain':
        run_chain(args)
    elif args.cmd == 'ros-tuning':
        fam = {61: [f'DWB_{k}' for k in params.DWB_GRID] + ['dwa_full'],
               62: [f'MPPI_{k}' for k in params.MPPI_GRID] + ['pp'],
               63: [f'RPP_{k}' for k in params.RPP_GRID] + ['dwa_textbook']}
        plan = [(d, ns, fam[d], [], f'{RAW}/ros_tuning_w{d}.jsonl', None) for d, ns in WORKERS]
        ros_stage(args, 'tuning', plan)
    elif args.cmd == 'ros-heldout':
        arms = ['dwa_full', 'dwa_textbook', 'pp', 'DWB_D00', 'MPPI_P00', 'RPP_R00',
                'dwa_no_m1', 'dwa_no_m2', 'dwa_no_m3']
        arms += [a for a in chosen_arms() if a not in arms]
        ids = heldout_ids()
        plan = [(d, ns, arms, ids[k::3], f'{RAW}/ros_heldout_w{d}.jsonl', None)
                for k, (d, ns) in enumerate(WORKERS)]
        ros_stage(args, 'heldout', plan)
    else:
        arms = ['dwa_full', 'dwa_textbook', 'pp', 'DWB_D00', 'MPPI_P00', 'RPP_R00']
        arms += [a for a in chosen_arms() if a not in arms]
        for seed in range(1, 6):
            plan = [(d, ns, arms, NOISE_IDS[k::3], f'{RAW}/ros_noise_w{d}.jsonl',
                     (seed, 0.02, 0.02)) for k, (d, ns) in enumerate(WORKERS)]
            ros_stage(args, 'noise', plan)
    return 0


if __name__ == '__main__':
    sys.exit(main())

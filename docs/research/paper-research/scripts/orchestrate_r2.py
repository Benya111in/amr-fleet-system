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
import params_r2
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


# ---------------------------------------------------------------------------------------------
# ROS 단계

FIXED_ARMS = ['dwa_full', 'dwa_R', 'pp', 'DWB_D00', 'MPPI_P00', 'RPP_R00']   # prereg_r2 §2 고정 팔


def selection():
    with open(os.path.join(RUNS_HOST, 'r2', 'tuning', 'selection_r2.json'), encoding='utf-8') as f:
        return json.load(f)


def heldout_arms():
    """고정 팔 + 튜닝 선택 팔 (DWB·MPPI·RPP 선택, 교과서 튜닝, DWA-R2)."""
    sel = selection()
    arms = list(FIXED_ARMS)
    arms += [sel[f]['chosen'] for f in ('DWB', 'MPPI', 'RPP') if sel[f]['chosen'] not in arms]
    arms += ['dwa_textbook_tuned', 'dwa_R2']
    return arms, sel['DWA_R2']['chosen_y'], sel['TB']['chosen_t']


def load_c(path_c):
    with open(ox.c2h(path_c), encoding='utf-8') as f:
        return json.load(f)


def make_job(stage, jid, arms, ids, paths, est_s, set_key=None, seed=0, rep=0, largs=(),
             extra_bench=(), **pf_kw):
    """1 라운드 orchestrate_x.make_job 과 같고, 디렉터리가 /runs/r2/<단계>, 지도가 경로 집합 것 (PR_MAP)."""
    sd = f'/runs/r2/{stage}'
    pf = os.path.join(ox.c2h(sd), 'params', f'{jid}.yaml')
    params_r2.ros_params_file(pf, arms, resets_prefix=f'{sd}/resets/{jid}', **pf_kw)
    for d in ('resets', 'traces', 'raw', 'logs'):
        os.makedirs(os.path.join(ox.c2h(sd), d), exist_ok=True)
    bench = ['--paths', ','.join(paths), '--ids', ','.join(ids), '--arms', ','.join(arms),
             '--tag', f'r2_{stage}', '--rep', str(rep), '--trace-dir', f'{sd}/traces/{jid}'] + \
        list(extra_bench)
    largs = list(largs)
    if seed:
        bench += ['--noise-seed', str(seed), '--odom-noise-v', '0.02', '--odom-noise-w', '0.02']
        largs += [f'noise_seed:={seed}', 'odom_noise_v:=0.02', 'odom_noise_w:=0.02']
    env = {'PR_MAP': set_map(set_key)} if set_key else {}
    return {'id': jid, 'arms': list(arms), 'ids': list(ids), 'seed': seed, 'rep': rep,
            'params': pf, 'largs': largs, 'bench': bench, 'env': env,
            'out': f'{sd}/raw/{jid}.jsonl', 'launch_log': f'{sd}/logs/{jid}.launch.log',
            'bench_log': f'{sd}/logs/{jid}.bench.log', 'timeout': 2.0 * est_s + 600.0}


def ros_stage(args):
    st = args.stage
    sdh = os.path.join(RUNS_HOST, 'r2', st)
    if os.path.exists(os.path.join(sdh, 'SEALED')):
        print(f'{sdh} 는 봉인됐다', file=sys.stderr)
        return 2
    for d in ('raw', 'logs', 'params'):
        os.makedirs(os.path.join(sdh, d), exist_ok=True)
    jobs = []
    if st == 'equiv':
        # prereg_r2 §4: X03(dwa_R) 대 Y00, 1 라운드 튜닝 4 경로 × 실시간 반복 5 (1 라운드 지도)
        tun = load_c(ox.TUNING_JSON)
        ids = [e['id'] for e in tun]
        arms = ['dwa_R', 'dwa_y_Y00']
        for rep in range(1, args.reps + 1):
            jobs.append(make_job(st, f'rep{rep}', arms, ids, [ox.TUNING_JSON],
                                 ox.est_seconds(tun, ids, arms), rep=rep))
    elif st == 'tuning':
        # prereg_r2 §2: 다섯 격자 × 12 × 튜닝-20 × {잡음 없음, seed 1–3} + 튜닝 O 6 × 1.
        # 작업 = 같은 번호 k 의 DWB·MPPI·RPP·T·Y 다섯 팔 × 경로 절반 (10 경로)
        ent = []
        for p in set_json('tune'):
            ent += load_c(p)
        ids = [e['id'] for e in ent]
        halves = (ids[0::2], ids[1::2])
        grids = list(zip(params.DWB_GRID, params.MPPI_GRID, params.RPP_GRID, params.TB_GRID,
                         params_r2.DWA_Y_GRID))
        for seed in (0, 1, 2, 3):
            for k, (d, m, r, t, y) in enumerate(grids):
                arms = [f'DWB_{d}', f'MPPI_{m}', f'RPP_{r}', params.tb_arm(t), params_r2.y_arm(y)]
                for h, sub in enumerate(halves):
                    jobs.append(make_job(st, f's{seed}_k{k:02d}_h{h}', arms, sub, set_json('tune'),
                                         ox.est_seconds(ent, sub, arms), set_key='tune',
                                         seed=seed))
        scenes = load_c(f"{SETS['tune']['dir']}/scenes_obstacles.json")
        for sc in scenes:
            for g in range(4):
                arms = []
                for k in range(3 * g, 3 * g + 3):
                    d, m, r, t, y = grids[k]
                    arms += [f'DWB_{d}', f'MPPI_{m}', f'RPP_{r}', params.tb_arm(t), params_r2.y_arm(y)]
                jobs.append(make_job(
                    st, f'O_{sc["id"]}_g{g}', arms, [sc['id']],
                    [f"{SETS['tune']['dir']}/scenes_obstacles.json"],
                    ox.est_seconds(scenes, [sc['id']], arms), set_key='tune',
                    largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                    extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O']))
    elif st in ('c1', 'c2'):
        arms, ysel, tsel = heldout_arms()
        stateful = st == 'c2'
        ent = []
        for p in set_json('c'):
            ent += load_c(p)
        ids = [e['id'] for e in ent]
        for j in range(20):
            sub = ids[j::20]
            jobs.append(make_job(st, f'j{j:02d}', arms, sub, set_json('c'),
                                 ox.est_seconds(ent, sub, arms), set_key='c',
                                 extra_bench=['--stateful', str(int(stateful)), '--condition', st.upper()],
                                 stateful=stateful, selected_y=ysel, selected_t=tsel))
    elif st == 'c3':
        arms, ysel, tsel = heldout_arms()
        arms = arms + ['dwa_x_X00']
        ent = []
        for p in set_json('c'):
            ent += load_c(p)
        sm = [e['id'] for e in load_c(set_json('c')[0])][:6]
        ast = [e['id'] for e in load_c(set_json('c')[2]) if e['meta']['length'] < 20.0][:3]
        ids = sm + NARROW_SUBSET + ast
        for seed in range(1, 6):
            for j in range(4):
                sub = ids[j::4]
                jobs.append(make_job(st, f's{seed}_j{j}', arms, sub, set_json('c'),
                                     ox.est_seconds(ent, sub, arms), set_key='c', seed=seed,
                                     extra_bench=['--condition', 'C3'],
                                     selected_y=ysel, selected_t=tsel))
    elif st == 'o':
        arms, ysel, tsel = heldout_arms()
        oj = f"{SETS['c']['dir']}/scenes_obstacles.json"
        scenes = load_c(oj)
        for rep in (1, 2, 3):
            for sc in scenes:
                jobs.append(make_job(
                    st, f'{sc["id"]}_r{rep}', arms, [sc['id']], [oj],
                    ox.est_seconds(scenes, [sc['id']], arms), set_key='c', rep=rep,
                    largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                    extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O'],
                    selected_y=ysel, selected_t=tsel))
    else:
        raise SystemExit(f'알 수 없는 단계 {st}')
    with open(os.path.join(sdh, 'jobs.json'), 'w', encoding='utf-8') as f:
        json.dump(jobs, f, indent=1)
    failed = ox.run_lanes(jobs, args, f'r2_{st}')
    print(json.dumps({'stage': f'r2_{st}', 'jobs': len(jobs), 'failed': failed}), flush=True)
    return 1 if failed else 0


# ---------------------------------------------------------------------------------------------
# chain_sim_x (prereg_r2 §3: DWA 팔 전부를 C1·C2·C3 대응 조건으로)

def chain_calls():
    arms = ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_textbook_tuned']
    calls = []
    for arm in arms:
        for sc in set_scenes('c'):
            calls.append((arm, sc, 0, 0))
            calls.append((arm, sc, 0, 1))
            for seed in range(1, 6):
                calls.append((arm, sc, seed, 0))
    return calls


def chain_call(args, arm, scenes, seed, stateful):
    fam = scenes.split('_')[-1][:-4]
    cond = 'clean' if seed == 0 else f's{seed}'
    cid = f'{arm}__{fam}__{cond}__st{stateful}'
    root = '/runs/r2/chain'
    out, rlog, tdir = f'{root}/raw/{cid}.jsonl', f'{root}/resets/{cid}.csv', f'{root}/traces/{cid}'
    os.makedirs(tdir, exist_ok=True)
    cfg = f'{root}/cfg/{arm}.cfg'
    n_exp = sum(1 for line in open(scenes, encoding='utf-8') if line.startswith('scene '))
    if ox.nlines(out) >= n_exp:
        return cid, 'skip'
    for fn in (out, rlog):
        if os.path.exists(fn):
            os.remove(fn)
    cmd = [ox.CHAIN_X, scenes, out, f'cfg={cfg}', f'arm={arm}', f'stateful={stateful}',
           f'reset_log={rlog}', f'trace_dir={tdir}']
    if seed:
        cmd += ox.NOISE_CHAIN + [f'seed={seed}']
    t0 = time.time()
    l0 = os.getloadavg()
    r = subprocess.run(cmd, capture_output=True, text=True)
    return cid, {'id': f'r2_chain_{cid}', 'testbed': 'chain_sim_x', 'commit': args.commit,
                 'dirty': args.dirty,
                 'config': {'arm': arm, 'cfg_sha': ox.sha(cfg), 'args': cmd[3:], 'rc': r.returncode,
                            'stderr': r.stderr[-500:]},
                 'paths': os.path.basename(scenes), 'paths_sha': ox.sha(scenes), 'seed': seed,
                 'stateful': stateful, 'start': t0, 'end': time.time(), 'raw': out,
                 'result': ox.summarize(out, arm) if os.path.exists(out) else None,
                 'loadavg': [l0, os.getloadavg()]}


def chain_inner(args):
    root = '/runs/r2/chain'
    for d in ('raw', 'resets', 'traces', 'cfg'):
        os.makedirs(f'{root}/{d}', exist_ok=True)
    with open('/runs/r2/tuning/selection_r2.json', encoding='utf-8') as f:
        sel = json.load(f)
    print('cfg', params_r2.write_cfgs(f'{root}/cfg', sel['DWA_R2']['chosen_y'],
                                      sel['TB']['chosen_t']), flush=True)
    calls = sorted(chain_calls(), key=lambda c: (c[2] == 0, c[1]))
    entries = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.procs) as ex:
        futs = [ex.submit(chain_call, args, *c) for c in calls]
        for k, fu in enumerate(concurrent.futures.as_completed(futs)):
            cid, entry = fu.result()
            if entry != 'skip':
                entries.append(entry)
                with open(f'{root}/runs_chain.jsonl', 'a', encoding='utf-8') as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + '\n')
            print(f'[r2 chain] {k + 1}/{len(calls)} {cid}', flush=True)
    bad = [e['id'] for e in entries if e['config']['rc'] != 0]
    print(json.dumps({'calls': len(calls), 'ran': len(entries), 'rc_nonzero': bad}), flush=True)
    return 1 if bad else 0


def chain_stage(args):
    root_h = os.path.join(RUNS_HOST, 'r2', 'chain')
    os.makedirs(root_h, exist_ok=True)
    ox.docker_rm_if_mine('amr-ai-pr-chain')
    fn = os.path.join(root_h, 'runs_chain.jsonl')
    n0 = ox.nlines(fn)
    cmd = ['docker', 'run', '--rm', '--name', 'amr-ai-pr-chain', '--cpus', str(args.procs),
           '--memory', '24g', '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro',
           '-v', f'{ox.WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs', ox.IMAGE,
           'python3', f'{SCRIPTS_C}/orchestrate_r2.py', 'chain-inner', '--commit', args.commit,
           '--dirty', str(args.dirty), '--procs', str(args.procs)]
    with open(os.path.join(root_h, 'chain_stage.log'), 'a', encoding='utf-8') as lf:
        rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
    if os.path.exists(fn):
        with open(fn, encoding='utf-8') as f:
            for i, line in enumerate(f):
                if i >= n0 and line.strip():
                    ox.log_run(json.loads(line))
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['paths', 'ros', 'chain', 'chain-inner'])
    ap.add_argument('stage', nargs='?', default='')
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    ap.add_argument('--ws', default=ox.WS_HOST)
    ap.add_argument('--sets', default='tune,c')
    ap.add_argument('--lanes', type=int, default=14)
    ap.add_argument('--lane0', type=int, default=0)
    ap.add_argument('--cpus', type=float, default=3.0)
    ap.add_argument('--memory', default='8g')
    ap.add_argument('--stagger', type=float, default=15.0)
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--procs', type=int, default=24)
    args = ap.parse_args()
    if args.cmd == 'paths':
        return paths_stage(args)
    if args.cmd == 'ros':
        return ros_stage(args)
    if args.cmd == 'chain':
        return chain_stage(args)
    return chain_inner(args)


if __name__ == '__main__':
    sys.exit(main())

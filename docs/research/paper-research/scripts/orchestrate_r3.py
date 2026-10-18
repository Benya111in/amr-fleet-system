#!/usr/bin/env python3
"""
3 라운드 실행 조율 (prereg_r3.md). 2 라운드 orchestrate_r2.py·1 라운드 orchestrate_x.py 의 레인 실행기를 그대로 쓰고
경로 집합 held-out D 와 3 라운드 단계·팔만 더한다. 원자료 /home/CAPYI_AI/paper_research/runs/r3/<단계>/.

    python3 orchestrate_r3.py paths --commit <sha> --dirty 0 --ws /home/CAPYI_AI/wt/dwa-abl   # held-out D + O 20
    python3 orchestrate_r3.py ros <tuning|c1|c2|c3|o> --commit <sha> --dirty 0 [--lanes 14]
    python3 orchestrate_r3.py chain --commit <sha> --dirty 0 [--procs 24]

- 튜닝 (prereg_r3 §2·§3): Nav2 새 격자 DWB_D12–D35·MPPI_P12–P35·RPP_R12–R35 와 Z00–Z11 을 2 라운드 튜닝-20
  (/runs/r2/paths_tune, 지도 그 집합) × {잡음 없음, seed 1–3} + 튜닝 O 6 에서. 작업 = 같은 번호 k 의 새 DWB·MPPI·RPP
  (+ k < 12 이면 Z_k) × 경로 절반; O 는 장면 × k 셋 묶음.
- held-out D 팔 (prereg_r3 §4): 튜닝 선택 DWB·MPPI·RPP·DWA-R3 + 고정 D00·P00·R00·pp·dwa_full·dwa_R(X03)·
  dwa_R2(Y11)·dwa_textbook_tuned(T06). C3 에는 1·2 라운드처럼 dwa_x_X00 (원본 동치, 재설정 기록) 을 더한다.
"""
import argparse
import concurrent.futures
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import orchestrate_r2 as o2  # noqa: E402
import orchestrate_x as ox  # noqa: E402
import params_r3  # noqa: E402

o2.SETS['d'] = {'dir': '/runs/r3/paths_d', 'seed': 20261021, 'n': (32, 8, 20), 'o': 20}
o2.PATHS_LOGS = 'r3/paths_logs'
RUNS_HOST = ox.RUNS_HOST
FIXED = ['dwa_full', 'dwa_R', 'dwa_R2', 'dwa_textbook_tuned', 'pp', 'DWB_D00', 'MPPI_P00', 'RPP_R00']


def selection():
    with open(os.path.join(RUNS_HOST, 'r3', 'tuning', 'selection_r3.json'), encoding='utf-8') as f:
        return json.load(f)


def heldout_arms():
    sel = selection()
    arms = list(FIXED)
    arms += [sel[f]['chosen'] for f in ('DWB', 'MPPI', 'RPP') if sel[f]['chosen'] not in arms]
    return arms + ['dwa_R3'], sel['DWA_R3']['chosen_z']


def make_job(stage, jid, arms, ids, paths, est_s, set_key, seed=0, rep=0, largs=(),
             extra_bench=(), **pf_kw):
    sd = f'/runs/r3/{stage}'
    pf = os.path.join(ox.c2h(sd), 'params', f'{jid}.yaml')
    params_r3.ros_params_file(pf, arms, resets_prefix=f'{sd}/resets/{jid}', **pf_kw)
    for d in ('resets', 'traces', 'raw', 'logs'):
        os.makedirs(os.path.join(ox.c2h(sd), d), exist_ok=True)
    bench = ['--paths', ','.join(paths), '--ids', ','.join(ids), '--arms', ','.join(arms),
             '--tag', f'r3_{stage}', '--rep', str(rep), '--trace-dir', f'{sd}/traces/{jid}'] + \
        list(extra_bench)
    largs = list(largs)
    if seed:
        bench += ['--noise-seed', str(seed), '--odom-noise-v', '0.02', '--odom-noise-w', '0.02']
        largs += [f'noise_seed:={seed}', 'odom_noise_v:=0.02', 'odom_noise_w:=0.02']
    return {'id': jid, 'arms': list(arms), 'ids': list(ids), 'seed': seed, 'rep': rep,
            'params': pf, 'largs': largs, 'bench': bench, 'env': {'PR_MAP': o2.set_map(set_key)},
            'out': f'{sd}/raw/{jid}.jsonl', 'launch_log': f'{sd}/logs/{jid}.launch.log',
            'bench_log': f'{sd}/logs/{jid}.bench.log', 'timeout': 2.0 * est_s + 600.0}


def ros_stage(args):
    st = args.stage
    sdh = os.path.join(RUNS_HOST, 'r3', st)
    if os.path.exists(os.path.join(sdh, 'SEALED')):
        print(f'{sdh} 는 봉인됐다', file=sys.stderr)
        return 2
    for d in ('raw', 'logs', 'params'):
        os.makedirs(os.path.join(sdh, d), exist_ok=True)
    jobs = []
    if st == 'tuning':
        ent = []
        for p in o2.set_json('tune'):
            ent += o2.load_c(p)
        ids = [e['id'] for e in ent]
        halves = (ids[0::2], ids[1::2])
        dk, mk, rk = list(params_r3.DWB_NEW), list(params_r3.MPPI_NEW), list(params_r3.RPP_NEW)
        zk = list(params_r3.build_z())

        def arms_k(k):
            a = [f'DWB_{dk[k]}', f'MPPI_{mk[k]}', f'RPP_{rk[k]}']
            if k < len(zk):
                a.append(params_r3.z_arm(zk[k]))
            return a
        for seed in (0, 1, 2, 3):
            for k in range(24):
                arms = arms_k(k)
                for h, sub in enumerate(halves):
                    jobs.append(make_job(st, f's{seed}_k{k:02d}_h{h}', arms, sub, o2.set_json('tune'),
                                         ox.est_seconds(ent, sub, arms), 'tune', seed=seed))
        oj = f"{o2.SETS['tune']['dir']}/scenes_obstacles.json"
        scenes = o2.load_c(oj)
        for sc in scenes:
            for g in range(8):
                arms = [a for k in range(3 * g, 3 * g + 3) for a in arms_k(k)]
                jobs.append(make_job(st, f'O_{sc["id"]}_g{g}', arms, [sc['id']], [oj],
                                     ox.est_seconds(scenes, [sc['id']], arms), 'tune',
                                     largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                                     extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O']))
    elif st in ('c1', 'c2'):
        arms, zsel = heldout_arms()
        stateful = st == 'c2'
        ent = []
        for p in o2.set_json('d'):
            ent += o2.load_c(p)
        ids = [e['id'] for e in ent]
        for j in range(20):
            sub = ids[j::20]
            jobs.append(make_job(st, f'j{j:02d}', arms, sub, o2.set_json('d'),
                                 ox.est_seconds(ent, sub, arms), 'd',
                                 extra_bench=['--stateful', str(int(stateful)), '--condition', st.upper()],
                                 stateful=stateful, selected_z=zsel))
    elif st == 'c3':
        arms, zsel = heldout_arms()
        arms = arms + ['dwa_x_X00']
        ent = []
        for p in o2.set_json('d'):
            ent += o2.load_c(p)
        sm = [e['id'] for e in o2.load_c(o2.set_json('d')[0])][:6]
        ast = [e['id'] for e in o2.load_c(o2.set_json('d')[2]) if e['meta']['length'] < 20.0][:3]
        ids = sm + o2.NARROW_SUBSET + ast
        for seed in range(1, 6):
            for j in range(4):
                sub = ids[j::4]
                jobs.append(make_job(st, f's{seed}_j{j}', arms, sub, o2.set_json('d'),
                                     ox.est_seconds(ent, sub, arms), 'd', seed=seed,
                                     extra_bench=['--condition', 'C3'], selected_z=zsel))
    elif st == 'o':
        arms, zsel = heldout_arms()
        oj = f"{o2.SETS['d']['dir']}/scenes_obstacles.json"
        scenes = o2.load_c(oj)
        for rep in (1, 2, 3):
            for sc in scenes:
                jobs.append(make_job(st, f'{sc["id"]}_r{rep}', arms, [sc['id']], [oj],
                                     ox.est_seconds(scenes, [sc['id']], arms), 'd', rep=rep,
                                     largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                                     extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O'],
                                     selected_z=zsel))
    else:
        raise SystemExit(f'알 수 없는 단계 {st}')
    with open(os.path.join(sdh, 'jobs.json'), 'w', encoding='utf-8') as f:
        json.dump(jobs, f, indent=1)
    failed = ox.run_lanes(jobs, args, f'r3_{st}')
    print(json.dumps({'stage': f'r3_{st}', 'jobs': len(jobs), 'failed': failed}), flush=True)
    return 1 if failed else 0


def chain_calls():
    calls = []
    for arm in ('dwa_full', 'dwa_R', 'dwa_R2', 'dwa_R3', 'dwa_textbook_tuned'):
        for sc in o2.set_scenes('d'):
            calls.append((arm, sc, 0, 0))
            calls.append((arm, sc, 0, 1))
            for seed in range(1, 6):
                calls.append((arm, sc, seed, 0))
    return calls


def chain_call(args, arm, scenes, seed, stateful):
    fam = scenes.split('_')[-1][:-4]
    cond = 'clean' if seed == 0 else f's{seed}'
    cid = f'{arm}__{fam}__{cond}__st{stateful}'
    root = '/runs/r3/chain'
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
    return cid, {'id': f'r3_chain_{cid}', 'testbed': 'chain_sim_x', 'commit': args.commit,
                 'dirty': args.dirty,
                 'config': {'arm': arm, 'cfg_sha': ox.sha(cfg), 'args': cmd[3:], 'rc': r.returncode,
                            'stderr': r.stderr[-500:]},
                 'paths': os.path.basename(scenes), 'paths_sha': ox.sha(scenes), 'seed': seed,
                 'stateful': stateful, 'start': t0, 'end': time.time(), 'raw': out,
                 'result': ox.summarize(out, arm) if os.path.exists(out) else None,
                 'loadavg': [l0, os.getloadavg()]}


def chain_inner(args):
    root = '/runs/r3/chain'
    for d in ('raw', 'resets', 'traces', 'cfg'):
        os.makedirs(f'{root}/{d}', exist_ok=True)
    with open('/runs/r3/tuning/selection_r3.json', encoding='utf-8') as f:
        zsel = json.load(f)['DWA_R3']['chosen_z']
    if zsel == 'Y11':
        names = params_r3.write_cfgs(f'{root}/cfg', None)
        import shutil
        shutil.copyfile(f'{root}/cfg/dwa_R2.cfg', f'{root}/cfg/dwa_R3.cfg')
    else:
        names = params_r3.write_cfgs(f'{root}/cfg', zsel)
    print('cfg', names, flush=True)
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
            print(f'[r3 chain] {k + 1}/{len(calls)} {cid}', flush=True)
    bad = [e['id'] for e in entries if e['config']['rc'] != 0]
    print(json.dumps({'calls': len(calls), 'ran': len(entries), 'rc_nonzero': bad}), flush=True)
    return 1 if bad else 0


def chain_stage(args):
    root_h = os.path.join(RUNS_HOST, 'r3', 'chain')
    os.makedirs(root_h, exist_ok=True)
    ox.docker_rm_if_mine('amr-ai-pr-chain')
    fn = os.path.join(root_h, 'runs_chain.jsonl')
    n0 = ox.nlines(fn)
    cmd = ['docker', 'run', '--rm', '--name', 'amr-ai-pr-chain', '--cpus', str(args.procs),
           '--memory', '24g', '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro',
           '-v', f'{ox.WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs', ox.IMAGE,
           'python3', f'{ox.SCRIPTS_C}/orchestrate_r3.py', 'chain-inner', '--commit', args.commit,
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
    if len(sys.argv) > 1 and sys.argv[1] == 'paths':
        sys.argv += ['--sets', 'd']
        os.makedirs(os.path.join(RUNS_HOST, 'r3', 'paths_logs'), exist_ok=True)
        return o2.main()
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['ros', 'chain', 'chain-inner'])
    ap.add_argument('stage', nargs='?', default='')
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    ap.add_argument('--ws', default=ox.WS_HOST)
    ap.add_argument('--lanes', type=int, default=14)
    ap.add_argument('--lane0', type=int, default=0)
    ap.add_argument('--cpus', type=float, default=3.0)
    ap.add_argument('--memory', default='8g')
    ap.add_argument('--stagger', type=float, default=15.0)
    ap.add_argument('--procs', type=int, default=24)
    args = ap.parse_args()
    if args.cmd == 'ros':
        return ros_stage(args)
    if args.cmd == 'chain':
        return chain_stage(args)
    return chain_inner(args)


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""
paper_research 실행 조율 (prereg.md §3·§4·§5·§7, contract.md). 원본 dwa-ablation/scripts/orchestrate.py 의
사본에서 출발했고 (log_run·summarize·sha·nlines 는 그대로), 실행 방식을 바꿨다:

- **호스트에서 돈다.** ROS 단계는 레인(lane) n 마다 컨테이너 amr-ai-pr-ros-<n> 하나 (항상 --rm), 도메인
  70 + n, ROS_LOCALHOST_ONLY=1, 이미지 amr-ai/rl:0.3. 작업(job) 하나 = 시험대 한 벌 기동
  (ros_worker_x.sh) + ros_bench_x.py 한 번. 작업마다 원자료 파일이 따로라 동시 쓰기가 없다.
  작업공간은 읽기 전용으로 붙인다: /ros2_ws = 빌드된 작업공간, /pr_ws = paper_research worktree (스크립트),
  /runs = /home/CAPYI_AI/paper_research/runs.
- chain_sim 단계는 컨테이너 amr-ai-pr-chain 하나 안에서 chain_sim_x 를 최대 --procs 개 동시에 돌린다
  (`chain-inner`). 실행마다 재설정 기록(reset_log=)과 궤적(trace_dir=)을 남긴다.
- 실행 기록: docs/research/paper-research/runs.jsonl (작업·호출마다 한 줄: commit·dirty·loadavg·seed·설정 해시).
  ROS 원자료에는 실행마다 commit·dirty·loadavg·cmd_rate_hz·seed·rep 가 함께 있다 (ros_bench_x.py).
- 재개: 같은 명령을 다시 부르면 끝난 (경로, 팔, seed, rep) 는 건너뛴다 (--skip-done 이 기본).

    python3 orchestrate_x.py paths   --commit <sha> --dirty <n> --ws /home/CAPYI_AI/wt/dwa-abl
    python3 orchestrate_x.py ros <dryrun|equiv|tuning|c1|c2|c3|o> --commit <sha> --dirty <n> [--lanes 10]
    python3 analyze_x.py select --raw /runs/tuning/raw > /runs/tuning/selection.json   (선택 J')
    python3 orchestrate_x.py chain   --commit <sha> --dirty <n> [--procs 24]

단계별 팔·경로·조건은 prereg 그대로다 (ros_stage 의 단계별 주석).
- equiv (prereg §3.3): dwa_full 대 dwa_x_X00, 튜닝 4 경로, 실시간 반복 5 회 (판정은 analyze_x.py equiv).
- tuning (prereg §4): 작업 = (조건 s ∈ {잡음 없음, seed 1–3}, 번호 k) 의 DWB_k·MPPI_k·RPP_k·X_k 넷.
- c1·c2·c3·o (prereg §5): 팔 = dwa_full·dwa_textbook·pp·DWB_D00·MPPI_P00·RPP_R00 + 선택 셋 + dwa_R.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
import subprocess
import sys
import time

import params_x as params

HERE = os.path.dirname(os.path.abspath(__file__))
PR_DIR = os.path.abspath(os.path.join(HERE, '..'))
WS_HOST = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))
RUNS_JSONL = os.path.join(PR_DIR, 'runs.jsonl')
RUNS_HOST = '/home/CAPYI_AI/paper_research/runs'
IMAGE = 'amr-ai/rl:0.3'
SCRIPTS_C = '/pr_ws/docs/research/paper-research/scripts'
P = '/runs/paths'
TUNING_JSON = f'{P}/tuning.json'
HELDOUT_JSON = [f'{P}/heldout_smooth.json', f'{P}/heldout_narrow.json', f'{P}/heldout_astar.json']
HELDOUT_SCENES = [f'{P}/scenes_heldout_smooth.txt', f'{P}/scenes_heldout_narrow.txt',
                  f'{P}/scenes_heldout_astar.txt']
OBST_JSON = f'{P}/scenes_obstacles.json'
CHAIN_X = '/ros2_ws/install/amr_nav_research/lib/amr_nav_research/chain_sim_x'
NOISE_CHAIN = ['odom_noise_v=0.02', 'odom_noise_w=0.02', 'pose_noise_xy=0.01',
               'pose_noise_yaw=0.005', 'pose_noise_tau=1.0', 'lidar=1']   # 원본 chain 잡음 조건
NARROW_SUBSET = ['N1', 'N4', 'N6']   # 원본 잡음 부분집합 규칙 (dwa-ablation orchestrate.py NOISE_IDS)
BASE_ARMS = ['dwa_full', 'dwa_textbook', 'pp', 'DWB_D00', 'MPPI_P00', 'RPP_R00']
DOMAIN0 = 70


def h2c(path):
    return path.replace(RUNS_HOST, '/runs', 1) if path.startswith(RUNS_HOST) else path


def c2h(path):
    return path.replace('/runs', RUNS_HOST, 1) if path.startswith('/runs') else path


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
            if i >= since and line.strip():
                rs.append(json.loads(line))
    if arm:
        rs = [r for r in rs if r['arm'] == arm]
    ok = sum(r['success'] for r in rs)
    ts = sorted(r['t_reach'] for r in rs if r['success'])
    out = {'n': len(rs), 'success': ok, 'median_t_reach': ts[len(ts) // 2] if ts else None}
    rates = [r['cmd_rate_hz'] for r in rs if 'cmd_rate_hz' in r]
    if rates:
        out['cmd_rate_min'] = min(rates)
        out['rate_low'] = sum(1 for v in rates if v < 18.0)
    return out


def nlines(fn):
    if not os.path.exists(fn):
        return 0
    with open(fn, encoding='utf-8') as f:
        return sum(1 for line in f if line.strip())


def load_json(path_c):
    with open(c2h(path_c), encoding='utf-8') as f:
        return json.load(f)


def heldout_entries():
    out = []
    for f in HELDOUT_JSON:
        out += load_json(f)
    return out


def noise_subset():
    """C3 부분집합 (prereg §5): 매끈 앞 6, 통로 N1·N4·N6 (원본 규칙), 길이 20 m 미만 A* 앞 3."""
    sm = [e['id'] for e in load_json(HELDOUT_JSON[0])][:6]
    ast = [e['id'] for e in load_json(HELDOUT_JSON[2]) if e['meta']['length'] < 20.0][:3]
    return sm + NARROW_SUBSET + ast


def selection():
    with open(os.path.join(RUNS_HOST, 'tuning', 'selection.json'), encoding='utf-8') as f:
        return json.load(f)


def heldout_arms():
    """ROS held-out 팔 (prereg §3.4): 기존 비교군 + 튜닝 선택 셋 + DWA-R."""
    sel = selection()
    arms = list(BASE_ARMS)
    arms += [sel[f]['chosen'] for f in ('DWB', 'MPPI', 'RPP') if sel[f]['chosen'] not in arms]
    return arms + ['dwa_R'], sel['DWA_R']['chosen_x']


# ---------------------------------------------------------------------------------------------
# ROS 레인

def docker_rm_if_mine(name):
    """이 하네스가 띄운 이름(amr-ai-pr-ros-<n>, amr-ai-pr-chain, amr-ai-pr-paths)만 정리한다."""
    assert name.startswith(('amr-ai-pr-ros-', 'amr-ai-pr-chain', 'amr-ai-pr-paths'))
    r = subprocess.run(['docker', 'ps', '-aq', '--filter', f'name=^{name}$'],
                       capture_output=True, text=True)
    if r.stdout.strip():
        subprocess.run(['docker', 'rm', '-f', name], capture_output=True)


def lane_cmd(lane, job, args):
    dom = DOMAIN0 + lane
    ns = f'amr_{dom}'
    cname = f'amr-ai-pr-ros-{lane}'
    bench = ['python3', f'{SCRIPTS_C}/ros_bench_x.py'] + job['bench'] + [
        '--out', job['out'], '--worker', ns, '--ns', '/@NS@', '--skip-done',
        '--commit', args.commit, '--dirty', str(args.dirty),
        '--ros-args', '-r', '__ns:=/@NS@', '--log-level', 'warn']
    cmd = ['docker', 'run', '--rm', '--name', cname, '--cpus', str(args.cpus),
           '--memory', args.memory, '--shm-size', '2g',
           '-e', f'ROS_DOMAIN_ID={dom}', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'ROS_WS=/ros2_ws',
           '-v', f'{args.ws}:/ros2_ws:ro', '-v', f'{WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs',
           IMAGE, 'bash', f'{SCRIPTS_C}/ros_worker_x.sh', str(dom), ns, h2c(job['params']),
           job['launch_log']] + job['largs'] + ['--'] + bench
    return cmd, cname, dom


def run_lanes(jobs, args, stage):
    """작업을 레인에 나눠 돈다. 레인 하나에 동시에 작업 하나. 실패한 작업은 기록하고 넘어간다."""
    pending = [j for j in jobs if not job_done(j)]
    print(f'[{stage}] 작업 {len(jobs)} 개 중 남은 것 {len(pending)} 개, 레인 {args.lanes}', flush=True)
    active = {}
    last_start = 0.0
    failed = []
    while pending or active:
        for lane in range(args.lanes):
            if lane in active or not pending or time.time() - last_start < args.stagger:
                continue
            job = pending.pop(0)
            cmd, cname, dom = lane_cmd(lane, job, args)
            docker_rm_if_mine(cname)
            os.makedirs(os.path.dirname(c2h(job['out'])), exist_ok=True)
            blog = open(c2h(job['bench_log']), 'a', encoding='utf-8')
            n0 = nlines(c2h(job['out']))
            p = subprocess.Popen(cmd, stdout=blog, stderr=subprocess.STDOUT)
            active[lane] = (p, job, time.time(), os.getloadavg(), n0, blog, cname, dom)
            last_start = time.time()
            print(f'[{stage}] lane {lane} 시작 {job["id"]}', flush=True)
        for lane in list(active):
            p, job, t0, l0, n0, blog, cname, dom = active[lane]
            rc = p.poll()
            if rc is None and time.time() - t0 > job['timeout']:
                subprocess.run(['docker', 'stop', '-t', '10', cname], capture_output=True)
                rc = p.wait()
                rc = rc if rc else -9
            if rc is None:
                continue
            blog.close()
            out_h = c2h(job['out'])
            res = summarize(out_h, None, n0) if os.path.exists(out_h) else None
            log_run({'id': f'{stage}_{job["id"]}', 'testbed': 'ros_kinematic',
                     'commit': args.commit, 'dirty': args.dirty,
                     'config': {'arms': job['arms'], 'params_sha': sha(job['params']),
                                'domain': dom, 'container': cname, 'largs': job['largs'],
                                'rc': rc, 'ws': args.ws},
                     'paths': job['ids'], 'seed': job.get('seed', 0), 'rep': job.get('rep', 0),
                     'start': t0, 'end': time.time(), 'raw': job['out'], 'result': res,
                     'loadavg': [l0, os.getloadavg()]})
            done = job_done(job)
            print(f'[{stage}] lane {lane} 끝 {job["id"]} rc={rc} 완료={done} {res}', flush=True)
            if not done:
                failed.append(job['id'])
            del active[lane]
        time.sleep(2.0)
    return failed


def job_done(job):
    out = c2h(job['out'])
    if not os.path.exists(out):
        return False
    have = set()
    with open(out, encoding='utf-8') as f:
        for line in f:
            if line.strip():
                d = json.loads(line)
                have.add((d['scene'], d['arm'], d.get('seed', 0), d.get('rep', 0)))
    need = {(i, a, job.get('seed', 0), job.get('rep', 0)) for i in job['ids'] for a in job['arms']}
    return need <= have


def make_job(stage_dir, jid, arms, ids, paths, params_file, est_s, seed=0, rep=0, largs=(),
             extra_bench=()):
    bench = ['--paths', ','.join(paths), '--ids', ','.join(ids), '--arms', ','.join(arms),
             '--tag', os.path.basename(stage_dir), '--rep', str(rep)] + list(extra_bench)
    if seed:
        bench += ['--noise-seed', str(seed), '--odom-noise-v', '0.02', '--odom-noise-w', '0.02']
        largs = list(largs) + [f'noise_seed:={seed}', 'odom_noise_v:=0.02', 'odom_noise_w:=0.02']
    return {'id': jid, 'arms': list(arms), 'ids': list(ids), 'seed': seed, 'rep': rep,
            'params': params_file, 'largs': list(largs), 'bench': bench,
            'out': f'/runs/{os.path.basename(stage_dir)}/raw/{jid}.jsonl',
            'launch_log': f'/runs/{os.path.basename(stage_dir)}/logs/{jid}.launch.log',
            'bench_log': f'/runs/{os.path.basename(stage_dir)}/logs/{jid}.bench.log',
            'timeout': 2.0 * est_s + 600.0}


def est_seconds(entries, ids, arms):
    """작업 시간 상한 추정: 실행마다 시간 상한 max(60, 3·T_pred + 30) 의 절반 + 15 s."""
    t = 0.0
    for e in entries:
        if e['id'] in ids:
            L = sum(((e['points'][i + 1][0] - e['points'][i][0]) ** 2 +
                     (e['points'][i + 1][1] - e['points'][i][1]) ** 2) ** 0.5
                    for i in range(len(e['points']) - 1))
            t += len(arms) * (0.5 * max(60.0, 3.0 * (L + 1.5) + 30.0) + 15.0)
    return t + 120.0


def ros_stage(args):
    st = args.stage
    stage_dir = os.path.join(RUNS_HOST, st)
    for d in ('raw', 'logs', 'params'):
        os.makedirs(os.path.join(stage_dir, d), exist_ok=True)
    if os.path.exists(os.path.join(stage_dir, 'SEALED')):
        print(f'{stage_dir} 는 봉인됐다', file=sys.stderr)
        return 2
    jobs = []
    if st == 'dryrun':
        # 원본 팔로 튜닝 4 경로, 잡음 없음 1 회: 사본 ros_bench_x 가 원본 튜닝 수치를 재현하는지 (하네스 확인)
        tun = load_json(TUNING_JSON)
        ids = [e['id'] for e in tun]
        groups = [[f'DWB_{d}', f'MPPI_{m}', f'RPP_{r}'] for d, m, r in
                  zip(params.DWB_GRID, params.MPPI_GRID, params.RPP_GRID)]
        groups.append(['dwa_full', 'pp', 'dwa_textbook'])
        for k, arms in enumerate(groups):
            pf = os.path.join(stage_dir, 'params', f'g{k:02d}.yaml')
            params.ros_params_file(pf, arms)
            jobs.append(make_job(stage_dir, f'g{k:02d}', arms, ids, [TUNING_JSON], pf,
                                 est_seconds(tun, ids, arms)))
    elif st == 'equiv':
        # prereg §3.3: 플러그인 X(스위치 끔) 대 원본 dwa_full, 튜닝 4 경로, 실시간 반복 5 회
        tun = load_json(TUNING_JSON)
        ids = [e['id'] for e in tun]
        arms = ['dwa_full', 'dwa_x_X00']
        pf = os.path.join(stage_dir, 'params', 'equiv.yaml')
        params.ros_params_file(pf, arms)
        for rep in range(1, args.reps + 1):
            jobs.append(make_job(stage_dir, f'rep{rep}', arms, ids, [TUNING_JSON], pf,
                                 est_seconds(tun, ids, arms), rep=rep))
    elif st == 'tuning':
        # prereg §4: 팔마다 12 설정 × 튜닝 4 경로 × (잡음 없음, seed 1–3). 작업 하나 = 같은 번호 k 의
        # DWB·MPPI·RPP·X 설정 넷 (제어기 종류가 레인에 고르게 섞이게)
        tun = load_json(TUNING_JSON)
        ids = [e['id'] for e in tun]
        for seed in (0, 1, 2, 3):
            for k, (d, m, r, x) in enumerate(zip(params.DWB_GRID, params.MPPI_GRID,
                                                 params.RPP_GRID, params.DWA_X_GRID)):
                arms = [f'DWB_{d}', f'MPPI_{m}', f'RPP_{r}', params.x_arm(x)]
                pf = os.path.join(stage_dir, 'params', f'k{k:02d}.yaml')
                if seed == 0:
                    params.ros_params_file(pf, arms)
                jobs.append(make_job(stage_dir, f's{seed}_k{k:02d}', arms, ids, [TUNING_JSON], pf,
                                     est_seconds(tun, ids, arms), seed=seed))
    elif st in ('c1', 'c2'):
        arms, xsel = heldout_arms()
        stateful = st == 'c2'
        pf = os.path.join(stage_dir, 'params', f'{st}.yaml')
        params.ros_params_file(pf, arms, stateful=stateful, selected_x=xsel)
        ent = heldout_entries()
        ids = [e['id'] for e in ent]
        nj = args.jobs or 20
        for j in range(nj):
            sub = ids[j::nj]
            jobs.append(make_job(stage_dir, f'j{j:02d}', arms, sub, HELDOUT_JSON, pf,
                                 est_seconds(ent, sub, arms),
                                 extra_bench=['--stateful', str(int(stateful)),
                                              '--condition', st.upper()]))
    elif st == 'c3':
        arms, xsel = heldout_arms()
        pf = os.path.join(stage_dir, 'params', 'c3.yaml')
        params.ros_params_file(pf, arms, selected_x=xsel)
        ent = heldout_entries()
        ids = noise_subset()
        for seed in range(1, 6):
            for j in range(4):
                sub = ids[j::4]
                jobs.append(make_job(stage_dir, f's{seed}_j{j}', arms, sub, HELDOUT_JSON, pf,
                                     est_seconds(ent, sub, arms), seed=seed,
                                     extra_bench=['--condition', 'C3']))
    elif st == 'o':
        arms, xsel = heldout_arms()
        pf = os.path.join(stage_dir, 'params', 'o.yaml')
        params.ros_params_file(pf, arms, selected_x=xsel)
        scenes = load_json(OBST_JSON)
        for rep in (1, 2, 3):
            for sc in scenes:
                jobs.append(make_job(
                    stage_dir, f'{sc["id"]}_r{rep}', arms, [sc['id']], [OBST_JSON], pf,
                    est_seconds(scenes, [sc['id']], arms), rep=rep,
                    largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                    extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O']))
    else:
        raise SystemExit(f'알 수 없는 단계 {st}')
    with open(os.path.join(stage_dir, 'jobs.json'), 'w', encoding='utf-8') as f:
        json.dump(jobs, f, indent=1)
    failed = run_lanes(jobs, args, st)
    print(json.dumps({'stage': st, 'jobs': len(jobs), 'failed': failed}), flush=True)
    return 1 if failed else 0


# ---------------------------------------------------------------------------------------------
# 경로 생성 (held-out B, O 장면)

def paths_stage(args):
    out_h = os.path.join(RUNS_HOST, 'paths')
    if os.path.exists(os.path.join(out_h, 'SEALED')):
        print('paths 는 봉인됐다', file=sys.stderr)
        return 2
    os.makedirs(os.path.join(RUNS_HOST, 'paths_logs'), exist_ok=True)
    base = ['docker', 'run', '--rm', '--name', 'amr-ai-pr-paths', '--cpus', '4', '--memory', '8g',
            '--shm-size', '2g', '-e', f'ROS_DOMAIN_ID={DOMAIN0}', '-e', 'ROS_LOCALHOST_ONLY=1',
            '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro', '-v', f'{WS_HOST}:/pr_ws:ro',
            '-v', f'{RUNS_HOST}:/runs', IMAGE]
    steps = [
        ('make_paths', ['python3', f'{SCRIPTS_C}/make_paths_x.py', '--out', P, '--seed',
                        str(args.seed)]),
        ('plan_astar', ['bash', f'{SCRIPTS_C}/ros_worker_x.sh', str(DOMAIN0), f'amr_{DOMAIN0}',
                        '/ros2_ws/install/amr_navigation/share/amr_navigation/config/nav2_params.yaml',
                        '/runs/paths_logs/launch_plan.log', '--',
                        'python3', f'{SCRIPTS_C}/plan_astar_x.py', '--pairs', f'{P}/astar_pairs.json',
                        '--out', f'{P}/heldout_astar.json', '--n', '20',
                        '--ros-args', '-r', '__ns:=/@NS@']),
        ('astar_scenes', ['python3', f'{SCRIPTS_C}/make_paths_x.py', '--out', P, '--astar-scenes']),
        ('obstacles', ['python3', f'{SCRIPTS_C}/scenes_obstacles.py', '--paths', P, '--seed',
                       str(args.seed)]),
    ]
    for name, cmd in steps:
        docker_rm_if_mine('amr-ai-pr-paths')
        t0 = time.time()
        l0 = os.getloadavg()
        with open(os.path.join(RUNS_HOST, 'paths_logs', f'{name}.log'), 'w',
                  encoding='utf-8') as lf:
            rc = subprocess.run(base + cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
        log_run({'id': f'paths_{name}', 'testbed': 'paths', 'commit': args.commit,
                 'dirty': args.dirty, 'config': {'cmd': cmd, 'ws': args.ws, 'rc': rc},
                 'seed': args.seed, 'start': t0, 'end': time.time(), 'raw': P,
                 'loadavg': [l0, os.getloadavg()]})
        print(name, rc, flush=True)
        if rc != 0:
            return rc
    return 0


# ---------------------------------------------------------------------------------------------
# chain_sim (컨테이너 안에서 돈다)

def chain_plan(sel_x):
    """
    prereg §5 chain_sim 팔·조건: dwa_full, R1–R4 단독(X02·X04·X05·X06), DWA-R(dwa_R) ×
    {잡음 없음, 잡음 seed 1–5} × stateful {0, 1} × held-out B 60 경로; dwa_no_m3 는 잡음 없음 × stateful {0, 1}.
    """
    arms = ['dwa_full'] + [params.x_arm(params.CHAIN_SINGLE[k]) for k in ('R1', 'R2', 'R3', 'R4')]
    arms.append('dwa_R')
    calls = []
    for stateful in (0, 1):
        for seed in (0, 1, 2, 3, 4, 5):
            for arm in arms:
                for sc in HELDOUT_SCENES:
                    calls.append((arm, sc, seed, stateful))
        for sc in HELDOUT_SCENES:
            calls.append(('dwa_no_m3', sc, 0, stateful))
    return calls


def chain_call(args, arm, scenes, seed, stateful):
    fam = scenes.split('_')[-1][:-4]
    cond = 'clean' if seed == 0 else f's{seed}'
    cid = f'{arm}__{fam}__{cond}__st{stateful}'
    root = '/runs/chain'
    out = f'{root}/raw/{cid}.jsonl'
    rlog = f'{root}/resets/{cid}.csv'
    tdir = f'{root}/traces/{cid}'
    os.makedirs(tdir, exist_ok=True)
    cfg = f'{root}/cfg/{arm}.cfg'
    n_exp = sum(1 for line in open(scenes, encoding='utf-8') if line.startswith('scene '))
    if nlines(out) >= n_exp:
        return cid, 'skip'
    if os.path.exists(out):
        os.remove(out)          # 끊긴 호출은 처음부터 (한 호출 = 한 경로 묶음, 결정적)
    cmd = [CHAIN_X, scenes, out, f'cfg={cfg}', f'arm={arm}', f'stateful={stateful}',
           f'reset_log={rlog}', f'trace_dir={tdir}']
    if seed:
        cmd += NOISE_CHAIN + [f'seed={seed}']
    t0 = time.time()
    l0 = os.getloadavg()
    r = subprocess.run(cmd, capture_output=True, text=True)
    entry = {'id': f'chain_{cid}', 'testbed': 'chain_sim_x', 'commit': args.commit,
             'dirty': args.dirty,
             'config': {'arm': arm, 'cfg_sha': sha(cfg), 'args': cmd[3:], 'rc': r.returncode,
                        'stderr': r.stderr[-500:]},
             'paths': os.path.basename(scenes), 'paths_sha': sha(scenes), 'seed': seed,
             'stateful': stateful, 'start': t0, 'end': time.time(), 'raw': out,
             'result': summarize(out, arm) if os.path.exists(out) else None,
             'loadavg': [l0, os.getloadavg()]}
    return cid, entry


def chain_inner(args):
    root = '/runs/chain'
    for d in ('raw', 'resets', 'traces', 'cfg'):
        os.makedirs(f'{root}/{d}', exist_ok=True)
    with open('/runs/tuning/selection.json', encoding='utf-8') as f:
        sel_x = json.load(f)['DWA_R']['chosen_x']
    print('cfg', params.write_cfgs(f'{root}/cfg', sel_x), flush=True)
    calls = chain_plan(sel_x)
    # 느린 호출(잡음 = LiDAR 코스트맵)을 먼저
    calls.sort(key=lambda c: (c[2] == 0, c[1]))
    entries = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.procs) as ex:
        futs = [ex.submit(chain_call, args, *c) for c in calls]
        for k, fu in enumerate(concurrent.futures.as_completed(futs)):
            cid, entry = fu.result()
            if entry != 'skip':
                entries.append(entry)
                with open(f'{root}/runs_chain.jsonl', 'a', encoding='utf-8') as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + '\n')
            print(f'[chain] {k + 1}/{len(calls)} {cid} '
                  f'{"skip" if entry == "skip" else entry["result"]}', flush=True)
    bad = [e['id'] for e in entries if e['config']['rc'] != 0]
    print(json.dumps({'calls': len(calls), 'ran': len(entries), 'rc_nonzero': bad}), flush=True)
    return 1 if bad else 0


def chain_stage(args):
    """호스트: 컨테이너 amr-ai-pr-chain 을 띄워 chain-inner 를 돌리고, 호출 기록을 runs.jsonl 로 옮긴다."""
    root_h = os.path.join(RUNS_HOST, 'chain')
    os.makedirs(root_h, exist_ok=True)
    docker_rm_if_mine('amr-ai-pr-chain')
    n0 = nlines(os.path.join(root_h, 'runs_chain.jsonl'))
    cmd = ['docker', 'run', '--rm', '--name', 'amr-ai-pr-chain', '--cpus', str(args.procs),
           '--memory', '24g', '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro',
           '-v', f'{WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs', IMAGE,
           'python3', f'{SCRIPTS_C}/orchestrate_x.py', 'chain-inner', '--commit', args.commit,
           '--dirty', str(args.dirty), '--procs', str(args.procs)]
    with open(os.path.join(root_h, 'chain_stage.log'), 'a', encoding='utf-8') as lf:
        rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
    fn = os.path.join(root_h, 'runs_chain.jsonl')
    if os.path.exists(fn):
        with open(fn, encoding='utf-8') as f:
            for i, line in enumerate(f):
                if i >= n0 and line.strip():
                    log_run(json.loads(line))
    return rc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['paths', 'ros', 'chain', 'chain-inner'])
    ap.add_argument('stage', nargs='?', default='')
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    ap.add_argument('--ws', default=WS_HOST, help='/ros2_ws 로 붙일 빌드된 작업공간 (호스트 경로)')
    ap.add_argument('--lanes', type=int, default=10)
    ap.add_argument('--cpus', type=float, default=3.0, help='레인 컨테이너 하나의 CPU 상한')
    ap.add_argument('--memory', default='8g')
    ap.add_argument('--stagger', type=float, default=15.0, help='작업 시작 간격 [s] (기동 부하 분산)')
    ap.add_argument('--jobs', type=int, default=0)
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--procs', type=int, default=24)
    ap.add_argument('--seed', type=int, default=20261018)
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

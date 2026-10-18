#!/usr/bin/env python3
"""
R3-C 실행 조율 (prereg_r3c.md §3·부록 A). 1–3 라운드 하네스(orchestrate_x·r2·r3, params_*)를 불러 쓰고,
기존 파일은 고치지 않는다. 원자료 /home/CAPYI_AI/paper_research/runs/r3c/<단계>/, 실행 기록은
docs/research/paper-research/runs_r3c.jsonl (pr-harness 의 runs.jsonl 에는 쓰지 않는다).

    python3 orchestrate_r3c.py ros <단계[,단계…]> --commit <sha> --dirty <n> [--ws <빌드>]
        e2      X00 (dwa_x_X00, 배포 규칙 동치, 재설정 기록 켬) × held-out D C1 60
        c1      E1 변형 + E3 dwa_R3 × held-out D C1 60 (잡음 없음, stateful false)   — 같은 작업에 함께
        c3      같은 팔 × C3 12 경로 × seed 1–5
        o       같은 팔 × O 20 장면 × 반복 3
        e4eq    E4 동치: 튜닝-20 매끈 앞 4 경로 × 5 팔 (감쌈·안 감쌈) × 반복 3 (빌드 ws_r3c)
        e4      E4 측정: held-out D C1 60 × 감싼 5 팔 (레인 2 개, 레인마다 물리 코어 4 개 cpuset, 빌드 ws_r3c)
    python3 orchestrate_r3c.py chain --commit <sha> --dirty <n> [--procs 8]   # E1 chain (컨테이너 amr-ai-pr-chain-r3c)

레인 (부록 A): 기본 14–19 (도메인 84–89, 컨테이너 amr-ai-pr-ros-14…19, --cpus 3 --memory 8g).
- 확장: R3 held-out 네 단계(runs/r3/{c1,c2,c3,o}/SEALED)가 모두 봉인되고 amr-ai-pr-ros-0…13 이 하나도 돌지
  않을 때(한 번 확인하면 고정) 레인 0–13 도 쓴다 (--expand).
- R3 감시: runs/r3/*/orchestrate.log 에 이 실행이 시작된 뒤 끝난 R3 작업 가운데 cmd_rate_min < 19.5 또는
  rate_low > 0 이 있으면 새 작업 기동을 멈추고 일지에 적는다. 그 뒤 R3 작업 3 개가 연달아 ≥ 19.7 Hz·rate_low 0
  으로 끝나거나, R3 레인 컨테이너(0–13)가 2 분 넘게 하나도 없으면 다시 기동한다.
"""
import argparse
import ast
import concurrent.futures
import glob
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import orchestrate_r3 as o3  # noqa: E402,F401  (o2.SETS['d'] = held-out D 를 등록한다)
import orchestrate_r2 as o2  # noqa: E402
import orchestrate_x as ox  # noqa: E402
import params_r3 as p3  # noqa: E402
import params_r3c as pc  # noqa: E402

ox.RUNS_JSONL = os.path.join(ox.PR_DIR, 'runs_r3c.jsonl')
RUNS_HOST = ox.RUNS_HOST
R3C = '/runs/r3c'
WS_R3C = '/home/CAPYI_AI/paper_research/ws_r3c'
JLOG = '/home/CAPYI_AI/paper_research/bin/jlog'
LANES_BASE = list(range(14, 20))
LANES_EXT = list(range(0, 14))
E4_CPUSETS = {14: '0-3,16-19', 15: '4-7,20-23'}     # 레인마다 물리 코어 4 개 (SMT 짝 포함 논리 8 개)
GUARD_TRIP, GUARD_OK = 19.5, 19.7


def jlog(msg):
    subprocess.run([JLOG, 'pr-r3c', msg], check=False)


def selection():
    sel = pc.committed_selection()
    if sel is None:
        raise SystemExit('tuning_selection_r3.json 이 아직 커밋되지 않았다')
    return sel


def e4_arms(sel):
    return ['dwa_R3', sel['DWB']['chosen'], sel['MPPI']['chosen'], sel['RPP']['chosen'], 'dwa_textbook_tuned']


# ---------------------------------------------------------------------------------------------
# 작업

def make_job(stage, jid, arms, ids, paths, est_s, set_key, zsel=None, seed=0, rep=0, largs=(),
             extra_bench=(), ws=None, bench='ros_bench_x.py', timed=None, **pf_kw):
    sd = f'{R3C}/{stage}'
    pf = os.path.join(ox.c2h(sd), 'params', f'{jid}.yaml')
    for d in ('resets', 'traces', 'raw', 'logs', 'params', 'timing'):
        os.makedirs(os.path.join(ox.c2h(sd), d), exist_ok=True)
    if zsel is None:
        p3.ros_params_file(pf, arms, resets_prefix=f'{sd}/resets/{jid}', **pf_kw)
    else:
        tm = None
        if timed:
            tm = {t: (inner, f'{sd}/timing/{jid}__{t}.csv') for t, inner in timed.items()}
        pc.ros_params_file(pf, arms, zsel, timed=tm, resets_prefix=f'{sd}/resets/{jid}', **pf_kw)
    bl = ['--paths', ','.join(paths), '--ids', ','.join(ids), '--arms', ','.join(arms),
          '--tag', f'r3c_{stage}', '--rep', str(rep), '--trace-dir', f'{sd}/traces/{jid}'] + list(extra_bench)
    largs = list(largs)
    if seed:
        bl += ['--noise-seed', str(seed), '--odom-noise-v', '0.02', '--odom-noise-w', '0.02']
        largs += [f'noise_seed:={seed}', 'odom_noise_v:=0.02', 'odom_noise_w:=0.02']
    return {'id': jid, 'stage': stage, 'arms': list(arms), 'ids': list(ids), 'seed': seed, 'rep': rep,
            'params': pf, 'largs': largs, 'bench': bl, 'bench_script': bench, 'ws': ws or ox.WS_HOST,
            'env': {'PR_MAP': o2.set_map(set_key)}, 'est': est_s,
            'out': f'{sd}/raw/{jid}.jsonl', 'launch_log': f'{sd}/logs/{jid}.launch.log',
            'bench_log': f'{sd}/logs/{jid}.bench.log', 'timeout': 2.0 * est_s + 600.0}


def d_entries():
    ent = []
    for p in o2.set_json('d'):
        ent += o2.load_c(p)
    return ent


def stage_jobs(st, args):
    jobs = []
    if st == 'e2':
        ent = d_entries()
        ids = [e['id'] for e in ent]
        arms = ['dwa_x_X00']
        for j in range(6):
            sub = ids[j::6]
            jobs.append(make_job(st, f'j{j:02d}', arms, sub, o2.set_json('d'), ox.est_seconds(ent, sub, arms),
                                 'd', extra_bench=['--stateful', '0', '--condition', 'C1'], stateful=False))
        return jobs
    sel = selection()
    zsel = sel['DWA_R3']['chosen_z']
    if st in ('c1', 'c3', 'o'):
        arms = ['dwa_R3'] + pc.run_arms(zsel)
        if st == 'c1':
            ent = d_entries()
            ids = [e['id'] for e in ent]
            for j in range(20):
                sub = ids[j::20]
                jobs.append(make_job(st, f'j{j:02d}', arms, sub, o2.set_json('d'), ox.est_seconds(ent, sub, arms),
                                     'd', zsel=zsel, extra_bench=['--stateful', '0', '--condition', 'C1'],
                                     stateful=False))
        elif st == 'c3':
            ent = d_entries()
            sm = [e['id'] for e in o2.load_c(o2.set_json('d')[0])][:6]
            ast_ = [e['id'] for e in o2.load_c(o2.set_json('d')[2]) if e['meta']['length'] < 20.0][:3]
            ids = sm + o2.NARROW_SUBSET + ast_
            for seed in range(1, 6):
                for j in range(4):
                    sub = ids[j::4]
                    jobs.append(make_job(st, f's{seed}_j{j}', arms, sub, o2.set_json('d'),
                                         ox.est_seconds(ent, sub, arms), 'd', zsel=zsel, seed=seed,
                                         extra_bench=['--condition', 'C3']))
        else:
            oj = f"{o2.SETS['d']['dir']}/scenes_obstacles.json"
            scenes = o2.load_c(oj)
            for rep in (1, 2, 3):
                for sc in scenes:
                    jobs.append(make_job(st, f'{sc["id"]}_r{rep}', arms, [sc['id']], [oj],
                                         ox.est_seconds(scenes, [sc['id']], arms), 'd', zsel=zsel, rep=rep,
                                         largs=[f'moving_obstacles:={sc["moving_obstacles"]}'],
                                         extra_bench=['--obstacles', sc['bench_obstacles'], '--condition', 'O']))
        return jobs
    if st == 'e4eq':
        arms5 = e4_arms(sel)
        timed = {f'tm_{a}': a for a in arms5}
        arms = arms5 + list(timed)
        ent = o2.load_c(o2.set_json('tune')[0])          # 튜닝-20 매끈 (held-out 아님)
        ids = [e['id'] for e in ent][:4]
        for rep in (1, 2, 3):
            for h, sub in enumerate((ids[0:2], ids[2:4])):
                jobs.append(make_job(st, f'r{rep}_h{h}', arms, sub, [o2.set_json('tune')[0]],
                                     ox.est_seconds(ent, sub, arms), 'tune', zsel=zsel, rep=rep,
                                     ws=WS_R3C, bench='ros_bench_r3c.py', timed=timed,
                                     extra_bench=['--stateful', '0', '--condition', 'E4EQ'], stateful=False))
        return jobs
    if st == 'e4':
        arms5 = e4_arms(sel)
        timed = {f'tm_{a}': a for a in arms5}
        arms = list(timed)
        ent = d_entries()
        ids = [e['id'] for e in ent]
        for j in range(10):
            sub = ids[j::10]
            jobs.append(make_job(st, f'j{j:02d}', arms, sub, o2.set_json('d'), ox.est_seconds(ent, sub, arms),
                                 'd', zsel=zsel, ws=WS_R3C, bench='ros_bench_r3c.py', timed=timed,
                                 extra_bench=['--stateful', '0', '--condition', 'C1'], stateful=False))
        return jobs
    raise SystemExit(f'알 수 없는 단계 {st}')


# ---------------------------------------------------------------------------------------------
# 레인

def lane_cmd(lane, job, args):
    dom = ox.DOMAIN0 + lane
    ns = f'amr_{dom}'
    cname = f'amr-ai-pr-ros-{lane}'
    bench = ['python3', f'{ox.SCRIPTS_C}/{job["bench_script"]}'] + job['bench'] + [
        '--out', job['out'], '--worker', ns, '--ns', '/@NS@', '--skip-done',
        '--commit', args.commit, '--dirty', str(args.dirty),
        '--ros-args', '-r', '__ns:=/@NS@', '--log-level', 'warn']
    envs = []
    for k, v in job.get('env', {}).items():
        envs += ['-e', f'{k}={v}']
    res = ['--cpus', str(args.cpus)]
    if getattr(args, 'e4', False):          # E4 측정: 레인마다 물리 코어 4 개 (논리 8 개)
        res = ['--cpuset-cpus', E4_CPUSETS[lane], '--cpus', '8']
    cmd = ['docker', 'run', '--rm', '--name', cname] + res + [
        '--memory', args.memory, '--shm-size', '2g',
        '-e', f'ROS_DOMAIN_ID={dom}', '-e', 'ROS_LOCALHOST_ONLY=1', '-e', 'ROS_WS=/ros2_ws'] + envs + [
        '-v', f'{job["ws"]}:/ros2_ws:ro', '-v', f'{ox.WS_HOST}:/pr_ws:ro', '-v', f'{RUNS_HOST}:/runs',
        ox.IMAGE, 'bash', f'{ox.SCRIPTS_C}/ros_worker_x.sh', str(dom), ns, ox.h2c(job['params']),
        job['launch_log']] + job['largs'] + ['--'] + bench
    return cmd, cname, dom


def running_containers():
    r = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'], capture_output=True, text=True)
    return set(r.stdout.split())


class R3Guard:
    """R3 작업 명령 주기 감시 (부록 A 4)."""

    PAT = re.compile(r'\] lane \d+ 끝 (\S+) rc=(\S+) 완료=\S+ (.*)$')

    def __init__(self):
        self.off = {}
        for fn in glob.glob(os.path.join(RUNS_HOST, 'r3', '*', 'orchestrate.log')):
            self.off[fn] = os.path.getsize(fn)
        self.paused = False
        self.good_run = 0
        self.idle_since = None
        self.trips = []

    def poll(self, mine):
        for fn in glob.glob(os.path.join(RUNS_HOST, 'r3', '*', 'orchestrate.log')):
            o = self.off.get(fn, 0)
            sz = os.path.getsize(fn)
            if sz <= o:
                continue
            with open(fn, encoding='utf-8', errors='replace') as f:
                f.seek(o)
                chunk = f.read()
            self.off[fn] = o + len(chunk.encode('utf-8'))
            for line in chunk.splitlines():
                m = self.PAT.search(line)
                if not m:
                    continue
                try:
                    res = ast.literal_eval(m.group(3))
                except (ValueError, SyntaxError):
                    res = None
                if not isinstance(res, dict) or 'cmd_rate_min' not in res:
                    continue
                stage = os.path.basename(os.path.dirname(fn))
                bad = res['cmd_rate_min'] < GUARD_TRIP or res.get('rate_low', 0) > 0
                good = res['cmd_rate_min'] >= GUARD_OK and res.get('rate_low', 0) == 0
                if bad:
                    self.good_run = 0
                    self.trips.append((time.time(), stage, m.group(1), res['cmd_rate_min'], res.get('rate_low')))
                    if not self.paused:
                        self.paused = True
                        jlog(f'R3 감시 멈춤: R3 {stage} 작업 {m.group(1)} cmd_rate_min {res["cmd_rate_min"]:.2f} Hz '
                             f'rate_low {res.get("rate_low")} → R3-C 새 작업 기동 중지 (도는 R3-C 작업 {len(mine)})')
                elif self.paused:
                    self.good_run = self.good_run + 1 if good else 0
                    if self.good_run >= 3:
                        self.paused = False
                        jlog(f'R3 감시 재개: 그 뒤 R3 작업 3 개 연속 ≥ {GUARD_OK} Hz (마지막 {stage} {m.group(1)} '
                             f'{res["cmd_rate_min"]:.2f} Hz)')
        if self.paused:
            r3_live = [c for c in running_containers() if re.fullmatch(r'amr-ai-pr-ros-(\d+)', c) and
                       int(c.rsplit('-', 1)[1]) < 14 and c not in mine]
            if r3_live:
                self.idle_since = None
            elif self.idle_since is None:
                self.idle_since = time.time()
            elif time.time() - self.idle_since > 120.0:
                self.paused = False
                jlog('R3 감시 재개: R3 레인 컨테이너(0–13)가 2 분 넘게 없음')
        return self.paused


def expand_ok(mine):
    sealed = all(os.path.exists(os.path.join(RUNS_HOST, 'r3', s, 'SEALED')) for s in ('c1', 'c2', 'c3', 'o'))
    if not sealed:
        return False
    live = [c for c in running_containers() if re.fullmatch(r'amr-ai-pr-ros-(\d+)', c) and
            int(c.rsplit('-', 1)[1]) < 14 and c not in mine]
    return not live


def run_lanes(jobs, args, tag):
    """ox.run_lanes 와 같은 방식 + 레인 집합·R3 감시·작업별 빌드/cpuset."""
    pending = [j for j in jobs if not ox.job_done(j)]
    pending.sort(key=lambda j: -j['est'])
    lanes = list(args.lane_list)
    print(f'[{tag}] 작업 {len(jobs)} 개 중 남은 것 {len(pending)} 개, 레인 {lanes}', flush=True)
    guard = R3Guard() if args.guard else None
    active = {}
    last_start = 0.0
    failed = []
    expanded = False
    last_check = 0.0
    while pending or active:
        mine = {a[6] for a in active.values()}
        paused = False
        if time.time() - last_check > 20.0:
            last_check = time.time()
            if guard is not None:
                paused = guard.poll(mine)
            if args.expand and not expanded and expand_ok(mine):
                expanded = True
                lanes = sorted(set(lanes) | set(LANES_EXT))
                jlog(f'R3-C {tag}: R3 held-out 네 단계 봉인·레인 0–13 비어 있음 확인 → 레인 0–13 도 쓴다 '
                     f'(총 {len(lanes)} 레인)')
                print(f'[{tag}] 레인 확장 {lanes}', flush=True)
        elif guard is not None:
            paused = guard.paused
        for lane in lanes:
            if paused or lane in active or not pending or time.time() - last_start < args.stagger:
                continue
            job = pending.pop(0)
            cmd, cname, dom = lane_cmd(lane, job, args)
            ox.docker_rm_if_mine(cname)
            os.makedirs(os.path.dirname(ox.c2h(job['out'])), exist_ok=True)
            blog = open(ox.c2h(job['bench_log']), 'a', encoding='utf-8')
            n0 = ox.nlines(ox.c2h(job['out']))
            p = subprocess.Popen(cmd, stdout=blog, stderr=subprocess.STDOUT)
            active[lane] = (p, job, time.time(), os.getloadavg(), n0, blog, cname, dom)
            last_start = time.time()
            print(f'[{tag}] lane {lane} 시작 {job["stage"]}/{job["id"]}', flush=True)
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
            out_h = ox.c2h(job['out'])
            res = ox.summarize(out_h, None, n0) if os.path.exists(out_h) else None
            ox.log_run({'id': f'r3c_{job["stage"]}_{job["id"]}', 'testbed': 'ros_kinematic',
                        'commit': args.commit, 'dirty': args.dirty,
                        'config': {'arms': job['arms'], 'params_sha': ox.sha(job['params']), 'domain': dom,
                                   'container': cname, 'largs': job['largs'], 'env': job.get('env', {}),
                                   'rc': rc, 'ws': job['ws'], 'bench': job['bench_script'],
                                   'cpuset': E4_CPUSETS.get(lane) if args.e4 else None},
                        'paths': job['ids'], 'seed': job.get('seed', 0), 'rep': job.get('rep', 0),
                        'start': t0, 'end': time.time(), 'raw': job['out'], 'result': res,
                        'loadavg': [l0, os.getloadavg()]})
            done = ox.job_done(job)
            print(f'[{tag}] lane {lane} 끝 {job["stage"]}/{job["id"]} rc={rc} 완료={done} {res}', flush=True)
            if not done:
                failed.append(f'{job["stage"]}/{job["id"]}')
            del active[lane]
        time.sleep(2.0)
    if guard is not None and guard.trips:
        print(f'[{tag}] R3 감시 멈춤 사건 {len(guard.trips)}: {guard.trips}', flush=True)
    return failed


def ros_stage(args):
    stages = args.stage.split(',')
    jobs = []
    for st in stages:
        sdh = os.path.join(RUNS_HOST, 'r3c', st)
        if os.path.exists(os.path.join(sdh, 'SEALED')):
            print(f'{sdh} 는 봉인됐다', file=sys.stderr)
            return 2
        js = stage_jobs(st, args)
        with open(os.path.join(sdh, 'jobs.json'), 'w', encoding='utf-8') as f:
            json.dump(js, f, indent=1)
        jobs += js
    args.lane_list = sorted(E4_CPUSETS) if args.e4 else list(LANES_BASE)
    failed = []
    for attempt in range(1 + args.retries):
        failed = run_lanes(jobs, args, f'r3c_{args.stage}' + (f'_retry{attempt}' if attempt else ''))
        if not failed:
            break
        jlog(f'R3-C {args.stage}: 실패 작업 {len(failed)} 개 ({", ".join(failed[:6])}) → --skip-done 으로 다시 돈다 '
             f'(시도 {attempt + 1})')
    print(json.dumps({'stage': f'r3c_{args.stage}', 'jobs': len(jobs), 'failed': failed}), flush=True)
    return 1 if failed else 0


# ---------------------------------------------------------------------------------------------
# chain_sim_x (E1 chain): dwa_R3 + 변형 × C1·C2·C3 대응 (R3 chain 과 같은 호출 꼴)

def chain_call(args, arm, scenes, seed, stateful, root):
    fam = scenes.split('_')[-1][:-4]
    cond = 'clean' if seed == 0 else f's{seed}'
    cid = f'{arm}__{fam}__{cond}__st{stateful}'
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
    return cid, {'id': f'r3c_chain_{cid}', 'testbed': 'chain_sim_x', 'commit': args.commit,
                 'dirty': args.dirty,
                 'config': {'arm': arm, 'cfg_sha': ox.sha(cfg), 'args': cmd[3:], 'rc': r.returncode,
                            'stderr': r.stderr[-500:]},
                 'paths': os.path.basename(scenes), 'paths_sha': ox.sha(scenes), 'seed': seed,
                 'stateful': stateful, 'start': t0, 'end': time.time(), 'raw': out,
                 'result': ox.summarize(out, arm) if os.path.exists(out) else None,
                 'loadavg': [l0, os.getloadavg()]}


def chain_inner(args):
    root = f'{R3C}/chain'
    for d in ('raw', 'resets', 'traces', 'cfg'):
        os.makedirs(f'{root}/{d}', exist_ok=True)
    zsel = args.zsel
    names = pc.write_cfgs(f'{root}/cfg', zsel)
    print('cfg', names, flush=True)
    arms = ['dwa_R3'] + pc.run_arms(zsel)
    calls = []
    for arm in arms:
        for sc in o2.set_scenes('d'):
            calls.append((arm, sc, 0, 0))
            calls.append((arm, sc, 0, 1))
            for seed in range(1, 6):
                calls.append((arm, sc, seed, 0))
    calls.sort(key=lambda c: (c[2] == 0, c[1]))
    entries = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.procs) as ex:
        futs = [ex.submit(chain_call, args, *c, root) for c in calls]
        for k, fu in enumerate(concurrent.futures.as_completed(futs)):
            cid, entry = fu.result()
            if entry != 'skip':
                entries.append(entry)
                with open(f'{root}/runs_chain.jsonl', 'a', encoding='utf-8') as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + '\n')
            print(f'[r3c chain] {k + 1}/{len(calls)} {cid}', flush=True)
    bad = [e['id'] for e in entries if e['config']['rc'] != 0]
    print(json.dumps({'calls': len(calls), 'ran': len(entries), 'rc_nonzero': bad}), flush=True)
    return 1 if bad else 0


def chain_stage(args):
    root_h = os.path.join(RUNS_HOST, 'r3c', 'chain')
    os.makedirs(root_h, exist_ok=True)
    zsel = selection()['DWA_R3']['chosen_z']
    name = 'amr-ai-pr-chain-r3c'
    ox.docker_rm_if_mine(name)
    fn = os.path.join(root_h, 'runs_chain.jsonl')
    n0 = ox.nlines(fn)
    cmd = ['docker', 'run', '--rm', '--name', name, '--cpus', str(args.procs), '--memory', '16g',
           '-e', 'ROS_WS=/ros2_ws', '-v', f'{args.ws}:/ros2_ws:ro', '-v', f'{ox.WS_HOST}:/pr_ws:ro',
           '-v', f'{RUNS_HOST}:/runs', ox.IMAGE, 'nice', '-n', '10',
           'python3', f'{ox.SCRIPTS_C}/orchestrate_r3c.py', 'chain-inner', '--commit', args.commit,
           '--dirty', str(args.dirty), '--procs', str(args.procs), '--zsel', zsel]
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
    ap.add_argument('cmd', choices=['ros', 'chain', 'chain-inner', 'jobs'])
    ap.add_argument('stage', nargs='?', default='')
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', type=int, required=True)
    ap.add_argument('--ws', default=ox.WS_HOST)
    ap.add_argument('--cpus', type=float, default=3.0)
    ap.add_argument('--memory', default='8g')
    ap.add_argument('--stagger', type=float, default=15.0)
    ap.add_argument('--procs', type=int, default=8)
    ap.add_argument('--zsel', default='')
    ap.add_argument('--retries', type=int, default=1)
    ap.add_argument('--no-guard', dest='guard', action='store_false')
    ap.add_argument('--expand', action='store_true', help='R3 held-out 봉인 뒤 레인 0–13 도 쓴다')
    ap.add_argument('--e4', action='store_true', help='E4 측정: 레인 14·15, 레인마다 cpuset')
    args = ap.parse_args()
    if args.cmd == 'ros':
        return ros_stage(args)
    if args.cmd == 'jobs':     # 작업 목록만 (확인용, 실행 안 함)
        for st in args.stage.split(','):
            js = stage_jobs(st, args)
            print(st, len(js), sum(len(j['ids']) * len(j['arms']) for j in js), js[0]['arms'])
        return 0
    if args.cmd == 'chain':
        return chain_stage(args)
    return chain_inner(args)


if __name__ == '__main__':
    sys.exit(main())

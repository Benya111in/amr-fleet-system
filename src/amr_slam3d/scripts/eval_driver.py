#!/usr/bin/env python3
"""
보너스 2 P 단계 재생 구동기 (호스트, 표준 라이브러리만) — 재개 가능, cpu_slot 안에서만 재생한다.

    eval_driver.py plan   --root R --stage e2          # 사전 등록한 작업 목록을 R/plans/<stage>.json 에 쓴다
    eval_driver.py run    --root R --wt WT             # nohup setsid 로 띄운다 (슬롯 일꾼 2)
    eval_driver.py status --root R

구조
- 작업 = (단계, bag, 행, 재생 번호). 출력 R/<단계>/<bag>/<행>/r<k>/ — 끝나면 DONE (rc·시각·loadavg).
- `run` 은 일꾼 둘(= cpu 슬롯 둘)을 돌린다. 일꾼은 `cpu_slot.sh b2-eval <이 파일> pool …` 로 슬롯을 잡고,
  pool 은 최대 K 개(R/control.json 의 k, 기본 4) 재생을 동시에 돌리다가 POOL_WALL_S 가 지나면 새 작업을 받지 않고
  돌던 것이 끝나면 슬롯을 놓는다 (다른 에이전트의 cpu_slot 대기를 굶기지 않게, 30 s 쉬고 다시 잡는다).
- 작업 집기는 `<출력>.claim` 디렉터리 mkdir (원자적) — 일꾼 둘이 같은 작업을 잡지 않는다. 구동기가 끊겨 남은
  claim 은 같은 이름 컨테이너가 없으면 부분 출력을 `<출력>.aborted.<시각>` 로 옮기고 다시 줄에 넣는다.
- 부하 규칙 (사전 등록): 새 재생은 (loadavg 1 분값 − 우리 컨테이너 CPU 코어 합) ≤ LOAD_MAX(48) 일 때만 띄운다.
  넘으면 30 s 마다 다시 본다 (기다린 기록은 R/load_wait.log). 우리 몫은 `docker stats` 의 amr-ai-b2-eval-* CPU%.
- 기동 실패 (구독 확인 시간 초과 rc 4, docker 오류 rc 125–127) 만 새 시도로 다시 돈다 (시도 상한 3, 이전 시도는
  `<출력>.attempt<n>` 로 남긴다). 재생이 돈 뒤의 결과(누락 > 0 등)는 다시 돌리지 않는다 — 분석이 실패로 센다.
- 의존: 작업의 `requires` 경로가 모두 있어야 집는다 (E3(b) 가 E2 의 지도를 기다린다).
- 재생 배속 = 목표 유효 sim 속도 E / 그 bag 의 녹화 RTF (recordings.md §4). E = 0.25 (사전 등록).
"""

import argparse
import datetime as dt
import fcntl
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

AGENT = 'b2-eval'
PREFIX = f'amr-ai-{AGENT}-'
CPU_SLOT = '/home/CAPYI_AI/env/cpu_slot.sh'
REC = '/home/CAPYI_AI/runs/b24/b2-rec/rec'
LOAD_MAX = 48.0
POOL_WALL_S = 3600
E_EFF = 0.25
DEFAULT_K = 4

#: bag 태그 → (녹화 RTF 실제, sim 구간 s, 사전 등록 여부) — recordings.md §3.1 표 그대로
BAGS = {
    's3_aisle_a1': (0.493, 146.1, True),
    's1_seed1_a1': (0.493, 167.4, True),
    's1_seed3_a2': (0.231, 224.4, True),
    's1_seed2x_a1': (0.203, 260.8, False),
    's6_open_a3': (0.249, 84.0, True),
    's6_queue_a1': (0.249, 84.2, True),
    's6_wall_a1': (0.248, 84.2, True),
    'map_long_a1': (0.248, 695.6, True),
}
#: 행 → (이미지, bag 디렉터리). rows.py 와 같아야 한다 (test_eval_driver.py 가 대조한다)
ROW_IMAGE = {
    'slam_toolbox': ('amr-ai/slam:0.1', 'bag'),
    'rtabmap2d_noimu': ('amr-ai/slam:0.1', 'bag'),
    'rtabmap3d_noimu': ('amr-ai/slam:0.1', 'bag'),
    'liosam_planar': ('amr-ai/liosam:0.1', 'bag_madg'),
    'liosam_loop_re': ('amr-ai/liosam:0.1', 'bag_madg'),
    'liosam_oracle': ('amr-ai/liosam:0.1', 'bag_madg'),
}
RETRY_RC = {4, 125, 126, 127}
MAX_ATTEMPTS = 3


def rate_for(bag: str, e_eff: float = E_EFF) -> float:
    """재생 배속 = E / 녹화 RTF (소수 셋째 자리)."""
    return round(e_eff / BAGS[bag][0], 3)


def now() -> str:
    """지역 시각 ISO."""
    return dt.datetime.now().astimezone().isoformat(timespec='seconds')


def load1() -> float:
    """1 분 loadavg."""
    return os.getloadavg()[0]


def loadavg() -> str:
    """'1 5 15' 분 loadavg 문자열."""
    return ' '.join(f'{v:.2f}' for v in os.getloadavg())


# ---------------------------------------------------------------- 작업 목록 (사전 등록)
def job(stage, bag, row, k, pri, domain, image=None, kind=None, env=None, requires=(),
        mounts=(), rate=None):
    """작업 한 건 (dict)."""
    img, knd = ROW_IMAGE.get(row, (None, None))
    return {'id': f'{stage}/{bag}/{row}/r{k}', 'stage': stage, 'bag': bag, 'row': row,
            'replay': k, 'priority': pri, 'domain': domain, 'image': image or img,
            'kind': kind or knd, 'rate': rate if rate is not None else rate_for(bag),
            'env': dict(env or {}), 'requires': list(requires), 'mounts': list(mounts)}


E2_ROWS_MAIN = ('slam_toolbox', 'rtabmap2d_noimu', 'rtabmap3d_noimu', 'liosam_planar')
E2_ROWS_REF = ('liosam_loop_re',)          # 6DoF 참고
E2_ROWS_AUX = ('liosam_oracle',)           # 오라클 상한 (보조)


def plan_e2():
    """E2·E3(a) 매핑: 행 6 × bag 8 × 재생 3. map_long 의 slam_toolbox·rtabmap3d 가 먼저 (E3(b) 지도)."""
    jobs = []
    for k in range(3):
        for row in E2_ROWS_MAIN + E2_ROWS_REF + E2_ROWS_AUX:
            for bag in BAGS:
                if bag == 'map_long_a1' and row in ('slam_toolbox', 'rtabmap3d_noimu'):
                    pri = 10                            # E3(b) 가 쓸 지도
                elif row in E2_ROWS_MAIN:
                    pri = 20 + k
                elif row in E2_ROWS_REF:
                    pri = 30 + k
                else:
                    pri = 40 + k
                jobs.append(job('e2', bag, row, k, pri, domain=45))
    return jobs


PLANS = {'e2': plan_e2}


def load_jobs(root):
    """R/plans/*.json 의 작업 전부 (우선순위·id 순)."""
    jobs = []
    pdir = os.path.join(root, 'plans')
    if not os.path.isdir(pdir):
        return jobs
    for name in sorted(os.listdir(pdir)):
        if name.endswith('.json'):
            with open(os.path.join(pdir, name), encoding='utf-8') as f:
                jobs += json.load(f)
    jobs.sort(key=lambda j: (j['priority'], j['id']))
    return jobs


def control(root):
    """R/control.json — k(슬롯당 동시 재생), pause, load_max."""
    try:
        with open(os.path.join(root, 'control.json'), encoding='utf-8') as f:
            c = json.load(f)
    except (OSError, ValueError):
        c = {}
    return {'k': int(c.get('k', DEFAULT_K)), 'pause': bool(c.get('pause', False)),
            'load_max': float(c.get('load_max', LOAD_MAX))}


def log(root, name, msg):
    """R/<name> 에 한 줄."""
    with open(os.path.join(root, name), 'a', encoding='utf-8') as f:
        f.write(f'{now()} {msg}\n')


# ---------------------------------------------------------------- 컨테이너
def cname(j):
    """작업 → 컨테이너 이름 amr-ai-b2-eval-<단계>-<bag>-<행>-r<k>."""
    return PREFIX + re.sub(r'[^A-Za-z0-9_.-]', '-', j['id'].replace('/', '-'))


def running_containers():
    """돌고 있는 우리 컨테이너 이름."""
    out = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'], capture_output=True,
                         text=True).stdout.split()
    return {n for n in out if n.startswith(PREFIX)}


def own_cpu_cores():
    """우리 컨테이너의 CPU 사용 합 [코어] (docker stats 한 번)."""
    out = subprocess.run(['docker', 'stats', '--no-stream', '--format',
                          '{{.Name}} {{.CPUPerc}}'], capture_output=True, text=True).stdout
    tot, per = 0.0, {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].startswith(PREFIX):
            try:
                v = float(parts[1].rstrip('%')) / 100.0
            except ValueError:
                continue
            per[parts[0]] = v
            tot += v
    return tot, per


def out_dir(root, j):
    """작업 출력 디렉터리."""
    return os.path.join(root, j['id'])


def is_done(root, j):
    """DONE 이 있으면 끝난 작업."""
    return os.path.isfile(os.path.join(out_dir(root, j), 'DONE'))


def attempts(root, j):
    """이전 기동 실패 시도 수 (.attempt<n>)."""
    base = out_dir(root, j)
    d = os.path.dirname(base)
    if not os.path.isdir(d):
        return 0
    pat = re.compile(re.escape(os.path.basename(base)) + r'\.attempt\d+$')
    return sum(1 for n in os.listdir(d) if pat.match(n))


def try_claim(root, j, live):
    """작업을 잡는다 (True). 이미 끝났거나 다른 일꾼이 잡았으면 False. 낡은 claim 은 정리한다."""
    o = out_dir(root, j)
    if is_done(root, j):
        return False
    os.makedirs(os.path.dirname(o), exist_ok=True)
    claim = o + '.claim'
    try:
        os.mkdir(claim)
        return True
    except FileExistsError:
        pass
    if cname(j) in live:
        return False
    # 낡은 claim: 컨테이너가 없다 — 다른 일꾼이 막 잡고 아직 띄우기 전일 수 있으니 120 s 지난 것만
    try:
        age = time.time() - os.path.getmtime(claim)
    except OSError:
        return False
    if age < 120:
        return False
    stamp = dt.datetime.now().strftime('%Y%m%dT%H%M%S')
    if os.path.isdir(o):
        os.rename(o, f'{o}.aborted.{stamp}')
    shutil.rmtree(claim, ignore_errors=True)
    log(root, 'driver.log', f'STALE {j["id"]} — 부분 출력 .aborted.{stamp}')
    try:
        os.mkdir(claim)
        return True
    except FileExistsError:
        return False


def wt_head(wt):
    """
    작업공간의 (HEAD, git status) — 스냅샷(git archive 사본)이면 SNAPSHOT_HEAD 파일.

    재생은 실행 중 바뀌지 않는 사본에서 돈다: bash 는 스크립트를 읽어 가며 실행하므로 작업 중인 worktree 를 마운트하면
    E3(b)·E4 코드를 고치는 동안 돌던 재생이 바뀐 replay_row.sh·rows.py 를 읽을 수 있다.
    """
    snap = os.path.join(wt, 'SNAPSHOT_HEAD')
    if os.path.isfile(snap):
        with open(snap, encoding='utf-8') as f:
            return f.read().strip(), 'snapshot (git archive, 미커밋 변경 없음)\n'
    head = subprocess.run(['git', '-C', wt, 'rev-parse', 'HEAD'], capture_output=True,
                          text=True).stdout.strip()
    st = subprocess.run(['git', '-C', wt, 'status', '--porcelain'], capture_output=True,
                        text=True).stdout
    return head, st


def start_job(root, wt, j):
    """출력 디렉터리에 HEAD·status·job.json 을 쓰고 컨테이너를 띄운다 (Popen). 작업에 wt 가 있으면 그 사본."""
    wt = j.get('wt') or wt
    o = out_dir(root, j)
    os.makedirs(o, exist_ok=True)
    head, st = wt_head(wt)
    with open(os.path.join(o, 'HEAD.txt'), 'w', encoding='utf-8') as f:
        f.write(head + '\n')
    with open(os.path.join(o, 'status.txt'), 'w', encoding='utf-8') as f:
        f.write(st)
    with open(os.path.join(o, 'job.json'), 'w', encoding='utf-8') as f:
        json.dump(dict(j, started=now(), load_start=loadavg(), head=head,
                       dirty_src=sum(1 for ln in st.splitlines() if ' src/' in ln)), f,
                  ensure_ascii=False, indent=1)
    bagdir = os.path.join(REC, j['bag'], j['kind'])
    cmd = ['docker', 'run', '--rm', '--name', cname(j), '--shm-size=2g',
           '-e', f'ROS_DOMAIN_ID={j["domain"]}', '-e', 'ROS_LOCALHOST_ONLY=1',
           '-e', f'GIT_HEAD={head}', '-e', f'IMAGE={j["image"]}',
           '-e', f'REPLAY_ID={AGENT}-{j["id"]}']
    for k, v in j['env'].items():
        cmd += ['-e', f'{k}={v}']
    cmd += ['-v', f'{wt}:/ros2_ws:ro', '-v', f'{bagdir}:/data/bag:ro', '-v', f'{o}:/out']
    for m in j['mounts']:
        cmd += ['-v', m]
    cmd += ['-w', '/ros2_ws', j['image'], 'bash', '-lc',
            f'/ros2_ws/src/amr_slam3d/scripts/replay_row.sh {j["row"]} /data/bag /out {j["rate"]}']
    host = open(os.path.join(o, 'host.log'), 'w', encoding='utf-8')
    p = subprocess.Popen(cmd, stdout=host, stderr=subprocess.STDOUT)
    log(root, 'driver.log', f'START {j["id"]} rate={j["rate"]} load={loadavg()} pid={p.pid}')
    return p, host


def finish_job(root, j, rc):
    """DONE 을 쓰고, 기동 실패면 출력을 .attempt<n> 으로 옮겨 다시 줄에 넣는다."""
    o = out_dir(root, j)
    with open(os.path.join(o, 'DONE'), 'w', encoding='utf-8') as f:
        json.dump({'rc': rc, 'ended': now(), 'load_end': loadavg()}, f)
    log(root, 'driver.log', f'END {j["id"]} rc={rc} load={loadavg()}')
    if rc in RETRY_RC and attempts(root, j) + 1 < MAX_ATTEMPTS:
        n = attempts(root, j) + 1
        os.rename(o, f'{o}.attempt{n}')
        log(root, 'driver.log', f'RETRY {j["id"]} rc={rc} → .attempt{n} (기동 실패만 다시)')
    shutil.rmtree(o + '.claim', ignore_errors=True)


def ready(j):
    """의존 경로가 모두 있나."""
    return all(os.path.exists(p) for p in j['requires'])


# ---------------------------------------------------------------- pool (슬롯 안)
def cmd_pool(ns):
    """슬롯 하나 안에서 최대 k 재생을 돌린다 (POOL_WALL_S 뒤 새 작업을 받지 않고 끝낸다)."""
    root, wt = ns.root, ns.wt
    t0 = time.monotonic()
    running = {}                                       # id → (job, Popen, file)
    log(root, 'driver.log', f'POOL{ns.worker} ACQ load={loadavg()}')
    while True:
        for jid, (j, p, fh) in list(running.items()):
            rc = p.poll()
            if rc is not None:
                fh.close()
                finish_job(root, j, rc)
                del running[jid]
        c = control(root)
        accepting = (time.monotonic() - t0 < POOL_WALL_S) and not c['pause']
        if not accepting and not running:
            break
        if accepting and len(running) < c['k']:
            live = running_containers()
            jobs = [j for j in load_jobs(root) if not is_done(root, j) and ready(j)
                    and j['id'] not in running and cname(j) not in live]
            if not jobs and not running:
                break
            if jobs:
                own, _ = own_cpu_cores()
                other = load1() - own
                if other > c['load_max']:
                    log(root, 'load_wait.log', f'POOL{ns.worker} wait other={other:.1f} '
                                               f'own={own:.1f} load={loadavg()}')
                else:
                    for j in jobs:
                        if try_claim(root, j, live):
                            p, fh = start_job(root, wt, j)
                            running[j['id']] = (j, p, fh)
                            time.sleep(5)               # 기동 겹침을 조금 푼다
                            break
                    else:
                        time.sleep(30)
                    continue
        time.sleep(30)
    log(root, 'driver.log', f'POOL{ns.worker} REL load={loadavg()}')
    return 0


# ---------------------------------------------------------------- run (일꾼 둘)
def remaining(root):
    """끝나지 않은 작업."""
    return [j for j in load_jobs(root) if not is_done(root, j)]


def worker(ns, i, stop):
    """슬롯 일꾼 — 남은 작업이 있는 동안 cpu_slot 으로 pool 을 되풀이한다."""
    while not stop.is_set():
        rem = remaining(ns.root)
        if not rem:
            break
        if not any(ready(j) for j in rem) or control(ns.root)['pause']:
            time.sleep(60)
            continue
        subprocess.run([CPU_SLOT, AGENT, sys.executable, os.path.abspath(__file__), 'pool',
                        '--root', ns.root, '--wt', ns.wt, '--worker', str(i)])
        time.sleep(30)


def sampler(ns, stop):
    """60 s 마다 loadavg 와 우리 컨테이너 CPU 를 R/cpu_samples.jsonl 에 (공유 부하 하 예비값)."""
    path = os.path.join(ns.root, 'cpu_samples.jsonl')
    while not stop.is_set():
        own, per = own_cpu_cores()
        with open(path, 'a', encoding='utf-8') as f:
            f.write(json.dumps({'t': now(), 'load': loadavg(), 'own_cores': round(own, 3),
                                'per': {k[len(PREFIX):]: round(v, 3) for k, v in per.items()}})
                    + '\n')
        stop.wait(60)


def cmd_run(ns):
    """구동기 (하나만 — driver.lock)."""
    fd = os.open(os.path.join(ns.root, 'driver.lock'), os.O_CREAT | os.O_RDWR)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit('구동기가 이미 돈다 (driver.lock)')
    os.write(fd, f'{os.getpid()}\n'.encode())
    log(ns.root, 'driver.log', f'RUN pid={os.getpid()} wt={ns.wt} load={loadavg()}')
    stop = threading.Event()
    ts = [threading.Thread(target=worker, args=(ns, i, stop)) for i in range(ns.workers)]
    sm = threading.Thread(target=sampler, args=(ns, stop), daemon=True)
    sm.start()
    for t in ts:
        t.start()
        time.sleep(10)
    for t in ts:
        t.join()
    stop.set()
    log(ns.root, 'driver.log', f'RUN-END load={loadavg()}')
    return 0


def cmd_plan(ns):
    """사전 등록 작업 목록을 R/plans/<단계>.json 에."""
    jobs = PLANS[ns.stage]()
    os.makedirs(os.path.join(ns.root, 'plans'), exist_ok=True)
    path = os.path.join(ns.root, 'plans', f'{ns.stage}.json')
    if os.path.exists(path) and not ns.force:
        sys.exit(f'이미 있다: {path} (--force)')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(jobs, f, ensure_ascii=False, indent=1)
    print(f'{len(jobs)} 작업 → {path}')
    return 0


def cmd_status(ns):
    """단계별 done / claimed / todo 수."""
    by = {}
    for j in load_jobs(ns.root):
        st = 'done' if is_done(ns.root, j) else (
            'claimed' if os.path.isdir(out_dir(ns.root, j) + '.claim') else 'todo')
        by[(j['stage'], st)] = by.get((j['stage'], st), 0) + 1
    for k in sorted(by):
        print(k, by[k])
    return 0


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('plan')
    s.add_argument('--root', required=True)
    s.add_argument('--stage', required=True, choices=sorted(PLANS))
    s.add_argument('--force', action='store_true')
    s = sub.add_parser('run')
    s.add_argument('--root', required=True)
    s.add_argument('--wt', required=True)
    s.add_argument('--workers', type=int, default=2)
    s = sub.add_parser('pool')
    s.add_argument('--root', required=True)
    s.add_argument('--wt', required=True)
    s.add_argument('--worker', type=int, default=0)
    s = sub.add_parser('status')
    s.add_argument('--root', required=True)
    ns = ap.parse_args(argv)
    return {'plan': cmd_plan, 'run': cmd_run, 'pool': cmd_pool, 'status': cmd_status}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))

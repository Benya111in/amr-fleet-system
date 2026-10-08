#!/usr/bin/env python3
"""
보너스 4 P5 드라이버 (호스트, 표준 라이브러리) — 재개 가능: 끝난 짝(<원자료>/pairs/<실험>_p<k>.done)은 건너뛴다.

반드시 `CPU_SLOTS=0 /home/CAPYI_AI/env/exclusive.sh b4-p5 python3 p5_driver.py …` 안에서
(gz 슬롯 둘을 함께 쥔다).
갈래마다: 계획 · 통신 모의 파일 → 스택 컨테이너(gz_slot.sh 경유, 포그라운드 자식) → 서버 컨테이너 1 (같은 네트워크 · IPC
이름공간) → clock.json 을 보며 장애 주입(docker kill · pause · unpause · 새 서버) → 스택이 끝나면 서버 정리 → 분석 · 실행 목록.
짝: 두 갈래를 동시에(--concurrent) 또는 차례로. 먼저 띄우는 갈래와 도메인을 짝마다 번갈아 (p5_plan.schedule).
무효 짝(준비 시간 초과 · /clock 정지 · 감시 중단)은 새 태그(…r<n>)로 다시 돈다. 같은 짝 연속 무효 3 번이면 그 실험을 멈추고 진단.
멈추려면: touch <원자료>/STOP (다음 짝 전에 멈춘다).
"""

import argparse
import datetime as dt
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p5_plan  # noqa: E402

IMG = 'amr-ai/rl:0.3'
AGENT = 'b4-p5'


def now_iso():
    return dt.datetime.now().astimezone().isoformat(timespec='seconds')


def load3():
    return ' '.join(f'{v:.2f}' for v in os.getloadavg())


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


class Driver:
    def __init__(self, a):
        self.a = a
        self.raw = a.raw
        os.makedirs(os.path.join(self.raw, 'pairs'), exist_ok=True)
        self.logf = os.path.join(self.raw, 'driver.log')

    def log(self, msg):
        line = f'{now_iso()} {msg}'
        with open(self.logf, 'a', encoding='utf-8') as f:
            f.write(line + '\n')
        print(line, flush=True)

    # ------------------------------------------------------------------ 갈래
    def ws_for(self, layout):
        return self.a.ws_old if layout == 'old' else self.a.ws_new

    def run_arm(self, exp, pair, arm_index, tag, domain, result):
        d = os.path.join(self.raw, tag)
        os.makedirs(d, exist_ok=True)
        e = p5_plan.EXPERIMENTS[exp]
        arm = e['arms'][arm_index]
        ws = self.ws_for(arm['layout'])
        plan_path = os.path.join(d, 'plan.json')
        comm_path = os.path.join(d, 'comm.yaml') if arm['comm'] else ''
        args = ['write', '--exp', exp, '--pair', str(pair), '--arm', str(arm_index),
                '--out', plan_path, '--t-start', str(self.a.t_start)]
        if comm_path:
            args += ['--comm-out', comm_path]
        if p5_plan.main(args) != 0:
            result['rc'] = 'plan'
            return
        commit = self.ws_commit(ws)
        with open(os.path.join(d, 'HEAD.txt'), 'w', encoding='utf-8') as f:
            f.write(f'{commit}\n갈래 {arm["label"]} ({arm["mode"]}, 배치 {arm["layout"]}), '
                    f'워크스페이스 {ws} (git archive 사본)\n')
        diff_src = os.path.join(ws, 'ARCHIVE_DIFF.txt')
        with open(os.path.join(d, 'diff_stat.txt'), 'w', encoding='utf-8') as f:
            if os.path.isfile(diff_src):
                with open(diff_src, encoding='utf-8') as g:
                    f.write(g.read())
            elif os.path.exists(os.path.join(ws, '.git')):
                f.write('git diff --stat (작업 트리):\n'
                        + sh(['git', '-C', ws, 'diff', '--stat']).stdout
                        + sh(['git', '-C', ws, 'status', '--porcelain']).stdout)
            else:
                f.write('git diff --stat: 없음 (git archive 사본)\n')
        img = sh(['docker', 'image', 'inspect', '-f', '{{.Id}}', IMG]).stdout.strip()
        with open(os.path.join(d, 'config.txt'), 'w', encoding='utf-8') as f:
            f.write(f'exp={exp} pair={pair} arm={arm_index} label={arm["label"]} '
                    f'mode={arm["mode"]} layout={arm["layout"]} domain={domain} '
                    f'partition=b4p5_{tag} image={IMG} {img} '
                    f't_start={self.a.t_start} comm={arm["comm"]} faults={arm["faults"]}\n')
        self.snap(os.path.join(d, 'start.txt'))
        stack = f'amr-ai-{AGENT}-gz-{tag}'
        rawc = f'/raw/{tag}'
        envs = ['-e', f'P5_TAG={tag}', '-e', f'P5_MODE={arm["mode"]}', '-e', f'P5_DOMAIN={domain}',
                '-e', f'P5_OUT={rawc}', '-e', f'P5_PLAN={rawc}/plan.json']
        if comm_path:
            envs += ['-e', f'P5_COMM={rawc}/comm.yaml']
        slot = ['/home/CAPYI_AI/env/gz_slot.sh', AGENT]
        if self.a.dryrun:
            envs += ['-e', 'P5_DRYRUN=1']
            slot = []                       # Gazebo 없음 — 슬롯을 쥐지 않는다
        cmd = slot + [
            'docker', 'run', '--rm', '--name', stack, '--gpus', 'all', '--shm-size=8g',
            '--ipc', 'shareable', '-v', f'{ws}:/ros2_ws', '-v', f'{self.raw}:/raw'] + envs + [
            '-w', '/ros2_ws', IMG, 'bash', '/ros2_ws/tests/integration/fleet_p5/p5_container.sh']
        out = open(os.path.join(d, 'runner.out'), 'w', encoding='utf-8')
        t_wall0 = time.time()
        proc = subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT)
        servers = {}
        faults_f = open(os.path.join(d, 'faults.jsonl'), 'a', buffering=1, encoding='utf-8')
        clock = {}

        def action_log(rec):
            rec = dict(rec, wall=round(time.time(), 3), iso=now_iso(),
                       sim=clock.get('sim'), t_base=clock.get('t_base'))
            faults_f.write(json.dumps(rec, ensure_ascii=False) + '\n')

        def server(k, extra):
            name = f'amr-ai-{AGENT}-srv-{tag}-{k}'
            senv = ['-e', f'P5_TAG={tag}', '-e', f'P5_MODE={arm["mode"]}',
                    '-e', f'P5_DOMAIN={domain}', '-e', f'P5_OUT={rawc}', '-e', f'P5_SERVER={k}']
            if comm_path:
                senv += ['-e', f'P5_COMM={rawc}/comm.yaml']
            so = open(os.path.join(d, f'srv{k}.out'), 'w', encoding='utf-8')
            p = subprocess.Popen(['docker', 'run', '--rm', '--name', name, '--network',
                                  f'container:{stack}', '--ipc', f'container:{stack}',
                                  '-v', f'{ws}:/ros2_ws', '-v', f'{self.raw}:/raw'] + senv + [
                '-w', '/ros2_ws', IMG, 'bash', '/ros2_ws/tests/integration/fleet_p5/p5_server.sh'],
                stdout=so, stderr=subprocess.STDOUT)
            servers[k] = (name, p)
            action_log(dict(extra, action='start', server=k, container=name))

        # 스택 컨테이너가 뜨기를 기다린 뒤 서버 1
        for _ in range(600):
            if proc.poll() is not None:
                break
            if sh(['docker', 'ps', '-q', '-f', f'name=^{stack}$']).stdout.strip():
                break
            time.sleep(1)
        if proc.poll() is None:
            time.sleep(3)
            server(1, {'scheduled': None})
        pending = sorted(arm['faults'], key=lambda f: f[0])
        ps_done = False
        over_since = None
        last_change = time.time()
        last_mtime = None
        aborted = None
        clock_path = os.path.join(d, 'clock.json')
        while proc.poll() is None:
            time.sleep(1.0)
            try:
                mt = os.path.getmtime(clock_path)
                with open(clock_path, encoding='utf-8') as f:
                    clock.update(json.load(f))
            except (OSError, ValueError):
                mt = None
            if mt != last_mtime:
                last_mtime, last_change = mt, time.time()
            tb, sim = clock.get('t_base'), clock.get('sim')
            while pending and tb is not None and sim is not None and sim >= tb + pending[0][0]:
                off, action, k = pending.pop(0)
                rec = {'scheduled': off, 'scheduled_sim': tb + off}
                if action == 'start':
                    server(k, rec)
                elif action in ('agent_kill', 'agent_restart'):
                    pre = f'{rawc}/agent_{k}'
                    ctl = ['python3', '/ros2_ws/tests/integration/fleet_p5/p5_agent_ctl.py']
                    if action == 'agent_kill':
                        r = sh(['docker', 'exec', stack] + ctl + ['kill', k, pre])
                    else:
                        r = sh(['docker', 'exec', '-d', stack] + ctl + [
                            'restart', pre, f'{rawc}/stack_env.sh'])
                    action_log(dict(rec, action=action, robot=k, rc=r.returncode,
                                    out=(r.stdout + r.stderr).strip()[:300]))
                else:
                    name = servers.get(k, (f'amr-ai-{AGENT}-srv-{tag}-{k}', None))[0]
                    r = sh(['docker', action, name])
                    action_log(dict(rec, action=action, server=k, rc=r.returncode,
                                    out=(r.stdout + r.stderr).strip()[:300]))
            if not ps_done and time.time() - t_wall0 > self.a.ps_after_s:
                # 스택 컨테이너 프로세스 표 (관리자 · 교통 관리자가 스택에 없어야 한다 — 서버는 다른 컨테이너)
                r = sh(['docker', 'exec', stack, 'ps', '-eo', 'pid,etime,pcpu,rss,args'])
                with open(os.path.join(d, 'ps_stack.txt'), 'w', encoding='utf-8') as f:
                    f.write(f'{now_iso()} sim={clock.get("sim")}\n' + r.stdout + r.stderr)
                r = sh(['docker', 'exec', stack, 'df', '-k'])     # shm 마운트 사용량 (UDP 전용이면 ~0)
                with open(os.path.join(d, 'df_stack.txt'), 'w', encoding='utf-8') as f:
                    f.write(f'{now_iso()}\n' + r.stdout + r.stderr)
                ps_done = True
            if clock.get('state') == 'END':
                last_change = time.time()
            stall = time.time() - last_change
            if stall > self.a.stall_s and not aborted:
                aborted = f'clock.json 이 {stall:.0f} s 동안 바뀌지 않음 (기록기 정지로 봄)'
            if time.time() - t_wall0 > self.a.arm_timeout_s and not aborted:
                aborted = f'갈래 상한 {self.a.arm_timeout_s:.0f} s 초과'
            l5 = os.getloadavg()[1]
            if l5 > self.a.load_abort:
                over_since = over_since or time.time()
                if time.time() - over_since >= self.a.load_abort_s and not aborted:
                    aborted = (f'부하 중단: loadavg(5 분) > {self.a.load_abort:g} 가 '
                               f'{self.a.load_abort_s:.0f} s 이어짐 (지금 {load3()})')
                    result['load_abort'] = True
            else:
                over_since = None
            if aborted:
                with open(os.path.join(d, 'ABORTED.txt'), 'w', encoding='utf-8') as f:
                    f.write(f'중단(감시): {aborted} — {now_iso()}\n')
                self.log(f'{tag} 중단: {aborted}')
                self.diag(stack, d)
                sh(['docker', 'stop', '-t', '90', stack])
                break
        rc = proc.wait()
        for k, (name, p) in servers.items():
            sh(['docker', 'unpause', name])
            sh(['docker', 'kill', name])
            try:
                p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                p.kill()
        faults_f.close()
        out.close()
        with open(os.path.join(d, 'rc.txt'), 'w', encoding='utf-8') as f:
            f.write(f'rc={rc}\n')
        self.snap(os.path.join(d, 'end.txt'))
        result.update({'rc': rc, 'aborted': aborted, 'started': dt.datetime.fromtimestamp(
            t_wall0).astimezone().isoformat(timespec='seconds'), 'ended': now_iso(),
            'commit': commit, 'domain': domain})

    @staticmethod
    def ws_commit(ws):
        """워크스페이스 코드 커밋: git archive 사본의 ARCHIVE_COMMIT.txt, 없으면 git rev-parse (+dirty)."""
        path = os.path.join(ws, 'ARCHIVE_COMMIT.txt')
        if os.path.isfile(path):
            with open(path, encoding='utf-8') as f:
                return f.read().strip()
        head = sh(['git', '-C', ws, 'rev-parse', 'HEAD']).stdout.strip() or '?'
        dirty = bool(sh(['git', '-C', ws, 'status', '--porcelain']).stdout.strip())
        return head + (' (dirty)' if dirty else '')

    def snap(self, path):
        gpu = sh(['nvidia-smi', '--query-gpu=memory.used,utilization.gpu',
                  '--format=csv,noheader']).stdout.strip().split('\n')[0]
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'{now_iso()}\n{load3()}\n{gpu}\n')

    def diag(self, stack, d):
        r = sh(['docker', 'exec', stack, 'ps', '-eo', 'pid,ppid,stat,wchan:32,etime,pcpu,rss,args',
                '--sort=pid'])
        with open(os.path.join(d, 'stall_ps.txt'), 'w', encoding='utf-8') as f:
            f.write(r.stdout + r.stderr)

    # ------------------------------------------------------------------ 분석 · 등록
    def analyze(self, exp, pair, tag, res, label, mode):
        d = os.path.join(self.raw, tag)
        info = {'run_id': f'P5_{tag}', 'stage': f'P5-{exp}', 'agent': AGENT, 'kind': 'gazebo',
                'sim': 'gazebo', 'commit': str(res.get('commit', '?'))[:7], 'dirty': False,
                'config': f'tests/integration/fleet_p5 실험 {exp} 짝 {pair} 갈래 {label} ({mode}), '
                          f'plan {d}/plan.json, 도메인 {res.get("domain")}',
                'seed': pair, 'started': res.get('started', now_iso()),
                'ended': res.get('ended', now_iso()), 'raw_dir': d,
                'label': f'{exp}/{label}', 'load': self.load_text(d)}
        with open(os.path.join(d, 'registry_info.json'), 'w', encoding='utf-8') as f:
            json.dump(info, f, ensure_ascii=False)
        r = sh(['docker', 'run', '--rm', '--name', f'amr-ai-{AGENT}-ana-{tag}',
                '--network', 'none',
                '-v', f'{HERE}:/p5:ro', '-v', f'{self.raw}:/raw', IMG, 'python3',
                '/p5/p5_analyze.py', 'run', f'/raw/{tag}', '--registry-info',
                f'/raw/{tag}/registry_info.json'])
        with open(os.path.join(d, 'analyze.out'), 'w', encoding='utf-8') as f:
            f.write(r.stdout + r.stderr)
        reg = os.path.join(d, 'registry.json')
        if os.path.isfile(reg):
            with open(reg, encoding='utf-8') as f:
                rec = json.load(f)
            rec['raw_dir'] = d               # 컨테이너 안 경로가 아니라 호스트 절대 경로
            with open(reg, 'w', encoding='utf-8') as f:
                json.dump(rec, f, ensure_ascii=False, indent=1)
            rr = sh(['python3', os.path.join(self.a.repo, 'scripts/notebook.py'), '--nb',
                     os.path.join(self.a.repo, 'docs/research/fleet-distributed/notebook'),
                     'register', '--file', reg])
            self.log(f'{tag} 분석 rc={r.returncode} {r.stdout.strip()[-200:]} | 등록 '
                     f'{(rr.stdout + rr.stderr).strip()[-120:]}')
        try:
            with open(os.path.join(d, 'metrics.json'), encoding='utf-8') as f:
                return bool(json.load(f).get('valid'))
        except (OSError, ValueError):
            return False

    def load_text(self, d):
        parts = []
        for name in ('start.txt', 'end.txt'):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                with open(p, encoding='utf-8') as f:
                    lines = f.read().split('\n')
                parts.append(name.split('.')[0] + ' ' + (lines[1] if len(lines) > 1 else '?'))
        return '; '.join(parts) + ' (loadavg 1/5/15 분, 공유 서버)'

    # ------------------------------------------------------------------ 짝
    def wait_quiet(self, key):
        t0, ok = time.time(), 0
        while True:
            l1, l5, l15 = os.getloadavg()
            ok = ok + 1 if (l1 < self.a.load_max and l5 < self.a.load5_max
                            and l15 < self.a.load15_max) else 0
            if ok >= 2 or time.time() - t0 > self.a.load_wait_max_s:
                break
            if os.path.exists(os.path.join(self.raw, 'STOP')):
                return False
            time.sleep(15)
        top = sh(['ps', '-eo', 'pcpu,user,comm', '--sort=-pcpu']).stdout.split('\n')[:8]
        cont = sh(['docker', 'ps', '--format', '{{.Names}} {{.Image}}']).stdout
        with open(os.path.join(self.raw, 'pairs', f'{key}.start_condition'), 'a',
                  encoding='utf-8') as f:
            f.write(f'{now_iso()} waited {time.time() - t0:.0f} s, load {load3()} (기준 load1 < '
                    f'{self.a.load_max} · load5 < {self.a.load5_max} · load15 < '
                    f'{self.a.load15_max} 두 번 연속, 상한 '
                    f'{self.a.load_wait_max_s:.0f} s)\n'
                    + '\n'.join(top) + '\n--- 컨테이너\n' + cont)
        self.log(f'{key} 시작 조건 (대기 {time.time() - t0:.0f} s, load {load3()})')
        return True

    def run_pair(self, exp, k, first, doms):
        key = f'{exp}_p{k}'
        if os.path.exists(os.path.join(self.raw, 'pairs', key + '.done')):
            return 'done'
        if os.path.exists(os.path.join(self.raw, 'STOP')):
            return 'stop'
        e = p5_plan.EXPERIMENTS[exp]
        n = 1
        while any(os.path.exists(os.path.join(self.raw, self.tag(exp, k, n, a['label'])))
                  for a in e['arms']):
            n += 1
        if not self.wait_quiet(key):
            return 'stop'
        order = [first, 1 - first]
        tags = {i: self.tag(exp, k, n, e['arms'][i]['label']) for i in (0, 1)}
        results = {0: {}, 1: {}}
        self.log(f'{key} 시작 (시도 {n}): ' + ', '.join(
            f'{tags[i]}(도메인 {doms[j]})' for j, i in enumerate(order))
            + (' 동시' if self.a.concurrent else ' 차례로'))
        threads = []
        for j, i in enumerate(order):
            if not self.a.concurrent and j == 1 and results[order[0]].get('load_abort'):
                self.log(f'{key}: 첫 갈래가 부하 중단 — 둘째 갈래를 띄우지 않는다')
                results[i].update({'rc': 'skipped', 'load_abort': True})
                os.makedirs(os.path.join(self.raw, tags[i]), exist_ok=True)
                with open(os.path.join(self.raw, tags[i], 'ABORTED.txt'), 'w',
                          encoding='utf-8') as f:
                    f.write(f'띄우지 않음: 첫 갈래 부하 중단 — {now_iso()}\n')
                break
            th = threading.Thread(target=self.run_arm, args=(exp, k, i, tags[i], doms[j],
                                                             results[i]))
            th.start()
            threads.append(th)
            if self.a.concurrent:
                time.sleep(2)
            else:
                th.join()
        for th in threads:
            th.join()
        valid = [self.analyze(exp, k, tags[i], results[i], e['arms'][i]['label'],
                              e['arms'][i]['mode']) for i in (0, 1)]
        if any(results[i].get('load_abort') for i in (0, 1)):
            self.log(f'{key} 부하 중단 — 짝 무효, 연속 실패 수에 넣지 않고 다시 돈다')
            return 'load'
        self.log(f'{key} 끝: 유효 {valid} rc {[results[i].get("rc") for i in (0, 1)]}')
        if all(valid):
            with open(os.path.join(self.raw, 'pairs', key + '.done'), 'w', encoding='utf-8') as f:
                f.write(json.dumps({'tags': tags, 'ended': now_iso()}))
            return 'ok'
        return 'invalid'

    @staticmethod
    def tag(exp, k, n, label):
        return f'p5_{exp}_p{k}_{label}' + ('' if n == 1 else f'r{n}')

    def analyze_exp(self, exp):
        r = sh(['docker', 'run', '--rm', '--name', f'amr-ai-{AGENT}-ana-{exp}',
                '--network', 'none', '-v', f'{HERE}:/p5:ro', '-v', f'{self.raw}:/raw', IMG,
                'python3', '/p5/p5_analyze.py', 'exp', exp, '/raw', '--out', '/raw/analysis'])
        self.log(f'{exp} 실험 분석 rc={r.returncode}')

    def run(self):
        self.log(f'드라이버 시작 pid {os.getpid()} EXCLUSIVE_HOLDER='
                 f'{os.environ.get("EXCLUSIVE_HOLDER")} load {load3()} 순서 {self.a.order} '
                 f'동시={self.a.concurrent} t_start={self.a.t_start}')
        stopped_exp = set()
        lo, hi = (int(v) for v in self.a.pair_range.split('-'))
        for exp, k, first, doms in p5_plan.schedule(self.a.order.split(',')):
            if exp in stopped_exp or not lo <= k <= hi:
                continue
            fails = 0
            while True:
                r = self.run_pair(exp, k, first, doms)
                if r in ('ok', 'done'):
                    break
                if r == 'stop':
                    self.log('STOP 파일 — 멈춤')
                    return 1
                if r == 'load':
                    continue                    # 환경(다른 사용자 부하) 탓 — 시작 조건을 다시 기다린다
                fails += 1
                if fails >= 3:
                    self.log(f'{exp}_p{k}: 무효 3 번 연속 — 이 실험을 멈추고 진단 (재시도 안 함)')
                    stopped_exp.add(exp)
                    break
            if k == p5_plan.EXPERIMENTS[exp]['pairs']:
                self.analyze_exp(exp)
        self.log('드라이버 끝')
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--raw', default='/home/CAPYI_AI/runs/b24/b4-p5/raw')
    ap.add_argument('--ws-new', required=True)
    ap.add_argument('--ws-old', required=True)
    ap.add_argument('--repo', default='/home/CAPYI_AI/wt/b4-p5')
    ap.add_argument('--order', default=','.join(p5_plan.ORDER))
    ap.add_argument('--t-start', type=float, default=p5_plan.T_START)
    ap.add_argument('--concurrent', action='store_true')
    ap.add_argument('--pair-range', default='1-99', help='짝 번호 범위 a-b (파일럿 단독/동시 나누기)')
    ap.add_argument('--dryrun', action='store_true',
                    help='파이프라인 시험 (Gazebo 없음, 실험 dry): 스택 대신 가짜 /clock')
    ap.add_argument('--stall-s', type=float, default=900.0)
    ap.add_argument('--ps-after-s', type=float, default=420.0,
                    help='갈래 시작 뒤 이만큼(벽시계) 지나 스택 컨테이너 프로세스 표를 남긴다')
    ap.add_argument('--arm-timeout-s', type=float, default=4 * 3600.0)
    ap.add_argument('--load-max', type=float, default=30.0,
                    help='짝 시작 조건 loadavg(1 분) < 이 값 두 번 연속 (사전 등록 8 → 결과 뒤 30, 노트 01:50)')
    ap.add_argument('--load15-max', type=float, default=80.0,
                    help='짝 시작 조건에 loadavg(15 분) < 이 값도 — 다른 사용자 일괄 작업이 멈춘 때만 (노트 03:05)')
    ap.add_argument('--load5-max', type=float, default=50.0,
                    help='짝 시작 조건에 loadavg(5 분) < 이 값도 (잠깐 내려간 1 분 값에 속지 않게, 노트 02:45)')
    ap.add_argument('--load-wait-max-s', type=float, default=10800.0)
    ap.add_argument('--load-abort', type=float, default=120.0,
                    help='갈래 중 loadavg(5 분)가 이 값을 load-abort-s 동안 넘으면 갈래 중단 · 짝 다시')
    ap.add_argument('--load-abort-s', type=float, default=600.0)
    a = ap.parse_args(argv)
    return Driver(a).run()


if __name__ == '__main__':
    sys.exit(main())

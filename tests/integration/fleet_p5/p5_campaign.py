#!/usr/bin/env python3
"""
보너스 4 P5 캠페인 (호스트, 표준 라이브러리) — 감독 에이전트가 끊겨도 측정이 이어지게 (COORDINATION §10).

  python3 p5_campaign.py wait   … : b2-rec 녹화 끝(아래 조건)까지 기다린 뒤
      `CPU_SLOTS=0 exclusive.sh b4-p5 python3 p5_campaign.py run …` 으로 넘어간다.
  python3 p5_campaign.py run    … : (exclusive 안) 파일럿 짝 1 차례로 → 짝 2 동시 → 사전 등록 규칙으로 동시 실행 · T_START
                                    결정(raw/pilot_decision.json) → 점검 통과면 본 실행(p5_plan.ORDER) 드라이버.

b2-rec 녹화 끝 조건 (COORDINATION §10.1): ① 명령줄에 '/wt/b2-rec/' 와 'rec_driver' 또는
'rec_attempt' 가 있는 프로세스가 없고
② b2-rec 작업 트리의 docs/research/slam3d/recordings.md §0 이 자리표시("(녹화 뒤 채운다)")가 아니다 — 둘 다 10 분 연속.
수동 진행: <원자료 루트>/GO 파일이 있으면 조건을 건너뛴다 (오케스트레이터 · 감독 에이전트 결정용, 로그에 남긴다).

파일럿 규칙 (사전 등록 항목 「파일럿」): 짝 2 의 두 갈래 RTF 가 모두 ≥ 0.6 × 짝 1 같은 모드 RTF 이고 짝 2 두 갈래가 유효면 동시,
아니면 차례로. T_START = max(T_FLOOR, 30 의 배수로 올림(1.25 × 파일럿 네 갈래 t_ready 최댓값)) — T_FLOOR 은 아래 주석.
점검 (도구 결함 감지 — 지표 값으로 규칙을 바꾸지 않는다): 파일럿 네 갈래 모두 유효, 정답 자세 표본 > 0,
분산 갈래 에이전트 상태 기록 있음, 완료 작업 ≥ 1, 스택 컨테이너에 관리자 · 교통 관리자가 없음(서버 컨테이너에만).
하나라도 어긋나면 본 실행을 시작하지 않고 멈춘다 (raw/CAMPAIGN_HALT.txt).
"""

import argparse
import datetime as dt
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import p5_plan  # noqa: E402

B2_WT = '/home/CAPYI_AI/wt/b2-rec'
B2_DOC = os.path.join(B2_WT, 'docs/research/slam3d/recordings.md')
PLACEHOLDER = '(녹화 뒤 채운다)'
# T_START 하한 [sim s]. 사전 등록은 240 이었다 — 파일럿 첫 갈래 준비가 sim 9.4 s 라 240 이면 갈래마다 sim ~230 s(벽시계 ~15 분)를
# 그냥 기다린다. 결과 뒤 변경 (운영 파라미터 — 지표 · 판정과 무관): 하한 = imu_filter bias_estimation_time 60 s (정지 평균 창).
T_FLOOR = 60.0


def now_iso():
    return dt.datetime.now().astimezone().isoformat(timespec='seconds')


def log(raw, msg):
    line = f'{now_iso()} {msg}'
    os.makedirs(raw, exist_ok=True)
    with open(os.path.join(raw, 'campaign.log'), 'a', encoding='utf-8') as f:
        f.write(line + '\n')
    print(line, flush=True)


def b2_recording_active() -> bool:
    out = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True).stdout
    return any('/wt/b2-rec/' in a and ('rec_driver' in a or 'rec_attempt' in a)
               for a in out.split('\n'))


def b2_doc_done(path: str = B2_DOC) -> bool:
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return False
    i = text.find('## 0.')
    j = text.find('\n## ', i + 1)
    sec = text[i:j] if i >= 0 else ''
    return bool(sec) and PLACEHOLDER not in sec


def wait_b2(raw, stable_s=600.0, poll_s=60.0):
    since = None
    last_state = None
    while True:
        if os.path.exists(os.path.join(raw, 'GO')):
            log(raw, 'GO 파일 — b2-rec 조건을 건너뛴다')
            return
        active, done = b2_recording_active(), b2_doc_done()
        state = (active, done)
        if state != last_state:
            log(raw, f'b2-rec 녹화 프로세스 {"있음" if active else "없음"}, recordings.md §0 '
                     f'{"채워짐" if done else "자리표시"}')
            last_state = state
        if not active and done:
            since = since or time.time()
            if time.time() - since >= stable_s:
                log(raw, f'b2-rec 녹화 끝 조건 {stable_s:.0f} s 유지 — 진행')
                return
        else:
            since = None
        time.sleep(poll_s)


def driver(a, extra):
    cmd = [sys.executable, os.path.join(HERE, 'p5_driver.py'), '--raw', a.raw,
           '--ws-new', a.ws_new, '--ws-old', a.ws_old, '--repo', a.repo,
           '--load-wait-max-s', str(a.load_wait_max_s)] + extra
    log(a.raw, 'driver: ' + ' '.join(cmd))
    return subprocess.run(cmd).returncode


def metrics(raw, tag):
    try:
        with open(os.path.join(raw, tag, 'metrics.json'), encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def latest_tag(raw, exp, k, label):
    base = f'p5_{exp}_p{k}_{label}'
    cands = [n for n in os.listdir(raw)
             if n == base or (n.startswith(base + 'r') and n[len(base) + 1:].isdigit())]
    return sorted(cands, key=lambda n: (len(n), n))[-1] if cands else None


def decide(raw):
    """사전 등록 파일럿 규칙 → (동시 여부, T_START, 점검 문제 목록, 근거 dict)."""
    arms = {}
    for k in (1, 2):
        for a in p5_plan.EXPERIMENTS['pilot']['arms']:
            tag = latest_tag(raw, 'pilot', k, a['label'])
            arms[(k, a['mode'])] = (tag, metrics(raw, tag) if tag else None)
    problems = []
    for (k, mode), (tag, m) in sorted(arms.items()):
        if m is None:
            problems.append(f'pilot 짝 {k} {mode}: metrics 없음 ({tag})')
            continue
        if not m.get('valid'):
            problems.append(f'pilot 짝 {k} {mode}: 무효 ({m.get("why")}, 중단 {m.get("aborted")})')
        if not m.get('distance_m') or not any(v > 0 for v in m['distance_m'].values()):
            problems.append(f'pilot 짝 {k} {mode}: 정답 자세 이동 없음')
        if (m.get('completed') or 0) < 1:
            problems.append(f'pilot 짝 {k} {mode}: 완료 작업 0')
        if mode == 'distributed' and not m.get('mode_transitions'):
            problems.append(f'pilot 짝 {k} {mode}: 에이전트 상태 기록 없음')
        if m.get('stack_has_server') is not False:
            problems.append(f'pilot 짝 {k} {mode}: 스택 컨테이너 프로세스 표 없음 또는 관리자 · 교통 '
                            f'관리자가 스택에도 떴다 (stack_has_server={m.get("stack_has_server")})')
    rtf = {key: (m or {}).get('rtf') for key, (_, m) in arms.items()}
    ready = [(m or {}).get('t_ready') for _, m in arms.values()]
    conc = all(rtf.get((2, md)) is not None and rtf.get((1, md)) is not None
               and rtf[(2, md)] >= 0.6 * rtf[(1, md)] for md in ('central', 'distributed'))
    conc = conc and all((arms[(2, md)][1] or {}).get('valid') for md in ('central', 'distributed'))
    tr = [t for t in ready if t is not None]
    t_start = max(T_FLOOR, 30.0 * math.ceil(1.25 * max(tr) / 30.0)) if tr else None
    if t_start is None:
        problems.append('t_ready 없음 — T_START 를 정할 수 없다')
    basis = {'rtf': {f'{k}/{md}': v for (k, md), v in rtf.items()}, 't_ready': ready,
             'tags': {f'{k}/{md}': tag for (k, md), (tag, _) in arms.items()},
             'rule_concurrent': '짝 2 두 갈래 RTF ≥ 0.6 × 짝 1 같은 모드 RTF, 짝 2 두 갈래 유효',
             'rule_t_start': f'max({T_FLOOR:g}, ceil30(1.25 × max t_ready))'}
    return conc, t_start, problems, basis


def run(a):
    raw = a.raw
    log(raw, f'캠페인 run 시작 EXCLUSIVE_HOLDER={os.environ.get("EXCLUSIVE_HOLDER")} '
             f'load {os.getloadavg()}')
    if not a.skip_pilot:
        tp = ['--t-start', str(T_FLOOR)]           # 파일럿도 하한 시각에 게시 (셈 밖 — RTF · t_ready 만 본다)
        driver(a, ['--order', 'pilot', '--pair-range', '1-1'] + tp)
        driver(a, ['--order', 'pilot', '--pair-range', '2-2', '--concurrent'] + tp)
    conc, t_start, problems, basis = decide(raw)
    dec = {'decided': now_iso(), 'concurrent': conc, 't_start': t_start, 'problems': problems,
           'basis': basis}
    with open(os.path.join(raw, 'pilot_decision.json'), 'w', encoding='utf-8') as f:
        json.dump(dec, f, ensure_ascii=False, indent=1)
    log(raw, f'파일럿 결정: 동시={conc} T_START={t_start} 문제={problems}')
    if problems:
        with open(os.path.join(raw, 'CAMPAIGN_HALT.txt'), 'w', encoding='utf-8') as f:
            f.write(f'{now_iso()} 파일럿 점검 실패 — 본 실행 시작 안 함:\n' + '\n'.join(problems) + '\n')
        log(raw, '본 실행 시작 안 함 (CAMPAIGN_HALT.txt)')
        return 3
    extra = ['--order', a.order, '--t-start', str(t_start)] + (['--concurrent'] if conc else [])
    return driver(a, extra)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['wait', 'run', 'decide'])
    ap.add_argument('--raw', default='/home/CAPYI_AI/runs/b24/b4-p5/raw')
    ap.add_argument('--ws-new', required=True)
    ap.add_argument('--ws-old', required=True)
    ap.add_argument('--repo', default='/home/CAPYI_AI/wt/b4-p5')
    ap.add_argument('--order', default=','.join(p5_plan.ORDER))
    ap.add_argument('--skip-pilot', action='store_true', help='파일럿이 이미 끝났을 때 (재개)')
    ap.add_argument('--load-wait-max-s', type=float, default=300.0,
                    help='짝 시작 전 loadavg < 8 을 기다리는 상한 [s] — 넘으면 그대로 시작하고 기록')
    a = ap.parse_args(argv)
    if a.cmd == 'decide':
        print(json.dumps(decide(a.raw), ensure_ascii=False, indent=1, default=str))
        return 0
    if a.cmd == 'wait':
        wait_b2(a.raw)
        cmd = ['/home/CAPYI_AI/env/exclusive.sh', 'b4-p5', sys.executable,
               os.path.abspath(__file__),
               'run', '--raw', a.raw, '--ws-new', a.ws_new, '--ws-old', a.ws_old, '--repo', a.repo,
               '--order', a.order, '--load-wait-max-s', str(a.load_wait_max_s)] + (
            ['--skip-pilot'] if a.skip_pilot else [])
        log(a.raw, 'exclusive: ' + ' '.join(cmd))
        return subprocess.run(cmd, env=dict(os.environ, CPU_SLOTS='0')).returncode
    return run(a)


if __name__ == '__main__':
    sys.exit(main())

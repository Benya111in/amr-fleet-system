#!/usr/bin/env python3
"""
단계 C 실행기 상태 도우미 (표준 라이브러리만 — 호스트에서 c_lane.sh·c_master.sh 가 부른다).

    c_state.py status <묶음 디렉터리> <start> <count>
        → 'DONE <n>' | 'FAILED <사유>' | 'TODO <첫 빠진 exec_index> <남은 수> <진전 없는 연속 시도 수>'
        묶음 디렉터리의 시도들(a1, a2, …)의 gzbench.csv 를 합쳐 [start, start+count) 의 exec_index 가 다 있으면 DONE
        표지(DONE 파일)를 쓴다. 진전 없는 시도가 3 번 이어지면 FAILED 표지를 쓴다 (같은 실패 세 번 — 진단 몫).
    c_state.py attempt-end <시도 디렉터리>
        → 시도가 끝난 뒤 행 수를 rows.txt 에 남긴다 (진전 판정용).
    c_state.py gate <gate 디렉터리>
        → 'READY <seed> <모델 파일 이름> <sha256>' | 'WAIT <사유>' (RL 팔을 돌려도 되는가: rl_ready.json 이 있고
        STOP_RL 이 없고 모델 sha256 이 맞을 때만)
    c_state.py pilot-decide <파일럿 디렉터리>
        → 사전 등록 파일럿 규칙: 레인별 RTF 중앙(오류 아닌 에피소드)이 둘 다 ≥ 0.75 면 레인 둘, 아니면 레인 하나.
        레인마다 20 행이 아니거나 오류 > 2 면 멈춤(진단). DECISION.json 을 쓴다.
    c_state.py lists <일정 tsv> <레인 목록 '35 36' 또는 '35'> <출력 디렉터리>
        → 레인마다 'batch ctrl start count' 목록 파일 (레인 하나면 pos 순으로 35·36 을 번갈아 합친다).
"""

import csv
import glob
import hashlib
import json
import os
import statistics
import sys
import time

MAX_NO_PROGRESS = 3
PILOT_RTF_MIN = 0.75
PILOT_N = 20
PILOT_MAX_ERR = 2


def rows_of(batch_dir):
    """묶음의 모든 시도 gzbench.csv 행 (시도 순서대로)."""
    out = []
    for a in sorted(glob.glob(os.path.join(batch_dir, 'a*')),
                    key=lambda p: int(os.path.basename(p)[1:] or 0)):
        f = os.path.join(a, 'gzbench.csv')
        if os.path.exists(f):
            with open(f, encoding='utf-8', newline='') as fh:
                for r in csv.DictReader(fh):
                    r['_attempt'] = os.path.basename(a)
                    out.append(r)
    return out


def attempts(batch_dir):
    """시도 디렉터리 목록 (번호 순)."""
    return sorted(glob.glob(os.path.join(batch_dir, 'a*')),
                  key=lambda p: int(os.path.basename(p)[1:] or 0))


def status(batch_dir, start, count):
    """묶음 상태 문자열 (DONE·FAILED 표지를 쓸 수 있다)."""
    if os.path.exists(os.path.join(batch_dir, 'DONE')):
        return 'DONE ' + open(os.path.join(batch_dir, 'DONE')).read().split()[0]
    if os.path.exists(os.path.join(batch_dir, 'FAILED')):
        txt = open(os.path.join(batch_dir, 'FAILED')).read().strip()
        return 'FAILED ' + txt.replace('\n', ' ')
    want = set(range(start, start + count))
    have = {int(r['exec_index']) for r in rows_of(batch_dir)
            if r.get('exec_index') not in (None, '')}
    missing = sorted(want - have)
    if not missing:
        with open(os.path.join(batch_dir, 'DONE'), 'w', encoding='utf-8') as f:
            f.write(f'{len(want)} {time.strftime("%Y-%m-%dT%H:%M:%S%z")} '
                    f'attempts={len(attempts(batch_dir))}\n')
        return f'DONE {len(want)}'
    streak = 0
    for a in reversed(attempts(batch_dir)):
        rt = os.path.join(a, 'rows.txt')
        if not os.path.exists(rt):
            break
        if int(open(rt).read().split()[0]) > 0:
            break
        streak += 1
    if streak >= MAX_NO_PROGRESS:
        with open(os.path.join(batch_dir, 'FAILED'), 'w', encoding='utf-8') as f:
            f.write(f'진전 없는 시도 {streak} 번 연속 — 진단 필요 ({time.strftime("%F %T")})\n')
        return f'FAILED no_progress_{streak}'
    return f'TODO {missing[0]} {start + count - missing[0]} {streak}'


def attempt_end(attempt_dir):
    """시도 끝: 행 수 기록."""
    f = os.path.join(attempt_dir, 'gzbench.csv')
    n = 0
    if os.path.exists(f):
        with open(f, encoding='utf-8', newline='') as fh:
            n = sum(1 for _ in csv.DictReader(fh))
    with open(os.path.join(attempt_dir, 'rows.txt'), 'w', encoding='utf-8') as fh:
        fh.write(f'{n}\n')
    return str(n)


def sha256(path):
    """파일 sha256."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def gate(gate_dir):
    """RL 팔 문 (READY / WAIT)."""
    if os.path.exists(os.path.join(gate_dir, 'STOP_RL')):
        return 'WAIT STOP_RL'
    f = os.path.join(gate_dir, 'rl_ready.json')
    if not os.path.exists(f):
        return 'WAIT no_rl_ready'
    g = json.load(open(f, encoding='utf-8'))
    model = os.path.join(os.path.dirname(os.path.abspath(gate_dir)), 'models', g['model_file'])
    if not os.path.exists(model) or sha256(model) != g['sha256']:
        return 'WAIT model_sha_mismatch'
    return f'READY {g["seed"]} {g["model_file"]} {g["sha256"]}'


def num(x):
    """CSV 칸 → float (빈 칸 nan)."""
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def is_error(r):
    """하네스 오류 행."""
    return r.get('setup_ok') not in ('True', '1') or r.get('outcome_contract') == 'error'


def pilot_decide(pilot_dir):
    """파일럿 규칙 → DECISION.json."""
    lanes = {}
    for lane in (35, 36):
        att = attempts(os.path.join(pilot_dir, f'l{lane}'))
        rs = []
        if att:     # 마지막 시도만 (주 드라이버가 파일럿 도중 다시 떴으면 앞 시도는 버려진 부분 실행)
            rs = [r for r in rows_of(os.path.join(pilot_dir, f'l{lane}'))
                  if r['_attempt'] == os.path.basename(att[-1])]
        ok = [r for r in rs if not is_error(r)]
        rtf = [num(r['rtf']) for r in ok if num(r['rtf']) == num(r['rtf'])]
        lanes[lane] = {'n': len(rs), 'errors': len(rs) - len(ok),
                       'rtf_median': statistics.median(rtf) if rtf else None,
                       'rtf_min': min(rtf) if rtf else None, 'rtf_max': max(rtf) if rtf else None}
    d = {'rule': f'레인별 RTF 중앙 ≥ {PILOT_RTF_MIN} 이면 레인 둘, 아니면 레인 하나; '
                 f'레인마다 {PILOT_N} 행·오류 ≤ {PILOT_MAX_ERR} 아니면 멈춤',
         'lanes_detail': lanes, 'decided': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    bad = [lane for lane, v in lanes.items()
           if v['n'] != PILOT_N or v['errors'] > PILOT_MAX_ERR or v['rtf_median'] is None]
    if bad:
        d['lanes'] = []
        d['stop'] = f'파일럿 레인 {bad} 이 완주·오류 조건을 못 맞췄다 — 진단'
    elif all(v['rtf_median'] >= PILOT_RTF_MIN for v in lanes.values()):
        d['lanes'] = [35, 36]
    else:
        d['lanes'] = [35]
    with open(os.path.join(pilot_dir, 'DECISION.json'), 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=1)
    return json.dumps(d, ensure_ascii=False)


def lists(schedule, lanes, out_dir):
    """레인별 목록 파일."""
    by = {}
    with open(schedule, encoding='utf-8') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            lane, pos, b, ctrl, start, count = line.split()
            by.setdefault(int(lane), []).append((int(pos), b, ctrl, start, count))
    for v in by.values():
        v.sort()
    lanes = [int(x) for x in lanes.split()]
    os.makedirs(out_dir, exist_ok=True)
    if len(lanes) == 2:
        plan = {lane: by[lane] for lane in lanes}
    else:
        merged = []
        for i in range(max(len(v) for v in by.values())):
            for lane in sorted(by):
                if i < len(by[lane]):
                    merged.append(by[lane][i])
        plan = {lanes[0]: merged}
    for lane, items in plan.items():
        with open(os.path.join(out_dir, f'lane{lane}.list'), 'w', encoding='utf-8') as f:
            for _, b, ctrl, start, count in items:
                f.write(f'{b} {ctrl} {start} {count}\n')
    return ' '.join(str(x) for x in plan)


def main():
    """명령행."""
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == 'status':
        print(status(args[0], int(args[1]), int(args[2])))
    elif cmd == 'attempt-end':
        print(attempt_end(args[0]))
    elif cmd == 'gate':
        print(gate(args[0]))
    elif cmd == 'pilot-decide':
        print(pilot_decide(args[0]))
    elif cmd == 'lists':
        print(lists(args[0], args[1], args[2]))
    else:
        print(f'모르는 명령 {cmd}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
